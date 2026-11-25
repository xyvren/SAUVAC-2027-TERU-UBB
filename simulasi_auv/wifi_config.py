"""Loopback-host clients for Pico W UDP control and authenticated OTA updates."""

import base64
import json
import math
import re
import secrets
import socket
import threading
import time
import urllib.error
import urllib.request


PICO_WIFI_ADDRESS = ("192.168.4.1", 4210)
PICO_FIRMWARE_UPDATE_URL = "http://192.168.4.1/update"
MAX_REQUEST_BYTES = 384
MAX_RESPONSE_BYTES = 1500
MAX_FIRMWARE_IMAGE_BYTES = 800 * 1024
MAX_REQUEST_ID = 2_147_483_647
MIN_REQUEST_INTERVAL_SECONDS = 0.05
WIFI_OPERATIONS = {
    "get",
    "command",
    "mission_upload",
    "set_pid",
    "set_thruster",
    "set_power_limit",
    "set_deadband",
    "set_balance",
    "set_autorun_timing",
    "apply",
    "save",
    "discard",
}


class WifiConfigError(RuntimeError):
    def __init__(self, message, outcome_unknown=False):
        super().__init__(message)
        self.outcome_unknown = outcome_unknown


class PicoFirmwareUpdateError(RuntimeError):
    def __init__(self, message, status_code=502, outcome_unknown=False):
        super().__init__(message)
        self.status_code = status_code
        self.outcome_unknown = outcome_unknown


def upload_firmware_via_wifi(image, password, filename="firmware.bin"):
    """Relay one compiled Pico W .bin image to the authenticated OTA endpoint."""
    if not isinstance(image, bytes) or not 1 <= len(image) <= MAX_FIRMWARE_IMAGE_BYTES:
        raise ValueError(
            f"Firmware image must contain 1-{MAX_FIRMWARE_IMAGE_BYTES} bytes"
        )
    if (
        not isinstance(filename, str)
        or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,99}\.bin", filename, re.I)
    ):
        raise ValueError("Firmware filename must be a plain .bin filename")
    if (
        not isinstance(password, str)
        or not 12 <= len(password) <= 63
        or not password.isascii()
        or any(ord(char) < 32 or ord(char) > 126 for char in password)
    ):
        raise ValueError("Pico Wi-Fi password must be 12-63 printable ASCII characters")
    if image.startswith(b"UF2\n"):
        raise ValueError("UF2 images must be flashed through BOOTSEL, not OTA")

    boundary = "----SAUVAC-" + secrets.token_hex(16)
    multipart = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
        "Content-Type: application/octet-stream\r\n\r\n"
    ).encode("ascii") + image + f"\r\n--{boundary}--\r\n".encode("ascii")
    credentials = f"sauv-update:{password}".encode("ascii")
    request = urllib.request.Request(
        PICO_FIRMWARE_UPDATE_URL,
        data=multipart,
        headers={
            "Authorization": "Basic "
            + base64.b64encode(credentials).decode("ascii"),
            "Content-Type": f"multipart/form-data; boundary={boundary}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            response_body = response.read(4096).decode("utf-8", errors="replace")
            if response.status != 202:
                raise PicoFirmwareUpdateError(
                    response_body or f"Pico OTA endpoint returned HTTP {response.status}",
                    status_code=response.status,
                )
            return {"accepted": True, "message": response_body}
    except urllib.error.HTTPError as error:
        message = error.read(4096).decode("utf-8", errors="replace")
        raise PicoFirmwareUpdateError(
            message or f"Pico OTA endpoint rejected the image (HTTP {error.code})",
            status_code=error.code,
        ) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise PicoFirmwareUpdateError(
            "Pico Wi-Fi update connection ended without confirmation; device outcome is unknown",
            status_code=504,
            outcome_unknown=True,
        ) from error


def _is_finite_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _is_integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_operation(payload):
    if not isinstance(payload, dict):
        raise ValueError("Request must be a JSON object")
    operation = payload.get("op")
    if not isinstance(operation, str) or operation not in WIFI_OPERATIONS:
        raise ValueError("Unsupported Pico Wi-Fi configuration operation")

    fields = {
        "get": set(),
        "command": {"command"},
        "mission_upload": {"json"},
        "apply": set(),
        "save": set(),
        "discard": set(),
        "set_pid": {"axis", "kp", "ki", "kd"},
        "set_thruster": {
            "id", "reverse", "trim", "pulse_min", "pulse_neutral", "pulse_max"
        },
        "set_power_limit": {"percent"},
        "set_deadband": {"microseconds"},
        "set_balance": {"axis", "percent"},
        "set_autorun_timing": {"delay_sec", "tare_sec", "level_sec"},
    }
    if set(payload) - {"op"} - fields[operation]:
        raise ValueError("Unsupported fields for Pico Wi-Fi configuration operation")

    if operation in {"get", "apply", "save", "discard"}:
        if len(payload) != 1:
            raise ValueError(f"{operation} does not accept parameters")
        return
    if operation == "command":
        command = payload.get("command")
        if not isinstance(command, str) or not 1 <= len(command) <= 256:
            raise ValueError("command must contain 1-256 characters")
        if any(marker in command for marker in ("\n", "\r", ";", "&&", "|", "`")):
            raise ValueError("command must be a single safe line")
    elif operation == "mission_upload":
        mission_json = payload.get("json")
        if not isinstance(mission_json, str) or not 2 <= len(mission_json) <= 3500:
            raise ValueError("mission JSON must contain 2-3500 characters")
    elif operation == "set_pid":
        axis = payload.get("axis")
        if not isinstance(axis, str) or axis not in {"depth", "roll"}:
            raise ValueError("axis must be depth or roll; pitch is unsupported")
        for key in ("kp", "ki", "kd"):
            value = payload.get(key)
            if not _is_finite_number(value) or not 0 <= value <= 100:
                raise ValueError(f"{key} must be a finite number from 0 to 100")
    elif operation == "set_thruster":
        thruster_id = payload.get("id")
        if not _is_integer(thruster_id) or not 1 <= thruster_id <= 6:
            raise ValueError("id must be an integer from 1 to 6")
        supplied = set(payload) - {"op", "id"}
        if not supplied:
            raise ValueError("set_thruster requires at least one setting")
        if "reverse" in payload and not isinstance(payload["reverse"], bool):
            raise ValueError("reverse must be boolean")
        if "trim" in payload and not _is_finite_number(payload["trim"]):
            raise ValueError("trim must be a finite number")
        for key in ("pulse_min", "pulse_neutral", "pulse_max"):
            if key in payload and (
                not _is_integer(payload[key]) or not 0 <= payload[key] <= 65_535
            ):
                raise ValueError(f"{key} must be an integer from 0 to 65535")
    elif operation == "set_power_limit":
        value = payload.get("percent")
        if not _is_integer(value) or not 1 <= value <= 100:
            raise ValueError("percent must be an integer from 1 to 100")
    elif operation == "set_deadband":
        value = payload.get("microseconds")
        if not _is_integer(value) or not 0 <= value <= 80:
            raise ValueError("microseconds must be an integer from 0 to 80")
    elif operation == "set_balance":
        axis = payload.get("axis")
        if not isinstance(axis, str) or axis not in {"horiz", "vert"}:
            raise ValueError("axis must be horiz or vert")
        value = payload.get("percent")
        if not _is_finite_number(value) or not -20 <= value <= 20:
            raise ValueError("percent must be a finite number from -20 to 20")
    elif operation == "set_autorun_timing":
        supplied = set(payload) - {"op"}
        if not supplied:
            raise ValueError("set_autorun_timing requires at least one timing value")
        for key in supplied:
            value = payload[key]
            if not _is_integer(value) or not 0 <= value <= 255:
                raise ValueError(f"{key} must be an integer from 0 to 255")


class PicoWifiConfigClient:
    """Exchange protocol-v1 JSON datagrams without exposing arbitrary UDP."""

    def __init__(
        self,
        socket_factory=None,
        timeout=0.5,
        read_retries=2,
        max_request_bytes=MAX_REQUEST_BYTES,
    ):
        if timeout <= 0 or read_retries < 0 or max_request_bytes < 1:
            raise ValueError("timeout, retry count, and request limit must be positive")
        self.socket_factory = socket_factory
        self.timeout = timeout
        self.read_retries = read_retries
        self.max_request_bytes = max_request_bytes
        self._lock = threading.Lock()
        self._next_request_id = 0
        self._last_request_at = None

    def _new_request_id(self):
        self._next_request_id = (self._next_request_id + 1) % (MAX_REQUEST_ID + 1)
        return self._next_request_id

    def request(self, payload):
        _validate_operation(payload)
        operation = payload["op"]
        with self._lock:
            request_id = self._new_request_id()
            message = {"req_id": request_id, **payload}
            encoded = json.dumps(
                message, separators=(",", ":"), allow_nan=False
            ).encode("utf-8")
            if len(encoded) > self.max_request_bytes:
                raise ValueError(
                    f"Request exceeds the {self.max_request_bytes}-byte protocol limit"
                )

            attempts = self.read_retries + 1 if operation == "get" else 1
            last_error = None
            for _ in range(attempts):
                try:
                    return self._exchange(encoded, request_id, operation)
                except (OSError, WifiConfigError) as error:
                    last_error = error
                    if operation != "get":
                        break

            if operation == "get":
                raise WifiConfigError(
                    f"Pico Wi-Fi readback timed out after {attempts} attempts"
                ) from last_error
            raise WifiConfigError(
                f"Pico Wi-Fi {operation} acknowledgement was not received; "
                "outcome is unknown. Read back the configuration before retrying.",
                outcome_unknown=True,
            ) from last_error

    def _exchange(self, encoded, request_id, request_operation):
        factory = self.socket_factory or socket.socket
        client = factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if self._last_request_at is not None:
                remaining = MIN_REQUEST_INTERVAL_SECONDS - (
                    time.monotonic() - self._last_request_at
                )
                if remaining > 0:
                    time.sleep(remaining)
            client.settimeout(self.timeout)
            client.sendto(encoded, PICO_WIFI_ADDRESS)
            self._last_request_at = time.monotonic()
            deadline = self._last_request_at + self.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout("Pico Wi-Fi response timed out")
                client.settimeout(remaining)
                packet, peer = client.recvfrom(MAX_RESPONSE_BYTES + 1)
                if peer != PICO_WIFI_ADDRESS:
                    continue
                if len(packet) > MAX_RESPONSE_BYTES:
                    raise WifiConfigError("Pico Wi-Fi response exceeds protocol limit")
                try:
                    response = json.loads(packet.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as error:
                    raise WifiConfigError("Pico Wi-Fi returned invalid JSON") from error
                if not isinstance(response, dict):
                    raise WifiConfigError("Pico Wi-Fi response must be a JSON object")
                response_id = response.get("req_id")
                if (
                    not _is_integer(response_id)
                    or response_id < 0
                    or response_id > MAX_REQUEST_ID
                ):
                    raise WifiConfigError("Pico Wi-Fi response has an invalid req_id")
                if response_id != request_id:
                    continue
                if not isinstance(response.get("ok"), bool):
                    raise WifiConfigError("Pico Wi-Fi response is missing boolean ok")
                if response["ok"]:
                    expected_operation = "config" if request_operation == "get" else "reply"
                    if response.get("op") != expected_operation:
                        raise WifiConfigError("Pico Wi-Fi response operation does not match request")
                return response
        finally:
            client.close()
