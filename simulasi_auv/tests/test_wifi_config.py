import importlib
import io
import json
import socket
import sys
import time
import urllib.error
import unittest
from pathlib import Path
from unittest.mock import patch

from simulasi_auv.wifi_config import (
    MAX_FIRMWARE_IMAGE_BYTES,
    MAX_REQUEST_BYTES,
    PICO_WIFI_ADDRESS,
    PicoFirmwareUpdateError,
    PicoWifiConfigClient,
    WifiConfigError,
    upload_firmware_via_wifi,
)


class FakeUdpPeer:
    def __init__(self):
        self.sent = []
        self.sent_at = []
        self.sockets = []
        self.drop_next = 0
        self.response_packets = None

    def socket_factory(self, family, socket_type):
        fake_socket = FakeUdpSocket(self)
        self.sockets.append(fake_socket)
        return fake_socket

    def respond(self, encoded, address):
        request = json.loads(encoded.decode("utf-8"))
        self.sent.append((request, address, encoded))
        self.sent_at.append(time.monotonic())
        if self.drop_next:
            self.drop_next -= 1
            return []
        if self.response_packets is not None:
            return self.response_packets(request)
        response = {
            "req_id": request["req_id"],
            "ok": True,
            "op": "config" if request["op"] == "get" else "reply",
            "protocol_version": 1,
            "dirty": False,
            "settings": {},
            "status": {},
        }
        return [(json.dumps(response).encode("utf-8"), PICO_WIFI_ADDRESS)]


class FakeUdpSocket:
    def __init__(self, peer):
        self.peer = peer
        self.pending = []
        self.closed = False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendto(self, payload, address):
        self.pending.extend(self.peer.respond(payload, address))
        return len(payload)

    def recvfrom(self, size):
        if not self.pending:
            raise socket.timeout("fake peer did not respond")
        payload, address = self.pending.pop(0)
        return payload[:size], address

    def close(self):
        self.closed = True


class PicoWifiConfigClientTests(unittest.TestCase):
    def setUp(self):
        self.peer = FakeUdpPeer()
        self.client = PicoWifiConfigClient(
            socket_factory=self.peer.socket_factory,
            timeout=0.01,
            read_retries=2,
        )

    def test_get_uses_protocol_v1_address_and_req_id(self):
        response = self.client.request({"op": "get"})

        self.assertTrue(response["ok"])
        request, address, encoded = self.peer.sent[0]
        self.assertEqual(address, PICO_WIFI_ADDRESS)
        self.assertEqual(request, {"req_id": 1, "op": "get"})
        self.assertLessEqual(len(encoded), MAX_REQUEST_BYTES)
        self.assertTrue(self.peer.sockets[0].closed)

    def test_get_retries_with_same_request_id(self):
        self.peer.drop_next = 1

        response = self.client.request({"op": "get"})

        self.assertTrue(response["ok"])
        self.assertEqual(len(self.peer.sent), 2)
        self.assertEqual(
            [request["req_id"] for request, _, _ in self.peer.sent],
            [1, 1],
        )

    def test_get_timeout_stops_after_bounded_retries(self):
        self.peer.drop_next = 10

        with self.assertRaises(WifiConfigError) as raised:
            self.client.request({"op": "get"})

        self.assertFalse(raised.exception.outcome_unknown)
        self.assertEqual(len(self.peer.sent), 3)

    def test_stage_apply_save_and_discard_use_the_firmware_operations(self):
        requests = (
            {"op": "set_power_limit", "percent": 35},
            {"op": "apply"},
            {"op": "save"},
            {"op": "discard"},
        )
        for request in requests:
            self.assertTrue(self.client.request(request)["ok"])

        self.assertEqual(
            [request for request, _, _ in self.peer.sent],
            [
                {"req_id": 1, "op": "set_power_limit", "percent": 35},
                {"req_id": 2, "op": "apply"},
                {"req_id": 3, "op": "save"},
                {"req_id": 4, "op": "discard"},
            ],
        )

    def test_parameter_payloads_keep_firmware_field_names_and_types(self):
        requests = (
            {"op": "set_pid", "axis": "roll", "kp": 0.8, "ki": 0, "kd": 0.01},
            {
                "op": "set_thruster",
                "id": 2,
                "reverse": True,
                "pulse_min": 1100,
                "pulse_neutral": 1500,
                "pulse_max": 1900,
            },
            {"op": "set_deadband", "microseconds": 30},
            {"op": "set_balance", "axis": "vert", "percent": -2.0},
            {
                "op": "set_autorun_timing",
                "delay_sec": 10,
                "tare_sec": 3,
                "level_sec": 6,
            },
        )
        for request in requests:
            self.client.request(request)

        self.assertEqual(
            [request for request, _, _ in self.peer.sent],
            [{"req_id": index, **request} for index, request in enumerate(requests, 1)],
        )
        self.assertIs(self.peer.sent[1][0]["reverse"], True)

    def test_requests_respect_firmware_minimum_interval(self):
        self.client.request({"op": "get"})
        self.client.request({"op": "get"})

        self.assertGreaterEqual(
            self.peer.sent_at[1] - self.peer.sent_at[0],
            0.045,
        )

    def test_write_timeout_is_not_retried_and_marks_outcome_unknown(self):
        self.peer.drop_next = 1

        with self.assertRaises(WifiConfigError) as raised:
            self.client.request({"op": "save"})

        self.assertTrue(raised.exception.outcome_unknown)
        self.assertEqual(len(self.peer.sent), 1)

    def test_stale_id_and_wrong_peer_are_ignored(self):
        def delayed_responses(request):
            stale = {"req_id": request["req_id"] - 1, "ok": True}
            current = {
                "req_id": request["req_id"],
                "ok": True,
                "op": "config",
                "protocol_version": 1,
                "settings": {},
                "status": {},
                "dirty": False,
            }
            return [
                (json.dumps(current).encode(), ("192.168.4.2", 4210)),
                (json.dumps(stale).encode(), PICO_WIFI_ADDRESS),
                (json.dumps(current).encode(), PICO_WIFI_ADDRESS),
            ]

        self.peer.response_packets = delayed_responses

        response = self.client.request({"op": "get"})

        self.assertEqual(response["req_id"], 1)
        self.assertEqual(len(self.peer.sent), 1)

    def test_only_supported_bounded_configuration_operations_are_sent(self):
        invalid_requests = (
            {"op": "mission_upload"},
            {"op": "set_pid", "axis": "pitch", "kp": 1, "ki": 0, "kd": 0},
            {"op": "set_power_limit", "percent": True},
            {"op": "set_thruster", "id": 1, "reverse": 1},
            {"op": "set_balance", "axis": "horiz", "percent": 1, "command": "stop"},
        )
        for request in invalid_requests:
            with self.subTest(request=request), self.assertRaises(ValueError):
                self.client.request(request)

        small_client = PicoWifiConfigClient(
            socket_factory=self.peer.socket_factory,
            max_request_bytes=16,
        )
        with self.assertRaisesRegex(ValueError, "protocol limit"):
            small_client.request(
                {"op": "set_pid", "axis": "depth", "kp": 1, "ki": 0, "kd": 0}
            )
        self.assertEqual(self.peer.sent, [])


class PicoFirmwareUpdateTests(unittest.TestCase):
    class _Response:
        status = 202

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, size):
            return b"Firmware accepted. Pico W is restarting."

    def test_valid_bin_is_sent_as_authenticated_multipart(self):
        response = self._Response()
        with patch(
            "simulasi_auv.wifi_config.urllib.request.urlopen",
            return_value=response,
        ) as urlopen:
            result = upload_firmware_via_wifi(
                b"compiled-pico-bin", "private-password-123"
            )

        request = urlopen.call_args.args[0]
        self.assertEqual(result["accepted"], True)
        self.assertIn(b'name="firmware"; filename="firmware.bin"', request.data)
        self.assertIn(b"compiled-pico-bin", request.data)
        self.assertTrue(request.get_header("Authorization").startswith("Basic "))
        self.assertEqual(urlopen.call_args.kwargs["timeout"], 60)

    def test_invalid_images_names_and_credentials_are_rejected_before_network(self):
        invalid = (
            (b"", "private-password-123", "firmware.bin"),
            (b"x" * (MAX_FIRMWARE_IMAGE_BYTES + 1), "private-password-123", "firmware.bin"),
            (b"UF2\nimage", "private-password-123", "firmware.bin"),
            (b"image", "private-password-123", "../firmware.bin"),
            (b"image", "short", "firmware.bin"),
        )
        with patch("simulasi_auv.wifi_config.urllib.request.urlopen") as urlopen:
            for image, password, filename in invalid:
                with self.subTest(filename=filename), self.assertRaises(ValueError):
                    upload_firmware_via_wifi(image, password, filename)
        urlopen.assert_not_called()

    def test_device_rejection_is_known_but_disconnect_is_unknown(self):
        rejected = urllib.error.HTTPError(
            "http://192.168.4.1/update", 409, "Conflict", {}, io.BytesIO(b"not idle")
        )
        with patch(
            "simulasi_auv.wifi_config.urllib.request.urlopen", side_effect=rejected
        ):
            with self.assertRaises(PicoFirmwareUpdateError) as caught:
                upload_firmware_via_wifi(b"compiled-bin", "private-password-123")
        self.assertEqual(caught.exception.status_code, 409)
        self.assertFalse(caught.exception.outcome_unknown)

        with patch(
            "simulasi_auv.wifi_config.urllib.request.urlopen",
            side_effect=TimeoutError("timeout"),
        ):
            with self.assertRaises(PicoFirmwareUpdateError) as caught:
                upload_firmware_via_wifi(b"compiled-bin", "private-password-123")
        self.assertEqual(caught.exception.status_code, 504)
        self.assertTrue(caught.exception.outcome_unknown)


with patch.dict(sys.modules, {"serial": None}):
    app = importlib.import_module("simulasi_auv.app")
app.bridge.shutdown()
app.bridge.worker_thread.join(timeout=1.0)


class _HandlerStub:
    def __init__(self, allowed=True, path="/api/wifi/status"):
        self.allowed = allowed
        self.path = path
        self.status = None
        self.error = None
        self.payload = None

    def _local_request_allowed(self):
        return self.allowed

    def send_error(self, code, message):
        self.status = code
        self.error = message

    def send_json(self, payload, code=200):
        self.status = code
        self.payload = payload

    def send_response(self, code):
        self.status = code

    def send_header(self, name, value):
        pass

    def end_headers(self):
        pass


class WifiApiRouteTests(unittest.TestCase):
    def test_request_guard_remains_loopback_and_same_origin_only(self):
        handler = _HandlerStub()
        handler.headers = {"Host": "192.168.4.2:8080"}
        self.assertFalse(app.UnifiedHttpHandler._local_request_allowed(handler))

        handler.headers = {
            "Host": "127.0.0.1:8080",
            "Origin": "http://192.168.4.2:8080",
        }
        self.assertFalse(app.UnifiedHttpHandler._local_request_allowed(handler))

        handler.headers = {
            "Host": "127.0.0.1:8080",
            "Origin": "http://127.0.0.1:8080",
        }
        self.assertTrue(app.UnifiedHttpHandler._local_request_allowed(handler))

    def test_remote_get_cannot_reach_wifi_status_client(self):
        handler = _HandlerStub(allowed=False)
        with patch.object(app.wifi_config_client, "request") as request:
            app.UnifiedHttpHandler.do_GET(handler)
        request.assert_not_called()
        self.assertEqual(handler.status, 403)

    def test_local_readback_route_uses_get_without_exposing_other_operations(self):
        handler = _HandlerStub()
        response = {
            "req_id": 1,
            "ok": True,
            "op": "config",
            "protocol_version": 1,
            "settings": {},
            "status": {},
            "dirty": False,
        }
        with patch.object(
            app.wifi_config_client, "request", return_value=response
        ) as request:
            app.UnifiedHttpHandler.do_GET(handler)
        request.assert_called_once_with({"op": "get"})
        self.assertTrue(handler.payload["success"])
        self.assertEqual(handler.payload["response"], response)

    def test_remote_post_cannot_reach_wifi_config_client(self):
        handler = _HandlerStub(allowed=False)
        with patch.object(app.wifi_config_client, "request") as request:
            app.UnifiedHttpHandler.do_POST(handler)
        request.assert_not_called()
        self.assertEqual(handler.status, 403)

    def test_wifi_post_is_configuration_allowlisted(self):
        handler = _HandlerStub()
        response = {"req_id": 7, "ok": True, "op": "reply", "dirty": True}
        with patch.object(app.wifi_config_client, "request", return_value=response) as request:
            app.UnifiedHttpHandler._handle_post(
                handler, "/api/wifi/config", {"op": "set_power_limit", "percent": 20}
            )
        request.assert_called_once_with({"op": "set_power_limit", "percent": 20})
        self.assertTrue(handler.payload["success"])
        self.assertEqual(handler.payload["response"], response)

        forbidden_operations = (
            {"op": "test_motor"},
            {"op": "hil_arm"},
            {"op": "emergency_stop"},
            {"op": "raw_terminal", "command": "save"},
        )
        for forbidden in forbidden_operations:
            with self.subTest(operation=forbidden["op"]), self.assertRaisesRegex(
                ValueError, "state-changing"
            ):
                app.UnifiedHttpHandler._handle_post(
                    handler, "/api/wifi/config", forbidden
                )
        request.assert_called_once_with({"op": "set_power_limit", "percent": 20})

        for allowed in (
            {"op": "command", "command": "stop"},
            {"op": "mission_upload", "json": '{"steps":[]}'},
        ):
            with self.subTest(operation=allowed["op"]):
                app.UnifiedHttpHandler._handle_post(
                    handler, "/api/wifi/config", allowed
                )

    def test_wifi_write_timeout_is_reported_as_unknown_outcome(self):
        handler = _HandlerStub()
        timeout = app.WifiConfigError("acknowledgement missing", outcome_unknown=True)
        with patch.object(app.wifi_config_client, "request", side_effect=timeout):
            app.UnifiedHttpHandler._handle_post(
                handler, "/api/wifi/config", {"op": "apply"}
            )
        self.assertEqual(handler.status, 504)
        self.assertTrue(handler.payload["outcome_unknown"])


class WifiFirmwareApiTests(unittest.TestCase):
    class _Handler(_HandlerStub):
        def __init__(self, body=b"compiled-bin", headers=None, allowed=True):
            super().__init__(allowed=allowed, path="/api/wifi/firmware")
            self.rfile = io.BytesIO(body)
            self.headers = {
                "Host": "127.0.0.1:8080",
                "Content-Type": "application/octet-stream",
                "Content-Length": str(len(body)),
                "X-Pico-Update-Password": "private-password-123",
                "X-Firmware-Filename": "firmware_rev0.1.bin",
            }
            if headers:
                self.headers.update(headers)

        def _handle_firmware_upload(self):
            return app.UnifiedHttpHandler._handle_firmware_upload(self)

    SAFE_DEVICE_RESPONSE = {
        "ok": True,
        "settings": {"autorun": {"enabled": False}},
        "status": {"armed": False, "fsm_state": 0},
    }

    def test_local_firmware_upload_preflights_device_then_relays_bytes(self):
        handler = self._Handler()
        with patch.object(
            app.wifi_config_client,
            "request",
            return_value=self.SAFE_DEVICE_RESPONSE,
        ) as readback, patch.object(
            app,
            "upload_firmware_via_wifi",
            return_value={"accepted": True, "message": "restarting"},
        ) as upload:
            app.UnifiedHttpHandler.do_POST(handler)

        readback.assert_called_once_with({"op": "get"})
        upload.assert_called_once_with(
            b"compiled-bin", "private-password-123", "firmware_rev0.1.bin"
        )
        self.assertEqual(handler.status, 200)
        self.assertTrue(handler.payload["success"])

    def test_firmware_update_rejects_unsafe_device_and_oversized_body(self):
        handler = self._Handler()
        unsafe_response = {
            **self.SAFE_DEVICE_RESPONSE,
            "settings": {"autorun": {"enabled": True}},
        }
        with patch.object(
            app.wifi_config_client, "request", return_value=unsafe_response
        ), patch.object(app, "upload_firmware_via_wifi") as upload:
            app.UnifiedHttpHandler.do_POST(handler)
        self.assertEqual(handler.status, 409)
        upload.assert_not_called()

        oversized = self._Handler(
            body=b"",
            headers={"Content-Length": str(MAX_FIRMWARE_IMAGE_BYTES + 1)},
        )
        with patch.object(app.wifi_config_client, "request") as readback:
            app.UnifiedHttpHandler.do_POST(oversized)
        self.assertEqual(oversized.status, 413)
        readback.assert_not_called()

    def test_remote_firmware_upload_is_blocked_before_relay(self):
        handler = self._Handler(allowed=False)
        with patch.object(app.wifi_config_client, "request") as readback, patch.object(
            app, "upload_firmware_via_wifi"
        ) as upload:
            app.UnifiedHttpHandler.do_POST(handler)
        self.assertEqual(handler.status, 403)
        readback.assert_not_called()
        upload.assert_not_called()

    def test_only_one_firmware_update_can_run_at_a_time(self):
        handler = self._Handler()
        app.wifi_firmware_update_lock.acquire()
        try:
            with patch.object(app.wifi_config_client, "request") as readback:
                app.UnifiedHttpHandler.do_POST(handler)
            self.assertEqual(handler.status, 409)
            readback.assert_not_called()
        finally:
            app.wifi_firmware_update_lock.release()


class WifiUiSeparationTests(unittest.TestCase):
    def test_wifi_ui_defaults_to_serial_and_only_fetches_config_endpoints(self):
        source = (Path(__file__).parents[1] / "index.html").read_text(encoding="utf-8")
        self.assertIn('<option value="serial" selected>', source)
        self.assertIn("Pico W Wi-Fi (kontrol &amp; parameter)", source)
        self.assertIn("requestUrl.pathname === '/api/wifi/status'", source)
        self.assertIn("'/api/wifi/config'", source)
        self.assertIn(
            "requestUrl.pathname.startsWith('/api/') && !allowedWifiRequest", source
        )
        self.assertIn('id="cfgSaveButton"', source)
        self.assertIn("executeTerminalCommand", source)

    def test_firmware_ota_ui_requires_wifi_status_and_explains_bootsel_first_flash(self):
        source = (Path(__file__).parents[1] / "index.html").read_text(encoding="utf-8")
        self.assertIn('id="wifiFirmwareFile"', source)
        self.assertIn('id="wifiFirmwarePassword"', source)
        self.assertIn("POST', '/api/wifi/firmware'", source)
        self.assertIn("X-Pico-Update-Password", source)
        self.assertIn("X-Firmware-Filename", source)
        self.assertIn("Auto-Run nonaktif", source)
        self.assertIn("pertama kali tetap harus dipasang lewat USB/BOOTSEL", source)
        self.assertIn("jangan upload ulang sebelum membaca status Pico", source)

    def test_wifi_outcome_helpers_are_scoped_outside_transport_switch(self):
        source = (Path(__file__).parents[1] / "index.html").read_text(encoding="utf-8")
        transport_switch = source.index("function setConfigTransport(")
        self.assertLess(source.index("function markWifiOutcomeUnknown("), transport_switch)
        self.assertLess(source.index("function clearWifiOutcomeUnknown("), transport_switch)
