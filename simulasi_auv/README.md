# SAUVAC host configuration

The host UI is served by `app.py` at `http://127.0.0.1:8080/`. It uses the
existing loopback-only Python HTTP server; the API is not exposed on the LAN.
USB Serial is the default transport. The configuration tab also offers a
Pico W Wi-Fi transport for the firmware's allowlisted setup/control commands,
status readback, and firmware updates.

## Pico W Wi-Fi configuration

1. Build firmware with a private `AUV_WIFI_AP_PASSWORD` in the local,
   uncommitted `wireless_secrets.h`. The firmware disables its AP when the
   password is missing, is the placeholder, or is outside 12–63 characters.
   After `pio run`, use `.pio/build/picow/firmware.uf2` for the first BOOTSEL
   install and the matching `.pio/build/picow/firmware.bin` for OTA.
2. Connect the laptop to the Pico W access point. Its SSID is
   `SAUVAC-XXXX` (the suffix is derived from the Pico W MAC); the AP address
   and UDP port are `192.168.4.1:4210`.
3. Open the host UI locally and select **Pico W Wi-Fi**. Configuration and
   control requests go through the loopback API as protocol-v1 JSON datagrams.
   The password is not needed for those requests. Firmware OTA asks for the AP
   password only when uploading; Python uses it for authenticated HTTP on the
   Pico and does not store or log it.

The client uses the firmware contract as implemented: `get`, `set_pid`,
`set_thruster`, `set_power_limit`, `set_deadband`, `set_balance`,
`set_autorun_timing`, `apply`, `save`, and `discard`. Requests include a
`req_id` in the range 0–2,147,483,647 and are limited to 384 bytes. Only
read-only `get` requests are retried. A timed-out write is not resent automatically;
its result is reported as unknown, and the operator should read back before
deciding what to do next.

Set operations stage a candidate. **Apply** changes active settings,
**Save** applies and persists them to EEPROM, and **Discard** restores the
last-saved settings. Firmware requires idle/disarmed state. The UI also
prevents timing changes while Auto-Run is enabled.

Wi-Fi does not provide continuous streamed telemetry, HIL actuator streaming,
or HIL/arming. It does provide parameter staging,
EEPROM save/discard, live status/sensor readback polling, terminal CLI allowlist,
motor testing, mission upload/run/stop, sensor calibration, PID master,
Auto-Run enable/disable, emergency stop, and factory reset command.

## Firmware OTA

The OTA-capable firmware must be installed once over USB/BOOTSEL before Wi-Fi
updates are available. Thereafter, the configuration tab accepts a compiled
Pico W `.bin` (up to 800 KiB) and relays it through Python to the authenticated
Pico endpoint. OTA is rejected unless a fresh device readback confirms idle,
disarmed, and Auto-Run disabled; the firmware independently checks the state
and stages the image in LittleFS. Control/UDP processing pauses during transfer
and reboot. Keep power connected, remove propellers, and use UF2/BOOTSEL for
recovery. A lost connection can leave the result unknown; read status before
trying again.

This laptop UI does not include the separate Android client. Firmware/AP
availability on a connected vehicle must be verified on that hardware; no
hardware commands are sent during software tests.
