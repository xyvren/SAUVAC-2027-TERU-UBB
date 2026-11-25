# SAUVAC Android

Native Jetpack Compose companion app for connecting to the Pico W, composing
and uploading mission paths, monitoring live sensors and thrusters, calibrating
depth/IMU sensors, editing supported parameters, and uploading firmware over
Wi-Fi. Its interface is independent of the web dashboard and does not require
a laptop or Python web server.

## Device setup

1. Copy `src/wireless_secrets.example.h` to `src/wireless_secrets.h`.
2. Set a private WPA2 passphrase from 12 to 63 characters. The local header is
   ignored by Git; do not commit or share the passphrase.
3. Build and flash the firmware. Wi-Fi setup is disabled until a valid local
   passphrase is compiled into the firmware.
4. Open the Pico W serial monitor and note the generated `SAUVAC-XXXX` SSID in
   its `[WIFI] Hidden AP` startup message. The access point does not broadcast
   its SSID.
5. In the app, enter that SSID and passphrase, choose whether to save the
   passphrase, then tap **Hubungkan ke Pico W**. Saved credentials are encrypted
   with Android Keystore on the device and can be removed from the same screen.
   The app requests the hidden network directly; Android may ask
   for Nearby Wi-Fi permission (Android 13+) or location permission (Android
   10–12) while joining a network without internet access.

The app connects directly to `192.168.4.1`. It tolerates two consecutive missed
telemetry polls before showing the connection as offline, and responds to
Android's network-lost callback. It asks for the Wi-Fi passphrase again when
uploading a mission or firmware image.
Mission paths are limited to 32 steps and 3,500 JSON bytes. Uploading replaces
the route stored on the Pico but never starts a mission. Uploads require an
idle, disarmed device, neutral thrusters, Auto-Run disabled, and no active
simulation injection.

The Sensor tab polls Pico telemetry about once per second and can tare depth
at the water surface or level the IMU on a stationary, flat surface. Both
calibrations are saved to EEPROM immediately and require fresh sensor data,
idle/disarmed state, neutral thrusters, Auto-Run disabled, and no unsaved
parameter draft. ESC/motor calibration remains USB-only because it can move
thrusters. Remove propellers during bench work; Wi-Fi is not an emergency stop.

## Configuration workflow

Use **Ringkasan** to connect, **Jalur** to create and upload a route, **Sensor**
to monitor telemetry and calibrate sensors, **Parameter** to edit supported
settings, and **Firmware** to upload a `.bin` image. In Parameter, review
changes and choose **Stage draft**. Then use **Apply** to update runtime
settings and **Save EEPROM** to persist them. **Discard** restores the last
saved settings. Firmware and mission uploads use the firmware's idle/disarmed
interlock.

Parameter provides two opt-in startup settings, both disabled by default:
**Nol kedalaman otomatis** and **Level IMU otomatis**. Enable depth zero only
when the pressure sensor will be at the water surface at boot; enable IMU level
only when the AUV will be stationary and level. The firmware waits up to 15
seconds for stable readings and runs only the selected calibration. If readings
are unstable, previous calibration is kept; thrusters are never activated by
this process. Save the settings to EEPROM and reboot for them to take effect.
If Auto-Run is enabled and a selected startup calibration fails, launch is
cancelled.

The **Firmware** tab accepts authenticated Pico W `.bin` images up to 800 KiB.
It sends the image directly to the Pico W over the AP and requires the device
to be idle, disarmed, and Auto-Run-off. The app asks for the AP password again
for the authenticated update; it is not saved. The first OTA-capable firmware
must still be installed over USB/BOOTSEL. If an upload loses its connection,
the outcome is treated as unknown; reconnect and read status before retrying.

USB Serial, physical HIL, and host audit/log APIs remain laptop-only. Android
Wi-Fi is for configuration, route storage, calibration, and monitoring; it is
not an emergency stop or general-purpose motor control path. Test on a bench
with propellers removed.

## Build

Open this `android-config` directory in Android Studio and build the `app`
debug variant. The project targets Android 10 (API 29) and uses the Android
Studio bundled JDK. No Wi-Fi password is embedded in the app.
