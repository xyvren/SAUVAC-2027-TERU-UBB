# AUV SAUVAC Bench Testbed (Rev 0.1)

> **Lightweight, Standalone 6-Thruster Precision Control & Sensor Tuning Firmware**  
> Designed for Raspberry Pi Pico W (RP2040) • Zero Wi-Fi & Network Overhead • 115200 Baud Interactive Serial CLI

```
╔═════════════════════════════════════════════════════════════════════╗
║               AUV SAUVAC BENCH TESTBED (REV 0.1)                    ║
║          Lightweight 6-Thruster Precision Control & Tuning          ║
╚═════════════════════════════════════════════════════════════════════╝
```

---

## 📌 Overview

**`auv-6thruster-rev-0.1`** is a dedicated bench testbed firmware developed for the AUV SAUVAC (Universitas Bangka Belitung) team. Unlike full autonomous or web-based builds, this firmware is **100% stripped of Wi-Fi, web servers, and network overhead**, dedicating all microcontroller cycles to:

1. **Safety Power Limiting:** Hardcapped at **25% default** to prevent burning motors, blowing ESCs, or damaging wiring on the workbench.
2. **Speed & Polarity Synchronization:** Independent PWM endpoints (`min`, `neutral`, `max`), software reverse flags, and speed trim multipliers (`trim`) per thruster.
3. **Continuous 50 Hz PWM Streaming:** Guaranteed active PWM generation during actuator testing to keep marine ESCs armed without signal timeout dropouts.
4. **Sensor Telemetry & Fusion:** Real-time complementary filter attitude estimation from **MPU-6050** (Pitch, Roll, 3-axis Gyro) and high-resolution depth/pressure from **MS5803**.
5. **Interactive Serial CLI:** Beautiful ANSI-colored box borders, live dashboards, and one-touch calibration commands.
6. **Non-Volatile EEPROM Persistence:** Checksum-verified EEPROM storage preserving thruster calibrations, level offsets, and PID gains across power cycles.

---

## 🔌 Hardware Pinout & Wiring

| Peripheral | Component | RP2040 GPIO | Description / Notes |
| :--- | :--- | :--- | :--- |
| **Thruster 1 (T1)** | Rear-Left ESC | **GP2** | Horizontal vector (45°), default normal (CW) |
| **Thruster 2 (T2)** | Front-Right ESC | **GP3** | Horizontal vector (45°), default normal (CW) |
| **Thruster 3 (T3)** | Front-Left ESC | **GP4** | Horizontal vector (45°), default normal (CCW) |
| **Thruster 4 (T4)** | Rear-Right ESC | **GP5** | Horizontal vector (45°), default normal (CCW) |
| **Thruster 5 (T5)** | Mid-Right Vertical ESC | **GP6** | Vertical lift (CCW blade), default reverse (`reverse 1`) |
| **Thruster 6 (T6)** | Mid-Left Vertical ESC | **GP7** | Vertical lift (CW blade), default reverse (`reverse 1`) |
| **I2C SDA** | Sensors Bus (Wire1) | **GP26** | Connected to MPU-6050 & MS5803 SDA |
| **I2C SCL** | Sensors Bus (Wire1) | **GP27** | Connected to MPU-6050 & MS5803 SCL |
| **Serial CLI** | USB CDC Interface | **USB** | 115200 baud, 8-N-1 interactive terminal |

---

## ⚡ Serial Command Reference

All commands can be typed in lowercase and tolerate an optional leading slash (e.g., `status` or `/status`, `tes 1 20` or `/tes 1 20`).

### 1. System & Telemetry

| Command | Syntax | Description |
| :--- | :--- | :--- |
| `help` or `?` | `help` | Displays the complete formatted command menu. |
| `status` or `stat` | `status` | Full system dashboard: IMU attitude, depth, arming state, active PWM pulses, and PID gains. |
| `sensor` | `sensor` | One-shot snapshot of current IMU tilt, gyro rates, pressure, and depth. |
| `sensor stream` | `sensor stream` | Starts continuous 5 Hz live telemetry stream. Any keypress stops the stream. |
| `sensor off` | `sensor off` | Stops active telemetry streaming. |
| `stop` | `stop` | **EMERGENCY STOP:** Immediately neutralizes all 6 motors to 1500 µs and disarms vehicle. |

### 2. Thruster Tuning & Bench Testing

| Command | Syntax | Description |
| :--- | :--- | :--- |
| `thrusters` or `motor` | `thrusters` | Displays ASCII table of all 6 thrusters with pins, PWM endpoints, polarity, and trim. |
| `limit` | `limit <1-100>` | Sets global safety power limit (default **25%**). Motors will strictly never exceed this cap. |
| `tes` or `test` | `tes <1-6\|all> <power>` | Spins motor(s) for **2.0 seconds** at commanded power (-limit to +limit %). Checks reverse polarity and trim. |
| `thruster` | `thruster <id> min <us>` | Sets reverse PWM pulse endpoint (800–1400 µs, default 1000 µs). |
| `thruster` | `thruster <id> max <us>` | Sets forward PWM pulse endpoint (1600–2200 µs, default 2000 µs). |
| `thruster` | `thruster <id> neutral <us>` | Sets electrical center trim (1400–1600 µs, default 1500 µs). |
| `thruster` | `thruster <id> reverse <0\|1>` | Inverts software direction polarity (0: Normal, 1: Inverted). |
| `thruster` | `thruster <id> trim <scale>` | Adjusts speed trim multiplier (0.10–2.50, default 1.00) to synchronize motor speeds. |

### 3. Sensor Calibration

| Command | Syntax | Description |
| :--- | :--- | :--- |
| `calib zero` | `calib zero` | Tares water depth sensor at surface. Locks current MS5803 atmospheric pressure as reference. |
| `calib zero <mbar>` | `calib zero 1013.25` | Manually sets surface atmospheric pressure (ideal for dry desk testing). |
| `calib level` | `calib level` | Zeroes IMU tilt. Current Pitch and Roll angles are saved as level offsets (0.0°/0.0°). |
| `calib level <p> <r>` | `calib level 0 0` | Manually sets Pitch and Roll offsets in degrees. |

### 4. Closed-Loop PID Stabilization

| Command | Syntax | Description |
| :--- | :--- | :--- |
| `pid on` | `pid on` | Enables active closed-loop stabilization. Disturbance or tilt will drive vertical/horizontal thrusters. |
| `pid off` | `pid off` | Disables stabilization and neutralizes thrusters. |
| `pid <axis> <kp> <ki> <kd>` | `pid roll 0.02 0.0 0.008` | Configures PID gains for `depth`, `roll`, or `pitch`. |

### 5. Memory & Persistence

| Command | Syntax | Description |
| :--- | :--- | :--- |
| `save` | `save` | Commits current configuration, calibrations, and PID gains to EEPROM with FNV-1a checksum. |
| `default` | `default` | Resets all settings in RAM to factory defaults. Type `save` to persist. |

---

## 🛡️ Safety Guarantees & Implementation Details

1. **Hardware Power Protection (25% Safety Cap):**  
   Brushless thrusters on bench testing can draw up to 15A–25A each if driven at 100% throttle in air, which can melt 3D-printed mounts, burn motor windings, or trip bench power supplies. Firmware Rev 0.1 enforces an initial global power ceiling of **25%** (`DEFAULT_POWER_LIMIT_PERCENT = 25`). Testing commands (`tes 1 50`) will be automatically clamped with a warning:
   ```
   [TEST] Running Motor T1 (GP2) at +25.0% (1625us) for 2.0s (CLAMPED by safety limit)
   ```

2. **Continuous 50 Hz PWM Output Guard:**  
   Standard brushless ESCs require continuous 50 Hz PWM signals (20 ms period) to remain initialized and armed. Single-pulse writes will cause the ESC to time out and halt. Firmware Rev 0.1 uses an active window timer (`if (testUntilMs > 0 && now < testUntilMs)`) inside the 50 Hz output loop to stream pulses continuously until the 2.0-second timer expires.

3. **Signed Intermediate Math Pattern (ARM Cortex-M0+ Bug Prevention):**  
   On 32-bit ARM Cortex-M microcontrollers, direct float-to-unsigned conversion of negative numbers saturates to `0`. Firmware Rev 0.1 computes intermediate offsets using signed 32-bit integers before clamping:
   ```cpp
   int32_t delta = (frac >= 0.0f) ? lroundf(frac * (pulseMax - pulseNeutral))
                                  : lroundf(frac * (pulseNeutral - pulseMin));
   int32_t calcPulse = static_cast<int32_t>(pulseNeutral) + delta;
   uint16_t pulse = constrain(calcPulse, pulseMin, pulseMax);
   ```

4. **ESC Reverse Asymmetry Compensation:**  
   Marine propeller blades have cambered geometry optimized for one rotational direction. Software polarity inversion (`reverse 1`) is enabled by default on vertical thrusters T5 (GP6) and T6 (GP7) to produce balanced vertical heave dynamics without physical wire swapping.

---

## 🚀 Flashing & Verification

### Pre-Compiled Release Binaries
The `firmware/` directory contains verified, ready-to-flash binaries:
- `firmware/firmware_rev0.1.uf2` (Drag-and-drop BOOTSEL flashing)
- `firmware/firmware_rev0.1.bin` (Direct binary flashing)

### Method 1: USB BOOTSEL (Quickest)
1. Hold down the white **BOOTSEL** button on the Raspberry Pi Pico W.
2. Plug the USB cable into your computer (a storage drive named `RPI-RP2` appears).
3. Drag and drop `firmware/firmware_rev0.1.uf2` onto the `RPI-RP2` drive.
4. The Pico W reboots instantly and starts the testbed firmware.

### Method 2: PlatformIO CLI Compilation
To recompile from source:
```bash
# Navigate to the project directory
cd "D:/Projcet/Source Code SAUVC/auv-6thruster-rev-0.1"

# Compile with 0 warnings & 0 errors
pio run

# Upload via USB (auto-reset or picotool)
pio run -t upload

# Open interactive Serial CLI
pio device monitor -b 115200
```

---

## 📊 Bench Testing Workflow Example

```
AUV> status
[Displays live status: sensors online, limit at 25%, all thrusters at neutral 1500us]

AUV> thrusters
[Displays ASCII table of T1..T6 configuration]

AUV> tes 1 20
[TEST] Running Motor T1 (GP2) at +20.0% (1600us) for 2.0s
[Motor T1 spins smoothly at 20% power, then halts at neutral]

AUV> tes all 15
[TEST] Running ALL 6 motors at +15.0% for 2.0s
[All 6 thrusters spin in unison at 15% power for 2 seconds]

AUV> calib level
OK: IMU level offsets recorded (Pitch: -0.42°, Roll: +0.81°). Robot leveled!
Type 'save' to commit to EEPROM.

AUV> save
[EEPROM] Configuration and calibrations successfully committed! Checksum: 0x8DF421A9

AUV> pid on
[PID] Active closed-loop stabilization ENABLED and ARMED.
[Tilting the AUV on the workbench causes vertical motors T5/T6 to react and counteract tilt!]

AUV> stop
[EMERGENCY STOP] All 6 thrusters neutralized immediately (1500us). Disarmed.
```
