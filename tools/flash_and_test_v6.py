import os
import sys
import time
import shutil
import string
import serial
import serial.tools.list_ports

COM_PORT = os.environ.get('AUV_COM_PORT', 'COM4')
BAUD_RATE = 115200
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
UF2_SRC = os.path.join(PROJECT_ROOT, 'firmware', 'firmware_rev0.1.uf2')

print(f"=== STEP 1: Flashing RP2040 Firmware v6 on {COM_PORT} ===")
if not os.path.exists(UF2_SRC):
    print(f"Error: {UF2_SRC} not found!")
    sys.exit(1)

print(f"Firmware UF2 size: {os.path.getsize(UF2_SRC)} bytes")

# 1. Reset to BOOTSEL via 1200 baud touch
print(f"Pulsing {COM_PORT} at 1200 baud to trigger BOOTSEL...")
try:
    ser_1200 = serial.Serial(COM_PORT, 1200)
    ser_1200.dtr = False
    time.sleep(0.1)
    ser_1200.close()
    print("1200 baud touch complete.")
except Exception as e:
    print(f"Note on 1200 baud touch: {e}")

# 2. Detect RPI-RP2 drive
print("Searching for RPI-RP2 drive...")
rpi_drive = None
start_time = time.time()
while time.time() - start_time < 15:
    for letter in string.ascii_uppercase:
        drive_path = f"{letter}:\\"
        info_file = os.path.join(drive_path, "INFO_UF2.TXT")
        if os.path.exists(info_file):
            try:
                with open(info_file, 'r', errors='ignore') as f:
                    content = f.read()
                    if "Raspberry Pi" in content or "RP2040" in content:
                        rpi_drive = drive_path
                        print(f"Found RP2040 bootloader at {drive_path}")
                        break
            except Exception:
                pass
    if rpi_drive:
        break
    time.sleep(0.5)

if not rpi_drive:
    print("Error: Could not detect RPI-RP2 drive within timeout.")
    sys.exit(1)

# 3. Copy firmware to drive
dest_uf2 = os.path.join(rpi_drive, "firmware.uf2")
print(f"Copying {UF2_SRC} -> {dest_uf2}...")
shutil.copyfile(UF2_SRC, dest_uf2)
print("UF2 copied. RP2040 will now flash and reboot automatically.")

# 4. Wait for COM_PORT to re-enumerate
print(f"Waiting for {COM_PORT} to re-enumerate...")
reconnected = False
start_time = time.time()
while time.time() - start_time < 20:
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if COM_PORT in ports:
        reconnected = True
        break
    time.sleep(0.5)

if not reconnected:
    print(f"Error: {COM_PORT} did not re-appear after flashing!")
    sys.exit(1)

print(f"{COM_PORT} reconnected successfully! Waiting 2.0s for firmware startup...")
time.sleep(2.0)

print("\n=== STEP 2: Verification of Audited Enhancements ===")
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1.0)
time.sleep(0.5)
ser.reset_input_buffer()

def send_and_read(cmd, wait_time=0.8):
    print(f"\n>>> SENDING: '{cmd}'")
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait_time)
    response = ser.read_all().decode('utf-8', errors='replace')
    print("<<< RESPONSE:\n" + response.strip())
    return response

# Wakeup / clear buffer
initial = send_and_read("", wait_time=0.4)

# Test 1: $POLL
print("\n--- TEST 1: $POLL machine command ---")
resp_poll = send_and_read("$POLL", wait_time=0.5)
poll_ok = "$TEL," in resp_poll
print(f"Result $POLL: {'PASS' if poll_ok else 'FAIL'}")
if poll_ok:
    for line in resp_poll.splitlines():
        if line.startswith("$TEL,"):
            print(f"  Received valid telemetry packet: {line}")
            fields = line.strip().split(',')
            print(f"  Field count: {len(fields)} (expected 17: $TEL + 16 fields)")
            print(f"  Depth: {fields[1]} cm, Pressure: {fields[2]} mbar, Pitch: {fields[3]} deg, Roll: {fields[4]} deg")
            print(f"  PWM Pulses: T1={fields[9]}, T2={fields[10]}, T3={fields[11]}, T4={fields[12]}, T5={fields[13]}, T6={fields[14]}")
            print(f"  Armed: {fields[15]}, FSM State: {fields[16]}")

# Test 2: tes 1 15
print("\n--- TEST 2: Motor T1 test ('tes 1 15') ---")
resp_tes = send_and_read("tes 1 15", wait_time=0.8)
tes_ok = "[TEST] Running Motor T1" in resp_tes
print(f"Result 'tes 1 15': {'PASS' if tes_ok else 'FAIL'}")

# Test 3: autorun status
print("\n--- TEST 3: autorun status ---")
resp_ar_stat = send_and_read("autorun status", wait_time=0.8)
ar_stat_ok = "Surface Tare" in resp_ar_stat and "IMU Leveling" in resp_ar_stat
print(f"Result 'autorun status': {'PASS' if ar_stat_ok else 'FAIL'}")

# Test 4: autorun timing
print("\n--- TEST 4: autorun timing command ---")
resp_timing = send_and_read("autorun timing 15 4 2", wait_time=0.8)
timing_ok = "OK: Auto-run timing updated" in resp_timing
print(f"Result 'autorun timing 15 4 2': {'PASS' if timing_ok else 'FAIL'}")

# Test 5: esc deadband
print("\n--- TEST 5: esc deadband ---")
resp_db = send_and_read("esc deadband 30", wait_time=0.8)
db_ok = "OK: ESC deadband set to 30 us" in resp_db
print(f"Result 'esc deadband 30': {'PASS' if db_ok else 'FAIL'}")

# Test 6: motor balance
print("\n--- TEST 6: motor balance (horizontal & vertical) ---")
resp_bal_h = send_and_read("motor balance horiz 3.5", wait_time=0.8)
resp_bal_v = send_and_read("motor balance vert -2.0", wait_time=0.8)
bal_ok = "Differential horizontal trim set to +3.5%" in resp_bal_h and "Differential vertical trim set to -2.0%" in resp_bal_v
print(f"Result 'motor balance': {'PASS' if bal_ok else 'FAIL'}")

# Test 7: Telemetry streaming
print("\n--- TEST 7: Telemetry streaming (telem 5) ---")
ser.write(b"telem 5\r\n")
ser.flush()
time.sleep(1.2)
telem_stream_data = ser.read_all().decode('utf-8', errors='replace')
ser.write(b"telem off\r\n")
ser.flush()
time.sleep(0.5)
ser.read_all()
tel_stream_ok = telem_stream_data.count("$TEL,") >= 3
print(f"Result 'telem 5' (received {telem_stream_data.count('$TEL,')} packets in 1.2s): {'PASS' if tel_stream_ok else 'FAIL'}")

# Save and emergency stop
send_and_read("save", wait_time=0.5)
send_and_read("stop", wait_time=0.5)

ser.close()
print(f"\n{COM_PORT} closed cleanly.")

print("\n=== FINAL TEST SUMMARY ===")
all_pass = poll_ok and tes_ok and ar_stat_ok and timing_ok and db_ok and bal_ok and tel_stream_ok
print(f"1. $POLL Telemetry: {'PASS' if poll_ok else 'FAIL'}")
print(f"2. Motor Test ('tes 1 15'): {'PASS' if tes_ok else 'FAIL'}")
print(f"3. Auto-Run Status: {'PASS' if ar_stat_ok else 'FAIL'}")
print(f"4. Auto-Run Timing Config: {'PASS' if timing_ok else 'FAIL'}")
print(f"5. ESC Deadband: {'PASS' if db_ok else 'FAIL'}")
print(f"6. Motor Balance: {'PASS' if bal_ok else 'FAIL'}")
print(f"7. $TEL Telemetry Streaming: {'PASS' if tel_stream_ok else 'FAIL'}")

if all_pass:
    print("\n>>> ALL AUDITED ENHANCEMENTS VERIFIED SUCCESSFULLY ON HARDWARE! <<<")
    sys.exit(0)
else:
    print("\n>>> SOME TESTS FAILED! <<<")
    sys.exit(1)
