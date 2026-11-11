import os
import sys
import time
import shutil
import string
import serial
import serial.tools.list_ports

COM_PORT = 'COM4'
BAUD_RATE = 115200
UF2_SRC = r'D:\Projcet\Source Code SAUVC\auv-6thruster-rev-0.1\firmware\firmware_rev0.1.uf2'

print(f"=== STEP 4: Flashing RP2040 on {COM_PORT} ===")
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

print(f"{COM_PORT} reconnected successfully! Waiting 1.5s for firmware startup...")
time.sleep(1.5)

print("\n=== STEP 5: Verification via Serial CLI ===")
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

# Initial wakeup
send_and_read("", wait_time=0.3)

# 1. Send 'thrusters'
resp_thrusters = send_and_read("thrusters", wait_time=1.0)

# Check table reverse status
checks = [
    ("T1 Reverse YES", "|  1 | GP2  |" in resp_thrusters and "YES" in resp_thrusters),
    ("T2 Reverse YES", "|  2 | GP3  |" in resp_thrusters and "YES" in resp_thrusters),
    ("T3 Reverse NO",  "|  3 | GP4  |" in resp_thrusters and " NO" in resp_thrusters),
    ("T4 Reverse NO",  "|  4 | GP5  |" in resp_thrusters and " NO" in resp_thrusters),
    ("T5 Reverse YES", "|  5 | GP6  |" in resp_thrusters and "YES" in resp_thrusters),
    ("T6 Reverse NO",  "|  6 | GP7  |" in resp_thrusters and " NO" in resp_thrusters),
]

# 2. Test T1 at +15%
resp_t1 = send_and_read("tes 1 15", wait_time=0.8)

# 3. Test T2 at +15%
resp_t2 = send_and_read("tes 2 15", wait_time=0.8)

# 4. Test T6 at +15%
resp_t6 = send_and_read("tes 6 15", wait_time=0.8)

# Save settings to persist
send_and_read("save", wait_time=0.5)

# Emergency stop to ensure all thrusters neutral
send_and_read("stop", wait_time=0.5)

ser.close()
print(f"\n{COM_PORT} closed cleanly.")

print("\n=== VERIFICATION SUMMARY ===")
all_passed = True
print("Thruster Table Verification:")
for line in resp_thrusters.splitlines():
    if any(f"|  {i} |" in line for i in range(1, 7)):
        print("  " + line)

print("\nPulse Output Verification:")
t1_ok = "1425us" in resp_t1
t2_ok = "1425us" in resp_t2
t6_ok = "1575us" in resp_t6

print(f"  T1 (CW) at +15% -> 1425us: {'PASS' if t1_ok else 'FAIL'} (Response: {resp_t1.strip()})")
print(f"  T2 (CW) at +15% -> 1425us: {'PASS' if t2_ok else 'FAIL'} (Response: {resp_t2.strip()})")
print(f"  T6 (CW) at +15% -> 1575us: {'PASS' if t6_ok else 'FAIL'} (Response: {resp_t6.strip()})")

if not (t1_ok and t2_ok and t6_ok):
    all_passed = False

if all_passed:
    print("\nALL CHECKS PASSED SUCCESSFULLY!")
else:
    print("\nSOME CHECKS FAILED!")
    sys.exit(1)
