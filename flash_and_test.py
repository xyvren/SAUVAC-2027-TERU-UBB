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

print("=== STEP 1: Flashing RP2040 on COM4 ===")
if not os.path.exists(UF2_SRC):
    print(f"Error: {UF2_SRC} not found!")
    sys.exit(1)

print(f"Firmware UF2: {UF2_SRC} ({os.path.getsize(UF2_SRC)} bytes)")

# 1. 1200 baud touch to enter BOOTSEL
print(f"Triggering 1200 baud touch on {COM_PORT}...")
try:
    ser_1200 = serial.Serial(COM_PORT, 1200)
    ser_1200.dtr = False
    time.sleep(0.1)
    ser_1200.close()
    print("1200 baud touch completed.")
except Exception as e:
    print(f"Note on 1200 baud touch: {e}")

# 2. Find RP2040 drive
print("Searching for RP2040 bootloader drive (RPI-RP2)...")
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
                        print(f"Found RP2040 drive at {drive_path}")
                        break
            except Exception:
                pass
    if rpi_drive:
        break
    time.sleep(0.5)

if not rpi_drive:
    print("Error: Could not detect RP2040 drive within 15 seconds!")
    sys.exit(1)

# 3. Copy firmware UF2
dest_uf2 = os.path.join(rpi_drive, "firmware.uf2")
print(f"Flashing UF2: {UF2_SRC} -> {dest_uf2}...")
shutil.copyfile(UF2_SRC, dest_uf2)
print("Firmware flashed! RP2040 rebooting...")

# 4. Wait for COM_PORT
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
    print(f"Error: {COM_PORT} did not re-appear!")
    sys.exit(1)

print(f"{COM_PORT} is back online! Waiting 2s for firmware initialization...")
time.sleep(2.0)

# 5. Open COM_PORT and verify commands
print("\n=== STEP 2: Verifying Serial CLI Commands on COM4 ===")
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

# Wake up CLI
send_and_read("", wait_time=0.3)

# Test 'help'
resp_help = send_and_read("help", wait_time=0.8)

# Test 'status'
resp_status = send_and_read("status", wait_time=0.8)

# Test 'autorun status'
resp_autorun = send_and_read("autorun status", wait_time=0.8)

# Test 'jalur'
resp_jalur = send_and_read("jalur", wait_time=0.8)

ser.close()
print(f"\n{COM_PORT} closed cleanly.")
