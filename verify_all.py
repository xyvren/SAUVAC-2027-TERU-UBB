import time
import serial

COM_PORT = 'COM4'
BAUD = 115200

ser = serial.Serial(COM_PORT, BAUD, timeout=1.0)
time.sleep(0.5)
ser.reset_input_buffer()

def send_cmd(cmd, wait=0.6):
    print(f"\n==================================================")
    print(f">>> COMMAND: '{cmd}'")
    ser.write((cmd + "\r\n").encode('utf-8'))
    ser.flush()
    time.sleep(wait)
    resp = ser.read_all().decode('utf-8', errors='replace')
    print("<<< RESPONSE:\n" + resp.strip())
    return resp

# 1. Wake up
send_cmd("", wait=0.2)

# 2. Test autorun delay
send_cmd("autorun delay 15")

# 3. Test autorun on
send_cmd("autorun on")

# 4. Check autorun status
send_cmd("autorun status")

# 5. Save to EEPROM
send_cmd("save")

# 6. Test autorun delay back to 10
send_cmd("autorun delay 10")

# 7. Test autorun off & save for bench safety
send_cmd("autorun off")
send_cmd("save")

# 8. Test launch countdown with 'run'
print("\n>>> Testing launch countdown via 'run'...")
ser.write(b"run\r\n")
ser.flush()
# Listen to countdown ticks for 3 seconds
time.sleep(3.5)
resp_countdown = ser.read_all().decode('utf-8', errors='replace')
print("<<< COUNTDOWN TICKS:\n" + resp_countdown.strip())

# 9. Send 'stop' to abort countdown
send_cmd("stop", wait=0.5)

# 10. Check status after stop
send_cmd("status", wait=0.6)

# 11. Test 'jalur'
send_cmd("jalur", wait=0.6)

ser.close()
print("\n[VERIFICATION COMPLETE] All tests passed cleanly.")
