import importlib
import sys
import threading
import time
import unittest
from unittest.mock import patch


with patch.dict(sys.modules, {"serial": None}):
    app = importlib.import_module("simulasi_auv.app")
app.bridge.shutdown()
app.bridge.worker_thread.join(timeout=1.0)


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass


class HilBridgeTests(unittest.TestCase):
    def setUp(self):
        self.bridge = app.HardwareBridge.__new__(app.HardwareBridge)
        self.bridge.state_lock = threading.RLock()
        self.bridge.hil_lock = threading.RLock()
        self.bridge.serial_transaction_lock = threading.Lock()
        self.bridge.serial_io_lock = threading.Lock()
        self.bridge.ser = FakeSerial()
        self.bridge.connected = True
        self.bridge.last_tel_time = time.time()
        self.bridge.armed = 0
        self.bridge.fsm_state = 0
        self.bridge.hil_active = False
        self.bridge.hil_armed_at = 0.0
        self.bridge.packets_tx = 0
        self.bridge._execute_serial_command = lambda command, timeout=1.0: {
            "success": True,
            "reply": "OK",
            "command": command,
        }

    def test_arm_requires_explicit_confirmation_and_fresh_idle_hardware(self):
        result = self.bridge.arm_hil(False)
        self.assertFalse(result["success"])
        self.assertEqual(self.bridge.ser.writes, [])

        self.bridge.last_tel_time = time.time() - 1.0
        result = self.bridge.arm_hil(True)
        self.assertFalse(result["success"])
        self.assertEqual(self.bridge.ser.writes, [])

    def test_arm_sends_neutral_seed_and_hil_frames(self):
        result = self.bridge.arm_hil(True)
        self.assertTrue(result["success"])
        self.assertTrue(self.bridge.hil_active)
        self.assertIn(b"$INJ,", self.bridge.ser.writes[0])
        self.assertIn(b"$HIL,1500,1500,1500,1500,1500,1500", self.bridge.ser.writes[0])

        frame = self.bridge.send_hil_frame(
            [1425, 1500, 1575, 1500, 1500, 1500],
            {"depth_cm": 50, "pitch": 2, "roll": -1, "gyro_dps": [0, 0, 4]},
        )
        self.assertTrue(frame["success"])
        self.assertIn(b"$INJ,50.00,2.00,-1.00,0.00,0.00,4.00", self.bridge.ser.writes[-1])
        self.assertIn(b"$HIL,1425,1500,1575,1500,1500,1500", self.bridge.ser.writes[-1])

        self.assertTrue(self.bridge.disarm_hil()["success"])
        self.assertFalse(self.bridge.hil_active)

    def test_unarmed_bridge_cannot_send_actuator_output(self):
        result = self.bridge.send_hil_frame(
            [1500] * 6,
            {"depth_cm": 0, "pitch": 0, "roll": 0, "gyro_dps": [0, 0, 0]},
        )
        self.assertFalse(result["success"])
        self.assertEqual(self.bridge.ser.writes, [])

    def test_generic_command_api_cannot_bypass_hil_interlock(self):
        result = self.bridge.execute_command("hil on")
        self.assertFalse(result["success"])
        self.assertEqual(self.bridge.ser.writes, [])

    def test_stale_telemetry_stops_hil_frames(self):
        self.bridge.hil_active = True
        self.bridge.last_tel_time = time.time() - 1.0
        result = self.bridge.send_hil_frame(
            [1500] * 6,
            {"depth_cm": 0, "pitch": 0, "roll": 0, "gyro_dps": [0, 0, 0]},
        )
        self.assertFalse(result["success"])
        self.assertFalse(self.bridge.hil_active)
        self.assertEqual(self.bridge.ser.writes, [])


if __name__ == "__main__":
    unittest.main()
