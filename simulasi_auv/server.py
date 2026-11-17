#!/usr/bin/env python3
"""
AUV SAUVAC - 2D Underwater Digital Twin Simulator Host Server & Config Dashboard
Combines a high-speed Serial Bridge to RP2040 Pico W (COM4 @ 115200 baud)
with an HTTP server delivering the interactive HTML5/Canvas simulator (index.html)
and the Production-Grade Web Configuration & Monitoring Dashboard (config.html).

Protocol:
  Host -> Pico W:   $INJ,<depth_cm>,<pitch>,<roll>,<gx>,<gy>,<gz>\r\n
  Pico W -> Host:   $ACT,<T1>,<T2>,<T3>,<T4>,<T5>,<T6>\r\n
"""

import copy
import http.server
import json
import math
import os
import queue
import re
import sys
import threading
import time
import urllib.parse

# Try importing serial (pyserial)
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not available in this Python environment. Hardware bridge disabled.")

PORT = 8080
DEFAULT_COM_PORT = 'COM4'
DEFAULT_BAUD = 115200
WEB_DIR = os.path.dirname(os.path.abspath(__file__))

# ANSI escape sequence stripper
ANSI_REGEX = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]')

DEFAULT_MISSION_STEPS = [
    {"step": 1, "name": "Menyelam ke Kedalaman 50 cm", "action": "DIVE", "power": 20, "duration": 5.0, "target_cm": 50},
    {"step": 2, "name": "Berenang Lurus Menembus Garis", "action": "MAJU", "power": 20, "duration": 15.0, "target_cm": 50},
    {"step": 3, "name": "Putar Haluan 180 Derajat", "action": "PUTAR", "power": 15, "duration": 4.0, "target_cm": 50},
    {"step": 4, "name": "Berenang Kembali ke Arah Start", "action": "MAJU", "power": 20, "duration": 15.0, "target_cm": 50},
    {"step": 5, "name": "Naik ke Permukaan Selesai Misi", "action": "SURFACE", "power": 15, "duration": 4.0, "target_cm": 0}
]

def get_default_config_dict():
    return {
        "power_limit": 25,
        "pid_enabled": False,
        "pid": {
            "depth": {"kp": 0.60, "ki": 0.000, "kd": 0.015},
            "roll":  {"kp": 0.80, "ki": 0.000, "kd": 0.008},
            "pitch": {"kp": 0.80, "ki": 0.000, "kd": 0.008}
        },
        "thrusters": [
            {"id": 1, "pin": 2, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Belakang-Kiri (CW)", "type": "horizontal"},
            {"id": 2, "pin": 3, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Depan-Kanan (CW)", "type": "horizontal"},
            {"id": 3, "pin": 4, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Depan-Kiri (CCW)", "type": "horizontal"},
            {"id": 4, "pin": 5, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Belakang-Kanan (CCW)", "type": "horizontal"},
            {"id": 5, "pin": 6, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Vertikal Kanan (CW Dive)", "type": "vertical"},
            {"id": 6, "pin": 7, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Vertikal Kiri (CCW Dive)", "type": "vertical"}
        ],
        "autorun": {
            "enabled": False,
            "delay": 10
        },
        "calib": {
            "depth_zero_pressure": 1013.25,
            "depth_zero_calibrated": False,
            "pitch_offset": 0.0,
            "roll_offset": 0.0,
            "last_calib_time": None
        },
        "last_save_checksum": None,
        "updated_at": time.time()
    }

class AuvSimulatorBridge:
    def __init__(self, com_port=DEFAULT_COM_PORT, baud=DEFAULT_BAUD):
        self.com_port = com_port
        self.baud = baud
        self.ser = None
        self.running = True
        self.connected = False
        self.injection_enabled = True

        # Simulated physical state (injected to Pico W)
        self.sim_depth = 30.0    # cm
        self.sim_pitch = 0.0     # deg
        self.sim_roll = 0.0      # deg
        self.sim_gx = 0.0        # deg/s
        self.sim_gy = 0.0
        self.sim_gz = 0.0
        self.sim_temp = 28.5     # deg C

        # Wave disturbance generator
        self.wave_enabled = False
        self.wave_phase = 0.0

        # Active actuator telemetry received from hardware ($ACT)
        self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
        self.last_act_raw = ""
        self.last_packet_time = 0.0
        self.test_until_time = 0.0

        # Mission tracker state
        self.mission_state = "IDLE"  # "IDLE", "COUNTDOWN", "RUNNING", "COMPLETED"
        self.countdown_remaining = 0
        self.countdown_start_time = 0.0
        self.countdown_duration = 10
        self.current_step = 0
        self.total_steps = len(DEFAULT_MISSION_STEPS)
        self.step_name = ""
        self.step_action = ""
        self.step_power = 0.0
        self.step_duration = 0.0
        self.step_target_cm = 0.0
        self.active_target_depth_cm = 0.0
        self.sim_vz = 0.0        # vertical velocity (cm/s) for closed-loop physics
        self.step_start_time = 0.0
        self.mission_start_time = 0.0
        self.steps_list = list(DEFAULT_MISSION_STEPS)

        # Config state cache
        self.config_state = get_default_config_dict()

        # Sync steps from jalur.json if present
        self.load_jalur_json()

        # Stats
        self.packets_sent = 0
        self.packets_received = 0
        self.rate_hz = 0.0
        self._hz_counter = 0
        self._hz_timer = time.time()

        # Threading & queues
        self.lock = threading.RLock()
        self.cmd_queue = queue.Queue()
        self.recent_logs = []
        self.max_logs = 80

        # Initial log
        self.add_log("AUV SAUVAC Simulator & Config Bridge initialized.")

        # Start bridge thread
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def load_jalur_json(self):
        jalur_candidates = [
            os.path.join(os.path.dirname(WEB_DIR), "jalur.json"),
            os.path.join(WEB_DIR, "jalur.json"),
            "D:/Hermes/jalur.json"
        ]
        for jalur_path in jalur_candidates:
            if os.path.exists(jalur_path):
                try:
                    with open(jalur_path, "r", encoding="utf-8") as f:
                        j_steps = json.load(f)
                        parsed_steps = []
                        for idx, s in enumerate(j_steps):
                            parsed_steps.append({
                                "step": s.get("langkah", s.get("step", idx + 1)),
                                "name": s.get("nama", s.get("name", f"Langkah {idx+1}")),
                                "action": str(s.get("aksi", s.get("action", "IDLE"))).upper(),
                                "power": float(s.get("daya", s.get("power", 15))),
                                "duration": float(s.get("durasi_detik", s.get("duration", 5.0))),
                                "target_cm": float(s.get("target_cm", s.get("target", 0.0)))
                            })
                        if parsed_steps:
                            with self.lock:
                                self.steps_list = parsed_steps
                                self.total_steps = len(parsed_steps)
                            print(f"[OK] Loaded {len(parsed_steps)} steps from {jalur_path}")
                            return True
                except Exception as e:
                    print(f"[WARN] Failed to load {jalur_path}: {e}")
        return False

    def add_log(self, msg: str):
        with self.lock:
            ts = time.strftime("%H:%M:%S")
            self.recent_logs.append(f"[{ts}] {msg}")
            if len(self.recent_logs) > self.max_logs:
                self.recent_logs.pop(0)

    def queue_command(self, cmd: str):
        cmd = cmd.strip()
        if not cmd:
            return
        self.cmd_queue.put(cmd)
        self.add_log(f">>> {cmd}")

        # Update optimistic config state immediately
        self._optimistic_config_update(cmd)

    def _optimistic_config_update(self, cmd: str):
        c = cmd.strip().lower()
        parts = c.split()
        if not parts:
            return

        with self.lock:
            root = parts[0]
            if root in ['stop', 'disarm', 'abort']:
                self.mission_state = "IDLE"
                self.countdown_remaining = 0
                self.active_target_depth_cm = 0.0
                self.sim_vz = 0.0
                self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
                self.test_until_time = 0.0

            elif root == 'run':
                if len(parts) > 1 and parts[1] == 'now':
                    self.mission_state = "RUNNING"
                    self.current_step = 1
                    self.step_start_time = time.time()
                    self.mission_start_time = time.time()
                    if self.steps_list:
                        s0 = self.steps_list[0]
                        self.step_name = s0.get("name", "Langkah 1")
                        self.step_action = s0.get("action", "DIVE")
                        self.step_power = s0.get("power", 20)
                        self.step_duration = s0.get("duration", 5.0)
                        self.active_target_depth_cm = float(s0.get("target_cm", 50.0))
                else:
                    self.mission_state = "COUNTDOWN"
                    self.countdown_duration = self.config_state["autorun"].get("delay", 10)
                    self.countdown_remaining = self.countdown_duration
                    self.countdown_start_time = time.time()

            elif root == 'limit' and len(parts) > 1:
                try:
                    val = max(1, min(100, int(parts[1])))
                    self.config_state["power_limit"] = val
                    self.config_state["updated_at"] = time.time()
                except ValueError:
                    pass

            elif root == 'pid':
                if len(parts) > 1:
                    sub = parts[1]
                    if sub in ['on', '1', 'enable']:
                        self.config_state["pid_enabled"] = True
                    elif sub in ['off', '0', 'disable']:
                        self.config_state["pid_enabled"] = False
                    elif sub in ['roll', 'pitch', 'depth'] and len(parts) >= 5:
                        try:
                            kp = float(parts[2])
                            ki = float(parts[3])
                            kd = float(parts[4])
                            self.config_state["pid"][sub] = {"kp": kp, "ki": ki, "kd": kd}
                        except ValueError:
                            pass
                self.config_state["updated_at"] = time.time()

            elif root == 'thruster' and len(parts) >= 4:
                try:
                    t_id = int(parts[1])
                    prop = parts[2].lower()
                    val = parts[3]
                    if 1 <= t_id <= 6:
                        idx = t_id - 1
                        t_item = self.config_state["thrusters"][idx]
                        if prop == 'reverse':
                            t_item['reverse'] = 1 if val in ['1', 'on', 'true', 'yes'] else 0
                        elif prop == 'trim':
                            t_item['trim'] = max(0.10, min(2.50, float(val)))
                        elif prop == 'min':
                            t_item['min'] = max(800, min(1400, int(val)))
                        elif prop == 'max':
                            t_item['max'] = max(1600, min(2200, int(val)))
                        elif prop == 'neutral':
                            t_item['neutral'] = max(1400, min(1600, int(val)))
                    self.config_state["updated_at"] = time.time()
                except Exception:
                    pass

            elif root == 'autorun':
                if len(parts) > 1:
                    sub = parts[1]
                    if sub in ['on', '1', 'enable']:
                        self.config_state["autorun"]["enabled"] = True
                    elif sub in ['off', '0', 'disable']:
                        self.config_state["autorun"]["enabled"] = False
                    elif sub == 'delay' and len(parts) >= 3:
                        try:
                            sec = max(3, min(60, int(parts[2])))
                            self.config_state["autorun"]["delay"] = sec
                        except ValueError:
                            pass
                self.config_state["updated_at"] = time.time()

            elif root == 'calib':
                if len(parts) > 1:
                    sub = parts[1]
                    if sub == 'zero':
                        pres = float(parts[2]) if len(parts) >= 3 else 1013.25
                        self.config_state["calib"]["depth_zero_pressure"] = pres
                        self.config_state["calib"]["depth_zero_calibrated"] = True
                        self.config_state["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")
                        self.sim_depth = 0.0
                    elif sub == 'level':
                        p_off = float(parts[2]) if len(parts) >= 4 else 0.0
                        r_off = float(parts[3]) if len(parts) >= 4 else 0.0
                        self.config_state["calib"]["pitch_offset"] = p_off
                        self.config_state["calib"]["roll_offset"] = r_off
                        self.config_state["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")
                        self.sim_pitch = 0.0
                        self.sim_roll = 0.0
                self.config_state["updated_at"] = time.time()

            elif root == 'default':
                self.config_state = get_default_config_dict()
                self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]

            elif root == 'save':
                self.config_state["last_save_checksum"] = f"0x{int(time.time()*1000) & 0xFFFFFFFF:08X}"
                self.config_state["updated_at"] = time.time()

            elif root in ['tes', 'test']:
                # Motor test simulation when offline
                if len(parts) >= 3:
                    try:
                        m_target = parts[1]
                        pwr = float(parts[2])
                        limit = float(self.config_state.get("power_limit", 25))
                        clamped_pwr = max(-limit, min(limit, pwr))
                        self.test_until_time = time.time() + 2.0

                        if m_target == 'all' or m_target == '0':
                            for i in range(6):
                                t = self.config_state["thrusters"][i]
                                frac = clamped_pwr / 100.0
                                if t.get("reverse", 0):
                                    frac = -frac
                                frac *= t.get("trim", 1.0)
                                delta = int(round(frac * 500.0))
                                self.pulses[i] = max(1000, min(2000, 1500 + delta))
                        else:
                            idx = int(m_target) - 1
                            if 0 <= idx < 6:
                                for i in range(6):
                                    if i == idx:
                                        t = self.config_state["thrusters"][i]
                                        frac = clamped_pwr / 100.0
                                        if t.get("reverse", 0):
                                            frac = -frac
                                        frac *= t.get("trim", 1.0)
                                        delta = int(round(frac * 500.0))
                                        self.pulses[i] = max(1000, min(2000, 1500 + delta))
                                    else:
                                        self.pulses[i] = 1500
                    except Exception:
                        pass

    def _simulate_cli_response_offline(self, cmd: str):
        c = cmd.strip()
        cl = c.lower()
        parts = cl.split()
        if not parts:
            return

        root = parts[0]
        if root in ['status', 'stat']:
            self.add_log("<<< [STATUS] State: DISARMED/STANDBY | PID Stabilization: " + ("[ ON ]" if self.config_state["pid_enabled"] else "[ OFF ]"))
            self.add_log(f"<<< [STATUS] Limit: {self.config_state['power_limit']}% | Auto-Run: {'ENABLED' if self.config_state['autorun']['enabled'] else 'DISABLED'} ({self.config_state['autorun']['delay']}s)")
            self.add_log(f"<<< [STATUS] Depth: {self.sim_depth:.1f} cm | Pitch: {self.sim_pitch:+.1f}° | Roll: {self.sim_roll:+.1f}° | Temp: {self.sim_temp:.1f}°C")
            self.add_log(f"<<< [STATUS] Pulses: T1={self.pulses[0]}us T2={self.pulses[1]}us T3={self.pulses[2]}us T4={self.pulses[3]}us T5={self.pulses[4]}us T6={self.pulses[5]}us")

        elif root in ['thrusters', 'motor', 'motors']:
            self.add_log("<<< [THRUSTERS] 6-Thruster Configuration Table:")
            for t in self.config_state["thrusters"]:
                self.add_log(f"<<<   T{t['id']} (GP{t['pin']}): Min={t['min']} Neutral={t['neutral']} Max={t['max']} Rev={'YES' if t['reverse'] else 'NO'} Trim={t['trim']:.2f}x")

        elif root in ['sensor', 'sensors']:
            pres = 1013.25 + self.sim_depth * 0.981
            self.add_log(f"<<< [SENSOR SNAPSHOT] Depth: {self.sim_depth:.1f} cm | Pres: {pres:.1f} mbar | Pitch: {self.sim_pitch:+.1f}° | Roll: {self.sim_roll:+.1f}° | Gyro: [0.0, 0.0, 0.0] dps | Temp: {self.sim_temp:.1f}°C")

        elif root == 'limit' and len(parts) > 1:
            self.add_log(f"<<< OK: Safety power limit set to {self.config_state['power_limit']}%. Type 'save' to commit to EEPROM.")

        elif root == 'pid':
            if len(parts) > 1:
                sub = parts[1]
                if sub in ['on', '1', 'enable']:
                    self.add_log("<<< [PID] Active closed-loop stabilization ENABLED and ARMED.")
                elif sub in ['off', '0', 'disable']:
                    self.add_log("<<< [PID] Active closed-loop stabilization DISABLED and DISARMED.")
                elif sub in ['roll', 'pitch', 'depth'] and len(parts) >= 5:
                    p = self.config_state["pid"][sub]
                    self.add_log(f"<<< OK: {sub} PID gains set to Kp={p['kp']:.4f}, Ki={p['ki']:.4f}, Kd={p['kd']:.4f}. Type 'save' to commit.")

        elif root == 'thruster' and len(parts) >= 4:
            self.add_log(f"<<< OK: Thruster T{parts[1]} {parts[2]} set to {parts[3]}. Type 'save' to commit to EEPROM.")

        elif root == 'autorun':
            if len(parts) > 1:
                sub = parts[1]
                if sub in ['on', '1']:
                    self.add_log(f"<<< OK: Auto-run on boot ENABLED ({self.config_state['autorun']['delay']} s countdown).")
                elif sub in ['off', '0']:
                    self.add_log("<<< OK: Auto-run on boot DISABLED.")
                elif sub == 'delay' and len(parts) >= 3:
                    self.add_log(f"<<< OK: Countdown delay set to {self.config_state['autorun']['delay']} seconds.")
                elif sub == 'status':
                    self.add_log(f"<<< [AUTORUN] Status: {'ENABLED' if self.config_state['autorun']['enabled'] else 'DISABLED'}, Delay: {self.config_state['autorun']['delay']}s")

        elif root == 'calib':
            if len(parts) > 1:
                sub = parts[1]
                if sub == 'zero':
                    self.add_log(f"<<< OK: Surface depth reference pressure set to {self.config_state['calib']['depth_zero_pressure']:.1f} mbar. Depth zeroed!")
                elif sub == 'level':
                    self.add_log(f"<<< OK: IMU level offsets recorded (Pitch: {self.config_state['calib']['pitch_offset']:+.2f}°, Roll: {self.config_state['calib']['roll_offset']:+.2f}°). Robot leveled!")

        elif root == 'save':
            self.add_log(f"<<< [EEPROM] Configuration and calibrations successfully committed! Checksum: {self.config_state['last_save_checksum']}")

        elif root == 'default':
            self.add_log("<<< [CFG] Factory default settings restored in RAM. Type 'save' to persist to EEPROM.")

        elif root in ['stop', 'disarm', 'abort']:
            self.add_log("<<< [EMERGENCY STOP] All 6 thrusters neutralized immediately (1500us). Disarmed.")

        elif root in ['tes', 'test']:
            if len(parts) >= 3:
                self.add_log(f"<<< [TEST] Running Motor {parts[1]} at {parts[2]}% for 2.0s")

        elif root == 'run':
            if len(parts) > 1 and parts[1] == 'now':
                self.add_log("<<< System ARMED. Starting Step 1/5: Menyelam [DIVE] Power: 20% (Duration: 5.0s)")
            else:
                self.add_log(f"<<< Launch countdown started: {self.config_state['autorun']['delay']} seconds. Type 'stop' to abort.")

        elif root in ['jalur', 'mission']:
            self.add_log(f"<<< [MISSION] Loaded {len(self.steps_list)} steps from /jalur.json")

    def update_control(self, depth=None, pitch=None, roll=None, wave=None, injection=None):
        with self.lock:
            if depth is not None:
                self.sim_depth = max(0.0, min(140.0, float(depth)))
                self.sim_vz = 0.0
            if pitch is not None:
                self.sim_pitch = max(-45.0, min(45.0, float(pitch)))
            if roll is not None:
                self.sim_roll = max(-45.0, min(45.0, float(roll)))
            if wave is not None:
                self.wave_enabled = bool(wave)
            if injection is not None:
                self.injection_enabled = bool(injection)

    def _parse_firmware_log(self, raw_line: str):
        clean = ANSI_REGEX.sub('', raw_line).strip()
        if not clean:
            return

        with self.lock:
            # Target depth log
            m_target = re.search(r'Target(?:\s*Depth)?:\s*([\d\.]+)', clean, re.IGNORECASE)
            if m_target:
                self.active_target_depth_cm = float(m_target.group(1))

            # Countdown start
            m_cd = re.search(r'Launch countdown started:\s*(\d+)\s*seconds', clean, re.IGNORECASE)
            if m_cd:
                self.mission_state = "COUNTDOWN"
                self.countdown_duration = int(m_cd.group(1))
                self.countdown_remaining = self.countdown_duration
                self.countdown_start_time = time.time()
                return

            # Countdown tick
            m_tick = re.search(r'T-(\d+)s', clean)
            if m_tick and self.mission_state == "COUNTDOWN":
                self.countdown_remaining = int(m_tick.group(1))
                return

            # Mission Step Start
            m_step1 = re.search(r'Starting Step\s*(\d+)/(\d+):\s*(.*?)\s*\|\s*Action:\s*(.*?)\s*\|\s*Power:\s*([\d\.]+)%\s*\|\s*Duration:\s*([\d\.]+)s', clean, re.IGNORECASE)
            m_step2 = re.search(r'Starting Step\s*(\d+)/(\d+):\s*(.*?)\s*\[(.*?)\]\s*Power:\s*([\d\.]+)%\s*\(Duration:\s*([\d\.]+)s\)', clean, re.IGNORECASE)
            m_step = m_step1 or m_step2
            if m_step:
                self.mission_state = "RUNNING"
                self.current_step = int(m_step.group(1))
                self.total_steps = int(m_step.group(2))
                self.step_name = m_step.group(3).strip()
                self.step_action = m_step.group(4).strip().upper()
                self.step_power = float(m_step.group(5))
                self.step_duration = float(m_step.group(6))
                self.step_start_time = time.time()
                if self.current_step == 1:
                    self.mission_start_time = time.time()

                if 1 <= self.current_step <= len(self.steps_list):
                    step_cfg = self.steps_list[self.current_step - 1]
                    t_val = step_cfg.get("target_cm", 0.0)
                    if t_val > 0.0:
                        self.active_target_depth_cm = float(t_val)
                if self.step_action == "SURFACE":
                    self.active_target_depth_cm = 0.0
                return

            # Mission Completed
            if "MISSION STEPS COMPLETED" in clean or "ALL MISSION STEPS COMPLETED" in clean:
                self.mission_state = "COMPLETED"
                self.current_step = self.total_steps
                self.active_target_depth_cm = 0.0
                self.sim_vz = 0.0
                return

            # Stop / Disarmed
            if "EMERGENCY STOP" in clean or "System disarmed" in clean:
                if self.mission_state != "COMPLETED":
                    self.mission_state = "IDLE"
                    self.active_target_depth_cm = 0.0
                    self.sim_vz = 0.0

            # Parse Configuration & Calibration Feedback
            # 1. PID State
            m_pid_stat = re.search(r'PID Stabilization:\s*\[\s*(ON|OFF)\s*\]', clean, re.IGNORECASE)
            if m_pid_stat:
                self.config_state["pid_enabled"] = (m_pid_stat.group(1).upper() == "ON")

            if "[PID] Active closed-loop stabilization ENABLED" in clean:
                self.config_state["pid_enabled"] = True
            elif "[PID] Active closed-loop stabilization DISABLED" in clean:
                self.config_state["pid_enabled"] = False

            # 2. Power limit
            m_pwr = re.search(r'(?:Safety Cap|limit set to|power limit:)\s*(\d+)%', clean, re.IGNORECASE)
            if m_pwr:
                self.config_state["power_limit"] = int(m_pwr.group(1))

            # 3. Auto-Run status
            m_ar = re.search(r'Auto-Run on Boot\s*:\s*(ENABLED|DISABLED)', clean, re.IGNORECASE)
            if m_ar:
                self.config_state["autorun"]["enabled"] = (m_ar.group(1).upper() == "ENABLED")

            m_ard = re.search(r'(?:Countdown Delay|countdown delay set to|delay:)\s*(\d+)\s*s', clean, re.IGNORECASE)
            if m_ard:
                self.config_state["autorun"]["delay"] = int(m_ard.group(1))

            # 4. PID Gains
            for axis in ['Depth', 'Roll', 'Pitch']:
                m_g = re.search(rf'{axis}\s*:\s*Kp=([\d\.]+),\s*Ki=([\d\.]+),\s*Kd=([\d\.]+)', clean, re.IGNORECASE)
                if m_g:
                    self.config_state["pid"][axis.lower()] = {
                        "kp": float(m_g.group(1)),
                        "ki": float(m_g.group(2)),
                        "kd": float(m_g.group(3))
                    }

            # 5. Thruster Table Row
            m_tt = re.search(r'\|\s*(\d)\s*\|\s*GP(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(YES|NO)\s*\|\s*([\d\.]+)x\s*\|', clean)
            if m_tt:
                t_id = int(m_tt.group(1))
                if 1 <= t_id <= 6:
                    t_entry = self.config_state["thrusters"][t_id - 1]
                    t_entry["pin"] = int(m_tt.group(2))
                    t_entry["min"] = int(m_tt.group(3))
                    t_entry["neutral"] = int(m_tt.group(4))
                    t_entry["max"] = int(m_tt.group(5))
                    t_entry["reverse"] = 1 if m_tt.group(6).upper() == "YES" else 0
                    t_entry["trim"] = float(m_tt.group(7))

            # 6. Calib Feedback
            m_zero = re.search(r'Surface depth reference pressure set to\s*([\d\.]+)\s*mbar', clean, re.IGNORECASE)
            if m_zero:
                self.config_state["calib"]["depth_zero_pressure"] = float(m_zero.group(1))
                self.config_state["calib"]["depth_zero_calibrated"] = True
                self.config_state["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")

            m_lvl = re.search(r'offsets (?:recorded|set to) (?:Pitch:\s*([+-]?[\d\.]+)[°]?,?\s*Roll:\s*([+-]?[\d\.]+)[°]?)', clean, re.IGNORECASE)
            if m_lvl:
                self.config_state["calib"]["pitch_offset"] = float(m_lvl.group(1))
                self.config_state["calib"]["roll_offset"] = float(m_lvl.group(2))
                self.config_state["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")

            # 7. Checksum
            m_cs = re.search(r'Checksum:\s*(0x[0-9a-fA-F]+)', clean)
            if m_cs:
                self.config_state["last_save_checksum"] = m_cs.group(1)

            # 8. Sensor snapshot
            m_snap = re.search(r'\[SENSOR SNAPSHOT\] Depth:\s*([\d\.]+)\s*cm\s*\|\s*Pres:\s*([\d\.]+)\s*mbar\s*\|\s*Pitch:\s*([+-]?[\d\.]+)[°]?\s*\|\s*Roll:\s*([+-]?[\d\.]+)[°]?', clean)
            if m_snap:
                self.sim_depth = float(m_snap.group(1))
                self.sim_pitch = float(m_snap.group(3))
                self.sim_roll = float(m_snap.group(4))

    def get_mission_status(self):
        with self.lock:
            now = time.time()
            elapsed_sec = 0.0
            remaining_sec = 0.0
            step_progress = 0.0

            if self.mission_state == "COUNTDOWN":
                if self.countdown_start_time > 0:
                    c_elapsed = now - self.countdown_start_time
                    self.countdown_remaining = max(0, int(round(self.countdown_duration - c_elapsed)))
                remaining_sec = float(self.countdown_remaining)

            elif self.mission_state == "RUNNING":
                if self.step_start_time > 0 and self.step_duration > 0:
                    elapsed_sec = max(0.0, now - self.step_start_time)
                    remaining_sec = max(0.0, self.step_duration - elapsed_sec)
                    step_progress = min(100.0, (elapsed_sec / self.step_duration) * 100.0)

            elif self.mission_state == "COMPLETED":
                step_progress = 100.0
                remaining_sec = 0.0

            total_dur = sum(s.get("duration", 0.0) for s in self.steps_list) or 43.0
            completed_dur = 0.0
            for i in range(min(self.current_step - 1, len(self.steps_list))):
                completed_dur += self.steps_list[i].get("duration", 0.0)
            if self.mission_state == "RUNNING":
                completed_dur += min(elapsed_sec, self.step_duration)
            elif self.mission_state == "COMPLETED":
                completed_dur = total_dur

            overall_progress = min(100.0, (completed_dur / total_dur) * 100.0) if total_dur > 0 else 0.0

            return {
                "state": self.mission_state,
                "countdown_remaining": self.countdown_remaining,
                "current_step": self.current_step,
                "total_steps": self.total_steps,
                "step_name": self.step_name or (self.steps_list[0]["name"] if self.steps_list else "Standby"),
                "step_action": self.step_action or (self.steps_list[0]["action"] if self.steps_list else "IDLE"),
                "step_power": self.step_power or (self.steps_list[0]["power"] if self.steps_list else 0.0),
                "step_duration": self.step_duration or (self.steps_list[0]["duration"] if self.steps_list else 0.0),
                "target_depth_cm": self.active_target_depth_cm,
                "elapsed_sec": round(elapsed_sec, 1),
                "remaining_sec": round(remaining_sec, 1),
                "step_progress_pct": round(step_progress, 1),
                "overall_progress_pct": round(overall_progress, 1),
                "total_duration_sec": total_dur,
                "steps": self.steps_list
            }

    def get_state(self):
        with self.lock:
            # Calculate power delta percent: (pulse - 1500) / 5
            deltas = [round((p - 1500) / 5.0, 1) for p in self.pulses]
            return {
                "connected": self.connected,
                "port": self.com_port,
                "baud": self.baud,
                "injection_enabled": self.injection_enabled,
                "rate_hz": round(self.rate_hz, 1),
                "packets_sent": self.packets_sent,
                "packets_received": self.packets_received,
                "sim": {
                    "depth": round(self.sim_depth, 1),
                    "pitch": round(self.sim_pitch, 1),
                    "roll": round(self.sim_roll, 1),
                    "wave": self.wave_enabled
                },
                "pulses": list(self.pulses),
                "deltas": deltas,
                "mission": self.get_mission_status(),
                "logs": list(self.recent_logs)
            }

    def get_config_state(self):
        with self.lock:
            deltas = [round((p - 1500) / 5.0, 1) for p in self.pulses]
            pres = round(1013.25 + self.sim_depth * 0.981, 1)
            return {
                "status": "ok",
                "connected": self.connected,
                "port": self.com_port,
                "baud": self.baud,
                "rate_hz": round(self.rate_hz, 1),
                "config": copy.deepcopy(self.config_state),
                "sensors": {
                    "depth_cm": round(self.sim_depth, 1),
                    "pressure_mbar": pres,
                    "pitch": round(self.sim_pitch, 1),
                    "roll": round(self.sim_roll, 1),
                    "temp_c": round(self.sim_temp, 1),
                    "gyro": [round(self.sim_gx, 1), round(self.sim_gy, 1), round(self.sim_gz, 1)]
                },
                "pulses": list(self.pulses),
                "deltas": deltas,
                "mission": self.get_mission_status()
            }

    def _open_serial(self):
        if not SERIAL_AVAILABLE:
            return None
        try:
            ser = serial.Serial(self.com_port, self.baud, timeout=0.08)
            time.sleep(0.1)
            ser.reset_input_buffer()
            self.add_log(f"Connected to {self.com_port} @ {self.baud} baud")
            # Query firmware initial configuration
            ser.write(b"status\r\nthrusters\r\n")
            ser.flush()
            return ser
        except Exception:
            return None

    def _run(self):
        last_inj_time = 0.0
        inj_interval = 0.033  # ~30 Hz injection loop (33 ms)

        while self.running:
            now = time.time()

            # Connection management
            if self.ser is None or not self.ser.is_open:
                self.connected = False
                self.ser = self._open_serial()
                if self.ser is None:
                    # Offline simulated mode: drain command queue and simulate responses
                    while not self.cmd_queue.empty():
                        try:
                            cmd = self.cmd_queue.get_nowait()
                            self._simulate_cli_response_offline(cmd)
                        except queue.Empty:
                            break

                    # Check test motor timeout in simulation
                    if self.test_until_time > 0 and now >= self.test_until_time:
                        self.test_until_time = 0.0
                        if self.mission_state != "RUNNING":
                            self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]

                    time.sleep(0.2)
                    continue
                else:
                    self.connected = True

            # 1. Process pending text commands to hardware
            try:
                while not self.cmd_queue.empty():
                    cmd = self.cmd_queue.get_nowait()
                    self.ser.write((cmd + "\r\n").encode('utf-8'))
                    self.ser.flush()
                    time.sleep(0.05)
                    while self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='replace').strip()
                        if line:
                            self.add_log(f"<<< {line}")
                            self._parse_firmware_log(line)
            except Exception as e:
                self.add_log(f"Command error: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.connected = False
                continue

            # 2. Sensor Injection Tick (~30 Hz)
            if self.injection_enabled and (now - last_inj_time >= inj_interval):
                dt = now - last_inj_time if last_inj_time > 0 else inj_interval
                last_inj_time = now

                with self.lock:
                    # Hydrodynamic closed-loop vertical simulation
                    t5_thrust = (self.pulses[4] - 1500.0) / 500.0
                    t6_thrust = (1500.0 - self.pulses[5]) / 500.0
                    heave_force = (t5_thrust + t6_thrust) * 0.5  # [-1.0, 1.0]

                    if self.mission_state == "RUNNING" or abs(heave_force) > 0.015:
                        accel_thrust = heave_force * 110.0  # cm/s^2 at full thrust
                        accel_buoyancy = -1.2 if self.sim_depth > 0.5 else 0.0
                        accel_drag = -2.2 * self.sim_vz - 0.05 * self.sim_vz * abs(self.sim_vz)

                        a_z = accel_thrust + accel_buoyancy + accel_drag
                        self.sim_vz += a_z * dt
                        self.sim_depth += self.sim_vz * dt

                        if self.sim_depth < 0.0:
                            self.sim_depth = 0.0
                            self.sim_vz = max(0.0, self.sim_vz)
                        elif self.sim_depth > 140.0:
                            self.sim_depth = 140.0
                            self.sim_vz = min(0.0, self.sim_vz)

                    if self.wave_enabled:
                        self.wave_phase += dt * 2.2
                        eff_roll = self.sim_roll + 14.0 * math.sin(self.wave_phase)
                        eff_pitch = self.sim_pitch + 4.0 * math.cos(self.wave_phase * 0.7)
                        eff_depth = max(0.0, self.sim_depth + 5.0 * math.sin(self.wave_phase * 0.6))
                        eff_gx = 14.0 * 2.2 * math.cos(self.wave_phase)
                        eff_gy = -4.0 * 1.5 * math.sin(self.wave_phase * 0.7)
                        eff_gz = 0.0
                    else:
                        eff_roll = self.sim_roll
                        eff_pitch = self.sim_pitch
                        eff_depth = self.sim_depth
                        eff_gx = self.sim_gx
                        eff_gy = self.sim_gy
                        eff_gz = self.sim_gz

                # Construct $INJ packet
                packet = f"$INJ,{eff_depth:.2f},{eff_pitch:.2f},{eff_roll:.2f},{eff_gx:.2f},{eff_gy:.2f},{eff_gz:.2f}\r\n"

                try:
                    self.ser.write(packet.encode('utf-8'))
                    self.ser.flush()
                    self.packets_sent += 1

                    # Read response line
                    resp = self.ser.readline().decode('utf-8', errors='replace').strip()

                    if resp.startswith("$ACT,"):
                        parts = resp[5:].split(',')
                        if len(parts) >= 6:
                            try:
                                new_pulses = [int(p) for p in parts[:6]]
                                with self.lock:
                                    self.pulses = new_pulses
                                    self.last_act_raw = resp
                                self.packets_received += 1
                                self._hz_counter += 1
                            except ValueError:
                                pass
                    elif resp:
                        self.add_log(f"<<< {resp}")
                        self._parse_firmware_log(resp)

                    # Drain extra pending logs
                    while self.ser.in_waiting:
                        more_line = self.ser.readline().decode('utf-8', errors='replace').strip()
                        if more_line:
                            if more_line.startswith("$ACT,"):
                                parts = more_line[5:].split(',')
                                if len(parts) >= 6:
                                    try:
                                        self.pulses = [int(p) for p in parts[:6]]
                                    except ValueError:
                                        pass
                            else:
                                self.add_log(f"<<< {more_line}")
                                self._parse_firmware_log(more_line)

                except Exception as e:
                    self.add_log(f"Serial IO error: {e}")
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                    self.connected = False
                    time.sleep(0.5)
                    continue

            # Update Hz calculation once per second
            if now - self._hz_timer >= 1.0:
                self.rate_hz = self._hz_counter / (now - self._hz_timer)
                self._hz_counter = 0
                self._hz_timer = now

            time.sleep(0.005)

# Global Bridge Instance
bridge = AuvSimulatorBridge()

class AuvHttpHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, format, *args):
        # Suppress verbose standard HTTP access logs
        pass

    def send_json_response(self, obj, code=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == '/api/state':
            self.send_json_response(bridge.get_state())
            return

        elif path == '/api/mission':
            self.send_json_response(bridge.get_mission_status())
            return

        elif path == '/api/config_state':
            self.send_json_response(bridge.get_config_state())
            return

        elif path == '/' or path == '/index.html':
            self.path = '/index.html'
        elif path == '/config' or path == '/config.html':
            self.path = '/config.html'

        return super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', 0))
        post_body = self.rfile.read(length).decode('utf-8', errors='replace')
        body = {}
        if post_body:
            try:
                body = json.loads(post_body)
            except Exception:
                pass

        if path == '/api/control':
            depth = body.get('depth')
            pitch = body.get('pitch')
            roll = body.get('roll')
            wave = body.get('wave')
            injection = body.get('injection')
            bridge.update_control(depth=depth, pitch=pitch, roll=roll, wave=wave, injection=injection)
            self.send_json_response({"status": "ok", "state": bridge.get_state()})
            return

        elif path == '/api/command':
            cmd = body.get('command', '').strip()
            if cmd:
                bridge.queue_command(cmd)
            self.send_json_response({"status": "queued", "command": cmd})
            return

        elif path == '/api/test_motor':
            motor_id = body.get('id', 0)     # 0 = all, 1..6 = single motor
            power = body.get('power', 15)     # percent, default 15%
            try:
                power = float(power)
            except (ValueError, TypeError):
                power = 15.0

            if motor_id == 0 or str(motor_id).lower() == 'all':
                cmd = f"tes all {power:.0f}"
            else:
                m_int = max(1, min(6, int(motor_id)))
                cmd = f"tes {m_int} {power:.0f}"

            bridge.queue_command(cmd)
            self.send_json_response({
                "status": "queued",
                "motor_id": motor_id,
                "power": power,
                "command": cmd
            })
            return

        elif path == '/api/mission_action':
            action = body.get('action', '').strip().lower()
            cmd = "stop"
            if action in ['run', 'start']:
                cmd = "run"
            elif action in ['run_now', 'now', 'instant']:
                cmd = "run now"
            elif action in ['stop', 'abort', 'cancel']:
                cmd = "stop"

            bridge.queue_command(cmd)
            self.send_json_response({
                "status": "queued",
                "action": action,
                "command": cmd,
                "mission": bridge.get_mission_status()
            })
            return

        elif path == '/api/maneuver':
            action = body.get('action', '').strip().lower()
            power = body.get('power', 15)
            try:
                power = float(power)
            except (ValueError, TypeError):
                power = 15.0
            power = max(1.0, min(100.0, power))

            commands = []
            if action == 'stop':
                commands = ["stop"]
            elif action == 'forward':
                commands = [
                    f"tes 1 {power:.0f}",
                    f"tes 2 {power:.0f}",
                    f"tes 3 {power:.0f}",
                    f"tes 4 {power:.0f}"
                ]
            elif action == 'backward':
                commands = [
                    f"tes 1 {-power:.0f}",
                    f"tes 2 {-power:.0f}",
                    f"tes 3 {-power:.0f}",
                    f"tes 4 {-power:.0f}"
                ]
            elif action == 'turn_left':
                commands = [
                    f"tes 2 {power:.0f}",
                    f"tes 4 {power:.0f}",
                    f"tes 1 {-power:.0f}",
                    f"tes 3 {-power:.0f}"
                ]
            elif action == 'turn_right':
                commands = [
                    f"tes 1 {power:.0f}",
                    f"tes 3 {power:.0f}",
                    f"tes 2 {-power:.0f}",
                    f"tes 4 {-power:.0f}"
                ]
            elif action == 'dive':
                commands = [
                    f"tes 5 {power:.0f}",
                    f"tes 6 {power:.0f}"
                ]
            elif action == 'surface':
                commands = [
                    f"tes 5 {-power:.0f}",
                    f"tes 6 {-power:.0f}"
                ]
            else:
                self.send_json_response({"status": "error", "message": f"Unknown action: {action}"}, code=400)
                return

            for c in commands:
                bridge.queue_command(c)

            self.send_json_response({
                "status": "queued",
                "action": action,
                "power": power,
                "commands": commands
            })
            return

        elif path == '/api/param_update':
            p_type = body.get('type', '').strip().lower()
            commands = []

            if p_type == 'limit':
                val = int(body.get('value', 25))
                val = max(1, min(100, val))
                commands.append(f"limit {val}")

            elif p_type == 'pid':
                axis = str(body.get('axis', 'depth')).lower()
                kp = float(body.get('kp', 0.0))
                ki = float(body.get('ki', 0.0))
                kd = float(body.get('kd', 0.0))
                commands.append(f"pid {axis} {kp:.4f} {ki:.4f} {kd:.4f}")

            elif p_type == 'pid_state':
                en = bool(body.get('enabled', True))
                commands.append("pid on" if en else "pid off")

            elif p_type == 'thruster':
                t_id = max(1, min(6, int(body.get('id', 1))))
                prop = str(body.get('prop', 'reverse')).lower()
                val = body.get('value')
                if prop == 'reverse':
                    val_str = "1" if val in [1, "1", True, "on", "yes"] else "0"
                elif prop == 'trim':
                    val_str = f"{float(val):.2f}"
                else:
                    val_str = str(val)
                commands.append(f"thruster {t_id} {prop} {val_str}")

            elif p_type == 'autorun':
                en = bool(body.get('enabled', False))
                commands.append("autorun on" if en else "autorun off")
                if 'delay' in body and body['delay'] is not None:
                    d = max(3, min(60, int(body['delay'])))
                    commands.append(f"autorun delay {d}")

            elif p_type == 'calib':
                target = str(body.get('target', 'zero')).lower()
                if target == 'zero':
                    mbar = body.get('mbar')
                    if mbar is not None and str(mbar).strip():
                        commands.append(f"calib zero {float(mbar):.1f}")
                    else:
                        commands.append("calib zero")
                elif target == 'level':
                    p = body.get('pitch')
                    r = body.get('roll')
                    if p is not None and r is not None and str(p).strip() and str(r).strip():
                        commands.append(f"calib level {float(p):.2f} {float(r):.2f}")
                    else:
                        commands.append("calib level")

            elif p_type == 'save':
                commands.append("save")

            elif p_type == 'default':
                commands.append("default")

            else:
                self.send_json_response({"status": "error", "message": f"Unknown param type: {p_type}"}, code=400)
                return

            for c in commands:
                bridge.queue_command(c)

            self.send_json_response({
                "status": "ok",
                "type": p_type,
                "commands": commands,
                "config": bridge.get_config_state()["config"]
            })
            return

        elif path == '/api/mission_reload':
            loaded = bridge.load_jalur_json()
            self.send_json_response({
                "status": "ok",
                "reloaded": loaded,
                "mission": bridge.get_mission_status()
            })
            return

        self.send_response(404)
        self.end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

def run_server():
    server_address = ('', PORT)
    httpd = http.server.ThreadingHTTPServer(server_address, AuvHttpHandler)
    print("=" * 70)
    print(" AUV SAUVAC 2D DIGITAL TWIN & CONFIG DASHBOARD SERVER")
    print(f" Web Simulator : http://localhost:{PORT}/index.html")
    print(f" Config Console : http://localhost:{PORT}/config.html")
    print(f" Hardware Serial: {DEFAULT_COM_PORT} @ {DEFAULT_BAUD} bps")
    print("=" * 70)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        bridge.running = False
        httpd.server_close()

if __name__ == '__main__':
    run_server()
