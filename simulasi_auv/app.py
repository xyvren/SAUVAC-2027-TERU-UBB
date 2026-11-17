#!/usr/bin/env python3
"""
AUV SAUVAC - Production Unified Host Server & Single Launcher
Universitas Bangka Belitung - SAUVC 2027

Features:
- Dual-channel Serial Bridge to RP2040 Pico W (COM4 @ 115200 baud)
- Real-time $TEL telemetry ingestion (10-15 Hz)
- Truth-based synchronous command execution (Zero fake toasts!)
- LittleFS route upload handler (POST /api/mission_upload)
- Multi-threaded HTTP server on port 8080 serving unified 6-tab industrial dark web app
- Clean shutdown on Ctrl+C without locking COM port
"""

import copy
import http.server
import json
import math
import os
import queue
import re
import signal
import sys
import threading
import time
import urllib.parse
import webbrowser

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not found in this environment. Bridge will run in fallback mock mode.")

PORT = 8080
DEFAULT_COM_PORT = 'COM4'
DEFAULT_BAUD = 115200
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
HERMES_DIR = os.path.dirname(BASE_DIR)

if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

try:
    from rl_agent import rl_agent
except ImportError:
    rl_agent = None

ANSI_REGEX = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]')

def clean_ansi(text: str) -> str:
    """Strips ANSI terminal escape sequences and trims whitespace."""
    return ANSI_REGEX.sub('', text).strip()

def get_default_motors_for_action(aksi: str, target_cm: float = 0.0) -> list:
    a = (aksi or "").upper().strip()
    if a in ["DIVE", "SURFACE", "HOVER", "SELAM", "NAIK", "DIAM"]:
        return [5, 6]
    elif a in ["MAJU", "MUNDUR", "PUTAR", "KIRI", "KANAN", "FORWARD", "BACKWARD", "TURN", "SWAY_LEFT", "SWAY_RIGHT"]:
        return [1, 2, 3, 4, 5, 6] if target_cm > 0.0 else [1, 2, 3, 4]
    elif a in ["TES_T1", "T1", "MOTOR_1"]:
        return [1]
    elif a in ["TES_T2", "T2", "MOTOR_2"]:
        return [2]
    elif a in ["TES_T3", "T3", "MOTOR_3"]:
        return [3]
    elif a in ["TES_T4", "T4", "MOTOR_4"]:
        return [4]
    elif a in ["TES_T5", "T5", "MOTOR_5"]:
        return [5]
    elif a in ["TES_T6", "T6", "MOTOR_6"]:
        return [6]
    elif a in ["TES_ALL", "ALL", "ALL_MOTORS", "SEMUA"]:
        return [1, 2, 3, 4, 5, 6]
    return [1, 2, 3, 4, 5, 6]

DEFAULT_STEPS = [
    {"langkah": 1, "nama": "Menyelam ke 50 cm", "aksi": "DIVE", "daya": 15, "durasi_detik": 5.0, "target_cm": 50.0, "motors": [5, 6]},
    {"langkah": 2, "nama": "Berenang Maju Lurus", "aksi": "MAJU", "daya": 20, "durasi_detik": 12.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
    {"langkah": 3, "nama": "Putar Haluan Kanan", "aksi": "PUTAR", "daya": 15, "durasi_detik": 4.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
    {"langkah": 4, "nama": "Berenang Kembali", "aksi": "MAJU", "daya": 20, "durasi_detik": 12.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
    {"langkah": 5, "nama": "Naik ke Permukaan", "aksi": "SURFACE", "daya": 15, "durasi_detik": 4.0, "target_cm": 0.0, "motors": [5, 6]}
]

def get_default_config():
    return {
        "power_limit": 25,
        "pid_enabled": False,
        "pid": {
            "depth": {"kp": 0.600, "ki": 0.000, "kd": 0.015},
            "roll":  {"kp": 0.800, "ki": 0.000, "kd": 0.008},
            "pitch": {"kp": 0.800, "ki": 0.000, "kd": 0.008}
        },
        "thrusters": [
            {"id": 1, "pin": 2, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Belakang-Kiri (CW)", "type": "horizontal"},
            {"id": 2, "pin": 3, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Depan-Kanan (CW)", "type": "horizontal"},
            {"id": 3, "pin": 4, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Depan-Kiri (CCW)", "type": "horizontal"},
            {"id": 4, "pin": 5, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Belakang-Kanan (CCW)", "type": "horizontal"},
            {"id": 5, "pin": 6, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 0, "trim": 1.00, "pos": "Vertikal Kanan (CW Dive)", "type": "vertical"},
            {"id": 6, "pin": 7, "min": 1000, "neutral": 1500, "max": 2000, "reverse": 1, "trim": 1.00, "pos": "Vertikal Kiri (CCW Dive)", "type": "vertical"}
        ],
        "balance": {
            "horiz": 0.0,
            "vert": -2.0
        },
        "esc": {
            "deadband": 30
        },
        "autorun": {
            "enabled": False,
            "delay": 10,
            "tare": 3,
            "level": 6
        },
        "calib": {
            "zero_pressure": 1014.1,
            "pitch_offset": 0.0,
            "roll_offset": 0.0,
            "calibrated": True,
            "last_calib_time": time.strftime("%H:%M:%S")
        },
        "last_save_checksum": None
    }


class HardwareBridge:
    """High-speed synchronous and asynchronous serial bridge to RP2040."""

    def __init__(self, port=DEFAULT_COM_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True
        self.connected = False

        self.state_lock = threading.RLock()
        self.cmd_lock = threading.Lock()
        self.cmd_event = threading.Event()
        self.pending_cmd = None
        self.cmd_reply = None
        self.cmd_reply_lines = []
        self.last_reply_line_time = 0.0

        # Telemetry from $TEL
        self.hardware_yaw = 0.0
        self.last_tel_time = 0.0
        self.gz_bias = 0.0
        self.sensors = {
            "depth_cm": 0.0,
            "pressure_mbar": 1013.25,
            "pitch": 0.0,
            "roll": 0.0,
            "temp_c": 30.0,
            "gyro": [0.0, 0.0, 0.0],
            "yaw": 0.0
        }
        self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
        self.armed = 0
        self.fsm_state = 0
        self.last_packet_time = 0.0

        # Rate calculations
        self.packets_rx = 0
        self.packets_tx = 0
        self.rate_hz = 0.0
        self._hz_counter = 0
        self._hz_timer = time.time()

        # Simulated state fallback & 2D Hydrodynamic digital twin
        self.sim_depth = 0.0
        self.sim_pitch = 0.0
        self.sim_roll = 0.0
        self.sim_temp = 30.0
        self.sim_wave = False
        self.sim_wave_phase = 0.0
        self.mock_test_end_time = 0.0

        # Configurable Physics Parameters (User configurable)
        self.sim_mass = 14.5            # kg (dry mass, 8.0 to 25.0 kg)
        self.sim_added_mass_z = 8.2     # kg
        self.sim_added_mass_x = 3.5     # kg
        self.sim_net_buoyancy_g = 150.0 # grams (+50 to +500g)
        self.sim_drag_scale = 1.0       # 0.5x to 2.0x

        self.sim_x = 100.0              # cm (0 to 1200 cm along pool length)
        self.sim_vx = 0.0               # cm/s (surge speed)
        self.sim_vz = 0.0               # cm/s (heave speed)
        self.sim_yaw = 0.0              # deg (heading 0 to 360)
        self.sim_vyaw = 0.0             # deg/s (yaw rate)
        self.sim_mission_active = False
        self.sim_mission_completed = False
        self.sim_mission_step_idx = 0
        self.sim_mission_step_start = 0.0

        # Mission state
        self.mission_state = "IDLE"  # IDLE, COUNTDOWN, RUNNING, COMPLETED
        self.countdown_remaining = 0
        self.countdown_duration = 10
        self.countdown_start = 0.0
        self.current_step = 0
        self.total_steps = 5
        self.step_name = ""
        self.step_action = ""
        self.step_power = 0.0
        self.step_duration = 0.0
        self.step_start_time = 0.0
        self.mission_start_time = 0.0
        self.active_target_depth = 0.0
        self.steps_list = list(DEFAULT_STEPS)

        # Config state cache
        self.config = get_default_config()
        self.load_local_jalur()

        # Console logs
        self.recent_logs = []
        self.max_logs = 100
        self.add_log(f"Bridge initialized for {self.port} @ {self.baud} baud.")

        # Start thread
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    def add_log(self, msg: str):
        with self.state_lock:
            ts = time.strftime("%H:%M:%S")
            self.recent_logs.append(f"[{ts}] {msg}")
            if len(self.recent_logs) > self.max_logs:
                self.recent_logs.pop(0)

    def load_local_jalur(self):
        paths = [
            os.path.join(HERMES_DIR, "jalur.json"),
            os.path.join(BASE_DIR, "jalur.json"),
            "D:/Hermes/jalur.json"
        ]
        for p in paths:
            if os.path.exists(p):
                try:
                    with open(p, "r", encoding="utf-8") as f:
                        raw = json.load(f)
                    steps = []
                    for idx, s in enumerate(raw):
                        aksi = str(s.get("aksi", s.get("action", "IDLE"))).upper()
                        target = float(s.get("target_cm", s.get("target", 0.0)))
                        raw_motors = s.get("motors", None)
                        if raw_motors is not None and isinstance(raw_motors, list):
                            motors = sorted(list(set(int(m) for m in raw_motors if 1 <= int(m) <= 6)))
                        else:
                            motors = get_default_motors_for_action(aksi, target)

                        steps.append({
                            "langkah": s.get("langkah", s.get("step", idx + 1)),
                            "nama": s.get("nama", s.get("name", f"Langkah {idx+1}")),
                            "aksi": aksi,
                            "daya": float(s.get("daya", s.get("power", 15.0))),
                            "durasi_detik": float(s.get("durasi_detik", s.get("duration", 5.0))),
                            "target_cm": target,
                            "motors": motors
                        })
                    if steps:
                        with self.state_lock:
                            self.steps_list = steps
                            self.total_steps = len(steps)
                        print(f"[OK] Loaded {len(steps)} steps from {p}")
                        return True
                except Exception as e:
                    print(f"[WARN] Failed to read {p}: {e}")
        return False

    def _open_port(self):
        if not SERIAL_AVAILABLE:
            return None
        target_port = self.port
        try:
            available = list(serial.tools.list_ports.comports())
            devs = [p.device for p in available]
            if target_port not in devs:
                for p in available:
                    # Look for RP2040 VID 0x2E8A or Pico description
                    if p.vid == 0x2E8A or (p.hwid and "2E8A" in p.hwid) or "Pico" in (p.description or "") or "USB Serial" in (p.description or ""):
                        target_port = p.device
                        break
        except Exception:
            pass

        try:
            ser = serial.Serial(target_port, self.baud, timeout=0.04)
            time.sleep(0.15)
            ser.reset_input_buffer()
            self.port = target_port
            self.add_log(f"TERHUBUNG ke {self.port} @ {self.baud} baud.")
            # Enable hardware telemetry stream at 10 Hz
            ser.write(b"telem on\r\nstatus\r\nthrusters\r\nautorun status\r\n")
            ser.flush()
            return ser
        except Exception:
            return None

    def _parse_tel_packet(self, payload: str):
        parts = payload.strip().split(',')
        if len(parts) >= 16:
            try:
                depth_cm = float(parts[0])
                press_mbar = float(parts[1])
                pitch = float(parts[2])
                roll = float(parts[3])
                temp_c = float(parts[4])
                gx = float(parts[5])
                gy = float(parts[6])
                gz = float(parts[7])
                t_pulses = [int(p) for p in parts[8:14]]
                armed = int(parts[14])
                fsm_state = int(parts[15])

                now = time.time()
                if self.last_tel_time > 0.0:
                    dt = max(0.001, min(0.25, now - self.last_tel_time))
                    eff_gz = gz - self.gz_bias
                    if abs(eff_gz) > 0.8:
                        self.hardware_yaw = (self.hardware_yaw + eff_gz * dt) % 360.0
                        if self.hardware_yaw < 0:
                            self.hardware_yaw += 360.0
                self.last_tel_time = now

                with self.state_lock:
                    self.sensors["depth_cm"] = depth_cm
                    self.sensors["pressure_mbar"] = press_mbar
                    self.sensors["pitch"] = pitch
                    self.sensors["roll"] = roll
                    self.sensors["temp_c"] = temp_c
                    self.sensors["gyro"] = [gx, gy, gz]
                    self.sensors["yaw"] = round(self.hardware_yaw, 1)
                    self.pulses = t_pulses
                    self.armed = armed
                    self.fsm_state = fsm_state
                    self.last_packet_time = now
                    self.packets_rx += 1
                    self._hz_counter += 1
            except (ValueError, IndexError):
                pass

    def _parse_cli_line(self, line: str):
        """Parses human CLI responses from RP2040 and updates config/mission states."""
        with self.state_lock:
            # 1. Mission countdown tick
            m_tick = re.search(r'T-(\d+)s', line)
            if m_tick:
                self.mission_state = "COUNTDOWN"
                self.countdown_remaining = int(m_tick.group(1))
                return

            m_cd = re.search(r'Launch countdown started:\s*(\d+)\s*seconds', line, re.IGNORECASE)
            if m_cd:
                self.mission_state = "COUNTDOWN"
                self.countdown_duration = int(m_cd.group(1))
                self.countdown_remaining = self.countdown_duration
                self.countdown_start = time.time()
                return

            # 2. Mission step start
            m_step = re.search(r'Starting Step\s*(\d+)/(\d+):\s*(.*?)\s*\|\s*Action:\s*(.*?)\s*\|\s*Power:\s*([\d\.]+)%\s*\|\s*Duration:\s*([\d\.]+)s', line, re.IGNORECASE)
            if not m_step:
                m_step = re.search(r'Starting Step\s*(\d+)/(\d+):\s*(.*?)\s*\[(.*?)\]\s*Power:\s*([\d\.]+)%\s*\(Duration:\s*([\d\.]+)s\)', line, re.IGNORECASE)
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
                return

            # 3. Mission completed or stopped
            if "MISSION STEPS COMPLETED" in line or "ALL MISSION STEPS COMPLETED" in line:
                self.mission_state = "COMPLETED"
                self.current_step = self.total_steps
                return

            if "EMERGENCY STOP" in line or "System disarmed" in line:
                if self.mission_state != "COMPLETED":
                    self.mission_state = "IDLE"
                    self.countdown_remaining = 0

            # 4. Config parameters
            m_pwr = re.search(r'(?:Safety Cap|limit set to|power limit:)\s*(\d+)%', line, re.IGNORECASE)
            if m_pwr:
                self.config["power_limit"] = int(m_pwr.group(1))

            m_pid = re.search(r'PID Stabilization:\s*\[\s*(ON|OFF)\s*\]', line, re.IGNORECASE)
            if m_pid:
                self.config["pid_enabled"] = (m_pid.group(1).upper() == "ON")

            # Parse PID gains from status or pid command responses
            m_pd = re.search(r'depth.*?kp=([\d\.]+).*?ki=([\d\.]+).*?kd=([\d\.]+)', line, re.IGNORECASE)
            if m_pd:
                self.config["pid"]["depth"]["kp"] = float(m_pd.group(1))
                self.config["pid"]["depth"]["ki"] = float(m_pd.group(2))
                self.config["pid"]["depth"]["kd"] = float(m_pd.group(3))
                if rl_agent:
                    rl_agent.sync_baseline_from_hardware(self.config["pid"])

            m_pr = re.search(r'roll.*?kp=([\d\.]+).*?ki=([\d\.]+).*?kd=([\d\.]+)', line, re.IGNORECASE)
            if m_pr:
                self.config["pid"]["roll"]["kp"] = float(m_pr.group(1))
                self.config["pid"]["roll"]["ki"] = float(m_pr.group(2))
                self.config["pid"]["roll"]["kd"] = float(m_pr.group(3))
                if rl_agent:
                    rl_agent.sync_baseline_from_hardware(self.config["pid"])

            m_pp = re.search(r'pitch.*?kp=([\d\.]+).*?ki=([\d\.]+).*?kd=([\d\.]+)', line, re.IGNORECASE)
            if m_pp:
                self.config["pid"]["pitch"]["kp"] = float(m_pp.group(1))
                self.config["pid"]["pitch"]["ki"] = float(m_pp.group(2))
                self.config["pid"]["pitch"]["kd"] = float(m_pp.group(3))
                if rl_agent:
                    rl_agent.sync_baseline_from_hardware(self.config["pid"])

            m_dead = re.search(r'ESC deadband set to\s*(\d+)\s*us', line, re.IGNORECASE)
            if m_dead:
                self.config["esc"]["deadband"] = int(m_dead.group(1))

            m_b_h = re.search(r'horizontal trim.*?([+-]?[\d\.]+)%', line, re.IGNORECASE)
            if m_b_h:
                self.config["balance"]["horiz"] = float(m_b_h.group(1))

            m_b_v = re.search(r'vertical trim.*?([+-]?[\d\.]+)%', line, re.IGNORECASE)
            if m_b_v:
                self.config["balance"]["vert"] = float(m_b_v.group(1))

            m_ar = re.search(r'Auto-Run on Boot\s*:\s*(ENABLED|DISABLED)', line, re.IGNORECASE)
            if m_ar:
                self.config["autorun"]["enabled"] = (m_ar.group(1).upper() == "ENABLED")

            m_zero = re.search(r'depth reference pressure set to\s*([\d\.]+)\s*mbar', line, re.IGNORECASE)
            if m_zero:
                self.config["calib"]["zero_pressure"] = float(m_zero.group(1))
                self.config["calib"]["calibrated"] = True
                self.config["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")

            m_lvl = re.search(r'offsets (?:recorded|set to) (?:Pitch:\s*([+-]?[\d\.]+)[°]?,?\s*Roll:\s*([+-]?[\d\.]+)[°]?)', line, re.IGNORECASE)
            if m_lvl:
                self.config["calib"]["pitch_offset"] = float(m_lvl.group(1))
                self.config["calib"]["roll_offset"] = float(m_lvl.group(2))
                self.config["calib"]["last_calib_time"] = time.strftime("%H:%M:%S")

            m_cs = re.search(r'Checksum:\s*(0x[0-9a-fA-F]+)', line)
            if m_cs:
                self.config["last_save_checksum"] = m_cs.group(1)

    def _worker_loop(self):
        """Dedicated background reader loop that isolates machine packets from CLI output."""
        while self.running:
            now = time.time()

            # Connection management
            if self.ser is None or not self.ser.is_open:
                self.connected = False
                self.ser = self._open_port()
                if self.ser is None:
                    # Mock simulation behavior when offline
                    self._mock_tick(now)
                    time.sleep(0.1)
                    continue
                else:
                    self.connected = True

            # Calculate telemetry rate Hz
            if now - self._hz_timer >= 1.0:
                self.rate_hz = self._hz_counter / (now - self._hz_timer)
                self._hz_counter = 0
                self._hz_timer = now

            # Read from serial port
            try:
                has_read = False
                while self.ser and self.ser.in_waiting:
                    has_read = True
                    raw = self.ser.readline().decode('utf-8', errors='replace')
                    cleaned = clean_ansi(raw)
                    if not cleaned:
                        continue

                    # 1. Check for $TEL packet anywhere in line
                    idx = cleaned.find('$TEL,')
                    if idx != -1:
                        # Extract the packet content
                        payload = cleaned[idx + 5:].strip().split()[0]
                        self._parse_tel_packet(payload)
                        # Remove the $TEL part from cleaned line
                        cleaned = (cleaned[:idx] + cleaned[idx + 5 + len(payload):]).strip()

                    # Strip prompt prefix
                    cleaned = re.sub(r'^AUV>\s*', '', cleaned).strip()
                    if not cleaned or cleaned == 'AUV>':
                        continue

                    # Add non-telemetry line to terminal logs
                    self.add_log(f"<<< {cleaned}")
                    self._parse_cli_line(cleaned)

                    # 2. Check if synchronous command is waiting for reply
                    with self.cmd_lock:
                        if self.pending_cmd is not None:
                            # Echo suppression: if line matches the sent command exactly, ignore it
                            if cleaned.lower() != self.pending_cmd.lower():
                                self.cmd_reply_lines.append(cleaned)
                                self.last_reply_line_time = time.time()

                if not has_read:
                    time.sleep(0.005)

            except Exception as e:
                self.add_log(f"Koneksi terputus: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.connected = False
                time.sleep(0.5)

    def start_sim_mission(self):
        with self.state_lock:
            self.sim_mission_active = True
            self.sim_mission_completed = False
            self.sim_mission_step_idx = 0
            self.sim_mission_step_start = time.time()
            self.sim_x = 100.0
            self.sim_depth = 0.0
            self.sim_yaw = 0.0
            self.sim_vx = 0.0
            self.sim_vz = 0.0
            self.sim_pitch = 0.0
            self.sim_roll = 0.0
        self.add_log("[SIMULASI] Simulasi Jalur Misi Virtual DIMULAI!")

    def stop_sim_mission(self):
        with self.state_lock:
            self.sim_mission_active = False
            self.sim_mission_completed = False
            self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
        self.add_log("[SIMULASI] Simulasi Jalur Misi Virtual DIHENTIKAN!")

    def get_sim_mission_status(self) -> dict:
        with self.state_lock:
            now = time.time()
            steps = self.steps_list
            total_steps = len(steps)
            cur_idx = self.sim_mission_step_idx
            
            cur_step = steps[cur_idx] if 0 <= cur_idx < total_steps else None
            step_dur = cur_step.get("durasi_detik", 5.0) if cur_step else 0.0
            step_elapsed = (now - self.sim_mission_step_start) if self.sim_mission_active and cur_step else 0.0
            step_rem = max(0.0, step_dur - step_elapsed) if cur_step else 0.0
            step_pct = min(100.0, (step_elapsed / step_dur) * 100.0) if step_dur > 0 else 0.0

            total_dur = sum(s.get("durasi_detik", 0.0) for s in steps) or 37.0
            done_dur = sum(steps[i].get("durasi_detik", 0.0) for i in range(min(cur_idx, total_steps)))
            if self.sim_mission_active and cur_step:
                done_dur += min(step_elapsed, step_dur)
            elif self.sim_mission_completed:
                done_dur = total_dur
            overall_pct = min(100.0, (done_dur / total_dur) * 100.0) if total_dur > 0 else 0.0

            return {
                "active": self.sim_mission_active,
                "completed": self.sim_mission_completed,
                "current_step": cur_idx + 1 if self.sim_mission_active else (total_steps if self.sim_mission_completed else 0),
                "total_steps": total_steps,
                "step_name": cur_step.get("nama", "Idle") if cur_step else ("Selesai" if self.sim_mission_completed else "Standby"),
                "step_action": cur_step.get("aksi", "IDLE") if cur_step else "IDLE",
                "step_power": cur_step.get("daya", 0.0) if cur_step else 0.0,
                "target_depth_cm": cur_step.get("target_cm", 0.0) if cur_step else 0.0,
                "step_duration": step_dur,
                "step_elapsed_sec": round(step_elapsed, 1),
                "step_remaining_sec": round(step_rem, 1),
                "step_progress_pct": round(step_pct, 1),
                "overall_progress_pct": round(overall_pct, 1),
                "x_cm": round(self.sim_x, 1),
                "depth_cm": round(self.sim_depth, 1),
                "yaw_deg": round(self.sim_yaw, 1),
                "pitch_deg": round(self.sim_pitch, 1),
                "roll_deg": round(self.sim_roll, 1),
                "vx_cms": round(self.sim_vx, 1),
                "vz_cms": round(self.sim_vz, 1)
            }

    def _mock_tick(self, now: float):
        """Provides simulated hydrodynamic telemetry and virtual mission execution."""
        dt = 0.05
        with self.state_lock:
            # 1. Virtual Mission Execution
            if self.sim_mission_active:
                steps = self.steps_list
                if 0 <= self.sim_mission_step_idx < len(steps):
                    step = steps[self.sim_mission_step_idx]
                    step_dur = step.get("durasi_detik", 5.0)
                    step_el = now - self.sim_mission_step_start
                    aksi = step.get("aksi", "IDLE").upper()
                    daya = float(step.get("daya", 15.0))
                    target_z = float(step.get("target_cm", 0.0))
                    
                    # Compute virtual thruster pulses based on action
                    if aksi in ["DIVE", "SELAM"]:
                        err_z = target_z - self.sim_depth
                        p_vert = 1500 + int(math.copysign(min(daya * 5.0, max(15.0, abs(err_z) * 3.5)), err_z))
                        if abs(err_z) < 2.0:
                            p_vert = 1515  # anti-buoyancy trim
                        self.pulses = [1500, 1500, 1500, 1500, p_vert, p_vert]
                    elif aksi in ["MAJU", "FORWARD"]:
                        err_z = target_z - self.sim_depth
                        p_vert = 1500 + int(math.copysign(min(15.0 * 5.0, max(10.0, abs(err_z) * 3.0)), err_z)) if target_z > 0 else 1500
                        p_fwd = 1500 + int(daya * 5.0)
                        self.pulses = [p_fwd, p_fwd, p_fwd, p_fwd, p_vert, p_vert]
                    elif aksi in ["MUNDUR", "BACKWARD"]:
                        p_bwd = 1500 - int(daya * 5.0)
                        self.pulses = [p_bwd, p_bwd, p_bwd, p_bwd, 1500, 1500]
                    elif aksi in ["PUTAR", "TURN"]:
                        p_turn = int(daya * 5.0)
                        self.pulses = [1500 - p_turn, 1500 + p_turn, 1500 - p_turn, 1500 + p_turn, 1500, 1500]
                    elif aksi in ["SURFACE", "NAIK"]:
                        self.pulses = [1500, 1500, 1500, 1500, 1470, 1470]
                    else:
                        self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
                        
                    # Advance step
                    if step_el >= step_dur:
                        self.sim_mission_step_idx += 1
                        self.sim_mission_step_start = now
                        if self.sim_mission_step_idx >= len(steps):
                            self.sim_mission_active = False
                            self.sim_mission_completed = True
                            self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
                            self.add_log("[SIMULASI] Seluruh langkah misi selesai dieksekusi!")
                        else:
                            next_s = steps[self.sim_mission_step_idx]
                            self.add_log(f"[SIMULASI] Masuk Langkah #{self.sim_mission_step_idx+1}: {next_s.get('nama', '')} [{next_s.get('aksi','')}]")

            # Check mock test timer
            elif self.mock_test_end_time > 0:
                if now >= self.mock_test_end_time:
                    self.mock_test_end_time = 0.0
                    self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]

            # 2. Hydrodynamic 2D Physics Integration with Configurable Mass & Buoyancy
            total_m_z = self.sim_mass + self.sim_added_mass_z
            total_m_x = self.sim_mass + self.sim_added_mass_x
            buoyancy_force_n = (self.sim_net_buoyancy_g / 1000.0) * 9.80665

            # Vertical dynamics (T5, T6)
            p_vert_avg = (self.pulses[4] + self.pulses[5]) / 2.0 - 1500.0
            f_thrust_z = p_vert_avg * 0.18
            f_buoy_z = -buoyancy_force_n * 3.0  # upwards acceleration scaled to cm/s^2
            f_drag_z = (-0.045 * self.sim_drag_scale) * self.sim_vz * abs(self.sim_vz) - (0.25 * self.sim_drag_scale) * self.sim_vz
            accel_z = (f_thrust_z * 22.0 / total_m_z) + (f_buoy_z * 22.0 / total_m_z) + f_drag_z
            self.sim_vz += accel_z * dt
            self.sim_depth += self.sim_vz * dt
            
            # Floor & Surface bounds
            if self.sim_depth < 0.0:
                self.sim_depth = 0.0
                if self.sim_vz < 0: self.sim_vz = 0.0
            elif self.sim_depth > 138.0:
                self.sim_depth = 138.0
                if self.sim_vz > 0: self.sim_vz = 0.0

            # Horizontal dynamics (T1..T4)
            p_surge_avg = ((self.pulses[0] - 1500) + (self.pulses[1] - 1500) + (self.pulses[2] - 1500) + (self.pulses[3] - 1500)) / 4.0
            f_thrust_x = p_surge_avg * 0.22 * (18.0 / total_m_x)
            f_drag_x = (-0.025 * self.sim_drag_scale) * self.sim_vx * abs(self.sim_vx) - (0.20 * self.sim_drag_scale) * self.sim_vx
            accel_x = f_thrust_x + f_drag_x
            self.sim_vx += accel_x * dt
            self.sim_x += self.sim_vx * dt * math.cos(math.radians(self.sim_yaw))
            self.sim_x = max(20.0, min(1180.0, self.sim_x))

            # Yaw rotation dynamics
            p_yaw = ((self.pulses[1] - 1500) + (self.pulses[3] - 1500)) - ((self.pulses[0] - 1500) + (self.pulses[2] - 1500))
            self.sim_vyaw += (p_yaw * 0.08 - 0.6 * self.sim_vyaw) * dt
            self.sim_yaw = (self.sim_yaw + self.sim_vyaw * dt) % 360.0

            # Pitch & Roll dynamics
            if self.sim_wave:
                self.sim_wave_phase += dt * 2.2
                wave_p = 5.0 * math.sin(self.sim_wave_phase * 0.8)
                wave_r = 8.0 * math.cos(self.sim_wave_phase)
            else:
                wave_p = 0.0
                wave_r = 0.0
                
            nose_down = -0.08 * self.sim_vx
            self.sim_pitch = max(-35.0, min(35.0, nose_down + wave_p))
            self.sim_roll = max(-35.0, min(35.0, wave_r))

            # Sync to sensor state
            self.sensors["depth_cm"] = round(self.sim_depth, 1)
            self.sensors["pressure_mbar"] = round(1013.25 + self.sim_depth * 0.981, 1)
            self.sensors["pitch"] = round(self.sim_pitch, 1)
            self.sensors["roll"] = round(self.sim_roll, 1)
            self.sensors["temp_c"] = round(self.sim_temp, 1)
            self.sensors["gyro"] = [round(self.sim_roll * 0.1, 1), round(self.sim_pitch * 0.1, 1), round(self.sim_vyaw, 1)]
            self.sensors["yaw"] = round(self.sim_yaw, 1)
            self.rate_hz = 12.0

    def execute_command(self, cmd: str, timeout: float = 0.85) -> dict:
        """Sends command synchronously and waits for real hardware confirmation."""
        cmd = cmd.strip()
        if not cmd:
            return {"success": False, "error": "Perintah kosong"}

        root = cmd.lower().split()[0] if cmd.split() else ""
        if root in ['status', 'stat', 'thrusters', 'motor', 'motors', 'help', '?', 'autorun', 'jalur', 'mission', 'sensor', 'sensors']:
            timeout = max(timeout, 1.4)

        # Check for compass tare shortcut
        if cmd.lower() in ['calib compass', 'calib yaw', 'tare compass', 'tare yaw', 'tare 0']:
            with self.state_lock:
                self.hardware_yaw = 0.0
                self.sim_yaw = 0.0
                if len(self.sensors.get("gyro", [])) >= 3:
                    self.gz_bias = self.sensors["gyro"][2]
                self.sensors["yaw"] = 0.0
            self.add_log("[KOMPAS] Kompas berhasil di-tare ke 0.0° Utara!")
            return {"success": True, "reply": "OK: Kompas berhasil dinolkan (Heading: 000° Utara)!", "command": cmd}

        self.add_log(f">>> {cmd}")

        if not self.connected or self.ser is None:
            # When disconnected, MUST RETURN success: False! Zero fake green toasts!
            return {
                "success": False,
                "error": f"Koneksi Hardware {self.port} sedang TERPUTUS! Hubungkan kabel USB Pico W ke laptop.",
                "command": cmd,
                "offline": True
            }

        with self.cmd_lock:
            self.cmd_reply_lines = []
            self.pending_cmd = cmd
            self.last_reply_line_time = 0.0
            try:
                self.ser.write((cmd + "\r\n").encode('utf-8'))
                self.ser.flush()
                self.packets_tx += 1
            except Exception as e:
                self.pending_cmd = None
                return {"success": False, "error": f"Gagal mengirim serial: {e}", "command": cmd}

        # Wait for reply lines to arrive and quiet down
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.cmd_lock:
                if self.cmd_reply_lines and (time.time() - self.last_reply_line_time >= 0.08):
                    break
            time.sleep(0.01)

        with self.cmd_lock:
            lines = list(self.cmd_reply_lines)
            self.pending_cmd = None
            self.cmd_reply_lines = []

        if lines:
            reply = "\n".join(lines)
            return {"success": True, "reply": reply, "command": cmd}
        else:
            return {"success": False, "error": f"Hardware timeout ({int(timeout*1000)}ms): Tidak ada respon untuk '{cmd}'", "command": cmd}

    def _mock_command_reply(self, cmd: str) -> str:
        c = cmd.lower()
        parts = c.split()
        root = parts[0] if parts else ""
        if root in ['tes', 'test']:
            m = parts[1] if len(parts) > 1 else '1'
            p = parts[2] if len(parts) > 2 else '15'
            self.mock_test_end_time = time.time() + 2.0
            return f"Uji motor {m} pada daya {p}% selama 2.0 detik"
        elif root == 'stop':
            self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
            self.mission_state = "IDLE"
            return "Semua motor dinetralkan (1500us). Emergency Stop aktif!"
        elif root == 'save':
            self.config["last_save_checksum"] = f"0x{int(time.time()*1000)&0xFFFFFFFF:08X}"
            return f"Konfigurasi tersimpan ke EEPROM! Checksum: {self.config['last_save_checksum']}"
        elif root == 'default':
            self.config = get_default_config()
            return "Konfigurasi dikembalikan ke default pabrik!"
        elif root == 'calib':
            sub = parts[1] if len(parts) > 1 else 'zero'
            if sub == 'zero':
                self.config["calib"]["zero_pressure"] = 1014.1
                self.sim_depth = 0.0
                return "Kedalaman berhasil dinolkan pada 1014.1 mbar!"
            elif sub == 'level':
                self.config["calib"]["pitch_offset"] = 0.0
                self.config["calib"]["roll_offset"] = 0.0
                self.sim_pitch = 0.0
                self.sim_roll = 0.0
                return "IMU level offsets berhasil direkam!"
        return f"Perintah '{cmd}' diterima (simulasi offline)"

    def upload_mission_steps(self, steps_data: list) -> dict:
        """Validates JSON, writes locally to jalur.json, and uploads to Pico W LittleFS."""
        if not isinstance(steps_data, list) or len(steps_data) == 0:
            return {"success": False, "error": "Data jalur harus berupa array JSON tidak kosong"}

        validated_steps = []
        for idx, s in enumerate(steps_data):
            try:
                langkah = int(s.get("langkah", s.get("step", idx + 1)))
                nama = str(s.get("nama", s.get("name", f"Langkah {idx+1}"))).strip()
                aksi = str(s.get("aksi", s.get("action", "IDLE"))).strip().upper()
                daya = float(s.get("daya", s.get("power", 15)))
                durasi = float(s.get("durasi_detik", s.get("duration", 5.0)))
                target = float(s.get("target_cm", s.get("target", 0.0)))
                
                raw_motors = s.get("motors", None)
                if raw_motors is not None and isinstance(raw_motors, list):
                    motors = sorted(list(set(int(m) for m in raw_motors if 1 <= int(m) <= 6)))
                else:
                    motors = get_default_motors_for_action(aksi, target)

                validated_steps.append({
                    "langkah": langkah,
                    "nama": nama,
                    "aksi": aksi,
                    "daya": daya,
                    "durasi_detik": durasi,
                    "target_cm": target,
                    "motors": motors
                })
            except Exception as ex:
                return {"success": False, "error": f"Langkah #{idx+1} tidak valid: {ex}"}

        # 1. Save locally to D:/Hermes/jalur.json and local dir
        json_str = json.dumps(validated_steps, indent=2, ensure_ascii=False)
        saved_paths = []
        for p in [os.path.join(HERMES_DIR, "jalur.json"), os.path.join(BASE_DIR, "jalur.json"), "D:/Hermes/jalur.json"]:
            try:
                with open(p, "w", encoding="utf-8") as f:
                    f.write(json_str)
                saved_paths.append(p)
            except Exception:
                pass

        with self.state_lock:
            self.steps_list = validated_steps
            self.total_steps = len(validated_steps)

        if not self.connected or self.ser is None:
            return {
                "success": False,
                "error": f"Robot TERPUTUS! Gagal mengunggah ke /jalur.json Pico W (Data hanya tersimpan lokal di laptop).",
                "count": len(validated_steps),
                "offline": True
            }

        # 2. Upload to Pico W via serial protocol
        with self.cmd_lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b"jalur upload\r\n")
                self.ser.flush()

                # Wait for [UPLOAD] READY
                ready = False
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    if self.ser.in_waiting:
                        line = clean_ansi(self.ser.readline().decode('utf-8', errors='replace'))
                        if "[UPLOAD] READY" in line:
                            ready = True
                            break
                    time.sleep(0.01)

                if not ready:
                    return {"success": False, "error": "Pico W tidak merespon kesiapan upload [UPLOAD] READY"}

                # Stream payload followed by EOF
                compact_payload = json.dumps(validated_steps)
                self.ser.write(compact_payload.encode('utf-8') + b"\r\n===EOF===\r\n")
                self.ser.flush()

                # Wait for [UPLOAD] SUCCESS
                confirmed = False
                reply_msg = ""
                t1 = time.time()
                while time.time() - t1 < 3.0:
                    if self.ser.in_waiting:
                        line = clean_ansi(self.ser.readline().decode('utf-8', errors='replace'))
                        if "[UPLOAD] SUCCESS" in line:
                            confirmed = True
                            reply_msg = line
                            break
                        elif "[UPLOAD] ERROR" in line:
                            return {"success": False, "error": line}
                    time.sleep(0.01)

                if confirmed:
                    return {
                        "success": True,
                        "count": len(validated_steps),
                        "reply": reply_msg or "Misi /jalur.json berhasil diverifikasi & disimpan ke LittleFS!"
                    }
                else:
                    return {"success": False, "error": "Timeout konfirmasi penyimpanan LittleFS Pico W"}

            except Exception as e:
                return {"success": False, "error": f"Kesalahan serial upload: {e}"}

    def get_mission_status(self) -> dict:
        now = time.time()
        elapsed_sec = 0.0
        remaining_sec = 0.0
        step_progress = 0.0

        if self.mission_state == "COUNTDOWN":
            if self.countdown_start > 0:
                c_el = now - self.countdown_start
                self.countdown_remaining = max(0, int(round(self.countdown_duration - c_el)))
            remaining_sec = float(self.countdown_remaining)

        elif self.mission_state == "RUNNING":
            if self.step_start_time > 0 and self.step_duration > 0:
                elapsed_sec = max(0.0, now - self.step_start_time)
                remaining_sec = max(0.0, self.step_duration - elapsed_sec)
                step_progress = min(100.0, (elapsed_sec / self.step_duration) * 100.0)

        elif self.mission_state == "COMPLETED":
            step_progress = 100.0
            remaining_sec = 0.0

        total_dur = sum(s.get("durasi_detik", 0.0) for s in self.steps_list) or 37.0
        completed_dur = 0.0
        for i in range(min(self.current_step - 1, len(self.steps_list))):
            completed_dur += self.steps_list[i].get("durasi_detik", 0.0)
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
            "step_name": self.step_name or (self.steps_list[0]["nama"] if self.steps_list else "Standby"),
            "step_action": self.step_action or (self.steps_list[0]["aksi"] if self.steps_list else "IDLE"),
            "step_power": self.step_power or (self.steps_list[0]["daya"] if self.steps_list else 0.0),
            "step_duration": self.step_duration or (self.steps_list[0]["durasi_detik"] if self.steps_list else 0.0),
            "target_depth_cm": self.active_target_depth,
            "elapsed_sec": round(elapsed_sec, 1),
            "remaining_sec": round(remaining_sec, 1),
            "step_progress_pct": round(step_progress, 1),
            "overall_progress_pct": round(overall_progress, 1),
            "total_duration_sec": total_dur,
            "steps": self.steps_list
        }

    def get_full_state(self) -> dict:
        with self.state_lock:
            deltas = [round((p - 1500) / 5.0, 1) for p in self.pulses]
            return {
                "connected": self.connected,
                "port": self.port,
                "baud": self.baud,
                "rate_hz": round(self.rate_hz, 1),
                "packets_rx": self.packets_rx,
                "packets_tx": self.packets_tx,
                "sensors": copy.deepcopy(self.sensors),
                "pulses": list(self.pulses),
                "deltas": deltas,
                "armed": self.armed,
                "fsm_state": self.fsm_state,
                "config": copy.deepcopy(self.config),
                "mission": self.get_mission_status(),
                "sim_mission": self.get_sim_mission_status(),
                "physics": {
                    "mass_kg": self.sim_mass,
                    "added_mass_z_kg": self.sim_added_mass_z,
                    "added_mass_x_kg": self.sim_added_mass_x,
                    "net_buoyancy_g": self.sim_net_buoyancy_g,
                    "buoyancy_force_n": round((self.sim_net_buoyancy_g / 1000.0) * 9.80665, 3),
                    "drag_scale": self.sim_drag_scale
                },
                "rl_pid": rl_agent.get_status() if rl_agent else None,
                "logs": list(self.recent_logs)
            }

    def shutdown(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                # Stop streaming and neutralize thrusters before closing
                self.ser.write(b"telem off\r\nstop\r\n")
                self.ser.flush()
                time.sleep(0.08)
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        print("[*] Serial bridge shut down cleanly.")


# Global Bridge Instance
bridge = HardwareBridge()


class UnifiedHttpHandler(http.server.SimpleHTTPRequestHandler):
    """Multi-threaded HTTP handler for unified web app and truthful JSON API."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def log_message(self, format, *args):
        # Silence routine 200 GET access logs to keep terminal readable
        pass

    def send_json(self, data: dict, code: int = 200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == '/api/state':
            self.send_json(bridge.get_full_state())
            return
        elif path == '/api/mission' or path == '/api/mission_load':
            self.send_json(bridge.get_mission_status())
            return
        elif path == '/api/sim_mission':
            self.send_json(bridge.get_sim_mission_status())
            return
        elif path == '/api/rl_pid/status':
            self.send_json(rl_agent.get_status() if rl_agent else {"status": "DISABLED", "enabled": False})
            return
        elif path == '/api/logs':
            self.send_json({"logs": bridge.recent_logs})
            return

        # Serve index.html by default
        if path == '/' or path == '/index.html' or path == '/config' or path == '/config.html':
            self.path = '/index.html'

        return super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', 0))
        raw_body = self.rfile.read(length).decode('utf-8', errors='replace')
        body = {}
        if raw_body:
            try:
                body = json.loads(raw_body)
            except Exception:
                pass

        if path == '/api/command':
            cmd = body.get('command', '').strip()
            res = bridge.execute_command(cmd)
            self.send_json(res)
            return

        elif path == '/api/tare_compass' or path == '/api/tare_yaw':
            with bridge.state_lock:
                bridge.hardware_yaw = 0.0
                bridge.sim_yaw = 0.0
                if len(bridge.sensors.get("gyro", [])) >= 3:
                    bridge.gz_bias = bridge.sensors["gyro"][2]
                bridge.sensors["yaw"] = 0.0
            bridge.add_log("[KOMPAS] Kompas berhasil di-tare ke 0.0° Utara!")
            self.send_json({"success": True, "reply": "Kompas berhasil dinolkan ke 0.0° Utara!", "yaw": 0.0})
            return

        elif path == '/api/emergency_stop':
            res = bridge.execute_command('stop')
            self.send_json(res)
            return

        elif path == '/api/test_motor':
            motor_id = body.get('id', 1)
            power = body.get('power', 15)
            try:
                p_val = float(power)
            except (ValueError, TypeError):
                p_val = 15.0

            if str(motor_id).lower() in ['all', '0']:
                cmd = f"tes all {p_val:.0f}"
            else:
                m_int = max(1, min(6, int(motor_id)))
                cmd = f"tes {m_int} {p_val:.0f}"

            res = bridge.execute_command(cmd)
            self.send_json(res)
            return

        elif path == '/api/mission_action':
            action = body.get('action', '').strip().lower()
            if action in ['run', 'start']:
                cmd = "run"
            elif action in ['run_now', 'now', 'instant']:
                cmd = "run now"
            elif action in ['stop', 'abort', 'cancel']:
                cmd = "stop"
            else:
                cmd = "stop"
            res = bridge.execute_command(cmd)
            res["mission"] = bridge.get_mission_status()
            self.send_json(res)
            return

        elif path == '/api/mission_upload':
            steps = body.get('steps', body if isinstance(body, list) else [])
            res = bridge.upload_mission_steps(steps)
            self.send_json(res)
            return

        elif path == '/api/param_update':
            p_type = body.get('type', '').strip().lower()
            cmd = None

            if p_type == 'limit':
                val = max(1, min(100, int(body.get('value', 25))))
                cmd = f"limit {val}"

            elif p_type == 'pid':
                axis = str(body.get('axis', 'depth')).lower()
                kp = float(body.get('kp', 0.6))
                ki = float(body.get('ki', 0.0))
                kd = float(body.get('kd', 0.015))
                cmd = f"pid {axis} {kp:.4f} {ki:.4f} {kd:.4f}"

            elif p_type == 'pid_state':
                en = bool(body.get('enabled', True))
                cmd = "pid on" if en else "pid off"

            elif p_type == 'thruster':
                t_id = max(1, min(6, int(body.get('id', 1))))
                prop = str(body.get('prop', 'reverse')).lower()
                val = body.get('value')
                if prop == 'reverse':
                    v_str = "1" if val in [1, "1", True, "on", "yes"] else "0"
                elif prop == 'trim':
                    v_str = f"{float(val):.2f}"
                else:
                    v_str = str(val)
                cmd = f"thruster {t_id} {prop} {v_str}"

            elif p_type == 'balance':
                axis = str(body.get('axis', 'horiz')).lower()
                pct = float(body.get('value', 0.0))
                pct = max(-20.0, min(20.0, pct))
                cmd = f"motor balance {axis} {pct:+.1f}"

            elif p_type == 'esc_deadband':
                val = max(0, min(80, int(body.get('value', 30))))
                cmd = f"esc deadband {val}"

            elif p_type == 'esc_calib':
                sub = str(body.get('sub', 'start')).lower()
                cmd = f"esc calib {sub}"

            elif p_type == 'autorun_timing':
                delay = max(3, min(60, int(body.get('delay', 10))))
                tare = max(1, min(delay - 1, int(body.get('tare', 3))))
                lvl = max(1, min(delay - 1, int(body.get('level', 6))))
                cmd = f"autorun timing {delay} {tare} {lvl}"

            elif p_type == 'autorun':
                en = bool(body.get('enabled', False))
                cmd = "autorun on" if en else "autorun off"

            elif p_type == 'calib':
                target = str(body.get('target', 'zero')).lower()
                if target == 'zero':
                    mbar = body.get('mbar')
                    cmd = f"calib zero {float(mbar):.1f}" if mbar else "calib zero"
                elif target == 'level':
                    cmd = "calib level"

            elif p_type == 'save':
                cmd = "save"

            elif p_type == 'default':
                cmd = "default"

            if cmd:
                res = bridge.execute_command(cmd)
                self.send_json(res)
            else:
                self.send_json({"success": False, "error": f"Tipe parameter tidak dikenal: {p_type}"}, code=400)
            return

        elif path == '/api/control':
            # Simulator override for 2D Digital Twin
            with bridge.state_lock:
                if 'depth' in body and body['depth'] is not None:
                    bridge.sim_depth = max(0.0, min(140.0, float(body['depth'])))
                if 'x' in body and body['x'] is not None:
                    bridge.sim_x = max(0.0, min(1200.0, float(body['x'])))
                if 'yaw' in body and body['yaw'] is not None:
                    bridge.sim_yaw = float(body['yaw']) % 360.0
                if 'pitch' in body and body['pitch'] is not None:
                    bridge.sim_pitch = max(-45.0, min(45.0, float(body['pitch'])))
                if 'roll' in body and body['roll'] is not None:
                    bridge.sim_roll = max(-45.0, min(45.0, float(body['roll'])))
                if 'wave' in body and body['wave'] is not None:
                    bridge.sim_wave = bool(body['wave'])
                if 'pulses' in body and isinstance(body['pulses'], list):
                    bridge.pulses = [int(p) for p in body['pulses'][:6]]
            self.send_json({"success": True, "state": bridge.get_full_state()})
            return

        elif path == '/api/sim_mission':
            action = body.get('action', 'start').lower()
            if action in ['start', 'run']:
                bridge.start_sim_mission()
            elif action in ['stop', 'abort', 'reset']:
                bridge.stop_sim_mission()
            self.send_json({"success": True, "sim_mission": bridge.get_sim_mission_status()})
            return

        elif path == '/api/rl_pid/train':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            dist = body.get('disturbance')
            episodes = int(body.get('episodes', 30))
            res = rl_agent.train_simulation(disturbance=dist, episodes=episodes)
            self.send_json({"success": True, "result": res})
            return

        elif path == '/api/rl_pid/apply':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            gains = rl_agent.adaptive_gains
            results = []
            d = gains['depth']
            results.append(bridge.execute_command(f"pid depth {d['kp']:.4f} {d['ki']:.4f} {d['kd']:.4f}"))
            r = gains['roll']
            results.append(bridge.execute_command(f"pid roll {r['kp']:.4f} {r['ki']:.4f} {r['kd']:.4f}"))
            p = gains['pitch']
            results.append(bridge.execute_command(f"pid pitch {p['kp']:.4f} {p['ki']:.4f} {p['kd']:.4f}"))
            results.append(bridge.execute_command("save"))
            
            all_ok = all(res.get('success', False) for res in results)
            any_offline = any(res.get('offline', False) for res in results)
            
            self.send_json({
                "success": all_ok,
                "offline": any_offline,
                "results": results,
                "applied_gains": gains,
                "reply": "Gains RL-PID berhasil diterapkan ke robot dan disimpan ke EEPROM!" if all_ok else ("Hardware offline: Parameter tersimpan pada simulator laptop" if any_offline else "Sebagian perintah PID gagal dieksekusi")
            })
            return

        elif path == '/api/rl_pid/toggle':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            en = bool(body.get('enabled', True))
            rl_agent.set_enabled(en)
            self.send_json({"success": True, "status": rl_agent.get_status()})
            return

        elif path == '/api/rl_pid/config':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            if 'disturbance' in body:
                rl_agent.set_disturbance(float(body['disturbance']))
            if 'target_depth_cm' in body:
                with rl_agent.lock:
                    rl_agent.target_depth_cm = float(body['target_depth_cm'])
                    rl_agent._simulate_step_response()
            if 'mass' in body or 'net_buoyancy_g' in body or 'drag_scale' in body:
                mass_val = float(body['mass']) if 'mass' in body else None
                buoy_val = float(body['net_buoyancy_g']) if 'net_buoyancy_g' in body else None
                drag_val = float(body['drag_scale']) if 'drag_scale' in body else None
                rl_agent.set_physics_parameters(mass=mass_val, net_buoyancy_g=buoy_val, drag_scale=drag_val)
                with bridge.state_lock:
                    if mass_val is not None: bridge.sim_mass = mass_val
                    if buoy_val is not None: bridge.sim_net_buoyancy_g = buoy_val
                    if drag_val is not None: bridge.sim_drag_scale = drag_val
            self.send_json({"success": True, "status": rl_agent.get_status()})
            return

        elif path == '/api/physics_config':
            with bridge.state_lock:
                if 'mass' in body:
                    bridge.sim_mass = max(5.0, min(50.0, float(body['mass'])))
                if 'net_buoyancy_g' in body:
                    bridge.sim_net_buoyancy_g = max(0.0, min(1000.0, float(body['net_buoyancy_g'])))
                if 'drag_scale' in body:
                    bridge.sim_drag_scale = max(0.2, min(5.0, float(body['drag_scale'])))
                mass_cur = bridge.sim_mass
                buoy_cur = bridge.sim_net_buoyancy_g
                drag_cur = bridge.sim_drag_scale
            if rl_agent:
                rl_agent.set_physics_parameters(mass=mass_cur, net_buoyancy_g=buoy_cur, drag_scale=drag_cur)
            self.send_json({
                "success": True,
                "mass_kg": mass_cur,
                "net_buoyancy_g": buoy_cur,
                "drag_scale": drag_cur,
                "buoyancy_force_n": round((buoy_cur / 1000.0) * 9.80665, 3)
            })
            return

        self.send_response(404)
        self.end_headers()


def run_app():
    server_address = ('', PORT)
    httpd = http.server.ThreadingHTTPServer(server_address, UnifiedHttpHandler)

    print("=" * 76)
    print(" AUV SAUVAC PUSAT KONTROL & UNIFIED DIGITAL TWIN")
    print(" Universitas Bangka Belitung — RoboNation SAUVC 2027")
    print(f" Web Server  : http://localhost:{PORT}")
    print(f" Hardware    : {DEFAULT_COM_PORT} @ {DEFAULT_BAUD} bps (Auto-detecting)")
    print(" Tekan Ctrl+C untuk menghentikan server dan menutup port secara bersih.")
    print("=" * 76)

    def launch_browser():
        time.sleep(1.0)
        webbrowser.open(f"http://localhost:{PORT}")

    b_thread = threading.Thread(target=launch_browser, daemon=True)
    b_thread.start()

    def handle_exit(signum, frame):
        print("\n[*] Menutup server & membersihkan port serial...")
        bridge.shutdown()
        httpd.server_close()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        handle_exit(None, None)


if __name__ == '__main__':
    run_app()
