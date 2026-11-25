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
MAX_MISSION_STEPS = 32
MAX_MISSION_BYTES = 3500
VALID_MISSION_ACTIONS = {
    "DIVE", "SELAM", "FORWARD", "MAJU", "BACKWARD", "MUNDUR", "REVERSE",
    "TURN", "PUTAR", "YAW", "SWAY_LEFT", "SWAYLEFT", "KIRI", "LEFT",
    "SWAY_RIGHT", "SWAYRIGHT", "KANAN", "RIGHT", "HOVER", "DIAM", "HOLD",
    "SURFACE", "NAIK", "TES_T1", "T1", "MOTOR_1", "M1", "TES_T2", "T2",
    "MOTOR_2", "M2", "TES_T3", "T3", "MOTOR_3", "M3", "TES_T4", "T4",
    "MOTOR_4", "M4", "TES_T5", "T5", "MOTOR_5", "M5", "TES_T6", "T6",
    "MOTOR_6", "M6", "TES_ALL", "ALL", "ALL_MOTORS", "SEMUA"
}

FIRMWARE_CONFIG_VERSION = 6
FIRMWARE_PROTOCOL_VERSION = 1
CONFIG_BACKUP_DIR = os.path.join(BASE_DIR, "backups")

if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

try:
    from rl_agent import rl_agent
except ImportError:
    rl_agent = None

from hil_safety import (
    HIL_MAX_DELTA_US,
    HIL_WATCHDOG_MS,
    validate_hil_pulses,
    validate_hil_sensors,
)
from wifi_config import (
    MAX_FIRMWARE_IMAGE_BYTES,
    PicoFirmwareUpdateError,
    PicoWifiConfigClient,
    WifiConfigError,
    upload_firmware_via_wifi,
)

ANSI_REGEX = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]')

def clean_ansi(text: str) -> str:
    """Strips ANSI terminal escape sequences and trims whitespace."""
    return ANSI_REGEX.sub('', text).strip()

def finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} harus berupa angka")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} harus berupa angka terbatas")
    return number

def validate_config_backup(payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("Backup konfigurasi harus berupa JSON object")
    if payload.get("format") != "auv-config-backup":
        raise ValueError("Format backup konfigurasi tidak dikenali")
    config = payload.get("config")
    if not isinstance(config, dict):
        raise ValueError("Backup tidak memuat object config")
    return config

def vertical_pulse_pair(t5_pulse: int) -> list:
    """Return T5/T6 pulses that produce the same heave direction."""
    pulse = max(1000, min(2000, int(t5_pulse)))
    return [pulse, 3000 - pulse]

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
        self.serial_io_lock = threading.Lock()
        self.serial_transaction_lock = threading.Lock()
        self.hil_lock = threading.RLock()
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
            "yaw": 0.0,
            "imu_ready": False,
            "pressure_ready": False,
            "i2c_ready": False
        }
        self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
        self.armed = 0
        self.fsm_state = 0
        self.last_packet_time = 0.0
        self.hil_active = False
        self.hil_armed_at = 0.0

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
        self.sim_mass = 14.5            # kg (dry mass, 5.0 to 50.0 kg)
        self.sim_added_mass_z = 8.2     # kg
        self.sim_added_mass_x = 3.5     # kg
        self.sim_net_buoyancy_g = 150.0 # grams (-1000 to +1000g)
        self.sim_drag_scale = 1.0       # 0.2x to 5.0x

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
                imu_ready = len(parts) >= 19 and parts[16] == "1"
                pressure_ready = len(parts) >= 19 and parts[17] == "1"
                i2c_ready = len(parts) >= 19 and parts[18] == "1"

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
                    self.sensors["imu_ready"] = imu_ready
                    self.sensors["pressure_ready"] = pressure_ready
                    self.sensors["i2c_ready"] = i2c_ready
                    self.pulses = t_pulses
                    self.armed = armed
                    self.fsm_state = fsm_state
                    self.last_packet_time = now
                    self.packets_rx += 1
                    self._hz_counter += 1
                    hil_lost = (
                        self.hil_active
                        and not armed
                        and now - self.hil_armed_at > HIL_WATCHDOG_MS / 1000.0
                    )
                    if hil_lost:
                        self.hil_active = False
                if hil_lost:
                    self.add_log("[HIL] Firmware reports disarmed; actuator output disabled.")
            except (ValueError, IndexError):
                pass

    def _parse_act_packet(self, packet: str):
        parts = packet.strip().split(',')
        if len(parts) != 7 or parts[0] != "$ACT":
            return
        try:
            pulses = [int(value) for value in parts[1:]]
            if any(pulse < 1000 or pulse > 2000 for pulse in pulses):
                return
        except ValueError:
            return
        with self.state_lock:
            self.pulses = pulses
            self.packets_rx += 1

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

            with self.state_lock:
                telemetry_stale = (
                    self.connected
                    and self.last_packet_time > 0.0
                    and now - self.last_packet_time > 0.5
                )
            if telemetry_stale:
                self.add_log("[SAFETY] Telemetry stale >500 ms; activating fail-safe neutral.")
                with self.state_lock:
                    self.hil_active = False
                    self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
                    self.sensors["imu_ready"] = False
                    self.sensors["pressure_ready"] = False
                    self.sensors["i2c_ready"] = False
                try:
                    with self.serial_io_lock:
                        if self.ser and self.ser.is_open:
                            self.ser.write(b"stop\r\n")
                            self.ser.flush()
                except Exception:
                    pass

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
                while self.ser:
                    with self.serial_io_lock:
                        if not self.ser or not self.ser.in_waiting:
                            break
                        raw = self.ser.readline().decode('utf-8', errors='replace')
                    has_read = True
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

                    if cleaned.startswith('$ACT,'):
                        self._parse_act_packet(cleaned)
                        continue
                    if cleaned.startswith('$HIL,ERR,'):
                        with self.state_lock:
                            self.hil_active = False
                        self.add_log(f"[HIL] Firmware rejected actuator packet: {cleaned}")
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
                    with self.serial_io_lock:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.connected = False
                with self.state_lock:
                    was_hil_active = self.hil_active
                    self.hil_active = False
                if was_hil_active:
                    self.add_log("[HIL] Serial connection lost; firmware watchdog must neutralize thrusters.")
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

    def stop_sim_mission(self, completed: bool = False):
        with self.state_lock:
            self.sim_mission_active = False
            self.sim_mission_completed = completed
            self.pulses = [1500, 1500, 1500, 1500, 1500, 1500]
        state = "SELESAI" if completed else "DIHENTIKAN"
        self.add_log(f"[SIMULASI] Simulasi Jalur Misi Virtual {state}!")

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
                        self.pulses = [1500, 1500, 1500, 1500] + vertical_pulse_pair(p_vert)
                    elif aksi in ["MAJU", "FORWARD"]:
                        err_z = target_z - self.sim_depth
                        p_vert = 1500 + int(math.copysign(min(15.0 * 5.0, max(10.0, abs(err_z) * 3.0)), err_z)) if target_z > 0 else 1500
                        p_fwd = 1500 + int(daya * 5.0)
                        self.pulses = [p_fwd, p_fwd, p_fwd, p_fwd] + vertical_pulse_pair(p_vert)
                    elif aksi in ["MUNDUR", "BACKWARD"]:
                        p_bwd = 1500 - int(daya * 5.0)
                        self.pulses = [p_bwd, p_bwd, p_bwd, p_bwd, 1500, 1500]
                    elif aksi in ["PUTAR", "TURN"]:
                        p_turn = int(daya * 5.0)
                        self.pulses = [1500 - p_turn, 1500 + p_turn, 1500 - p_turn, 1500 + p_turn, 1500, 1500]
                    elif aksi in ["SURFACE", "NAIK"]:
                        self.pulses = [1500, 1500, 1500, 1500] + vertical_pulse_pair(1470)
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
            p_vert_avg = ((self.pulses[4] - 1500.0) + (1500.0 - self.pulses[5])) / 2.0
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
            self.sensors["imu_ready"] = True
            self.sensors["pressure_ready"] = True
            self.sensors["i2c_ready"] = True
            self.rate_hz = 12.0

    def get_hil_status(self) -> dict:
        with self.state_lock:
            return {
                "active": self.hil_active,
                "max_delta_us": HIL_MAX_DELTA_US,
                "watchdog_ms": HIL_WATCHDOG_MS,
                "last_telemetry_age_ms": (
                    max(0, int((time.time() - self.last_tel_time) * 1000))
                    if self.last_tel_time > 0.0 else None
                )
            }

    def arm_hil(self, safe_to_test: bool) -> dict:
        if safe_to_test is not True:
            return {"success": False, "error": "Safety confirmation is required before arming HIL."}

        with self.hil_lock:
            with self.state_lock:
                telemetry_fresh = self.last_tel_time > 0.0 and time.time() - self.last_tel_time <= 0.5
                if not self.connected or self.ser is None or not self.ser.is_open:
                    return {"success": False, "error": "Pico W is not connected."}
                if not telemetry_fresh:
                    return {"success": False, "error": "Fresh hardware telemetry is required to arm HIL."}
                if self.armed or self.fsm_state != 0:
                    return {"success": False, "error": "HIL requires hardware to be idle and disarmed."}

            with self.serial_transaction_lock:
                result = self._execute_serial_command("hil on", timeout=1.5)
            if not result.get("success"):
                return result

            with self.state_lock:
                self.hil_active = True
                self.hil_armed_at = time.time()
            neutral_result = self.send_hil_frame(
                [1500] * 6,
                {"depth_cm": 0.0, "pitch": 0.0, "roll": 0.0, "gyro_dps": [0.0, 0.0, 0.0]}
            )
            if not neutral_result.get("success"):
                self.disarm_hil()
                return neutral_result
            return {
                "success": True,
                "reply": result.get("reply", "HIL armed."),
                "hil": self.get_hil_status()
            }

    def send_hil_frame(self, pulses, sensors) -> dict:
        validated = validate_hil_pulses(pulses)
        validated_sensors = validate_hil_sensors(sensors)
        with self.hil_lock:
            with self.state_lock:
                if not self.hil_active:
                    return {"success": False, "error": "HIL is not armed."}
                if not self.connected or self.ser is None or not self.ser.is_open:
                    return {"success": False, "error": "Pico W is not connected."}
                if self.last_tel_time <= 0.0 or time.time() - self.last_tel_time > 0.5:
                    self.hil_active = False
                    return {"success": False, "error": "Hardware telemetry is stale; HIL output stopped."}
                ser = self.ser

            sensor_values = [
                validated_sensors["depth_cm"],
                validated_sensors["pitch"],
                validated_sensors["roll"],
                *validated_sensors["gyro_dps"],
            ]
            inj_packet = "$INJ," + ",".join(f"{value:.2f}" for value in sensor_values) + "\r\n"
            hil_packet = "$HIL," + ",".join(str(pulse) for pulse in validated) + "\r\n"
            packet = (inj_packet + hil_packet).encode("ascii")
            try:
                serial_lost = False
                with self.serial_io_lock:
                    if self.ser is not ser or not ser.is_open:
                        serial_lost = True
                    else:
                        ser.write(packet)
                        ser.flush()
                if serial_lost:
                    with self.state_lock:
                        self.hil_active = False
                    return {"success": False, "error": "Pico W serial connection was lost."}
                self.packets_tx += 2
                return {"success": True, "pulses": validated}
            except OSError as e:
                with self.state_lock:
                    self.hil_active = False
                return {"success": False, "error": f"Gagal mengirim output HIL: {e}"}

    def disarm_hil(self) -> dict:
        with self.hil_lock:
            with self.state_lock:
                self.hil_active = False
                connected = self.connected and self.ser is not None and self.ser.is_open
            if not connected:
                return {
                    "success": False,
                    "error": "Pico W disconnected; firmware watchdog will neutralize outputs."
                }
            with self.serial_transaction_lock:
                return self._execute_serial_command("hil off", timeout=1.5)

    def execute_command(self, cmd: str, timeout: float = 0.85) -> dict:
        """Sends command synchronously and waits for real hardware confirmation."""
        cmd = cmd.strip()
        if not cmd:
            return {"success": False, "error": "Perintah kosong"}

        root = cmd.lower().split()[0] if cmd.split() else ""
        if root == "hil" or cmd.lower().startswith("$hil"):
            return {
                "success": False,
                "error": "Gunakan kontrol HIL khusus agar konfirmasi keselamatan dan interlock tetap aktif."
            }
        if root in ["stop", "disarm", "abort"]:
            with self.state_lock:
                self.hil_active = False
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

        with self.serial_transaction_lock:
            return self._execute_serial_command(cmd, timeout)

    def _execute_serial_command(self, cmd: str, timeout: float) -> dict:
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
                with self.serial_io_lock:
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
            if any(re.search(r'(^|\s)(ERROR:|\[ERROR\])', line, re.IGNORECASE) for line in lines):
                return {"success": False, "error": reply, "command": cmd}
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
        if not isinstance(steps_data, list) or not steps_data:
            return {"success": False, "error": "Data jalur harus berupa array JSON tidak kosong"}
        if len(steps_data) > MAX_MISSION_STEPS:
            return {"success": False, "error": f"Misi maksimal {MAX_MISSION_STEPS} langkah"}

        validated_steps = []
        for idx, s in enumerate(steps_data):
            try:
                if not isinstance(s, dict):
                    raise ValueError("langkah harus berupa objek")
                raw_step = s.get("langkah", s.get("step", idx + 1))
                langkah_value = finite_number(raw_step, "nomor langkah")
                if not langkah_value.is_integer() or not 1 <= langkah_value <= 65535:
                    raise ValueError("nomor langkah harus bilangan bulat 1-65535")
                langkah = int(langkah_value)
                nama = s.get("nama", s.get("name", f"Langkah {idx+1}"))
                if not isinstance(nama, str):
                    raise ValueError("nama langkah harus berupa teks")
                nama = nama.strip()
                if not nama or len(nama.encode("utf-8")) >= 36:
                    raise ValueError("nama langkah harus berisi 1-35 byte UTF-8")
                aksi = s.get("aksi", s.get("action", ""))
                if not isinstance(aksi, str):
                    raise ValueError("aksi harus berupa teks")
                aksi = aksi.strip().upper()
                if aksi not in VALID_MISSION_ACTIONS:
                    raise ValueError(f"aksi tidak didukung: {aksi or '(kosong)'}")
                daya = finite_number(s.get("daya", s.get("power", 15)), "daya")
                durasi = finite_number(s.get("durasi_detik", s.get("duration", s.get("durasi", s.get("duration_sec", 5.0)))), "durasi")
                target = finite_number(s.get("target_cm", s.get("target", s.get("depth_cm", 0.0))), "target")
                if not 0.0 <= daya <= 100.0:
                    raise ValueError("daya harus 0-100%")
                if not 0.1 <= durasi <= 600.0:
                    raise ValueError("durasi harus 0.1-600 detik")
                if not 0.0 <= target <= 140.0:
                    raise ValueError("target harus 0-140 cm")

                raw_motors = s.get("motors")
                if raw_motors is not None:
                    if not isinstance(raw_motors, list) or not raw_motors:
                        raise ValueError("motors harus berupa array ID 1-6 yang tidak kosong")
                    parsed_motors = []
                    for motor in raw_motors:
                        motor_value = finite_number(motor, "ID motor")
                        if not motor_value.is_integer() or not 1 <= motor_value <= 6:
                            raise ValueError("ID motor harus bilangan bulat 1-6")
                        parsed_motors.append(int(motor_value))
                    motors = sorted(set(parsed_motors))
                elif "motor_mask" in s:
                    mask_value = finite_number(s["motor_mask"], "mask motor")
                    if not mask_value.is_integer() or not 1 <= mask_value <= 63:
                        raise ValueError("mask motor harus bilangan bulat 1-63")
                    motors = [motor_id for motor_id in range(1, 7) if int(mask_value) & (1 << (motor_id - 1))]
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

        json_str = json.dumps(validated_steps, separators=(",", ":"), ensure_ascii=False)
        if len(json_str.encode("utf-8")) > MAX_MISSION_BYTES:
            return {"success": False, "error": f"Payload misi melebihi batas {MAX_MISSION_BYTES} byte"}
        saved_paths = []
        for p in [os.path.join(HERMES_DIR, "jalur.json"), os.path.join(BASE_DIR, "jalur.json"), "D:/Hermes/jalur.json"]:
            try:
                with open(p, "w", encoding="utf-8") as f:
                    f.write(json_str)
                saved_paths.append(p)
            except OSError as ex:
                self.add_log(f"[MISSION] Tidak dapat menyimpan {p}: {ex}")

        if not saved_paths:
            return {"success": False, "error": "Tidak dapat menyimpan salinan lokal misi"}

        if not self.connected or self.ser is None:
            with self.state_lock:
                self.steps_list = validated_steps
                self.total_steps = len(validated_steps)
            return {
                "success": False,
                "error": f"Robot TERPUTUS! Gagal mengunggah ke /jalur.json Pico W (Data hanya tersimpan lokal di laptop).",
                "count": len(validated_steps),
                "offline": True
            }

        # 2. Upload to Pico W via serial protocol
        with self.serial_transaction_lock:
            try:
                with self.serial_io_lock:
                    self.ser.reset_input_buffer()
                    self.ser.write(b"jalur upload\r\n")
                    self.ser.flush()

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

                    self.ser.write(json_str.encode("utf-8") + b"\r\n===EOF===\r\n")
                    self.ser.flush()

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
                    with self.state_lock:
                        self.steps_list = validated_steps
                        self.total_steps = len(validated_steps)
                    return {
                        "success": True,
                        "count": len(validated_steps),
                        "reply": reply_msg or "Misi /jalur.json berhasil diverifikasi & disimpan ke LittleFS!"
                    }
                else:
                    return {"success": False, "error": "Timeout konfirmasi penyimpanan LittleFS Pico W"}

            except (OSError, ValueError, UnicodeError) as e:
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
            sensors = copy.deepcopy(self.sensors)
            sensor_age_ms = None
            if self.connected and self.last_packet_time > 0.0:
                sensor_age_ms = max(0, int((time.time() - self.last_packet_time) * 1000))
            if self.connected and (sensor_age_ms is None or sensor_age_ms > 500):
                sensors["imu_ready"] = False
                sensors["pressure_ready"] = False
                sensors["i2c_ready"] = False
            telemetry_health = (
                "hardware_live" if self.connected and sensor_age_ms is not None and sensor_age_ms <= 500
                else ("hardware_stale" if self.connected else "simulation_offline")
            )
            return {
                "connected": self.connected,
                "port": self.port,
                "baud": self.baud,
                "rate_hz": round(self.rate_hz, 1),
                "telemetry_health": telemetry_health,
                "watchdog_ms": 500,
                "state_stale": not self.connected or sensor_age_ms is None or sensor_age_ms > 500,
                "firmware": {
                    "config_version": FIRMWARE_CONFIG_VERSION,
                    "protocol_version": FIRMWARE_PROTOCOL_VERSION,
                    "compatibility": "expected",
                    "board": "RP2040 Pico W",
                },
                "packets_rx": self.packets_rx,
                "packets_tx": self.packets_tx,
                "sensors": sensors,
                "sensor_source": (
                    "hil_virtual_sensors" if self.hil_active
                    else ("hardware" if self.connected else "simulation")
                ),
                "sensor_age_ms": sensor_age_ms,
                "pulses": list(self.pulses),
                "deltas": deltas,
                "armed": self.armed,
                "fsm_state": self.fsm_state,
                "hil": self.get_hil_status(),
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
        with self.state_lock:
            self.hil_active = False
        if self.ser and self.ser.is_open:
            try:
                # Stop streaming and neutralize thrusters before closing
                with self.serial_io_lock:
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
wifi_config_client = PicoWifiConfigClient()
wifi_firmware_update_lock = threading.Lock()


def relay_wifi_firmware_update(image, password, filename):
    if not wifi_firmware_update_lock.acquire(blocking=False):
        raise PicoFirmwareUpdateError(
            "Another firmware update is already in progress", status_code=409
        )
    try:
        device_response = wifi_config_client.request({"op": "get"})
        settings = device_response.get("settings")
        status = device_response.get("status")
        autorun = settings.get("autorun") if isinstance(settings, dict) else None
        if (
            device_response.get("ok") is not True
            or not isinstance(status, dict)
            or status.get("armed") is not False
            or type(status.get("fsm_state")) is not int
            or status.get("fsm_state") != 0
            or not isinstance(autorun, dict)
            or autorun.get("enabled") is not False
        ):
            raise PicoFirmwareUpdateError(
                "OTA requires a confirmed idle, disarmed Pico W with Auto-Run disabled.",
                status_code=409,
            )
        return upload_firmware_via_wifi(image, password, filename)
    finally:
        wifi_firmware_update_lock.release()


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
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _local_request_allowed(self) -> bool:
        host = self.headers.get('Host', '').lower()
        if host not in {f'localhost:{PORT}', f'127.0.0.1:{PORT}'}:
            return False
        origin = self.headers.get('Origin')
        return origin is None or origin.lower() == f'http://{host}'

    def do_OPTIONS(self):
        self.send_response(405)
        self.send_header('Allow', 'GET, POST')
        self.end_headers()

    def do_GET(self):
        if not self._local_request_allowed():
            self.send_error(403, "Local requests only")
            return
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
        elif path == '/api/hil/status':
            self.send_json(bridge.get_hil_status())
            return
        elif path == '/api/rl_pid/status':
            self.send_json(rl_agent.get_status() if rl_agent else {"status": "DISABLED", "enabled": False})
            return
        elif path == '/api/logs':
            self.send_json({"logs": bridge.recent_logs})
            return
        elif path == '/api/config/export':
            backup = {
                "format": "auv-config-backup",
                "config_version": FIRMWARE_CONFIG_VERSION,
                "protocol_version": FIRMWARE_PROTOCOL_VERSION,
                "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "config": copy.deepcopy(bridge.config),
            }
            os.makedirs(CONFIG_BACKUP_DIR, exist_ok=True)
            filename = f"auv-config-{time.strftime('%Y%m%d-%H%M%S')}.json"
            backup_path = os.path.join(CONFIG_BACKUP_DIR, filename)
            with open(backup_path, "w", encoding="utf-8") as backup_file:
                json.dump(backup, backup_file, ensure_ascii=False, indent=2)
            self.send_json({
                "success": True,
                "filename": filename,
                "backup": backup,
            })
            return
        elif path == '/api/wifi/status':
            try:
                response = wifi_config_client.request({"op": "get"})
            except WifiConfigError as ex:
                self.send_json({
                    "success": False,
                    "error": str(ex),
                    "outcome_unknown": False,
                }, code=504)
                return
            self.send_json({
                "success": response.get("ok") is True,
                "response": response,
                "error": response.get("error"),
            }, code=200 if response.get("ok") is True else 409)
            return

        if path == '/config' or path == '/config.html':
            self.send_response(302)
            self.send_header('Location', '/#konfigurasi')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            return

        # Serve index.html by default
        if path == '/' or path == '/index.html':
            self.path = '/index.html'

        return super().do_GET()

    def do_POST(self):
        if not self._local_request_allowed():
            self.send_json({"success": False, "error": "Local same-origin requests only"}, code=403)
            return
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path == '/api/wifi/firmware':
            self._handle_firmware_upload()
            return
        if self.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/json':
            self.send_json({"success": False, "error": "Content-Type must be application/json"}, code=415)
            return
        try:
            length = int(self.headers.get('Content-Length', ''))
        except ValueError:
            self.send_json({"success": False, "error": "Valid Content-Length is required"}, code=411)
            return
        if length < 1:
            self.send_json({"success": False, "error": "Request body is required"}, code=400)
            return
        if length > 65536:
            self.send_json({"success": False, "error": "Request body exceeds 65536 bytes"}, code=413)
            return
        try:
            body = json.loads(self.rfile.read(length).decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.send_json({"success": False, "error": "Request body must be valid UTF-8 JSON"}, code=400)
            return
        if path == '/api/mission_upload':
            if not isinstance(body, (dict, list)):
                self.send_json({"success": False, "error": "Mission upload must be an object or array"}, code=400)
                return
        elif not isinstance(body, dict):
            self.send_json({"success": False, "error": "Request body must be a JSON object"}, code=400)
            return

        try:
            self._handle_post(path, body)
        except (ValueError, TypeError, OverflowError, AttributeError) as ex:
            self.send_json({"success": False, "error": str(ex) or "Invalid request value"}, code=400)

    def _handle_firmware_upload(self):
        if self.headers.get('Content-Type', '').split(';', 1)[0].strip().lower() != 'application/octet-stream':
            self.send_json({"success": False, "error": "Content-Type must be application/octet-stream"}, code=415)
            return
        try:
            length = int(self.headers.get('Content-Length', ''))
        except ValueError:
            self.send_json({"success": False, "error": "Valid Content-Length is required"}, code=411)
            return
        if not 1 <= length <= MAX_FIRMWARE_IMAGE_BYTES:
            self.send_json({
                "success": False,
                "error": f"Firmware image must contain 1-{MAX_FIRMWARE_IMAGE_BYTES} bytes",
            }, code=413 if length > MAX_FIRMWARE_IMAGE_BYTES else 400)
            return

        image = self.rfile.read(length)
        if len(image) != length:
            self.send_json({"success": False, "error": "Firmware upload ended before the full image was received"}, code=400)
            return

        password = self.headers.get('X-Pico-Update-Password', '')
        filename = self.headers.get('X-Firmware-Filename', '')
        try:
            result = relay_wifi_firmware_update(image, password, filename)
        except WifiConfigError as ex:
            self.send_json({
                "success": False,
                "error": str(ex),
                "outcome_unknown": ex.outcome_unknown,
            }, code=504)
            return
        except ValueError as ex:
            self.send_json({"success": False, "error": str(ex)}, code=400)
            return
        except PicoFirmwareUpdateError as ex:
            self.send_json({
                "success": False,
                "error": str(ex),
                "outcome_unknown": ex.outcome_unknown,
            }, code=ex.status_code)
            return
        self.send_json({"success": True, **result})

    def _handle_post(self, path: str, body):
        if path == '/api/config/import':
            config = validate_config_backup(body)
            backup_version = body.get("config_version")
            if backup_version is not None and backup_version != FIRMWARE_CONFIG_VERSION:
                raise ValueError(
                    f"Versi konfigurasi backup {backup_version} tidak cocok dengan firmware config v{FIRMWARE_CONFIG_VERSION}"
                )
            if not bridge.connected:
                bridge.add_log("[CONFIG-AUDIT] Import rejected: serial disconnected")
                self.send_json({"success": False, "offline": True, "error": "Serial hardware terputus; backup tidak diterapkan."}, code=409)
                return

            commands = []
            if "power_limit" in config:
                power_limit = finite_number(config["power_limit"], "power_limit")
                if not power_limit.is_integer() or not 1 <= power_limit <= 100:
                    raise ValueError("power_limit backup must be an integer from 1 to 100")
                commands.append(f"limit {int(power_limit)}")
            if "esc" in config and "deadband" in config["esc"]:
                deadband = finite_number(config["esc"]["deadband"], "deadband")
                if not deadband.is_integer() or not 0 <= deadband <= 80:
                    raise ValueError("deadband backup must be an integer from 0 to 80")
                commands.append(f"esc deadband {int(deadband)}")
            if "balance" in config:
                horiz = float(config["balance"].get("horiz", 0.0))
                vert = float(config["balance"].get("vert", 0.0))
                if not -20 <= horiz <= 20 or not -20 <= vert <= 20:
                    raise ValueError("balance backup values must be from -20 to 20")
                commands.append(f"motor balance horiz {horiz:+.1f}")
                commands.append(f"motor balance vert {vert:+.1f}")
            for axis in ("depth", "roll"):
                gains = config.get("pid", {}).get(axis)
                if gains:
                    values = [finite_number(gains.get(key, 0.0), f"{axis} {key}") for key in ("kp", "ki", "kd")]
                    if any(value < 0 or value > 100 for value in values):
                        raise ValueError("PID backup gains must be from 0 to 100")
                    commands.append(
                        f"pid {axis} {values[0]:.4f} {values[1]:.4f} {values[2]:.4f}"
                    )

            results = []
            for command in commands:
                result = bridge.execute_command(command)
                results.append(result)
                if not result.get("success"):
                    bridge.add_log(f"[CONFIG-AUDIT] Import stopped: {command} rejected")
                    self.send_json({
                        "success": False,
                        "error": f"Import berhenti pada perintah: {command}",
                        "results": results,
                    }, code=409)
                    return
            save_result = bridge.execute_command("save")
            results.append(save_result)
            success = save_result.get("success", False)
            bridge.add_log(
                "[CONFIG-AUDIT] Import confirmed and saved"
                if success else "[CONFIG-AUDIT] Import applied but save rejected"
            )
            os.makedirs(CONFIG_BACKUP_DIR, exist_ok=True)
            imported_path = os.path.join(
                CONFIG_BACKUP_DIR,
                f"imported-{time.strftime('%Y%m%d-%H%M%S')}.json",
            )
            with open(imported_path, "w", encoding="utf-8") as backup_file:
                json.dump(body, backup_file, ensure_ascii=False, indent=2)
            self.send_json({
                "success": success,
                "results": results,
                "error": None if success else "Konfigurasi diterapkan tetapi SAVE EEPROM gagal",
            }, code=200 if success else 409)
            return

        if path == '/api/wifi/config':
            operation = body.get('op')
            if operation not in {
                'set_pid', 'set_thruster', 'set_power_limit', 'set_deadband',
                'set_balance', 'set_autorun_timing', 'apply', 'save', 'discard',
                'command', 'mission_upload'
            }:
                raise ValueError("A state-changing Pico Wi-Fi configuration operation is required")
            try:
                response = wifi_config_client.request(body)
            except WifiConfigError as ex:
                bridge.add_log(f"[CONFIG-AUDIT] Wi-Fi {operation} failed: {ex}")
                self.send_json({
                    "success": False,
                    "error": str(ex),
                    "outcome_unknown": ex.outcome_unknown,
                }, code=504)
                return
            if response.get("ok") is True:
                bridge.add_log(f"[CONFIG-AUDIT] Wi-Fi {operation} confirmed")
            else:
                bridge.add_log(
                    f"[CONFIG-AUDIT] Wi-Fi {operation} rejected: {response.get('error') or 'unknown error'}"
                )
            self.send_json({
                "success": response.get("ok") is True,
                "reply": response.get("error") or response.get("op"),
                "response": response,
                "error": response.get("error"),
            }, code=200 if response.get("ok") is True else 409)
            return

        if path == '/api/hil/arm':
            safe_to_test = body.get('safe_to_test')
            if not isinstance(safe_to_test, bool):
                raise ValueError("safe_to_test must be a boolean")
            result = bridge.arm_hil(safe_to_test)
            self.send_json(result, code=200 if result.get("success") else 409)
            return

        elif path == '/api/hil/frame':
            result = bridge.send_hil_frame(body.get('pulses'), body.get('sensors'))
            self.send_json(result, code=200 if result.get("success") else 409)
            return

        elif path == '/api/hil/stop':
            result = bridge.disarm_hil()
            self.send_json(result, code=200 if result.get("success") else 409)
            return

        elif path == '/api/command':
            cmd = body.get('command', '').strip()
            allowed_prefixes = (
                "status", "stat", "help", "thrusters", "sensor", "telem ",
                "stop", "limit ", "pid ", "thruster ", "esc ", "motor ",
                "balance ", "autorun ", "calib ", "tare ", "save", "discard",
                "run", "mission ", "hil "
            )
            if not cmd or not cmd.lower().startswith(allowed_prefixes):
                bridge.add_log("[CRITICAL-AUDIT] Terminal command rejected by whitelist")
                self.send_json({
                    "success": False,
                    "error": "Perintah ditolak: hanya prefix CLI aman yang diizinkan.",
                }, code=400)
                return
            if any(marker in cmd.lower() for marker in ("\n", "\r", ";", "&&", "|", "`")):
                raise ValueError("Terminal command must be one line")
            res = bridge.execute_command(cmd)
            bridge.add_log(
                f"[CRITICAL-AUDIT] Terminal command {'confirmed' if res.get('success') else 'rejected'}: {cmd}"
            )
            self.send_json(res)
            return

        elif path == '/api/tare_compass' or path == '/api/tare_yaw':
            with bridge.state_lock:
                bridge.hardware_yaw = 0.0
                bridge.sim_yaw = 0.0
                if len(bridge.sensors.get("gyro", [])) >= 3:
                    bridge.gz_bias = bridge.sensors["gyro"][2]
                if not bridge.connected:
                    bridge.sensors["yaw"] = 0.0
            bridge.add_log("[KOMPAS] Referensi heading lokal/server diatur ke 0°.")
            self.send_json({
                "success": True,
                "reply": "Referensi heading lokal/server diatur ke 0°. Kalibrasi sensor firmware tidak diubah.",
                "yaw": 0.0
            })
            return

        elif path == '/api/emergency_stop':
            bridge.add_log("[CRITICAL-AUDIT] Emergency stop requested")
            if bridge.get_hil_status().get("active"):
                res = bridge.disarm_hil()
                if not res.get("success"):
                    fallback = bridge.execute_command('stop')
                    if fallback.get("success"):
                        res = fallback
            else:
                res = bridge.execute_command('stop')
            self.send_json(res)
            return

        elif path == '/api/test_motor':
            if not bridge.connected:
                bridge.add_log("[CRITICAL-AUDIT] Motor test rejected: serial disconnected")
                self.send_json({"success": False, "offline": True, "error": "Motor test ditolak: serial terputus."}, code=409)
                return
            if bridge.armed or bridge.fsm_state != 0:
                bridge.add_log("[CRITICAL-AUDIT] Motor test rejected: device armed or not idle")
                self.send_json({"success": False, "error": "Motor test ditolak: perangkat harus disarmed dan idle."}, code=409)
                return
            motor_id = body.get('id', 1)
            p_val = finite_number(body.get('power', 15), "power")
            if not -100.0 <= p_val <= 100.0:
                raise ValueError("power must be between -100 and 100")

            if str(motor_id).lower() in ['all', '0']:
                cmd = f"tes all {p_val:.0f}"
            else:
                m_value = finite_number(motor_id, "motor ID")
                if not m_value.is_integer() or not 1 <= m_value <= 6:
                    raise ValueError("motor ID must be 1-6 or 'all'")
                m_int = int(m_value)
                cmd = f"tes {m_int} {p_val:.0f}"

            res = bridge.execute_command(cmd)
            bridge.add_log(
                f"[CRITICAL-AUDIT] Motor test {'confirmed' if res.get('success') else 'rejected'}: {cmd}"
            )
            self.send_json(res)
            return

        elif path == '/api/mission_action':
            action = body.get('action', '').strip().lower()
            if action in ('run', 'start', 'run_now', 'now', 'instant'):
                if not bridge.connected:
                    bridge.add_log("[CRITICAL-AUDIT] Mission start rejected: serial disconnected")
                    self.send_json({"success": False, "offline": True, "error": "Misi ditolak: serial terputus."}, code=409)
                    return
                if bridge.armed or bridge.fsm_state != 0:
                    bridge.add_log("[CRITICAL-AUDIT] Mission start rejected: device armed or not idle")
                    self.send_json({"success": False, "error": "Misi ditolak: perangkat harus disarmed dan idle."}, code=409)
                    return
            if action in ['run', 'start']:
                cmd = "run"
            elif action in ['run_now', 'now', 'instant']:
                cmd = "run now"
            elif action in ['stop', 'abort', 'cancel']:
                cmd = "stop"
            else:
                raise ValueError("action must be start, run_now, or stop")
            res = bridge.execute_command(cmd)
            res["mission"] = bridge.get_mission_status()
            bridge.add_log(
                f"[CRITICAL-AUDIT] Mission {action} {'confirmed' if res.get('success') else 'rejected'}"
            )
            self.send_json(res)
            return

        elif path == '/api/mission_upload':
            steps = body.get('steps', []) if isinstance(body, dict) else body
            if not bridge.connected:
                bridge.add_log("[CRITICAL-AUDIT] Mission upload rejected: serial disconnected")
                self.send_json({"success": False, "offline": True, "error": "Upload misi ditolak: serial terputus."}, code=409)
                return
            res = bridge.upload_mission_steps(steps)
            bridge.add_log(
                f"[CRITICAL-AUDIT] Mission upload {'confirmed' if res.get('success') else 'rejected'}"
            )
            self.send_json(res)
            return

        elif path == '/api/param_update':
            p_type = body.get('type', '').strip().lower()
            cmd = None

            if p_type == 'limit':
                value = finite_number(body.get('value', 25), "limit")
                if not value.is_integer() or not 1 <= value <= 100:
                    raise ValueError("limit must be an integer from 1 to 100")
                val = int(value)
                cmd = f"limit {val}"

            elif p_type == 'pid':
                axis = str(body.get('axis', 'depth')).lower()
                if axis not in {'depth', 'roll'}:
                    raise ValueError("PID axis must be depth or roll; pitch is unsupported")
                kp = finite_number(body.get('kp', 0.6), "kp")
                ki = finite_number(body.get('ki', 0.0), "ki")
                kd = finite_number(body.get('kd', 0.015), "kd")
                if any(gain < 0.0 or gain > 100.0 for gain in (kp, ki, kd)):
                    raise ValueError("PID gains must be between 0 and 100")
                cmd = f"pid {axis} {kp:.4f} {ki:.4f} {kd:.4f}"

            elif p_type == 'pid_state':
                en = body.get('enabled', True)
                if not isinstance(en, bool):
                    raise ValueError("enabled must be a boolean")
                cmd = "pid on" if en else "pid off"

            elif p_type == 'thruster':
                t_value = finite_number(body.get('id', 1), "thruster ID")
                if not t_value.is_integer() or not 1 <= t_value <= 6:
                    raise ValueError("thruster ID must be an integer from 1 to 6")
                t_id = int(t_value)
                prop = str(body.get('prop', 'reverse')).lower()
                val = body.get('value')
                if prop == 'reverse':
                    if val in [1, "1", True, "on", "yes"]:
                        v_str = "1"
                    elif val in [0, "0", False, "off", "no"]:
                        v_str = "0"
                    else:
                        raise ValueError("reverse must be a boolean or 0/1")
                elif prop == 'trim':
                    trim = finite_number(val, "trim")
                    if not 0.1 <= trim <= 2.5:
                        raise ValueError("trim must be between 0.1 and 2.5")
                    v_str = f"{trim:.2f}"
                elif prop in {'min', 'neutral', 'max'}:
                    pulse = finite_number(val, "pulse")
                    limits = {'min': (800, 1400), 'neutral': (1400, 1600), 'max': (1600, 2200)}
                    low, high = limits[prop]
                    if not pulse.is_integer() or not low <= pulse <= high:
                        raise ValueError(f"{prop} pulse must be an integer from {low} to {high}")
                    v_str = str(int(pulse))
                else:
                    raise ValueError("thruster prop must be min, neutral, max, reverse, or trim")
                cmd = f"thruster {t_id} {prop} {v_str}"

            elif p_type == 'balance':
                axis = str(body.get('axis', 'horiz')).lower()
                pct = finite_number(body.get('value', 0.0), "balance")
                if axis not in {'horiz', 'vert'} or not -20.0 <= pct <= 20.0:
                    raise ValueError("balance axis must be horiz/vert and value -20 to 20")
                cmd = f"motor balance {axis} {pct:+.1f}"

            elif p_type == 'esc_deadband':
                value = finite_number(body.get('value', 30), "deadband")
                if not value.is_integer() or not 0 <= value <= 80:
                    raise ValueError("deadband must be an integer from 0 to 80")
                val = int(value)
                cmd = f"esc deadband {val}"

            elif p_type == 'esc_calib':
                sub = str(body.get('sub', 'start')).lower()
                if sub not in {'start', 'confirm', 'max', 'min', 'neutral', 'stop'}:
                    raise ValueError("unsupported ESC calibration operation")
                cmd = f"esc calib {sub}"

            elif p_type == 'autorun_timing':
                delay_value = finite_number(body.get('delay', 10), "delay")
                tare_value = finite_number(body.get('tare', 3), "tare")
                level_value = finite_number(body.get('level', 6), "level")
                if (not delay_value.is_integer() or not tare_value.is_integer() or
                        not level_value.is_integer() or not 3 <= delay_value <= 60 or
                        not 1 <= tare_value <= delay_value or
                        not 1 <= level_value <= delay_value):
                    raise ValueError("autorun timing must use integer delay 3-60 and tare/level 1-delay")
                delay, tare, lvl = int(delay_value), int(tare_value), int(level_value)
                cmd = f"autorun timing {delay} {tare} {lvl}"

            elif p_type == 'autorun':
                en = body.get('enabled', False)
                if not isinstance(en, bool):
                    raise ValueError("enabled must be a boolean")
                cmd = "autorun on" if en else "autorun off"

            elif p_type == 'calib':
                target = str(body.get('target', 'zero')).lower()
                if target == 'zero':
                    mbar = body.get('mbar')
                    if mbar is None:
                        cmd = "calib zero"
                    else:
                        pressure = finite_number(mbar, "pressure")
                        if not 500 <= pressure <= 2000:
                            raise ValueError("pressure must be 500-2000 mbar")
                        cmd = f"calib zero {pressure:.1f}"
                elif target == 'level':
                    cmd = "calib level"
                else:
                    raise ValueError("calibration target must be zero or level")

            elif p_type == 'save':
                cmd = "save"

            elif p_type == 'default':
                cmd = "default"

            if cmd:
                res = bridge.execute_command(cmd)
                if res.get("success"):
                    bridge.add_log(f"[CONFIG-AUDIT] Serial {p_type} confirmed: {cmd}")
                else:
                    bridge.add_log(
                        f"[CONFIG-AUDIT] Serial {p_type} rejected: {res.get('error') or res.get('reply') or 'unknown error'}"
                    )
                self.send_json(res)
            else:
                self.send_json({"success": False, "error": f"Tipe parameter tidak dikenal: {p_type}"}, code=400)
            return

        elif path == '/api/control':
            updates = {}
            limits = {
                'depth': (0.0, 140.0), 'x': (0.0, 1200.0),
                'pitch': (-45.0, 45.0), 'roll': (-45.0, 45.0)
            }
            for key, (low, high) in limits.items():
                if key in body and body[key] is not None:
                    value = finite_number(body[key], key)
                    if not low <= value <= high:
                        raise ValueError(f"{key} must be between {low} and {high}")
                    updates[key] = value
            if 'yaw' in body and body['yaw'] is not None:
                updates['yaw'] = finite_number(body['yaw'], 'yaw') % 360.0
            if 'wave' in body:
                if not isinstance(body['wave'], bool):
                    raise ValueError("wave must be a boolean")
                updates['wave'] = body['wave']
            if 'pulses' in body:
                pulses = body['pulses']
                if not isinstance(pulses, list) or len(pulses) != 6:
                    raise ValueError("pulses must contain exactly six values")
                parsed_pulses = []
                for pulse in pulses:
                    value = finite_number(pulse, "pulse")
                    if not value.is_integer() or not 1000 <= value <= 2000:
                        raise ValueError("pulse values must be integer microseconds from 1000 to 2000")
                    parsed_pulses.append(int(value))
                updates['pulses'] = parsed_pulses
            with bridge.state_lock:
                if 'depth' in updates: bridge.sim_depth = updates['depth']
                if 'x' in updates: bridge.sim_x = updates['x']
                if 'yaw' in updates: bridge.sim_yaw = updates['yaw']
                if 'pitch' in updates: bridge.sim_pitch = updates['pitch']
                if 'roll' in updates: bridge.sim_roll = updates['roll']
                if 'wave' in updates: bridge.sim_wave = updates['wave']
                if 'pulses' in updates: bridge.pulses = updates['pulses']
            self.send_json({"success": True, "state": bridge.get_full_state()})
            return

        elif path == '/api/sim_mission':
            action = body.get('action', 'start').lower()
            if action in ['start', 'run']:
                bridge.start_sim_mission()
            elif action in ['complete', 'completed']:
                bridge.stop_sim_mission(completed=True)
            elif action in ['stop', 'abort', 'reset']:
                bridge.stop_sim_mission()
            else:
                raise ValueError("action must be start or stop")
            self.send_json({"success": True, "sim_mission": bridge.get_sim_mission_status()})
            return

        elif path == '/api/rl_pid/train':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            dist = body.get('disturbance')
            if dist is not None:
                dist = finite_number(dist, "disturbance")
            episodes_value = finite_number(body.get('episodes', 30), "episodes")
            if not episodes_value.is_integer() or not 1 <= episodes_value <= 1000:
                raise ValueError("episodes must be an integer from 1 to 1000")
            episodes = int(episodes_value)
            res = rl_agent.train_simulation(disturbance=dist, episodes=episodes)
            self.send_json({"success": True, "result": res})
            return

        elif path == '/api/rl_pid/apply':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            gains = rl_agent.adaptive_gains
            d = gains['depth']
            r = gains['roll']
            commands = [
                f"pid depth {d['kp']:.4f} {d['ki']:.4f} {d['kd']:.4f}",
                f"pid roll {r['kp']:.4f} {r['ki']:.4f} {r['kd']:.4f}"
            ]
            results = []
            for command in commands:
                result = bridge.execute_command(command)
                results.append(result)
                if not result.get('success', False):
                    break

            if len(results) == len(commands) and all(result.get('success', False) for result in results):
                results.append(bridge.execute_command("save"))

            all_ok = len(results) == len(commands) + 1 and all(res.get('success', False) for res in results)
            any_offline = any(res.get('offline', False) for res in results)
            failure = next((res for res in results if not res.get('success', False)), None)
            if all_ok:
                reply = "Depth/roll gains applied and saved; pitch control is unsupported."
            elif any_offline and len(results) == 1:
                reply = "Hardware offline; no gain commands were applied or saved."
            else:
                error = (failure.get('error') or failure.get('reply') or "") if failure else ""
                reply = f"Apply stopped at the first failed command; save was skipped. Earlier changes may remain active in RAM. {error}"
            self.send_json({
                "success": all_ok,
                "offline": any_offline,
                "results": results,
                "applied_gains": gains,
                "reply": reply
            })
            return

        elif path == '/api/rl_pid/toggle':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            en = bool(body.get('enabled', True))
            if not isinstance(body.get('enabled', True), bool):
                raise ValueError("enabled must be a boolean")
            rl_agent.set_enabled(en)
            self.send_json({"success": True, "status": rl_agent.get_status()})
            return

        elif path == '/api/rl_pid/config':
            if not rl_agent:
                self.send_json({"success": False, "error": "Modul RL Agent tidak tersedia"}, code=500)
                return
            disturbance = finite_number(body['disturbance'], "disturbance") if 'disturbance' in body else None
            if disturbance is not None and not 0.0 <= disturbance <= 1.0:
                raise ValueError("disturbance must be between 0 and 1")
            target_depth = finite_number(body['target_depth_cm'], "target_depth_cm") if 'target_depth_cm' in body else None
            if target_depth is not None and not 0.0 <= target_depth <= 140.0:
                raise ValueError("target_depth_cm must be between 0 and 140")
            mass_val = finite_number(body['mass'], "mass") if 'mass' in body else None
            buoy_val = finite_number(body['net_buoyancy_g'], "net_buoyancy_g") if 'net_buoyancy_g' in body else None
            drag_val = finite_number(body['drag_scale'], "drag_scale") if 'drag_scale' in body else None
            if mass_val is not None and not 5.0 <= mass_val <= 50.0:
                raise ValueError("mass must be between 5 and 50 kg")
            if buoy_val is not None and not -1000.0 <= buoy_val <= 1000.0:
                raise ValueError("net_buoyancy_g must be between -1000 and 1000")
            if drag_val is not None and not 0.2 <= drag_val <= 5.0:
                raise ValueError("drag_scale must be between 0.2 and 5")

            if disturbance is not None:
                rl_agent.set_disturbance(disturbance)
            if 'target_depth_cm' in body:
                with rl_agent.lock:
                    rl_agent.target_depth_cm = target_depth
                    rl_agent._simulate_step_response()
            if 'mass' in body or 'net_buoyancy_g' in body or 'drag_scale' in body:
                rl_agent.set_physics_parameters(mass=mass_val, net_buoyancy_g=buoy_val, drag_scale=drag_val)
                with bridge.state_lock:
                    if mass_val is not None: bridge.sim_mass = mass_val
                    if buoy_val is not None: bridge.sim_net_buoyancy_g = buoy_val
                    if drag_val is not None: bridge.sim_drag_scale = drag_val
            self.send_json({"success": True, "status": rl_agent.get_status()})
            return

        elif path == '/api/physics_config':
            updates = {}
            for key, low, high in (
                ('mass', 5.0, 50.0),
                ('net_buoyancy_g', -1000.0, 1000.0),
                ('drag_scale', 0.2, 5.0)
            ):
                if key in body:
                    value = finite_number(body[key], key)
                    if not low <= value <= high:
                        raise ValueError(f"{key} must be between {low} and {high}")
                    updates[key] = value
            with bridge.state_lock:
                if 'mass' in updates: bridge.sim_mass = updates['mass']
                if 'net_buoyancy_g' in updates: bridge.sim_net_buoyancy_g = updates['net_buoyancy_g']
                if 'drag_scale' in updates: bridge.sim_drag_scale = updates['drag_scale']
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


def run_app(open_browser=True):
    server_address = ('127.0.0.1', PORT)
    httpd = http.server.ThreadingHTTPServer(server_address, UnifiedHttpHandler)

    print("=" * 76)
    print(" AUV SAUVAC PUSAT KONTROL & UNIFIED DIGITAL TWIN")
    print(" Universitas Bangka Belitung — RoboNation SAUVC 2027")
    print(f" Web Server  : http://localhost:{PORT}")
    print(f" Hardware    : {DEFAULT_COM_PORT} @ {DEFAULT_BAUD} bps (Auto-detecting)")
    print(" Tekan Ctrl+C untuk menghentikan server dan menutup port secara bersih.")
    print("=" * 76)

    if open_browser:
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
    run_app(open_browser='--no-browser' not in sys.argv[1:])
