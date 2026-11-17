# AUV SAUVAC — Kendali Presisi 6-Thruster & Digital Twin Bawah Air

<div align="center">

[![Microcontroller](https://img.shields.io/badge/MCU-Raspberry%20Pi%20Pico%20W%20(RP2040)-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Target](https://img.shields.io/badge/Target-SAUVC%202027-0052CC?style=for-the-badge&logo=target&logoColor=white)](https://sauvc.org/)
[![Physics](https://img.shields.io/badge/Hydrodynamics-Fossen%206--DOF%20%2F%20SNAME-00875A?style=for-the-badge)](simulasi_auv/)
[![Simulation](https://img.shields.io/badge/Digital%20Twin-Three.js%203D%20%2B%20Canvas%202D-FF5630?style=for-the-badge&logo=three.js&logoColor=white)](simulasi_auv/)
[![Build Status](https://img.shields.io/badge/Firmware-Verified%20(0%20Warnings)-success?style=for-the-badge&logo=platformio&logoColor=white)](firmware/)

**Pusat Kendali Presisi 6-Thruster & Simulator Digital Twin Bawah Air**

---

### 🌐 Bahasa
[**English**](README.md) • **Bahasa Indonesia**

---

</div>

## 📸 Galeri Visual

<div align="center">

| 3D Digital Twin (Three.js & CAD Mesh) | 2D Fossen Hydrodynamics & Wave Engine |
| :---: | :---: |
| <img src="docs/images/sim3d_view.png" alt="3D Simulation" width="460"/> | <img src="docs/images/sim2d_view.png" alt="2D Simulation" width="460"/> |

| Industrial Web Dashboard & Telemetry | Authentic CAD Projections |
| :---: | :---: |
| <img src="docs/images/dashboard_view.png" alt="Dashboard" width="460"/> | <img src="docs/images/cad_views.png" alt="CAD Views" width="460"/> |

</div>

---

### Daftar Isi
- [Ringkasan Sistem](#ringkasan-sistem)
- [Fitur Utama](#fitur-utama)
- [Pinout Hardware & Geometri 6-Thruster](#pinout-hardware--geometri-6-thruster)
- [Panduan Simulator Digital Twin](#panduan-simulator-digital-twin)
- [Referensi Perintah Serial CLI](#referensi-perintah-serial-cli)
- [Format Misi Otonom (`/jalur.json`)](#format-misi-otonom-jalurjson)
- [Alat Pengujian & Flashing](#alat-pengujian--flashing)
- [Struktur Direktori Repositori](#struktur-direktori-repositori)

---

## 📌 Ringkasan Sistem
Proyek ini mengimplementasikan arsitektur kontrol ganda (*dual-rate control*):
1. **Fast-Loop Tertanam (RP2040 @ 50 Hz / 20 ms):** Firmware mikroprosesor C++ deterministik tanpa overhead Wi-Fi/web server, memprioritaskan pembangkitan pulsa PWM motor yang presisi, sensor fusion (MPU-6050 & MS5803), stabilisasi loop tertutup PID, dan eksekusi trajektori otonom LittleFS.
2. **Companion Digital Twin (Host PC @ 60 FPS / Port 8080):** Simulator hidrodinamika Fossen 2D/3D interaktif berbasis web, mendukung simulasi murni (*Software-in-the-Loop*) maupun koneksi perangkat keras (*Hardware-in-the-Loop* via USB serial `COM4 @ 115200 baud`).

---

## 🌟 Fitur Utama
1. **Loop Kontrol 50 Hz Deterministik:** Bebas jitter jaringan untuk menjamin sinyal PWM ESC brushless tidak pernah terputus.
2. **Konfigurasi Vektor 6-Thruster:** 4 thruster horizontal bersudut 45° (vektor X) untuk surge, sway, dan yaw, serta 2 thruster vertikal untuk heave, pitch, dan roll.
3. **Fisika Hidrodinamika Nonlinier Fossen:** Memperhitungkan massa bodi ($14.5\text{ kg}$), massa tambah fluida (*added mass*), seretan air kuadratik, momen pemulih metasentrik, gaya apung positif netral (+150g fail-to-float), dan simulasi gelombang linier Airy.
4. **Stabilisasi Sikap Loop Tertutup PID:** Kontrol aktif untuk *Depth Hold* (kedalaman air), *Roll leveling*, dan *Pitch leveling*.
5. **Mesin Misi LittleFS:** Membaca skenario `/jalur.json` dengan dukungan istilah navigasi bahasa Indonesia (`selam`, `maju`, `mundur`, `putar`, `kiri`, `kanan`, `hover`, `naik`).
6. **Proteksi & Failsafe:** Pembatasan daya default **25%** untuk pengujian meja (*bench test*), kompensasi *deadband* ESC ($25\ \mu\text{s}$), dan watchdog failsafe (500 ms).

---

## 🔌 Pinout Hardware & Geometri 6-Thruster

```text
                    ▲ [HALUAN / DEPAN]
                    │
      [T3: Depan-Kiri]  ╲            ╱ [T2: Depan-Kanan]
         (GP4, CCW)      ╲          ╱     (GP3, CW)
                          ┌────────┐
        [T6: Vert-Kiri]   │ RP2040 │  [T5: Vert-Kanan]
           (GP7, CW)      │ PICO W │     (GP6, CCW)
                          └────────┘
      [T1: Belakang-Kiri]╱          ╲ [T4: Belakang-Kanan]
          (GP2, CW)     ╱            ╲    (GP5, CCW)
                    │
                    ▼ [BURITAN / BELAKANG]
```

| Komponen / Aktuator | GPIO RP2040 | Polaritas / Putaran | Fungsi / Peran |
| :--- | :---: | :---: | :--- |
| **Thruster 1 (T1)** | **GP2** | CW (Reverse: 1) | Vektor horizontal belakang-kiri (45°), surge/yaw |
| **Thruster 2 (T2)** | **GP3** | CW (Reverse: 1) | Vektor horizontal depan-kanan (45°), surge/yaw |
| **Thruster 3 (T3)** | **GP4** | CCW (Reverse: 0) | Vektor horizontal depan-kiri (45°), surge/yaw |
| **Thruster 4 (T4)** | **GP5** | CCW (Reverse: 0) | Vektor horizontal belakang-kanan (45°), surge/yaw |
| **Thruster 5 (T5)** | **GP6** | CCW (Reverse: 0) | Vektor vertikal tengah-kanan, heave/depth, roll trim |
| **Thruster 6 (T6)** | **GP7** | CW (Reverse: 1) | Vektor vertikal tengah-kiri, heave/depth, roll trim |
| **I2C SDA** | **GP26** | Standar I2C Wire1 | Sensor MPU-6050 (0x68) & MS5803 (0x76) |
| **I2C SCL** | **GP27** | Standar I2C Wire1 | Sensor MPU-6050 (0x68) & MS5803 (0x76) |
| **Serial CLI / Telemetri** | **USB** | 115200 baud, 8-N-1 | Terminal interaktif & paket mesin `$TEL`/`$INJ` |

---

## 🌊 Panduan Simulator Digital Twin
Folder `simulasi_auv/` berisi web server terpadu dan antarmuka Three.js 3D:
```bash
# Jalankan simulator
cd simulasi_auv

# Opsi 1 (Windows Batch):
start.bat

# Opsi 2 (Python Langsung):
python app.py
```
Buka browser pada **`http://localhost:8080`**. Antarmuka menyediakan 6 tab kontrol:
* **🏠 Home / Dashboard:** Telemetri live (kedalaman, sudut pitch/roll, pulsa 6 motor, status arming).
* **🌊 Simulasi 2D & 3D:** Kolam lomba Three.js ($12\text{m} \times 5\text{m} \times 2\text{m}$) dengan gawang pipa PVC, kaustik animasi, panah gaya dorong, dan grafik gerak fluida 2D Fossen.
* **⚙️ Konfigurasi:** Pengaturan limit daya, deadband ESC, reverse thruster, dan gain PID.
* **🎯 Kalibrasi Sensor:** Tare nol kedalaman air dan perataan sudut IMU level.
* **🗺️ Pembuat Jalur:** Desain lintasan misi visual dengan upload langsung via serial ke LittleFS.
* **💻 Terminal Hardware:** Terminal serial ANSI langsung ke mikroprosesor RP2040.

---

## ⚡ Referensi Perintah Serial CLI
Koneksikan serial terminal pada baud rate **115200**:
* `status` / `stat`: Menampilkan dashboard sistem lengkap.
* `sensor stream`: Streaming data sensor langsung (tekan sembarang tombol untuk berhenti).
* `telem on [hz]`: Menyalakan pengiriman telemetri non-blocking mesin `$TEL` (default 10 Hz).
* `tes <1-6|all> <power>`: Memutar motor selama 2 detik pada daya persentase tertentu.
* `esc calib`: Melakukan 3 tahap kalibrasi rentang throttle ESC otomatis (2000µs $\to$ 1000µs $\to$ 1500µs).
* `calib zero`: Mengunci tekanan atmosfer saat ini sebagai referensi permukaan 0 cm.
* `calib level`: Mengunci sudut tilt saat ini sebagai offset level (0.0°/0.0°).
* `pid on` / `pid off`: Mengaktifkan atau menonaktifkan stabilisasi tertutup.
* `pid <axis> <kp> <ki> <kd>`: Mengatur konstanta PID untuk `depth`, `roll`, atau `pitch`.
* `save`: Menyimpan konfigurasi dan kalibrasi ke EEPROM (ber-checksum FNV-1a).
* `stop`: **EMERGENCY STOP**, menetralkan seluruh thruster ke 1500 µs seketika.

---

## 🗺️ Format Misi Otonom (`/jalur.json`)
Skenario disimpan di LittleFS mikroprosesor:
```json
[
  {"step": 1, "name": "Menyelam ke 50 cm", "action": "DIVE", "power": 15, "duration": 5.0, "target_cm": 50.0, "motors": [5, 6]},
  {"step": 2, "name": "Berenang Maju Lurus", "action": "MAJU", "power": 20, "duration": 12.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
  {"step": 3, "name": "Putar Haluan Kanan", "action": "PUTAR", "power": 15, "duration": 4.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
  {"step": 4, "name": "Berenang Kembali", "action": "MAJU", "power": 20, "duration": 12.0, "target_cm": 50.0, "motors": [1, 2, 3, 4, 5, 6]},
  {"step": 5, "name": "Naik ke Permukaan", "action": "SURFACE", "power": 15, "duration": 4.0, "target_cm": 0.0, "motors": [5, 6]}
]
```

---

## 🛠️ Alat Pengujian & Flashing
Folder [`tools/`](tools/) menyediakan skrip pengujian Python lintas-platform:
```bash
# Siklus otomatis flash BOOTSEL & verifikasi fungsional
python tools/flash_and_verify.py

# Pengujian regresi skema EEPROM v6 & injeksi sensor
python tools/flash_and_test_v6.py

# Verifikasi perintah serial aktif tanpa flash ulang
python tools/verify_all.py
```

Kompilasi source code via PlatformIO:
```bash
pio run              # Kompilasi firmware C++
pio run -t upload    # Upload ke Pico W via USB
pio device monitor -b 115200 # Buka monitor serial
```

---

## 📂 Struktur Direktori Repositori
```text
auv-6thruster-rev-0.1/
├── platformio.ini                # Konfigurasi build PlatformIO untuk RP2040 Pico W
├── README.md                     # Dokumentasi utama bahasa Inggris
├── README.id.md                  # Dokumentasi utama bahasa Indonesia (file ini)
├── .gitignore                    # Aturan ignore git bersih
│
├── src/                          # Source code firmware C++ RP2040
│   ├── config.h                  # Skema EEPROM, pinout, matriks mixer
│   ├── main.cpp                  # Loop kontrol 50 Hz, FSM, dan stabilisasi PID
│   ├── sensors.h / sensors.cpp   # Driver MPU-6050 & MS5803 serta injeksi HIL
│   ├── mission.h / mission.cpp   # Parser & eksekutor trajektori LittleFS /jalur.json
│   └── cli.h / cli.cpp           # Terminal CLI ANSI & protokol mesin terisolasi
│
├── firmware/                     # File biner terverifikasi
│   ├── firmware_rev0.1.uf2       # Biner flash drag-and-drop BOOTSEL (835 KB)
│   └── firmware_rev0.1.bin       # Biner flash langsung (417 KB)
│
├── simulasi_auv/                 # Digital Twin & Web Control Center
│   ├── app.py                    # Server companion Python & serial bridge (Port 8080)
│   ├── rl_agent.py               # Modul optimasi agen RL-PID adaptif
│   ├── index.html                # Antarmuka web terpadu 6-tab industrial dark theme
│   ├── hydrodynamics.js          # Mesin fisika fluida 2D Fossen & gelombang Airy
│   ├── sim3d.js                  # Engine visualisasi 3D Three.js
│   ├── start.bat                 # Shortcut launcher Windows sekali klik
│   ├── assets/                   # Model 3D STL CAD asli & sprite robot
│   └── libs/                     # Three.js r128, OrbitControls, STLLoader
│
├── tools/                        # Skrip utilitas flashing & pengujian
│   ├── flash_and_verify.py       # Skrip utama flash & regression testing otomatis
│   ├── flash_and_test_v6.py      # Pengujian regresi skema v6
│   └── verify_all.py             # Verifikasi perintah serial aktif
│
└── docs/                         # Gambar aset dokumentasi
    └── images/                   # Tangkapan layar resolusi tinggi
```
