#!/usr/bin/env python3
"""
AUV SAUVAC Configuration & Control Center Launcher
Starts the bridge server (if not already running) and opens http://localhost:8080/config.html in your default browser.
"""

import os
import sys
import time
import webbrowser
import subprocess
import urllib.request

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
SERVER_PY = os.path.join(SIM_DIR, "server.py")
CONFIG_URL = "http://localhost:8080/config.html"

def is_server_running():
    try:
        with urllib.request.urlopen("http://localhost:8080/api/state", timeout=0.8) as r:
            return r.status == 200
    except Exception:
        return False

def main():
    print("=" * 68)
    print("  AUV SAUVAC — PUSAT KONTROL & KONFIGURASI PARAMETER")
    print("  Universitas Bangka Belitung • SAUVC 2027")
    print("=" * 68)

    server_proc = None
    if not is_server_running():
        print("[*] Menyalakan server bridge di latar belakang (COM4 @ 115200)...")
        server_proc = subprocess.Popen([sys.executable, SERVER_PY], cwd=SIM_DIR)
        time.sleep(1.2)
    else:
        print("[+] Server bridge sudah aktif menyala di port 8080.")

    print(f"[*] Membuka browser pada: {CONFIG_URL}")
    webbrowser.open(CONFIG_URL)
    print("[+] Dashboard konfigurasi berhasil dibuka di browser!")
    print("\n[INFO] Tekan Ctrl+C di terminal ini jika ingin mematikan server.")

    try:
        if server_proc:
            server_proc.wait()
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n[*] Menutup aplikasi...")
        if server_proc:
            server_proc.terminate()
            try:
                server_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                server_proc.kill()
        print("[+] Selesai. Sampai jumpa lagi, Kakak!")

if __name__ == '__main__':
    main()
