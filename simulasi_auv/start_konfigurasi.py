#!/usr/bin/env python3
"""
AUV SAUVAC Configuration & Control Center Launcher
Starts the unified application server when needed and opens its settings tab.
"""

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
import webbrowser

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
APP_PY = os.path.join(SIM_DIR, "app.py")
APP_URL = "http://127.0.0.1:8080/#konfigurasi"
STATE_URL = "http://127.0.0.1:8080/api/state"


def is_app_running():
    try:
        with urllib.request.urlopen(STATE_URL, timeout=1.0) as response:
            payload = json.load(response)
            return (
                response.status == 200
                and isinstance(payload, dict)
                and {"connected", "config", "mission"}.issubset(payload)
            )
    except (urllib.error.URLError, TimeoutError, OSError, ValueError):
        return False


def wait_for_app(server_proc, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if is_app_running():
            return
        exit_code = server_proc.poll()
        if exit_code is not None:
            raise RuntimeError(f"app.py berhenti sebelum siap (exit code {exit_code}).")
        time.sleep(0.25)
    raise TimeoutError(f"app.py tidak merespons /api/state dalam {timeout} detik.")


def main():
    print("=" * 68)
    print("  AUV SAUVAC — PUSAT KONTROL & KONFIGURASI PARAMETER")
    print("  Universitas Bangka Belitung • SAUVC 2027")
    print("=" * 68)

    if not is_app_running():
        print("[*] Menyalakan server aplikasi terpadu...")
        creation_flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
        server_proc = subprocess.Popen(
            [sys.executable, APP_PY, "--no-browser"],
            cwd=SIM_DIR,
            creationflags=creation_flags,
        )
        wait_for_app(server_proc)
    else:
        print("[+] Server aplikasi terpadu sudah aktif di port 8080.")

    print(f"[*] Membuka tab Pengaturan: {APP_URL}")
    if not webbrowser.open(APP_URL):
        raise RuntimeError(f"Tidak dapat membuka browser otomatis. Buka URL ini: {APP_URL}")
    print("[+] Dashboard konfigurasi berhasil dibuka di browser!")

if __name__ == '__main__':
    main()
