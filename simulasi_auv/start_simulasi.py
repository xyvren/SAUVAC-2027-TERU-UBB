#!/usr/bin/env python3
"""
AUV SAUVAC Simulator Launcher
Starts the digital twin server and automatically opens http://localhost:8080 in your default browser.
"""

import os
import sys
import time
import webbrowser
import subprocess

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
SERVER_PY = os.path.join(SIM_DIR, "app.py")
URL = "http://localhost:8080"

def main():
    print("=" * 65)
    print("  AUV SAUVAC — 2D UNDERWATER DIGITAL TWIN SIMULATOR")
    print("  Universitas Bangka Belitung • RoboNation SAUVC 2027")
    print("=" * 65)
    print(f"[*] Starting background bridge server on COM4...")
    print(f"[*] Opening browser interface at {URL}...")

    # Launch server
    proc = subprocess.Popen([sys.executable, SERVER_PY], cwd=SIM_DIR)

    # Wait 1.2s for server to start listening
    time.sleep(1.2)

    try:
        webbrowser.open(URL)
        print(f"[+] Web UI launched successfully at {URL}")
        print("[*] Press Ctrl+C in this console to stop the simulator.")
        proc.wait()
    except KeyboardInterrupt:
        print("\n[*] Stopping simulator server...")
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
        print("[+] Simulator stopped cleanly.")

if __name__ == '__main__':
    main()
