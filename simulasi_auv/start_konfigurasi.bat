@echo off
title AUV SAUVAC Pusat Konfigurasi & Monitoring
cd /d "%~dp0"

set PYTHON_BIN=python
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
    set PYTHON_BIN="%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
)

echo ======================================================================
echo   AUV SAUVAC - PUSAT KONTROL & KONFIGURASI PARAMETER
echo   Universitas Bangka Belitung - SAUVC 2027
echo ======================================================================
echo.

%PYTHON_BIN% -c "
import urllib.request, webbrowser, time, subprocess, sys, os

url = 'http://localhost:8080/config.html'
server_running = False

try:
    with urllib.request.urlopen('http://localhost:8080/api/state', timeout=1.0) as r:
        if r.status == 200:
            server_running = True
except Exception:
    server_running = False

if not server_running:
    print('[*] Menyalakan server bridge...')
    subprocess.Popen([sys.executable, 'server.py'], cwd=r'%~dp0')
    time.sleep(1.2)

print('[*] Membuka halaman konfigurasi di browser...')
webbrowser.open(url)
print('[+] Sukses dibuka di browser!')
"

echo.
echo Halaman konfigurasi sudah terbuka di browser!
echo Tekan tombol apa saja untuk menutup jendela hitam ini...
pause >nul
