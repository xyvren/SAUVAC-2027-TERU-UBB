@echo off
title AUV SAUVAC Pusat Konfigurasi & Monitoring
cd /d "%~dp0"

set "PYTHON_BIN=python"
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
    set "PYTHON_BIN=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
)

echo ======================================================================
echo   AUV SAUVAC - PUSAT KONTROL & KONFIGURASI PARAMETER
echo   Universitas Bangka Belitung - SAUVC 2027
echo ======================================================================
echo.

"%PYTHON_BIN%" "%~dp0start_konfigurasi.py"
if errorlevel 1 (
    echo.
    echo Gagal membuka tab Pengaturan. Pastikan Python tersedia.
    pause
)
