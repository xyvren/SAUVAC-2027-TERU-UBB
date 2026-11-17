@echo off
title AUV SAUVAC Pusat Kontrol Terpadu & Simulasi 2D/3D
cd /d "%~dp0"

echo ======================================================================
echo   AUV SAUVAC - PUSAT KONTROL TERPADU & SIMULASI 2D/3D
echo   Universitas Bangka Belitung - SAUVC 2027
echo ======================================================================
echo.

set PYTHON_BIN=python
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
    set PYTHON_BIN="%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
) else if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" (
    set PYTHON_BIN="%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
)

echo [*] Menjalankan server AUV SAUVAC pada port 8080...
echo [*] Serial port: COM4 @ 115200 baud (atau fallback mock simulation)
echo [*] Browser akan terbuka otomatis di http://localhost:8080
echo.
%PYTHON_BIN% app.py

pause
