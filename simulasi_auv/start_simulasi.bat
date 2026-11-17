@echo off
title AUV SAUVAC 2D Digital Twin Simulator
cd /d "%~dp0"

echo ======================================================================
echo   AUV SAUVAC 2D UNDERWATER DIGITAL TWIN SIMULATOR
echo   Universitas Bangka Belitung - SAUVC 2027
echo ======================================================================
echo.

set PYTHON_BIN=""

if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
    set PYTHON_BIN="%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
    goto RUN
)
if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" (
    set PYTHON_BIN="%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
    goto RUN
)

where py >nul 2>nul
if %errorlevel% equ 0 (
    set PYTHON_BIN=py -3
    goto RUN
)

where python >nul 2>nul
if %errorlevel% equ 0 (
    set PYTHON_BIN=python
    goto RUN
)

echo [ERROR] Python not found in PATH or standard directory!
pause
exit /b 1

:RUN
echo Starting simulator with %PYTHON_BIN%...
%PYTHON_BIN% start_simulasi.py
if %errorlevel% neq 0 (
    echo.
    echo [NOTE] Server terminated.
    pause
)
