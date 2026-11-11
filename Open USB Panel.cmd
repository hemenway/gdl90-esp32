@echo off
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 (
    python tools\usb_panel.py %*
) else (
    py -3 tools\usb_panel.py %*
)
if errorlevel 1 pause
