@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0export-ufp2-firmware.ps1" %*
exit /b %ERRORLEVEL%
