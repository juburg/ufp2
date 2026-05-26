@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash-ufp2.ps1" %*
exit /b %ERRORLEVEL%
