@echo off
chcp 65001 >nul
title X72_FOC UART Link Check
cd /d "%~dp0"

if exist "serial_link_check.py" goto run
echo [ERROR] serial_link_check.py not found next to this bat.
pause
exit /b 2

:run
python serial_link_check.py %*
set RC=%ERRORLEVEL%
echo.
echo exit code = %RC%
pause
exit /b %RC%
