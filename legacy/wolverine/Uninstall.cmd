@echo off
setlocal
title Wolverine Anatomy Tool 2.0 Beta 1 Uninstaller
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Install.ps1" -Mode Uninstall
if errorlevel 1 echo Uninstall failed. Read the message above; backups were retained.
echo.
pause
