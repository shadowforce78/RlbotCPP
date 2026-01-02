@echo off
echo Setting up Python environment...
set PYTHONHOME=C:\Users\saumondeluxe\AppData\Local\Programs\Python\Python311

echo Changing directory to build folder...
@REM Adjust the path if your build folder is located elsewhere
cd /d N:\Code\GigaLearnCPP-Leak\build\RelWithDebInfo

echo Launching GigaLearnBot without --render...
GigaLearnBot.exe

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Bot crashed or exited with error.
)
pause
