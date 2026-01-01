@echo off
echo Setting up Python environment...
set PYTHONHOME=C:\Users\saumondeluxe\AppData\Local\Programs\Python\Python311

echo Changing directory to build folder...
cd build\RelWithDebInfo

echo Launching GigaLearnBot without --render...
GigaLearnBot.exe

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Bot crashed or exited with error.
)
pause
