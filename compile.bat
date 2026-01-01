@echo off
echo Building GigaLearnCPP (RelWithDebInfo)...
cmake --build build --config RelWithDebInfo
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Build FAILED.
    pause
    exit /b %ERRORLEVEL%
)
echo.
echo Build SUCCESSFUL.
pause
