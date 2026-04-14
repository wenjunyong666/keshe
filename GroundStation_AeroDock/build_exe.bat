@echo off
setlocal
cd /d "%~dp0"

echo [1/3] Checking Python...
python --version || goto :fail

echo [2/3] Building desktop app...
python -m PyInstaller --clean --noconfirm GroundStation.spec || goto :fail

echo [3/3] Done.
echo EXE path: "%~dp0dist\地面站\地面站.exe"
echo Simulator EXE: "%~dp0dist\地面站_模拟版\地面站_模拟版.exe"
pause
exit /b 0

:fail
echo Build failed.
pause
exit /b 1
