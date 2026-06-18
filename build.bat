@echo off
echo ============================================
echo  BUILD - Controlador PID
echo ============================================

set PYTHON=python
set SCRIPT=%~dp0interface_pid_controller.py

echo [1/3] Instalando dependencias...
%PYTHON% -m pip install pyinstaller pyserial matplotlib numpy

echo [2/3] Gerando executavel...
%PYTHON% -m PyInstaller --onefile --windowed --name "ControladorPID" "%SCRIPT%"

echo [3/3] Concluido!
echo O executavel esta em: dist\ControladorPID.exe
pause