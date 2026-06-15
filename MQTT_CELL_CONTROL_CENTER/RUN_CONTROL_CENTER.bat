@echo off
setlocal
cd /d "%~dp0"
title Robot Cell Factory Supervisor - Persistent Sorter
echo Starting corrected coordinator from:
echo %CD%
python -m pip install -r requirements.txt
python cell_control_center.py
pause
