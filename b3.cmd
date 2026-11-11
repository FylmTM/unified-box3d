@echo off
rem b3, the unified-box3d CLI (tools\): builds itself, then runs. Needs odin on PATH.
setlocal
set "root=%~dp0"
if not exist "%root%.out" mkdir "%root%.out"
odin build "%root%tools" -o:minimal "-out:%root%.out\b3.exe"
if errorlevel 1 exit /b 1
"%root%.out\b3.exe" %*
exit /b %errorlevel%
