@echo off
setlocal
set "HERE=%~dp0"
set "PY=C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if not exist "%PY%" set "PY="
if not defined PY (
  for /f "delims=" %%P in ('where python 2^>nul') do if not defined PY set "PY=%%P"
)
if not defined PY (
  echo [ERROR] Python 3 was not found.
  pause
  exit /b 1
)

echo ============================================================
echo   PX4 Shelf Inventory Ground Station
echo   Web UI  : http://127.0.0.1:8765/
echo   WS link : port 8890 (aircraft connects to this PC)
echo   Camera  : see config/runtime.json -> camera.url
echo ============================================================
"%PY%" "%HERE%server.py" --no-mock --open %*
pause
endlocal
