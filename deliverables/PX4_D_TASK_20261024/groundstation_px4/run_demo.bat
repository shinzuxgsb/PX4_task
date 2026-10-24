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
"%PY%" "%HERE%server.py" --mock --time-scale 3 --open %*
pause
endlocal
