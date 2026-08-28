@echo off
setlocal EnableDelayedExpansion
rem ============================================================================
rem  start_service.bat - dev launcher for the pick_ik FastAPI service
rem
rem    start_service.bat
rem        If the service already answers /health: report its PID, open the
rem        browser anyway, and exit. Otherwise start
rem        "python -m service.main" in its own console window (title
rem        "ik_service" - that is where the logs go; Ctrl+C there stops the
rem        service cleanly), wait for /health, then open the browser.
rem
rem    start_service.bat /restart     (also accepted: restart, -r)
rem        Kill any running instance first (only if the PID on the port is
rem        really a python process), then start a fresh one.
rem
rem  Port: IK_SERVICE_PORT env var, default 8081 (same as service/main.py).
rem  Note: System32 executables are called by full path where a Git-for-Windows
rem        /usr/bin lookalike (e.g. GNU timeout.exe) could shadow them on PATH.
rem ============================================================================

set "ROOT=%~dp0"
cd /d "%ROOT%"

set "PORT=8081"
if defined IK_SERVICE_PORT set "PORT=%IK_SERVICE_PORT%"
set "BASE=http://127.0.0.1:%PORT%"

rem ---- argument parsing -----------------------------------------------------
if /I "%~1"=="/restart" goto do_restart_arg
if /I "%~1"=="restart" goto do_restart_arg
if /I "%~1"=="-r" goto do_restart_arg
if not "%~1"=="" (
  echo Unknown argument - usage: start_service.bat [/restart]
  exit /b 2
)
goto preflight

:do_restart_arg
set "MODE=restart"
goto preflight

rem ---- preflight ------------------------------------------------------------
:preflight
where python >NUL 2>&1 || (
  echo ERROR: python was not found on PATH.
  exit /b 1
)
where curl >NUL 2>&1 || (
  echo ERROR: curl.exe was not found ^(needed for the /health probe^).
  exit /b 1
)
set "PYD="
for %%f in ("%ROOT%dist\pickik*.pyd") do set "PYD=%%~nf"
if not defined PYD (
  echo WARNING: no pickik*.pyd found in dist\ - the service will crash on import.
  echo          Build the binding first; see HANDOVER.md section 3.
)

rem ---- what is on the port? --------------------------------------------------
call :find_pid
if not defined SVC_PID goto start_new
call :is_up
if errorlevel 1 (
  echo Port !PORT! is in use ^(PID !SVC_PID!^) but %BASE%/health does not answer.
  echo If a stale ik_service is stuck there, run: start_service.bat /restart
  exit /b 1
)
if /I "%MODE%"=="restart" goto do_kill
echo ik_service is already running on !BASE! ^(PID !SVC_PID!^).
echo Use "start_service.bat /restart" to kill and restart it.
start "" "!BASE!/"
exit /b 0

rem ---- /restart: kill the old instance ---------------------------------------
:do_kill
call :pid_is_python
if errorlevel 1 (
  echo Refusing to kill PID !SVC_PID! on port !PORT! ^(not a python process^).
  exit /b 1
)
echo Stopping ik_service ^(PID !SVC_PID!^)...
taskkill /F /PID !SVC_PID! >NUL
set /a TRIES=0
:wait_free
rem sleep ~1s via loopback ping: timeout.exe aborts under redirected stdin
%SystemRoot%\System32\ping.exe -n 2 127.0.0.1 >NUL
call :is_up
if errorlevel 1 goto start_new
set /a TRIES+=1
if !TRIES! LSS 5 goto wait_free
echo Could not stop the old instance within 5s - aborting.
exit /b 1

rem ---- start the service -----------------------------------------------------
:start_new
start "ik_service" /D "%ROOT%" cmd /k "python -m service.main"
echo Starting ik_service in its own console window ^(port !PORT!^), waiting for health...
set /a TRIES=0
:wait_up
rem sleep ~1s via loopback ping: timeout.exe aborts under redirected stdin
%SystemRoot%\System32\ping.exe -n 2 127.0.0.1 >NUL
call :is_up
if not errorlevel 1 goto opened
set /a TRIES+=1
if !TRIES! LSS 20 goto wait_up
echo.
echo ik_service did not answer !BASE!/health within 20s.
echo Look at the "ik_service" console window for the traceback, then try again.
exit /b 1

:opened
echo ik_service is up:  !BASE!/   ^(OpenAPI docs: !BASE!/docs^)
start "" "!BASE!/"
echo The service keeps running in the "ik_service" window; close it to stop.
exit /b 0

rem :is_up - errorlevel 0 if /health answers (2xx/3xx)
:is_up
curl -sf -o NUL --max-time 2 "!BASE!/health" >NUL 2>&1
exit /b %ERRORLEVEL%

rem :find_pid - sets SVC_PID to the PID listening on :!PORT! (empty if none).
rem Uses Get-NetTCPConnection instead of netstat because netstat localizes
rem its state names ("LISTENING" vs German "ABHOREN") on non-English systems.
:find_pid
set "SVC_PID="
for /f "usebackq delims=" %%p in (`powershell -NoProfile -Command "(Get-NetTCPConnection -LocalPort !PORT! -State Listen -ErrorAction SilentlyContinue).OwningProcess"`) do set "SVC_PID=%%p"
goto :eof

rem :pid_is_python - errorlevel 0 if SVC_PID belongs to python.exe
:pid_is_python
set "PNAME="
for /f "tokens=1 delims=," %%a in ('tasklist /FI "PID eq !SVC_PID!" /FO CSV /NH') do set "PNAME=%%a"
if /I "!PNAME:~1,-1!"=="python.exe" exit /b 0
exit /b 1
