@echo off
setlocal EnableDelayedExpansion
rem ============================================================================
rem  stop_service.bat - stop a running ik_service instance
rem
rem    stop_service.bat
rem        Find the PID listening on the service port (IK_SERVICE_PORT,
rem        default 8081) and stop it: first a graceful request
rem        (taskkill without /F), waiting up to 5s, then a forced
rem        taskkill /F if it is still alive. Refuses to touch a PID that
rem        does not belong to a python process, and verifies the port is
rem        free before reporting success.
rem
rem  If you can see the "ik_service" console window, pressing CTRL+C in it
rem  is the cleanest stop (uvicorn's graceful shutdown). This script is
rem  for when that window is not at hand.
rem  Note: System32 executables are called by full path where a
rem        Git-for-Windows /usr/bin lookalike (e.g. GNU timeout.exe)
rem        could shadow them on PATH.
rem ============================================================================

set "PORT=8081"
if defined IK_SERVICE_PORT set "PORT=%IK_SERVICE_PORT%"
set "BASE=http://127.0.0.1:%PORT%"

call :find_pid
if not defined SVC_PID (
  echo No ik_service found on port !PORT! ^(nothing listening^). Nothing to stop.
  exit /b 0
)
call :pid_is_python
if errorlevel 1 (
  echo Port !PORT! is held by PID !SVC_PID! which is NOT a python process - refusing to touch it.
  exit /b 1
)

echo Stopping ik_service ^(PID !SVC_PID!^) - graceful request first...
taskkill /PID !SVC_PID! >NUL 2>&1
set /a TRIES=0
:wait_graceful
rem sleep ~1s via loopback ping: timeout.exe aborts under redirected stdin
%SystemRoot%\System32\ping.exe -n 2 127.0.0.1 >NUL
call :find_pid
if not defined SVC_PID goto stopped
set /a TRIES+=1
if !TRIES! LSS 5 goto wait_graceful

echo Still running after 5s - forcing ^(taskkill /F^)...
taskkill /F /PID !SVC_PID! >NUL
set /a TRIES=0
:wait_forced
rem sleep ~1s via loopback ping: timeout.exe aborts under redirected stdin
%SystemRoot%\System32\ping.exe -n 2 127.0.0.1 >NUL
call :find_pid
if not defined SVC_PID goto stopped
set /a TRIES+=1
if !TRIES! LSS 5 goto wait_forced
echo ERROR: could not stop PID !SVC_PID! within 5s.
exit /b 1

:stopped
echo ik_service stopped - port !PORT! is free again.
echo The "ik_service" console window ^(if still open^) can simply be closed.
exit /b 0

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
