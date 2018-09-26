@echo off
setlocal EnableDelayedExpansion

set args=0
set rosrun_prefix=
set DEBUG=0

call :getargc args %*

:handleargs
if "%1" equ "--help" goto :usage
if "%1" equ "-h" goto :usage
if "%1" equ "--prefix" goto :prefix
if "%1" equ "-p" goto :prefix
if "%1" equ "--debug" ( 
  set /A DEBUG=1
  goto :nextarg 
)
if "%1" equ "-d" ( 
  set DEBUG=1 
  goto :nextarg 
)
goto :rosrun

:prefix
set rosrun_prefix=%2
shift /1
set /A args-=1

:nextarg
shift /1
set /A args-=1
if %args% gtr 0 goto :handleargs

:rosrun
if %args% lss 2 goto :usage

set catbin=
set rosrun_search_path=
set catkin_find_search_path=
set rospack_find_search_path=
if NOT "%CMAKE_PREFIX_PATH%"=="" (
  for /f "delims=" %%g in ('catkin_find --without-underlays --libexec --share %1') do (
    set "catkin_find_search_path=!catkin_find_search_path!;%%g"
  )
  call :debug "Looking in catkin libexec dirs: !catkin_find_search_path!"
)

for /f "delims=" %%a in ('rospack find %1') do (
  set "rospack_find_search_path=!rospack_find_search_path!;%%a"
)
call :debug "Looking in rospack dir: %rospack_find_search_path%"

if "%catkin_find_search_path%" == "" (
  if "%rospack_find_search_path%" == "" (
    exit /b 2
  )
)

for /f "delims=" %%a in ('catkin_find --bin') do (
  set "catbin=!catbin!;%%a"
)

REM on Windows, we will have a pecking order, libexec, pkgdir, global bin dir
set "rosrun_search_path=%catkin_find_search_path%;%rospack_find_search_path%;%catbin%"

if "%~x2"=="" (
  REM iterate through PATHEXT if no file extension specified.
  for %%a in (%PATHEXT%) do (
    call :debug "Searching for %2%%a"
    for %%i in (%2%%a) do (
      set "exepath=%%~$rosrun_search_path:i"
      if NOT "!exepath!" == "" (
        goto :process_exeargs
      )
    )
  )
) else (
  REM directly search for the file if extension specified.
  call :debug "Searching for %2"
  for %%i in (%2) do (
    set "exepath=%%~$rosrun_search_path:i"
  )
)

if "!exepath!" == "" (
  echo [rosrun] Couldn't find executable named %2
  exit /b 3
)

:process_exeargs
set exeargs=
if NOT %args% gtr 2 goto :start
shift /1
shift /1
set /A args-=2
:argloop
set n=%1%
set v=%2%
set "exeargs=%exeargs% %n%=%v%"
shift /1
shift /1
set /A args-=2
call :debug "%exeargs%"
if %args% gtr 0 goto :argloop

:start
call :debug "Running %rosrun_prefix% %exepath% %exeargs%"
call %rosrun_prefix% %exepath% %exeargs%
exit /b %ERRORLEVEL%

:debug
if %DEBUG% == 1 (
  echo [rosrun] %~1
)  
goto :eof

:usage
echo Usage: rosrun [--prefix cmd] [--debug] PACKAGE EXECUTABLE [ARGS]
echo   rosrun will locate PACKAGE and try to find
echo   an executable named EXECUTABLE in the PACKAGE tree.
echo   If it finds it, it will run it with ARGS.
exit /b 0

:getargc
  set getargc_v0=%1
  set /A "%getargc_v0% = 0"
:getargc_loop
  if not x%2x==xx (
    shift
    set /A "%getargc_v0% = %getargc_v0% + 1"
    goto :getargc_loop
  )
  set getargc_v0=
  goto :eof
