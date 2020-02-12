@echo off
setlocal EnableDelayedExpansion

set DEBUG=0

set rosrun_args=%*
set rosrun_prefix=
set rosrun_package=
set rosrun_executable=

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
goto :find_rosrun_parameters

:prefix
set rosrun_prefix=%2
shift /1
call :trim_first_arg result_args !rosrun_args!
set rosrun_args=!result_args!

:nextarg
shift /1
call :trim_first_arg result_args !rosrun_args!
set rosrun_args=!result_args!

if not "%1"=="" goto :handleargs

:find_rosrun_parameters
if "%1"=="" goto :usage
set rosrun_package=%1
call :trim_first_arg result_args !rosrun_args!
set rosrun_args=!result_args!
shift /1

if "%1"=="" goto :usage
set rosrun_executable=%1
set rosrun_executable_extension=%~x1
call :trim_first_arg result_args !rosrun_args!
set rosrun_args=!result_args!
shift /1

set catbin=
set rosrun_search_path=
set catkin_find_search_path=
set rospack_find_search_path=
if NOT "%CMAKE_PREFIX_PATH%"=="" (
  for /f "delims=" %%g in ('catkin_find --without-underlays --libexec --share !rosrun_package!') do (
    set "catkin_find_search_path=!catkin_find_search_path!;%%g"
  )
  call :debug "Looking in catkin libexec dirs: !catkin_find_search_path!"
)

for /f "delims=" %%a in ('rospack find !rosrun_package!') do (
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

if "!rosrun_executable_extension!"=="" (
  REM iterate through PATHEXT if no file extension specified.
  for %%a in (%PATHEXT%) do (
    call :debug "Searching for executable !rosrun_executable!%%a"
    for %%i in (!rosrun_executable!%%a) do (
      set "exepath=%%~$rosrun_search_path:i"
      if NOT "!exepath!" == "" (
        goto :run_rosrun_internal
      )
    )
  )
)

REM search in the catkin and rospack path
call :debug "Searching for executable !rosrun_executable!"
for %%i in (!rosrun_executable!) do (
  set "exepath=%%~$rosrun_search_path:i"
  if NOT "!exepath!" == "" (
    goto :run_rosrun_internal
  )
)

if "!exepath!" == "" (
  echo [rosrun] Couldn't find executable named !rosrun_executable!
  exit /b 3
)

:run_rosrun_internal
call :debug "Running %rosrun_prefix% %exepath% %rosrun_args%"
for %%a in ("%exepath%") do (
  set exepath_extension=%%~xa
)
if "!exepath_extension!" == ".py" (
  call %rosrun_prefix% "%PYTHONHOME%\python.exe" %exepath% %rosrun_args%
) else if "!exepath_extension!" == "" (
  call %rosrun_prefix% "%PYTHONHOME%\python.exe" %exepath% %rosrun_args%
) else (
  call %rosrun_prefix% %exepath% %rosrun_args%
)
exit /b %ERRORLEVEL%

:debug
if %DEBUG% == 1 (
  echo [rosrun] %*
)
goto :eof

:usage
echo Usage: rosrun [--prefix cmd] [--debug] PACKAGE EXECUTABLE [ARGS]
echo   rosrun will locate PACKAGE and try to find
echo   an executable named EXECUTABLE in the PACKAGE tree.
echo   If it finds it, it will run it with ARGS.
exit /b 0

:trim_first_arg
setlocal EnableDelayedExpansion
set params=%*
for /f "tokens=2*" %%a in ("!params!") do EndLocal & set %1=%%b
exit /b
