@echo off
setlocal EnableDelayedExpansion

set DEBUG=0

set roscp_args=%*
set roscp_package=
set roscp_filename=

:handleargs
if "%1" equ "--help" goto :usage
if "%1" equ "-h" goto :usage
if "%1" equ "--debug" (
  set /A DEBUG=1
  goto :nextarg
)
if "%1" equ "-d" (
  set DEBUG=1
  goto :nextarg
)
goto :find_roscp_parameters

:nextarg
shift /1
call :trim_first_arg result_args !roscp_args!
set roscp_args=!result_args!

if not "%1"=="" goto :handleargs

:find_roscp_parameters
if "%1"=="" goto :usage
set roscp_package=%1
call :trim_first_arg result_args !roscp_args!
set roscp_args=!result_args!
shift /1

if "%1"=="" goto :usage
set roscp_filename=%1
call :trim_first_arg result_args !roscp_args!
set roscp_args=!result_args!
shift /1

set catbin=
set roscp_search_path=
set catkin_find_search_path=
set rospack_find_search_path=
if NOT "%CMAKE_PREFIX_PATH%"=="" (
  for /f "delims=" %%g in ('catkin_find --without-underlays --libexec --share !roscp_package!') do (
    set "catkin_find_search_path=!catkin_find_search_path!;%%g"
  )
  call :debug "Looking in catkin libexec dirs: !catkin_find_search_path!"
)

for /f "delims=" %%a in ('rospack find !roscp_package!') do (
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

set catkin_find_search_path=%catkin_find_search_path:~1%
set rospack_find_search_path=%rospack_find_search_path:~1%
set catbin=%catbin:~1%

:: on Windows, we will have a pecking order, libexec, pkgdir, global bin dir
set "roscp_search_path=%catkin_find_search_path%;%rospack_find_search_path%;%catbin%"

:: search in the catkin and rospack path
call :debug "Searching for !roscp_filename! from !roscp_search_path!"
for %%i in (!roscp_filename!) do (
  set "src_filepath=%%~$roscp_search_path:i"
  if NOT "!src_filepath!" == "" (
    goto :run_roscp_internal
  )
)

if "!src_filepath!" == "" (
  echo [roscp] Couldn't find file named !roscp_filename!
  exit /b 3
)

:run_roscp_internal
call :debug "copy %src_filepath% %roscp_args%"
copy %src_filepath% %roscp_args% > NUL
exit /b %ERRORLEVEL%

:debug
if %DEBUG% == 1 (
  echo [roscp] %*
)
goto :eof

:usage
echo usage: roscp package filename target
echo.
echo.
echo Copy a file from a package to target location.
exit /b 0

:trim_first_arg
setlocal EnableDelayedExpansion
set params=%*
for /f "tokens=2*" %%a in ("!params!") do EndLocal & set %1=%%b
exit /b
