@echo off
setlocal EnableDelayedExpansion

set args=0
set rosrun_prefix=
set DEBUG=0

for %%i in (%*) do set /A args+=1

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

set catkin_package_libexec_dirs=
set pkgdir=
if NOT EXIST %CMAKE_PREFIX_PATH% (
  set prevline=
  for /f "delims=" %%g in ('catkin_find --without-underlays --libexec --share %1') do (
    if NOT DEFINED prevline (
      set "prevline=TRUE"
      set "catkin_package_libexec_dirs=%%g"
    ) else (
      set catkin_package_libexec_dirs=!catkin_package_libexec_dirs! %%g
    )
  )
  call :debug "Looking in catkin libexec dirs: %catkin_package_libexec_dirs%"
)

for /f "delims=" %%a in ('rospack find %1') do set "pkgdir=%%a"
call :debug "Looking in rospack dir: %pkgdir%"

if "%catkin_package_libexec_dirs%" == "" (
  if "%pkgdir%" == "" {
    exit /b 2
  )
)

set wildchar=%2:~-1%
if NOT "%wildchar%" == "*" (
  call :debug "Searching for %2.exe"
  REM on Windows, we will have a pecking order, libexec, pkgdir, global bin dir
  set nexes=0
  if NOT "%catkin_package_libexec_dirs%" == "" (
    for %%g in (%catkin_package_libexec_dirs%) do (
      call :findexe %2.exe %%g
    )
  )
  call :findexe %2.exe %pkgdir%
  for /f "delims=" %%a in ('catkin_find --bin') do set "catbin=%%a"
  call :findexe %2.exe !catbin!

  REM Select the first exe in the list
  if !nexes! EQU 0 (
    echo [rosrun] Couldn't find executable named %2.exe
    exit /b 3
  )
  if !nexes! GTR 1 (
    echo [rosrun] You have chosen a non-unique executable, selecting the first.
  )

  set nexes=0
  set exepath=!exepaths_0!
) else (
  set absname=%pkgdir%/%2.exe
  call :debug "Path given. Looking for %absname%"
  if NOT EXIST "%absname%" (
    echo Couldn't find executable named %absname%
    exit /b 3
  )
  set exepath=%absname%
)

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
start cmd.exe /c "%rosrun_prefix% %exepath% %exeargs%"

goto :eof

:findexe
REM Convert forward slashes and make sure dirs end with slashes
set lpath=%2
set lpath=%lpath:/=\%
if NOT EXIST %lpath% goto :eof
pushd
cd %lpath%
for /r %%d in (%1) do (
  set exepaths_!nexes!=%%d
  set /A nexes+=1
)
popd
goto :eof

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
