@echo off
setlocal EnableDelayedExpansion

set "LIB_ROOT=%~dp0"
set "OUT_DIR=%LIB_ROOT%build"

if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if exist "%OUT_DIR%\*.obj" del /q "%OUT_DIR%\*.obj"
if exist "%OUT_DIR%\*.o" del /q "%OUT_DIR%\*.o"
if exist "%OUT_DIR%\control_algo.lib" del /q "%OUT_DIR%\control_algo.lib"

where cl.exe >nul 2>nul
if not errorlevel 1 goto build_msvc

where gcc.exe >nul 2>nul
if not errorlevel 1 goto build_gcc

where clang.exe >nul 2>nul
if not errorlevel 1 goto build_clang

echo No supported C compiler found. Install MSVC, GCC, or Clang, then rerun this script.
exit /b 1

:build_msvc
cl.exe /nologo /c /O2 /W3 /TC /I"%LIB_ROOT%include" /I"%LIB_ROOT%src" "%LIB_ROOT%src\*.c" /Fo"%OUT_DIR%\\"
if errorlevel 1 exit /b 1
set "OBJ_LIST="
for %%O in ("%OUT_DIR%\*.obj") do set OBJ_LIST=!OBJ_LIST! "%%~fO"
lib.exe /nologo /OUT:"%OUT_DIR%\control_algo.lib" !OBJ_LIST!
if errorlevel 1 exit /b 1
goto done

:build_gcc
where ar.exe >nul 2>nul
if errorlevel 1 (
	echo ar.exe not found. Install binutils or use an MSVC toolchain, then rerun this script.
	exit /b 1
)
for %%F in ("%LIB_ROOT%src\*.c") do (
	gcc.exe -std=c99 -O2 -Wall -Wextra -I"%LIB_ROOT%include" -I"%LIB_ROOT%src" -c "%%~fF" -o "%OUT_DIR%\%%~nF.o"
	if errorlevel 1 exit /b 1
)
set "OBJ_LIST="
for %%O in ("%OUT_DIR%\*.o") do set OBJ_LIST=!OBJ_LIST! "%%~fO"
ar.exe rcs "%OUT_DIR%\control_algo.lib" !OBJ_LIST!
if errorlevel 1 exit /b 1
goto done

:build_clang
where ar.exe >nul 2>nul
if errorlevel 1 (
	echo ar.exe not found. Install binutils or use an MSVC toolchain, then rerun this script.
	exit /b 1
)
for %%F in ("%LIB_ROOT%src\*.c") do (
	clang.exe -std=c99 -O2 -Wall -Wextra -I"%LIB_ROOT%include" -I"%LIB_ROOT%src" -c "%%~fF" -o "%OUT_DIR%\%%~nF.o"
	if errorlevel 1 exit /b 1
)
set "OBJ_LIST="
for %%O in ("%OUT_DIR%\*.o") do set OBJ_LIST=!OBJ_LIST! "%%~fO"
ar.exe rcs "%OUT_DIR%\control_algo.lib" !OBJ_LIST!
if errorlevel 1 exit /b 1
goto done

:done
echo Built "%OUT_DIR%\control_algo.lib"
endlocal
