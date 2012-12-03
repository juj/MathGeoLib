@echo off
SET OLDPATH=%PATH%
SET PATH=%PATH:cygwin=cygwin_unused%
echo mingw32-make --version
mingw32-make --version
echo.
echo gcc --version
gcc --version
mingw32-make
SET PATH=%OLDPATH%
pause
