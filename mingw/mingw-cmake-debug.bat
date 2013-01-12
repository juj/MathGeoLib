set OLDPATH=%PATH%
set PATH=%PATH:cygwin=cygwin_unused%
set PATH=%PATH:git=git_unused%
call g++ --version
del CMakeCache.txt
cmake -DMINGW=1 -G "MinGW Makefiles" .. %1 %2 %3 %4 %5 %6 %7 %8 %9
set PATH=%OLDPATH%
pause