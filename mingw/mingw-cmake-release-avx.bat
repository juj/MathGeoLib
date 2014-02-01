set OLDPATH=%PATH%
set PATH=%PATH:cygwin=cygwin_unused%
set PATH=%PATH:git=git_unused%
call g++ --version
del CMakeCache.txt
cmake -DMATH_TESTS_EXECUTABLE=1 -DMINGW=1 -DMATH_AVX=1 -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" .. %*
set PATH=%OLDPATH%
pause
