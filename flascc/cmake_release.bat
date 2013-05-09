@echo off
del cmakecache.txt

set PTH=%PATH%
:: Make sure no custom cygwins are present in PATH. 
set PATH=%PATH:cygwin=cygwin_unused%
:: git cannot be present in PATH when mingw32-make is invoked! Or otherwise it fails with some DLL confusion!
set PATH=%PATH:git=git_unused%

echo Assuming FlasCC is at C:\FlasCC_1.0.1
set PATH=%PATH%;C:\FlasCC_1.0.1\cygwin\bin
cmake -DFLASCC=1 -DFLASCC_BUILD_EXE=1 -DCMAKE_CXX_COMPILER_WORKS=TRUE -DCMAKE_C_COMPILER_WORKS=TRUE -DCMAKE_C_COMPILER=C:/FlasCC_1.0.1/sdk/usr/bin/gcc.exe -DCMAKE_CXX_COMPILER=C:/FlasCC_1.0.1/sdk/usr/bin/g++.exe -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=mingw32-make -DMATH_TESTS_EXECUTABLE=1 -G "MinGW Makefiles" ..
set PATH=%PTH%
set PTH=

pause

