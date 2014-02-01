set OLDPATH=%PATH%
set PATH=%PATH:cygwin=cygwin_unused%
set PATH=%PATH:git=git_unused%
call g++ --version
del CMakeCache.txt
set LLVM_PATH=C:/Projects/emsdk_uqm/clang/3.2_64bit/bin
cmake -DGENERATE_ASM_LISTING=TRUE -DCMAKE_C_COMPILER=%LLVM_PATH%/clang.exe -DCMAKE_CXX_COMPILER=%LLVM_PATH%/clang++.exe -DMATH_AVX=1 -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" .. %*
set PATH=%OLDPATH%
pause
