call g++ --version
del CMakeCache.txt
cmake -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release .. %1 %2 %3 %4 %5 %6 %7 %8 %9
