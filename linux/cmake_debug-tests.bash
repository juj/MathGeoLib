#!/bin/bash

rm CMakeCache.txt
cmake -DMATH_TESTS_EXECUTABLE=1 -DLINUX=1 -DCMAKE_BUILD_TYPE=Debug -DBUILD_FOR_GCOV=1 -G "Unix Makefiles" ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version
