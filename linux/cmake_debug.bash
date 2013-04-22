#!/bin/bash

rm CMakeCache.txt
cmake -DLINUX=1 -DCMAKE_BUILD_TYPE=Debug $* -G "Unix Makefiles" ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version
