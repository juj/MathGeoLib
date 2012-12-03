#!/bin/bash

rm CMakeCache.txt
cmake -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version

