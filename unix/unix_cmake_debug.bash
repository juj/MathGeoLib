#!/bin/bash
cmake -G -DCMAKE_BUILD_TYPE=Debug "Unix Makefiles" ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version
