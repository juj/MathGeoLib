#!/bin/bash
cmake -G -DCMAKE_BUILD_TYPE=Release "Unix Makefiles" ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version
