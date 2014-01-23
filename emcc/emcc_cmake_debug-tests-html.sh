#!/bin/bash

emconfigure cmake -DEMSCRIPTEN=1 -DMATH_TESTS_EXECUTABLE=1 -DFAIL_USING_EXCEPTIONS=1 -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ..

