# This file is a 'toolchain description file' for CMake.
# It teaches CMake about the Emscripten compiler, so that CMake can generate Unix Makefiles
# from CMakeLists.txt that invoke emcc.

# the name of the target operating system
SET(CMAKE_SYSTEM_NAME Emscripten)

# Specify the compilers to use for C and C++
SET(CMAKE_C_COMPILER emccstub)
SET(CMAKE_CXX_COMPILER emccstub)

# Specify the program to use when building static libraries.
SET(CMAKE_CXX_ARCHIVE_CREATE "emccstub -o <TARGET> -emit-llvm <LINK_FLAGS> <OBJECTS>")
SET(CMAKE_C_ARCHIVE_CREATE "emccstub -o <TARGET> -emit-llvm <LINK_FLAGS> <OBJECTS>")

SET(CMAKE_C_OUTPUT_EXTENSION_REPLACE 1)
SET(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

# Set a global EMSCRIPTEN variable that can be used in client CMakeLists.txt to detect when building using Emscripten.
SET(EMSCRIPTEN 1)
