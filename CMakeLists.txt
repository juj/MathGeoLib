cmake_minimum_required(VERSION 2.8)
project(MathGeoLib)

include(CommonOptions.cmake)

# If requested from the command line, run Visual Studio 2012 static code analysis. Warning: this is very slow!
if (MSVC11 AND RUN_VS2012_ANALYZE)
	add_definitions(/analyze)
endif()

include_directories(./src)

include_directories(../CodeLib/src)
include_directories(../GraphicsEngine/src)

file(GLOB_RECURSE sourceFiles ./src/*.cpp)
file(GLOB_RECURSE headerFiles ./src/*.h)

add_library(MathGeoLib STATIC ${sourceFiles} ${headerFiles})

