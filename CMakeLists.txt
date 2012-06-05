cmake_minimum_required(VERSION 2.8)
project(MathGeoLib)

include(CommonOptions.cmake)

include_directories(./src)

include_directories(../CodeLib/src)
include_directories(../GraphicsEngine/src)

file(GLOB_RECURSE sourceFiles ./src/*.cpp)
file(GLOB_RECURSE headerFiles ./src/*.h)

add_library(MathGeoLib STATIC ${sourceFiles} ${headerFiles})
set_target_properties(MathGeoLib PROPERTIES 
	ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib
	LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

