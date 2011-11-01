cmake_minimum_required(VERSION 2.8)
project(MathGeoLib)

add_definitions(-DMATH_ENABLE_STL_SUPPORT)
include_directories(./src)
file(GLOB_RECURSE sourceFiles1 ./src/Algorithm/*.cpp)
file(GLOB_RECURSE sourceFiles2 ./src/Math/*.cpp)
file(GLOB_RECURSE sourceFiles2 ./src/Geometry/*.cpp)
add_library(MathGeoLib STATIC ${sourceFiles1} ${sourceFiles2} ${sourceFiles3})
