cmake_minimum_required(VERSION 2.8)
project(MathGeoLib)

include(CommonOptions.cmake)

# If requested from the command line, run Visual Studio 2012 static code analysis. Warning: this is very slow!
if (MSVC11 AND RUN_VS2012_ANALYZE)
	add_definitions(/analyze)
endif()

if (BUILD_FOR_GCOV)
	add_definitions(-DFAIL_USING_EXCEPTIONS=1)
endif()

if (MATH_TESTS_EXECUTABLE)
	add_definitions(-DMATH_TESTS_EXECUTABLE)
endif()

if (MINGW AND BUILD_FOR_GCOV)
	add_definitions(-fprofile-arcs -ftest-coverage -DFAIL_USING_EXCEPTIONS=1)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
	set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs")
	set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs")
endif()

include_directories(./src)

include_directories(../CodeLib/src)
include_directories(../GraphicsEngine/src)

file(GLOB_RECURSE sourceFiles ./src/*.cpp)
file(GLOB_RECURSE headerFiles ./src/*.h)

if (BUILD_FOR_GCOV)

	file(REMOVE "MathGeoLib_code_files.txt")
	foreach(filename ${sourceFiles})	
		message(STATUS "${filename}")
		get_filename_component(filebasename ${filename} NAME)
		file(APPEND "MathGeoLib_code_files.txt" "${filebasename} ")
	endforeach()

	file(GLOB_RECURSE testSourceFiles ./tests/*.cpp)
	file(GLOB_RECURSE testHeaderFiles ./tests/*.h)
	set(sourceFiles ${sourceFiles} ${testSourceFiles})
	set(headerFiles ${headerFiles} ${testHeaderFiles})
endif()

if (MATH_TESTS_EXECUTABLE)
	add_executable(MathGeoLib ${sourceFiles} ${headerFiles})
else()
	add_library(MathGeoLib STATIC ${sourceFiles} ${headerFiles})
endif()

if (LINUX)
	# clock_gettime() is found from the library librt on linux. 
	target_link_libraries(MathGeoLib rt)
endif()

if (WIN8RT)
	set_target_properties(MathGeoLib PROPERTIES VS_WINRT_EXTENSIONS TRUE)
endif()
