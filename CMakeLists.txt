cmake_minimum_required(VERSION 2.8)
project(MathGeoLib)

# Undef WIN32 when Windows is only used as a host system
if (EMSCRIPTEN OR NACL OR ANDROID OR FLASCC)
	SET(WIN32)
endif()

include(CommonOptions.cmake)

# If requested from the command line, run Visual Studio 2012 static code analysis. Warning: this is very slow!
if (MSVC11 AND RUN_VS2012_ANALYZE)
	add_definitions(/analyze)
endif()

if (FAIL_USING_EXCEPTIONS)
	add_definitions(-DFAIL_USING_EXCEPTIONS)

	if (EMSCRIPTEN)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
		set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
	endif()
endif()

if (MATH_TESTS_EXECUTABLE)
	add_definitions(-DMATH_TESTS_EXECUTABLE)

	if (BUILD_FOR_GCOV)
		add_definitions(-fprofile-arcs -ftest-coverage)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs")
		set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs")
	endif()
endif()

# N.B. These exist for inter-opping to other libraries written by the author.
# Feel free to remove. 
include_directories(../CodeLib/src)
include_directories(../GraphicsEngine/src)

file(GLOB_RECURSE sourceFiles ./src/*.cpp)
file(GLOB_RECURSE headerFiles ./src/*.h)

if (BUILD_FOR_GCOV)
	file(REMOVE "MathGeoLib_code_files.txt")
	foreach(filename ${sourceFiles})
		get_filename_component(filebasename ${filename} NAME)
		file(APPEND "MathGeoLib_code_files.txt" "${filebasename} ")
	endforeach()
endif()

if (BUILD_FOR_GCOV OR MATH_TESTS_EXECUTABLE OR FAIL_USING_EXCEPTIONS) # TODO: Use a separate MATH_INCLUDE_TESTS?
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
	# Ignore warning LNK4264: archiving object file compiled with /ZW into a static library; note that when authoring Windows Runtime types it is not recommended to link with a static library that contains Windows Runtime metadata
	#    since we are not authoring any WinRT components from this code. 
	set_target_properties(MathGeoLib PROPERTIES STATIC_LIBRARY_FLAGS "/ignore:4264")
endif()
