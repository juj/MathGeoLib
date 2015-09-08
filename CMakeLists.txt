cmake_minimum_required(VERSION 2.8)

if (CMAKE_VERSION VERSION_GREATER 3.0.99999) # Want to say "version >= 3.1", but no idea how to achieve that otherwise.
	cmake_policy(SET CMP0054 NEW) # Don't expand vars inside quotes, i.e. don't expand a line of form: ''if ("MSVC")'', but require ''if ("${MSVC}")''
endif()

project(MathGeoLib)

include(CommonOptions.cmake)

# N.B. These exist for inter-opping to other libraries written by the author.
# Feel free to remove. 
include_directories(../CodeLib/src)
include_directories(../GraphicsEngine/src)

file(GLOB_RECURSE sourceFiles ./src/*.cpp ./src/*.c)
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
	if (IOS)
		file(GLOB_RECURSE testObjCFiles ./tests/*.m)
		set(sourceFiles ${sourceFiles} ${testObjCFiles})
	endif()
endif()

if (MATH_TESTS_EXECUTABLE)
	add_executable(MathGeoLib ${sourceFiles} ${headerFiles})
	if (EMSCRIPTEN)
		set_target_properties(MathGeoLib PROPERTIES LINK_FLAGS "--js-library ${CMAKE_CURRENT_LIST_DIR}/src/emscripten/library_system.js -g")
	endif()
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

# install lib and header files if testing is disabled
if (NOT MATH_TESTS_EXECUTABLE)
	install (TARGETS MathGeoLib DESTINATION "lib")
	install (DIRECTORY "src/" DESTINATION "include/MathGeoLib" FILES_MATCHING PATTERN "*.h")
	install (DIRECTORY "src/" DESTINATION "include/MathGeoLib" FILES_MATCHING PATTERN "*.inl")
endif (NOT MATH_TESTS_EXECUTABLE)
