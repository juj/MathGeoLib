cmake_minimum_required(VERSION 3.0)

if (CMAKE_VERSION VERSION_GREATER 3.0.99999) # Want to say "version >= 3.1", but no idea how to achieve that otherwise.
	cmake_policy(SET CMP0054 NEW) # Don't expand vars inside quotes, i.e. don't expand a line of form: ''if ("MSVC")'', but require ''if ("${MSVC}")''
endif()

project(MathGeoLib)

set(MATHGEOLIB_MAJOR_VERSION 1)
set(MATHGEOLIB_MINOR_VERSION 5)
set(MATHGEOLIB_PATCH_VERSION 0)
set(MATHGEOLIB_VERSION
  ${MATHGEOLIB_MAJOR_VERSION}.${MATHGEOLIB_MINOR_VERSION}.${MATHGEOLIB_PATCH_VERSION})

#Define cmake install path
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

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
	# Tests use C++11 syntax. (at least for now the main library does not need to)
	set (CMAKE_CXX_STANDARD 11)
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
	install (TARGETS MathGeoLib
	  EXPORT ${PROJECT_NAME}Targets
	  ARCHIVE DESTINATION lib
	  LIBRARY DESTINATION lib
	  RUNTIME DESTINATION bin)
	install (DIRECTORY "src/" DESTINATION "include/MathGeoLib" FILES_MATCHING PATTERN "*.h")
	install (DIRECTORY "src/" DESTINATION "include/MathGeoLib" FILES_MATCHING PATTERN "*.inl")

# Handle CMake Config
# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}/include")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${MATHGEOLIB_CMAKE_DIR}/${REL_INCLUDE_DIR}/${PROJECT_NAME}")
configure_file(${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

endif (NOT MATH_TESTS_EXECUTABLE)
