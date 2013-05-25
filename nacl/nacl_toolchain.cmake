if (NACL_SDK_ROOT)
	set(NACL_PATH ${NACL_SDK_ROOT} CACHE STRING "Native Client SDK Root Path")
elseif ("$ENV{NACL_SDK_ROOT}" STREQUAL "")
	message(STATUS "Environment variable NACL_SDK_ROOT was not set! Assuming NaCl SDK is found from C:/nacl_sdk")
	set(NACL_PATH "C:/nacl_sdk" CACHE STRING "Native Client SDK Root Path")
else()
	FILE(TO_CMAKE_PATH "$ENV{NACL_SDK_ROOT}" NACL_SDK_ROOTPATH)
	set(NACL_PATH ${NACL_SDK_ROOTPATH} CACHE STRING "Native Client SDK Root Path")
endif()

if (NOT NACL_TOOLCHAIN)
	message(STATUS "NaCl toolchain was not set. Using 'pepper_canary' as default. Specify -DNACL_TOOLCHAIN=pepper_xx to configure")
	set(NACL_TOOLCHAIN "pepper_canary")
endif()

set(NACL_TAG pnacl)
set(NACL_HOST win_x86_pnacl)

#set(NACL_TAG i686-nacl)
#if (APPLE)
#	set(NACL_HOST mac_x86)
#else()
#	set(NACL_HOST win_x86_newlib)
#endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_FIND_ROOT_PATH ${NACL_PATH}/${NACL_TOOLCHAIN}/toolchain/${NACL_HOST}/nacl)

SET (CMAKE_C_OUTPUT_EXTENSION_REPLACE 1)
SET (CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

#if (NACL_64BIT) # Set NACL_64BIT on the commandline when invoking cmake to control this.
#  set(NACL_ARCH "-m64")
#  add_definitions("-m64")
#else()
#  set(NACL_ARCH "-m32")
#  add_definitions("-m32")
#endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NACL_ARCH}")
set(CMAKE_C_FLAGS_RELEASE "-O2 ${NACL_ARCH}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NACL_ARCH}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 ${NACL_ARCH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${NACL_ARCH}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${NACL_ARCH}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${NACL_ARCH}")

#set(CMAKE_ASM_COMPILER ${NACL_PATH}/${NACL_TOOLCHAIN}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-as.exe)
#set(CMAKE_C_COMPILER   ${NACL_PATH}/${NACL_TOOLCHAIN}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-gcc.exe)
#set(CMAKE_CXX_COMPILER ${NACL_PATH}/${NACL_TOOLCHAIN}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-g++.exe)

set(TOOLPATH "${NACL_PATH}/${NACL_TOOLCHAIN}/toolchain/${NACL_HOST}/newlib/bin")
set(CMAKE_AR "${TOOLPATH}/${NACL_TAG}-ar.bat" CACHE FILEPATH "PNaCl ar")
set(CMAKE_RANLIB "${TOOLPATH}/${NACL_TAG}-ranlib.bat" CACHE FILEPATH "PNaCl ranlib")
set(CMAKE_ASM_COMPILER "${TOOLPATH}/${NACL_TAG}-as.bat")
set(CMAKE_C_COMPILER "${TOOLPATH}/${NACL_TAG}-clang.bat")
set(CMAKE_CXX_COMPILER "${TOOLPATH}/${NACL_TAG}-clang++.bat")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set a global NACL variable that can be used in client CMakeLists.txt to detect when building using NaCl.
SET(NACL 1)

#add_definitions(-DPEPPER)
#add_definitions(-DNACL)
include_directories(${NACL_PATH}/${NACL_TOOLCHAIN}/include)

message(STATUS "Assuming we are building PNaCl executable!") # TODO: Allow configuring this!
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	link_directories(${NACL_PATH}/${NACL_TOOLCHAIN}/lib/pnacl/Debug)
else()
	link_directories(${NACL_PATH}/${NACL_TOOLCHAIN}/lib/pnacl/Release)
endif()

# Skip the platform compiler checks for cross compiling
set (CMAKE_CXX_COMPILER_WORKS TRUE)
set (CMAKE_C_COMPILER_WORKS TRUE)
