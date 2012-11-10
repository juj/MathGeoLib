if ($ENV{NACL_SDK_ROOT} STREQUAL "")
	message(STATUS "Environment variable NACL_SDK_ROOT was not set! Assuming NaCl SDK is found from C:/nacl_sdk/pepper_19")
	set(NACL_PATH "C:/nacl_sdk/pepper_19" CACHE STRING "Native Client SDK Root Path")
else()
	set(NACL_PATH $ENV{NACL_SDK_ROOT} CACHE STRING "Native Client SDK Root Path")
endif()

set(NACL_TAG i686-nacl)
if (APPLE)
	set(NACL_HOST mac_x86)
else()
	set(NACL_HOST win_x86_newlib)
endif()

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_FIND_ROOT_PATH ${NACL_PATH}/toolchain/${NACL_HOST}/nacl)

SET (CMAKE_C_OUTPUT_EXTENSION_REPLACE 1)
SET (CMAKE_CXX_OUTPUT_EXTENSION_REPLACE 1)

if (NACL_64BIT) # Set NACL_64BIT on the commandline when invoking cmake to control this.
  set(NACL_ARCH "-m64")
  add_definitions("-m64")
else()
  set(NACL_ARCH "-m32")
  add_definitions("-m32")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NACL_ARCH}")
set(CMAKE_C_FLAGS_RELEASE "-O2 ${NACL_ARCH}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NACL_ARCH}")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 ${NACL_ARCH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${NACL_ARCH}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${NACL_ARCH}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${NACL_ARCH}")

set(CMAKE_ASM_COMPILER ${NACL_PATH}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-as.exe)
set(CMAKE_C_COMPILER   ${NACL_PATH}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-gcc-4.4.3.exe)
set(CMAKE_CXX_COMPILER ${NACL_PATH}/toolchain/${NACL_HOST}/bin/${NACL_TAG}-c++.exe)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Set a global NACL variable that can be used in client CMakeLists.txt to detect when building using NaCl.
SET(NACL 1)

add_definitions(-DPEPPER)
add_definitions(-DNACL)
