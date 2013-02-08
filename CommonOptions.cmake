if (WIN32 AND NOT MINGW)
	add_definitions(/EHsc -D_CRT_SECURE_NO_WARNINGS)
	if (NOT MSVC11) # Visual Studio 2012 always has SSE2 enabled, so the command line option no longer exists.
		add_definitions(/arch:SSE2)
	endif()
endif()

if (EMSCRIPTEN)
	add_definitions(-emit-llvm)
	SET(CMAKE_LINK_LIBRARY_SUFFIX "")

	SET(CMAKE_STATIC_LIBRARY_PREFIX "")
	SET(CMAKE_STATIC_LIBRARY_SUFFIX ".bc")
	SET(CMAKE_SHARED_LIBRARY_PREFIX "")
	SET(CMAKE_SHARED_LIBRARY_SUFFIX ".bc")
	SET(CMAKE_EXECUTABLE_SUFFIX ".html")
	SET(CMAKE_DL_LIBS "" )

	SET(CMAKE_FIND_LIBRARY_PREFIXES "")
	SET(CMAKE_FIND_LIBRARY_SUFFIXES ".bc")
endif()

if (FLASCC)
	SET(CMAKE_LINK_LIBRARY_SUFFIX "")

	SET(CMAKE_STATIC_LIBRARY_PREFIX "")
	SET(CMAKE_STATIC_LIBRARY_SUFFIX ".a")
	SET(CMAKE_SHARED_LIBRARY_PREFIX "")
	SET(CMAKE_SHARED_LIBRARY_SUFFIX ".a")
	SET(CMAKE_EXECUTABLE_SUFFIX ".swf")
	SET(CMAKE_DL_LIBS "" )

	SET(CMAKE_FIND_LIBRARY_PREFIXES "")
	SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
endif()

if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER MATCHES ".*(gcc|clang|emcc).*")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Wno-variadic-macros")
else()
  message(WARNING "Unknown compiler '${CMAKE_C_COMPILER}' used! Cannot set up max warning level.")
endif()
