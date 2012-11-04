if (WIN32)
	add_definitions(/EHsc /arch:SSE2 -D_CRT_SECURE_NO_WARNINGS)
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

if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  # Update if necessary
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic -Wno-variadic-macros")
endif()
