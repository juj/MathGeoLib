# If true, output assembly source code (.asm) for examination.
# Set this in calling code
# SET(GENERATE_ASM_LISTING TRUE)

set(optFlags "-DMATH_ENABLE_INSECURE_OPTIMIZATIONS")
set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${optFlags}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${optFlags}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO     "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${optFlags}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${optFlags}")

if (MSVC)
	# Exception handling model: Catch C++ exceptions only, assume that "extern C" functions will never throw a C++ exception.
	add_definitions(/EHsc)
	# Ignore noisy VS warnings that complain about bad std library functions (for now?)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)

	# Perform extremely aggressive optimization on Release builds:
	# Flags on Visual Studio 2010 and newer:
	# Runtime library: Multi-threaded (/MT) as opposed to default 'Multi-threaded DLL' - static runtime library allows better LTCG inlining opportunities.
	# Optimization: Full Optimization (/Ox)
	# Inline Function Expansion: Any Suitable (/Ob2)
	# Enable Intrinsic Functions: Yes (/Oi)
	# Favor Size Or Speed: Favor fast code (/Ot)
	# Enable Fiber-Safe Optimizations: Yes (/GT)
	# Enable String Pooling: Yes (/GF)
	# Buffer Security Check: No (/GS-)
	# Floating Point Model: Fast (/fp:fast)
	# Enable Floating Point Exceptions: No (/fp:except-)
	# Build with Multiple Processes (/MP)
	set(relFlags "/MT /Ox /Ob2 /Oi /Ot /GT /GF /GS- /fp:fast /fp:except- /MP")

	# Disable all forms of MSVC debug iterator checking in new and old Visual Studios.
	set(relFlags "${relFlags} /D_SECURE_SCL=0 /D_SCL_SECURE_NO_WARNINGS /D_ITERATOR_DEBUG_LEVEL=0 /D_HAS_ITERATOR_DEBUGGING=0")

	# Since Visual Studio 2012 the IDE has an option: Secure Development Lifecycle (SDL) flags: No (/sdl-)
	# but that is implied by /GS- already above, so no need to set that.

	# Disable Incremental Linking (/INCREMENTAL:NO) This is incompatible with LTCG, but RelWithDebInfo has this default on.
	# Perform identical COMDAT folding (/OPT:ICF)
	set(relLinkFlags "/OPT:ICF /INCREMENTAL:NO")
	if (NOT GENERATE_ASM_LISTING) # When outputting assembly we want to have all functions in, independent of whether they're used.
		# Remove unreferenced data (/OPT:REF)
		set(relLinkFlags "${relLinkFlags} /OPT:REF")
	endif()

	# XXX Work around MSVC bug with x64 + /GL + /O2 /arch:AVX, see https://connect.microsoft.com/VisualStudio/feedback/details/814682/visual-studio-2013-x64-compiler-generates-faulty-code-with-gl-o2-arch-avx-flags-enabled
	if (MATH_AVX AND CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(VS_BUG TRUE)
		message(STATUS "NOTE: Whole Program Optimization is disabled due to detected MSVC bug with x64+/O2+/GL+/arch:AVX!")
	endif()
	if (NOT VS_BUG)
		# Whole Program Optimization: Yes (/GL)
		set(relFlags "${relFlags} /GL")
		# Link-time Code Generation (/LTCG)
		set(relLinkFlags "${relLinkFlags} /LTCG")
	endif()

	# Omit Frame Pointers: Yes (/Oy)
	set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} /Oy ${relFlags}")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Oy ${relFlags}")

	# Don't omit frame pointers, but add Debug database (/Zi).
	set(CMAKE_C_FLAGS_RELWITHDEBINFO     "${CMAKE_C_FLAGS_RELWITHDEBINFO} /Zi ${relFlags}")
	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /Zi ${relFlags}")

	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${relLinkFlags}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${relLinkFlags}")
	set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${relLinkFlags}")

	set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} ${relLinkFlags}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} ${relLinkFlags}")
	set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO} ${relLinkFlags}")

	set(outputAsmCodeFlags "/FAs /Fa$(IntDir) /GL /DOPTIMIZED_RELEASE")
	set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${outputAsmCodeFlags}")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${outputAsmCodeFlags}")
	set(outputAsmCodeLinkFlags "/OPT:NOREF /LTCG")
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")
	set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")

else()
#	GCC 4.7.2 generates broken code that fails Float4Normalize4 test and others under -O3 -ffast-math, so don't do that.
#	set(OPT_FLAGS "-O3 -ffast-math")
#	set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${OPT_FLAGS}")
#	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OPT_FLAGS}")
#	set(CMAKE_C_FLAGS_RELWITHDEBINFO     "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
#	set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${OPT_FLAGS}")
endif()

if (EMSCRIPTEN)
	SET(CMAKE_LINK_LIBRARY_SUFFIX "")

	SET(CMAKE_STATIC_LIBRARY_PREFIX "")
	SET(CMAKE_STATIC_LIBRARY_SUFFIX ".bc")
	SET(CMAKE_SHARED_LIBRARY_PREFIX "")
	SET(CMAKE_SHARED_LIBRARY_SUFFIX ".bc")
	if (EMSCRIPTEN_BUILD_JS)
		SET(CMAKE_EXECUTABLE_SUFFIX ".js")
	else()
		SET(CMAKE_EXECUTABLE_SUFFIX ".html")
	endif()
	SET(CMAKE_DL_LIBS "" )

	SET(CMAKE_FIND_LIBRARY_PREFIXES "")
	SET(CMAKE_FIND_LIBRARY_SUFFIXES ".bc")
	
	add_definitions(-Wno-warn-absolute-paths)
endif()

if (FLASCC)
	SET(CMAKE_LINK_LIBRARY_SUFFIX "")

	SET(CMAKE_STATIC_LIBRARY_PREFIX "")
	SET(CMAKE_STATIC_LIBRARY_SUFFIX ".a")
	SET(CMAKE_SHARED_LIBRARY_PREFIX "")
	SET(CMAKE_SHARED_LIBRARY_SUFFIX ".a")
	if (FLASCC_BUILD_EXE)
		SET(CMAKE_EXECUTABLE_SUFFIX ".exe")
	else()
		SET(CMAKE_EXECUTABLE_SUFFIX ".swf")
	endif()
	SET(CMAKE_DL_LIBS "" )

	SET(CMAKE_FIND_LIBRARY_PREFIXES "")
	SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

	set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE} -O4")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O4")
endif()

if (LINUX)
	add_definitions(-DLINUX)
endif()

if (MINGW)
	# Require Windows XP.
	# See http://msdn.microsoft.com/en-us/library/6sehtctf.aspx
	add_definitions(-DWINVER=0x0501)
	add_definitions(-D_WIN32_WINNT=0x0501)
endif()

if(MSVC)
  # Force to always compile with W4
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif(CMAKE_C_COMPILER_ID MATCHES "PathScale")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long")
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER MATCHES ".*(gcc|clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(GCC|Clang|emcc).*")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -Wno-variadic-macros")
else()
  message(WARNING "Unknown compiler '${CMAKE_C_COMPILER}'/'${CMAKE_C_COMPILER_ID}' used! Cannot set up max warning level.")
endif()

if (MATH_AVX)
	add_definitions(-DMATH_AVX)
	if (MSVC)
		add_definitions(/arch:AVX)
	else()
		add_definitions(-mavx)
	endif()
endif()

if (MATH_SSE41)
	add_definitions(-DMATH_SSE41)
endif()

if (MATH_SSE3)
	add_definitions(-DMATH_SSE3)
endif()

if (MATH_SSE2)
	add_definitions(-DMATH_SSE2)
endif()

if (MATH_SSE)
	add_definitions(-DMATH_SSE)
endif()

if (MATH_NEON)
	add_definitions(-DMATH_NEON)
	if (NOT MSVC)
		add_definitions(-mfpu=neon)
	endif()
endif()

if (MATH_AUTOMATIC_SSE)
	add_definitions(-DMATH_AUTOMATIC_SSE)
endif()
