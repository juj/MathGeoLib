if (CMAKE_C_COMPILER MATCHES ".*(clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(Clang|emcc).*")
	set(COMPILER_IS_CLANG TRUE)
endif()

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER MATCHES ".*(gcc|clang|emcc).*" OR CMAKE_C_COMPILER_ID MATCHES ".*(GCC|Clang|emcc).*")
	set(IS_GCC_LIKE TRUE)
else()
	set(IS_GCC_LIKE FALSE)
endif()

if (IS_GCC_LIKE AND NOT COMPILER_IS_CLANG)
	set(COMPILER_IS_GCC TRUE)
endif()

# Undef WIN32 when Windows is only used as a host system
if (EMSCRIPTEN OR NACL OR ANDROID OR FLASCC)
	SET(WIN32)
endif()

option(GENERATE_ASM_LISTING "Generate assembly listing of all compiled code" FALSE)

if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux" AND NOT EMSCRIPTEN AND NOT NACL AND NOT ANDROID AND NOT FLASCC)
	set(LINUX TRUE)
endif()

if (WIN32 AND IS_GCC_LIKE)
	add_definitions(-DWIN32)
endif()

# Add the global _DEBUG flag from WIN32 platform to all others, which is universally used in MGL to
# perform debug-mode-specific compilation.
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -D_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")

if (MINGW)
	set(CMAKE_C_FLAGS_RELEASE   "-O3")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

if (IOS)
	add_definitions(-DAPPLE_IOS)
endif()

set(optFlags "-DMATH_ENABLE_INSECURE_OPTIMIZATIONS -DNDEBUG -DMATH_SILENT_ASSUME -DRELEASE -DOPTIMIZED_RELEASE")

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
	set(relFlags "/Ox /Ob2 /Oi /Ot /GT /GF /GS- /fp:fast /fp:except- /MP")

	# Set up flags that affect ABI and linking to other projects as well, but only in unit test runner:
	if (MATH_TESTS_EXECUTABLE)
		set(relFlags "${relFlags} /MT")

		# Disable all forms of MSVC debug iterator checking in new and old Visual Studios.
		set(relFlags "${relFlags} /D_SECURE_SCL=0 /D_SCL_SECURE_NO_WARNINGS /D_ITERATOR_DEBUG_LEVEL=0 /D_HAS_ITERATOR_DEBUGGING=0")
	endif()

	# Since Visual Studio 2012 the IDE has an option: Secure Development Lifecycle (SDL) flags: No (/sdl-)
	# but that is implied by /GS- already above, so no need to set that.

	# Disable Incremental Linking (/INCREMENTAL:NO) This is incompatible with LTCG, but RelWithDebInfo has this default on.
	# Perform identical COMDAT folding (/OPT:ICF)
	set(relLinkFlags "/OPT:ICF /INCREMENTAL:NO")
	if (NOT GENERATE_ASM_LISTING) # When outputting assembly we want to have all functions in, independent of whether they're used.
		# Remove unreferenced data (/OPT:REF)
		set(relLinkFlags "${relLinkFlags} /OPT:REF")
	endif()

	if (MSVC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 18.0.31101.0)
		# XXX Work around MSVC bug with x64 + /GL + /O2 /arch:AVX, see https://connect.microsoft.com/VisualStudio/feedback/details/814682/visual-studio-2013-x64-compiler-generates-faulty-code-with-gl-o2-arch-avx-flags-enabled
		if (MATH_AVX AND CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(VS_BUG TRUE)
			message(STATUS "NOTE: Whole Program Optimization is disabled due to detected MSVC bug with x64+/O2+/GL+/arch:AVX! Install VS2013 Update 4 or newer to avoid this issue.") # First fix was actually in VS2013 Update 2 already, but don't know what the version of that compiler was.
		endif()
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

	if (GENERATE_ASM_LISTING)
		set(outputAsmCodeFlags "/FAs /Fa$(IntDir) /GL")
		set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${outputAsmCodeFlags}")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${outputAsmCodeFlags}")
		set(outputAsmCodeLinkFlags "/OPT:NOREF /LTCG")
		set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")
		set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")
		set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${outputAsmCodeLinkFlags}")
	endif()
else()
	if (IS_GCC_LIKE AND GENERATE_ASM_LISTING)
		# -fkeep-inline-functions: Inline everything, but also keep a separate inlined copy for asm outputting purposes.
		# Add -ffast-math and -fno-math-errno
		set(outputAsmCodeFlags "-S -fkeep-inline-functions -fverbose-asm -g")

		# Prefer outputting Intel syntax for assembly.
		if (COMPILER_IS_CLANG)
			set(outputAsmCodeFlags "${outputAsmCodeFlags} -mllvm --x86-asm-syntax=intel")
		else()
			set(outputAsmCodeFlags "${outputAsmCodeFlags} -masm=intel -Wa,-alnd") # GCC
		endif()

		# To interleave source code, run 'as -alhnd file.s > file.lst'
		set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} ${outputAsmCodeFlags}")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${outputAsmCodeFlags}")
	endif()
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

	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s PRECISE_F32=2 --separate-asm")
	SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -s ASSERTIONS=2 -s SAFE_HEAP=1 -s GL_ASSERTIONS=1")
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

# MATH_NO_WINVER: Set this to TRUE to have MathGeoLib build to not define the Windows version to target itself.
# MATH_WINVER: Set this to a custom Windows version that MathGeoLib should target.
if (MINGW AND NOT MATH_NO_WINVER)
	if (NOT MATH_WINVER)
		set(MATH_WINVER 0x0501)
	endif()
	# Require Windows XP.
	# See http://msdn.microsoft.com/en-us/library/6sehtctf.aspx
	add_definitions(-DWINVER=${MATH_WINVER})
	add_definitions(-D_WIN32_WINNT=${MATH_WINVER})
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

if (COMPILER_IS_GCC)
	if (MATH_SSE OR MATH_SSE2 OR MATH_SSE3 OR MATH_SSE41 OR MATH_AVX)
		add_definitions(-mfpmath=sse)
	endif()
endif()

if (MATH_FMA4 OR MATH_FMA3)
	# Between FMA3 and FMA4, the intrinsics are the same so C code doesn't need to know which to call,
	# it can just call _mm_fmadd_ps(), so this passed #define doesn't need to distinguish.
	add_definitions(-DMATH_FMA)
	# However for GCC codegen, it needs to know which instruction set to target:
	if (IS_GCC_LIKE)
		# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/i386-and-x86-64-Options.html#i386-and-x86-64-Options
		if (MATH_FMA4)
			add_definitions(-mfma4)
		else()
			add_definitions(-mfma)
		endif()
	endif()
endif()

if (MATH_AVX)
	add_definitions(-DMATH_AVX)
	if (MSVC)
		add_definitions(/arch:AVX)
	elseif (IS_GCC_LIKE)
		# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/i386-and-x86-64-Options.html#i386-and-x86-64-Options
		add_definitions(-mavx -march=corei7-avx -mtune=corei7-avx)
	endif()
elseif (MATH_SSE41)
	add_definitions(-DMATH_SSE41)
	if (MSVC AND MSVC_VERSION LESS 1800) # VS2013 no longer has /arch:SSE2, it's always enabled.
		add_definitions(/arch:SSE2) # No equivalent for Visual Studio, after SSE2, arch jumps to AVX.
	elseif (IS_GCC_LIKE)
		add_definitions(-msse4.1)
		# Note: corei7 also requires SSE 4.2
		if (NOT EMSCRIPTEN)
			add_definitions(-march=corei7 -mtune=corei7)
		endif()
	endif()
elseif (MATH_SSE3)
	add_definitions(-DMATH_SSE3)
	if (MSVC AND MSVC_VERSION LESS 1800) # VS2013 no longer has /arch:SSE2, it's always enabled.
		add_definitions(/arch:SSE2) # No equivalent for Visual Studio, after SSE2, arch jumps to AVX.
	elseif (IS_GCC_LIKE)
		add_definitions(-msse3)
		if (NOT EMSCRIPTEN)
			add_definitions(-march=core2 -mtune=core2)
		endif()
	endif()
elseif (MATH_SSE2)
	add_definitions(-DMATH_SSE2)
	if (MSVC AND MSVC_VERSION LESS 1800) # VS2013 no longer has /arch:SSE2, it's always enabled.
		add_definitions(/arch:SSE2)
	elseif (IS_GCC_LIKE)
		add_definitions(-msse2)
		if (NOT EMSCRIPTEN)
			add_definitions(-march=pentium4 -mtune=pentium4)
		endif()
	endif()
elseif (MATH_SSE)
	add_definitions(-DMATH_SSE)
	if (MSVC)
		add_definitions(/arch:SSE)
	elseif (IS_GCC_LIKE)
		add_definitions(-msse)
		if (NOT EMSCRIPTEN)
			add_definitions(-march=pentium3 -mtune=pentium3)
		endif()
	endif()
endif()

if (MATH_ENABLE_UNCOMMON_OPERATIONS)
	add_definitions(-DMATH_ENABLE_UNCOMMON_OPERATIONS)
endif()

if (MATH_NEON)
	add_definitions(-DMATH_NEON)
	if (IS_GCC_LIKE)
		add_definitions(-mfpu=neon)
	endif()
endif()

if (MATH_AUTOMATIC_SSE)
	add_definitions(-DMATH_AUTOMATIC_SSE)
endif()

# If requested from the command line, run Visual Studio 2012 static code analysis. Warning: this is very slow!
if (MSVC11 AND RUN_VS2012_ANALYZE)
	add_definitions(/analyze)
endif()

if (FAIL_USING_EXCEPTIONS)
	set(CMAKE_C_FLAGS_DEBUG     "${CMAKE_C_FLAGS_DEBUG} -DFAIL_USING_EXCEPTIONS")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DFAIL_USING_EXCEPTIONS")

	if (EMSCRIPTEN)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
		set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -s DISABLE_EXCEPTION_CATCHING=0")
	endif()
endif()

if (MATH_TESTS_EXECUTABLE)
	add_definitions(-DMATH_TESTS_EXECUTABLE)

	if (BUILD_FOR_GCOV)
		if (IS_GCC_LIKE)
			add_definitions(-fprofile-arcs -ftest-coverage)
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
			set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs")
			set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs")
		endif()
	endif()
endif()
