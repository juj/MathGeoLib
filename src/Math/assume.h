/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file assume.h
       @author Jukka Jylänki
	@brief Global compilation defines. */
#pragma once

#include "../MathBuildConfig.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <sstream>
#include <string>
#endif
#include "MathNamespace.h"
#include <stdio.h>
#include "myassert.h"
#include "MathLog.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Container/UString.h"
#endif

#ifndef MARK_UNUSED
/// If a variable is labelled with this directive, the compiler should not emit a warning even if it is unused in the code.
#define MARK_UNUSED(...) ((void)((void)__VA_ARGS__))
#endif

#ifdef __GNUC__
/// If a variable or a function definition is labelled with this directive, the compiler should not emit a warning even if it is unused
/// in the code.
#define DONT_WARN_UNUSED __attribute__((unused))
#else
#define DONT_WARN_UNUSED
#endif

#ifndef ARRAY_LENGTH
#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))
#endif

// MathGeoLib uses three types of runtime condition check macros:
//  - assert(): the regular assert() check, that compiles out in release builds (when NDEBUG or OPTIMIZED_RELEASE is defined).
//              Execution is aborted if an assert condition fails.
//  - assume(): Performs a runtime condition check. If the check fails, prints out an error log entry. If MATH_ASSERT_ON_ASSUME
//              is defined, behaves like assert(). (otherwise execution continues). If MathBreakOnAssume() is enabled, executes
//              a statement on failure to invoke the system debugger. Unlike assert(), assume() macro checks are present in NDEBUG
//              builds, but disabled in MATH_SILENT_ASSUME and OPTIMIZED_RELEASE modes.
//  - mathassert(): Similar to assert(), but used internally by MathGeoLib to verify programming errors inside MathGeoLib implementation
//              itself. MathAsserts are enabled if building with MATH_ASSERT_CORRECTNESS, otherwise disabled.
// The assume() macro is used to check preconditions on the math-related functions, e.g. whether vectors are normalized, check that division by zero doesn't occur, orthonormal bases, and so on.

// The assume() macro operates differently depending on which #defines are present:
// #define FAIL_USING_EXCEPTIONS - the assume() macro throws an exception
// #define BREAK_ON_ERROR_PRINTS - if an error message is printed (with LOGE()), it is treated as if an assume() had failed, breaking to debugger.
// #define BREAK_ON_WARNING_PRINTS - if a warning message is printed (with LOGW()), it is treated as if an assume() had failed, breaking to debugger.
//                                   Implies BREAK_ON_ERROR_PRINTS.
// #define MATH_ASSERT_ON_ASSUME - the assume() macro resolves to the assert() macro.
// #define MATH_STARTUP_BREAK_ON_ASSUME - MathGeoLib execution will start with MathBreakOnAssume() behavior enabled, i.e. assume() failures
//                                        will invoke the debugger. (this behavior can be controlled at runtime with SetMathBreakOnAssume(bool))
// #define MATH_SILENT_ASSUME   - the assume() macro is silent, and disabled altogether. (no prints or breaks or anything, the checks by assume() are ignored)
// If neither of the above is defined (default), then
//  - WIN32: if MathBreakOnAssume() == true, the system will break to debugger using a call to DebugBreak().
//  - Other: if MathBreakOnAssume() == true, the assume() macro is equal to the assert() macro.
//  -   All: if MathBreakOnAssume() == false, the assume() macro uses printf() to log warnings of failed math-related assumptions.

MATH_BEGIN_NAMESPACE

/// Assigns mathBreakOnAssume = isEnabled;
void SetMathBreakOnAssume(bool isEnabled);

/// Returns the current state of the math break-on-assume flag.
/// The default startup value for this flag is false.
bool MathBreakOnAssume();

/// Breaks to debugger if math break-on-assume flag
/// Returns the current state of the math break-on-assume flag.
bool AssumeFailed();

template<typename T>
inline StringT ObjToString(const T &obj)
{
	return obj.ToString();
}

template<>
inline StringT ObjToString<const char * const>(const char * const & obj)
{
	return StringT(obj);
}

template<>
inline StringT ObjToString<StringT>(const StringT &obj)
{
	return obj;
}

template<>
inline StringT ObjToString<float>(const float &obj)
{
#if defined(MATH_CONTAINERLIB_SUPPORT)
	return String::FromFloat(obj);
#else
	std::stringstream ss;
	ss << obj;
	return ss.str();
#endif
}

template<>
inline StringT ObjToString<double>(const double &obj)
{
#if defined(MATH_CONTAINERLIB_SUPPORT)
	return String::FromDouble(obj);
#else
	std::stringstream ss;
	ss << obj;
	return ss.str();
#endif
}

template<>
inline StringT ObjToString<int>(const int &obj)
{
#if defined(MATH_CONTAINERLIB_SUPPORT)
	return String::FromInt(obj);
#else
	std::stringstream ss;
	ss << obj;
	return ss.str();
#endif
}

template<>
inline StringT ObjToString<bool>(const bool &obj)
{
	return obj ? "true" : "false";
}

template<>
inline StringT ObjToString<u32>(const u32 &obj)
{
#if defined(MATH_CONTAINERLIB_SUPPORT)
	return String::FromUInt(obj);
#else
	std::stringstream ss;
	ss << obj;
	return ss.str();
#endif
}

template<>
inline StringT ObjToString<u64>(const u64 &obj)
{
#if defined(MATH_CONTAINERLIB_SUPPORT)
	return String::FromUInt64(obj);
#else
	std::stringstream ss;
	ss << obj;
	return ss.str();
#endif
}

MATH_END_NAMESPACE

#ifdef FAIL_USING_EXCEPTIONS
#include <stdexcept>
#define mgl_assume_failed(message) throw std::runtime_error((message))
#elif defined(MATH_ASSERT_ON_ASSUME)
#define mgl_assume(x) mgl_assert(x)
#define mgl_assume_failed(message) do { \
		LOGE("Assumption %s failed! in file %s, line %d!", message, __FILE__, __LINE__); \
		mgl_assert(false); \
	} while(0) 
#elif defined(MATH_SILENT_ASSUME)
#define mgl_assume(x) ((void)0)
#define mgl_assume_failed(message) ((void)0)
#else
#define mgl_assume_failed(message) do { \
		LOGE("Assumption \"%s\" failed! in file %s, line %d!", message, __FILE__, __LINE__); \
		AssumeFailed(); \
	} while (0)
#endif

#ifndef mgl_assume
#define mgl_assume(x) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			mgl_assume_failed(#x " in " __FILE__ ":" STRINGIZE(__LINE__)); \
	MULTI_LINE_MACRO_END
#endif

// In assume1-assume4, print1-print4 are additional helper parameters that get printed out to log in case of failure.
#define mgl_assume1(x, print1) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			mgl_assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define mgl_assert1 mgl_assume1

#define mgl_assume2(x, print1, print2) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			mgl_assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define mgl_assert2 mgl_assume2

#define mgl_assume3(x, print1, print2, print3) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			mgl_assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (", " #print3 ": ") + MATH_NS::ObjToString(print3) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define mgl_assert3 mgl_assume3

#define mgl_assume4(x, print1, print2, print3, print4) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			mgl_assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (", " #print3 ": ") + MATH_NS::ObjToString(print3) + \
			                  (", " #print4 ": ") + MATH_NS::ObjToString(print4) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define mgl_assert4 mgl_assume4

// If MATH_ASSERT_CORRECTNESS is defined, the function mathassert() is enabled to test
// that all forms of optimizations inside the math library produce proper results.
#ifdef MATH_ASSERT_CORRECTNESS
#define mgl_mathassert(x) mgl_assert(x)
#define mgl_mathassert1 mgl_assert1
#define mgl_mathassert2 mgl_assert2
#define mgl_mathassert3 mgl_assert3
#define mgl_mathassert4 mgl_assert4
#else
#define mgl_mathassert(x) ((void)0)
#define mgl_mathassert1(...) ((void)0)
#define mgl_mathassert2(...) ((void)0)
#define mgl_mathassert3(...) ((void)0)
#define mgl_mathassert4(...) ((void)0)
#endif

// Kill both mgl_assume() and mathassert() macros in OPTIMIZED_RELEASE builds.
#ifdef OPTIMIZED_RELEASE
#ifdef mgl_assume
#undef mgl_assume
#endif
#ifdef mgl_assume1
#undef mgl_assume1
#endif
#ifdef mgl_assume2
#undef mgl_assume2
#endif
#ifdef mgl_assume3
#undef mgl_assume3
#endif
#ifdef mgl_assume4
#undef mgl_assume4
#endif
#ifdef mgl_assert
#undef mgl_assert
#endif
#ifdef mgl_assert1
#undef mgl_assert1
#endif
#ifdef mgl_assert2
#undef mgl_assert2
#endif
#ifdef mgl_assert3
#undef mgl_assert3
#endif
#ifdef mgl_assert4
#undef mgl_assert4
#endif
#ifdef mgl_mathassert
#undef mgl_mathassert
#endif
#ifdef mgl_mathassert1
#undef mgl_mathassert1
#endif
#ifdef mgl_mathassert2
#undef mgl_mathassert2
#endif
#ifdef mgl_mathassert3
#undef mgl_mathassert3
#endif
#ifdef mgl_mathassert4
#undef mgl_mathassert4
#endif

#define mgl_assume(...) ((void)0)
#define mgl_assume1(...) ((void)0)
#define mgl_assume2(...) ((void)0)
#define mgl_assume3(...) ((void)0)
#define mgl_assume4(...) ((void)0)
#define mgl_assert(...) ((void)0)
#define mgl_assert1(...) ((void)0)
#define mgl_assert2(...) ((void)0)
#define mgl_assert3(...) ((void)0)
#define mgl_assert4(...) ((void)0)
#define mgl_mathassert(...) ((void)0)
#define mgl_mathassert1(...) ((void)0)
#define mgl_mathassert2(...) ((void)0)
#define mgl_mathassert3(...) ((void)0)
#define mgl_mathassert4(...) ((void)0)

#endif
