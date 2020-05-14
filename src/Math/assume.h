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
#define assume_failed(message) throw std::runtime_error((message))
#elif defined(MATH_ASSERT_ON_ASSUME)
#define assume(x) assert(x)
#define assume_failed(message) do { \
		LOGE("Assumption %s failed! in file %s, line %d!", message, __FILE__, __LINE__); \
		assert(false); \
	} while(0) 
#elif defined(MATH_SILENT_ASSUME)
#define assume(x) ((void)0)
#define assume_failed(message) ((void)0)
#else
#define assume_failed(message) do { \
		LOGE("Assumption \"%s\" failed! in file %s, line %d!", message, __FILE__, __LINE__); \
		AssumeFailed(); \
	} while (0)
#endif

#ifndef assume
#define assume(x) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			assume_failed(#x " in " __FILE__ ":" STRINGIZE(__LINE__)); \
	MULTI_LINE_MACRO_END
#endif

// In assume1-assume4, print1-print4 are additional helper parameters that get printed out to log in case of failure.
#define assume1(x, print1) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define assert1 assume1

#define assume2(x, print1, print2) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define assert2 assume2

#define assume3(x, print1, print2, print3) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (", " #print3 ": ") + MATH_NS::ObjToString(print3) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define assert3 assume3

#define assume4(x, print1, print2, print3, print4) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			assume_failed((("\"" #x "\", " #print1 ": ") + MATH_NS::ObjToString(print1) + \
			                  (", " #print2 ": ") + MATH_NS::ObjToString(print2) + \
			                  (", " #print3 ": ") + MATH_NS::ObjToString(print3) + \
			                  (", " #print4 ": ") + MATH_NS::ObjToString(print4) + \
			                  (" in " __FILE__ ":" STRINGIZE(__LINE__))).c_str()); \
	MULTI_LINE_MACRO_END
#define assert4 assume4

// If MATH_ASSERT_CORRECTNESS is defined, the function mathassert() is enabled to test
// that all forms of optimizations inside the math library produce proper results.
#ifdef MATH_ASSERT_CORRECTNESS
#define mathassert(x) assert(x)
#define mathassert1 assert1
#define mathassert2 assert2
#define mathassert3 assert3
#define mathassert4 assert4
#else
#define mathassert(x) ((void)0)
#define mathassert1(...) ((void)0)
#define mathassert2(...) ((void)0)
#define mathassert3(...) ((void)0)
#define mathassert4(...) ((void)0)
#endif

// Kill both assume() and mathassert() macros in OPTIMIZED_RELEASE builds.
#ifdef OPTIMIZED_RELEASE
#ifdef assume
#undef assume
#endif
#ifdef assume1
#undef assume1
#endif
#ifdef assume2
#undef assume2
#endif
#ifdef assume3
#undef assume3
#endif
#ifdef assume4
#undef assume4
#endif
#ifdef assert
#undef assert
#endif
#ifdef assert1
#undef assert1
#endif
#ifdef assert2
#undef assert2
#endif
#ifdef assert3
#undef assert3
#endif
#ifdef assert4
#undef assert4
#endif
#ifdef mathassert
#undef mathassert
#endif
#ifdef mathassert1
#undef mathassert1
#endif
#ifdef mathassert2
#undef mathassert2
#endif
#ifdef mathassert3
#undef mathassert3
#endif
#ifdef mathassert4
#undef mathassert4
#endif

#define assume(...) ((void)0)
#define assume1(...) ((void)0)
#define assume2(...) ((void)0)
#define assume3(...) ((void)0)
#define assume4(...) ((void)0)
#define assert(...) ((void)0)
#define assert1(...) ((void)0)
#define assert2(...) ((void)0)
#define assert3(...) ((void)0)
#define assert4(...) ((void)0)
#define mathassert(...) ((void)0)
#define mathassert1(...) ((void)0)
#define mathassert2(...) ((void)0)
#define mathassert3(...) ((void)0)
#define mathassert4(...) ((void)0)

#endif
