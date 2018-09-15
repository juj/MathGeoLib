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

#include <sstream>
#include "../MathBuildConfig.h"
#include "MathNamespace.h"
#include <stdio.h>
#include "myassert.h"
#include "MathLog.h"

#ifndef MARK_UNUSED
/// If a variable is labelled with this directive, the compiler should not emit a warning even if it is unused in the code.
#define MARK_UNUSED(x) ((void)x)
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

// The assume() macro is used to check preconditions on the math-related functions, e.g. whether vectors are normalized, check that division by zero doesn't occur, orthonormal bases, and so on.

// The assume() macro operates differently depending on which #defines are present:
// #define MATH_ASSERT_ON_ASSUME - the assume() macro resolves to the assert() macro.
// #define MATH_DISABLE_ASSUME   - the assume() macro is silent, and disabled altogether. (no prints or breaks or anything, the checks by assume() are ignored)
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
inline std::string ObjToString(const T &obj)
{
	return obj.ToString();
}
/*
template<>
inline std::string ObjToString<const char*>(const char * const & obj)
{
	return obj;
}
*/
template<>
inline std::string ObjToString<std::string>(const std::string &obj)
{
	return obj;
}

template<>
inline std::string ObjToString<float>(const float &obj)
{
	std::stringstream ss;
	ss << obj;
	return ss.str();
}

template<>
inline std::string ObjToString<int>(const int &obj)
{
	std::stringstream ss;
	ss << obj;
	return ss.str();
}

template<>
inline std::string ObjToString<bool>(const bool &obj)
{
	std::stringstream ss;
	ss << obj;
	return ss.str();
}

template<>
inline std::string ObjToString<u32>(const u32 &obj)
{
	std::stringstream ss;
	ss << obj;
	return ss.str();
}

MATH_END_NAMESPACE

// If MATH_ENABLE_INSECURE_OPTIMIZATIONS is defined, all input data is assumed to be correct and will
// not be checked against at runtime.
// If this flag is undefined (the default), all input is sanity checked so that user cannot crash the system
// e.g. with out-of-bounds accesses.
//#define MATH_ENABLE_INSECURE_OPTIMIZATIONS

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
#define assume_failed(message) LOGE("Assumption \"%s\" failed! in file %s, line %d!", message, __FILE__, __LINE__)
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
#else
#define mathassert(x) ((void)0)
#endif

// Kill both assume() and mathassert() macros in OPTIMIZED_RELEASE builds.
#ifdef OPTIMIZED_RELEASE
#ifdef assume
#undef assume
#endif
#ifdef mathassert
#undef mathassert
#endif
#define assume(x) ((void)0)
#define mathassert(x) ((void)0)
#endif
