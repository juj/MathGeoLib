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
#include "MathNamespace.h"
#include <stdio.h>
#ifdef WIN32
#include "myassert.h"
#endif
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

MATH_END_NAMESPACE

// If MATH_ENABLE_INSECURE_OPTIMIZATIONS is defined, all input data is assumed to be correct and will
// not be checked against at runtime.
// If this flag is undefined (the default), all input is sanity checked so that user cannot crash the system
// e.g. with out-of-bounds accesses.
//#define MATH_ENABLE_INSECURE_OPTIMIZATIONS

#ifdef MATH_ASSERT_ON_ASSUME
#define assume(x) assert(x)
#elif defined(MATH_SILENT_ASSUME)
#define assume(x) ((void)0)
#elif defined(FAIL_USING_EXCEPTIONS)

#include <stdexcept>

#define assume(x) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			throw std::runtime_error(#x); \
	MULTI_LINE_MACRO_END

#elif defined(_MSC_VER)

#define assume(x) (void)((!!(x)) || ( printf("Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__) && MATH_NS::AssumeFailed()) )

#elif defined(ANDROID)

#include <android/log.h>
#define assume(x) do { if (!(x)) { __android_log_print(ANDROID_LOG_ERROR, "native-activity", "Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)
#ifdef assert
#undef assert
#endif
#define assert(x) do { if (!(x)) { __android_log_print(ANDROID_LOG_ERROR, "native-activity", "Assertion \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)

#else // All other platforms

#define assume(x) do { if (!(x)) { printf("Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)

#endif

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
