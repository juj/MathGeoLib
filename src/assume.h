#pragma once

#include "Math/MathNamespace.h"
#include <stdio.h>
#ifdef WIN32
#include <assert.h>
#endif

//#define MATH_ASSERT_ON_ASSUME

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

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

MATH_END_NAMESPACE

// If MATH_ENABLE_INSECURE_OPTIMIZATIONS is defined, all input data is assumed to be correct and will
// not be checked against at runtime.
// If this flag is undefined (the default), all input is sanity checked so that user cannot crash the system 
// e.g. with out-of-bounds accesses.
//#define MATH_ENABLE_INSECURE_OPTIMIZATIONS

#ifdef MATH_ASSERT_ON_ASSUME
#define assume(x) assert(x)
#elif !defined(MATH_SILENT_ASSUME) 

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#define assume(x) do { if (!(x)) { printf("Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); if (MathBreakOnAssume()) DebugBreak(); } } while(0)
#elif defined(ANDROID)
#include <android/log.h>
#define assume(x) do { if (!(x)) { __android_log_print(ANDROID_LOG_ERROR, "native-activity", "Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)
#ifdef assert
#undef assert
#endif
#define assert(x) do { if (!(x)) { __android_log_print(ANDROID_LOG_ERROR, "native-activity", "Assertion \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)
#else
#define assume(x) do { if (!(x)) { printf("Assumption \"%s\" failed! in file %s, line %d!\n", #x, __FILE__, __LINE__); } } while(0)
#endif
#else
#define assume(x) 
#endif

// If MATH_ASSERT_CORRECTNESS is defined, the function mathassert() is enabled to test
// that all forms of optimizations inside the math library produce proper results.
#ifdef MATH_ASSERT_CORRECTNESS
#define mathassert(x) assert(x)
#else
#define mathassert(x)
#endif
