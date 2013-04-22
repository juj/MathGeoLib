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
#pragma once

/** @file Types.h
	@brief Provides platform-independent fixed size types. */

#ifdef MATH_TUNDRA_INTEROP
#include "CoreTypes.h"
#else

#ifndef KNET_NO_FIXEDWIDTH_TYPES

// As a reminder: http://predef.sourceforge.net/prestd.html

// If we have C99, take the types from there.
#if (__STDC_VERSION__ >= 199901L) || (_MSC_VER >= 1600)

#include <cstdint>

typedef uint8_t u8; ///< a single byte: 0-255.
typedef uint16_t u16; ///< 2 bytes: 0 - 65535.
typedef uint32_t u32; ///< 4 bytes: 0 - 4,294,967,295 ~ 4000 million or 4e9.
typedef uint64_t u64; ///< 8 bytes: 18,446,744,073,709,551,615 ~1.8e19.

typedef int8_t s8; ///< a single byte: -128 - 127.
typedef int16_t s16; ///< 2 bytes: -32768 - 32767.
typedef int32_t s32; ///< 4 bytes signed: max 2,147,483,647 ~ 2000 million or 2e9.
typedef int64_t s64; ///< 8 bytes signed. 9,223,372,036,854,775,807 ~ 9e18.

// Otherwise, if we have boost, we can also pull the types from there.
#elif defined(KNET_USE_BOOST)

#include <boost/cstdint.hpp>

typedef boost::uint8_t u8; ///< a single byte: 0-255.
typedef boost::uint16_t u16; ///< 2 bytes: 0 - 65535.
typedef boost::uint32_t u32; ///< 4 bytes: 0 - 4,294,967,295 ~ 4000 million or 4e9.
typedef boost::uint64_t u64; ///< 8 bytes: 18,446,744,073,709,551,615 ~1.8e19.

typedef boost::int8_t s8; ///< a single byte: -128 - 127.
typedef boost::int16_t s16; ///< 2 bytes: -32768 - 32767.
typedef boost::int32_t s32; ///< 4 bytes signed: max 2,147,483,647 ~ 2000 million or 2e9.
typedef boost::int64_t s64; ///< 8 bytes signed. 9,223,372,036,854,775,807 ~ 9e18.

#else // No boost or unknown if we have C99. Have to guess the following are correct.

#include <limits.h>

//#pragma warning "Not using boost and C99 not defined. Guessing the built-ins for fixed-width types!"

typedef unsigned char u8; ///< a single byte: 0-255.
typedef unsigned short u16; ///< 2 bytes: 0 - 65535.
typedef unsigned long long u64; ///< 8 bytes: 18,446,744,073,709,551,615 ~1.8e19.

typedef signed char s8; ///< a single byte: -128 - 127.
typedef signed short s16; ///< 2 bytes: -32768 - 32767.

#if ULONG_MAX == 0xffffffff
typedef unsigned long u32; ///< 4 bytes: 0 - 4,294,967,295 ~ 4000 million or 4e9.
typedef long s32; ///< 4 bytes signed: max 2,147,483,647 ~ 2000 million or 2e9.
#elif UINT_MAX == 0xffffffff
typedef unsigned int u32; ///< 4 bytes: 0 - 4,294,967,295 ~ 4000 million or 4e9.
typedef int s32; ///< 4 bytes signed: max 2,147,483,647 ~ 2000 million or 2e9.
#endif

typedef signed long long s64; ///< 8 bytes signed. 9,223,372,036,854,775,807 ~ 9e18.

#endif

#endif // ~KNET_NO_FIXEDWIDTH_TYPES

#endif

#ifdef _MSC_VER
#define STATIC_ASSERT static_assert
#else
// From http://stackoverflow.com/questions/3385515/static-assert-in-c
#define COMPILE_TIME_ASSERT4(COND,MSG) typedef char static_assertion_##MSG[(!!(COND))*2-1]
#define COMPILE_TIME_ASSERT3(X,L) COMPILE_TIME_ASSERT4(X,static_assertion_at_line_##L)
#define COMPILE_TIME_ASSERT2(X,L) COMPILE_TIME_ASSERT3(X,L)
#define STATIC_ASSERT(X, msg)    COMPILE_TIME_ASSERT2(X,__LINE__)
#endif

STATIC_ASSERT(sizeof(u8) == 1, "Typedef for fixed-width type u8 is incorrect!");
STATIC_ASSERT(sizeof(s8) == 1, "Typedef for fixed-width type s8 is incorrect!");
STATIC_ASSERT(sizeof(u16) == 2, "Typedef for fixed-width type u16 is incorrect!");
STATIC_ASSERT(sizeof(s16) == 2, "Typedef for fixed-width type s16 is incorrect!");
STATIC_ASSERT(sizeof(u32) == 4, "Typedef for fixed-width type u32 is incorrect!");
STATIC_ASSERT(sizeof(s32) == 4, "Typedef for fixed-width type s32 is incorrect!");
STATIC_ASSERT(sizeof(u64) == 8, "Typedef for fixed-width type u64 is incorrect!");
STATIC_ASSERT(sizeof(s64) == 8, "Typedef for fixed-width type s64 is incorrect!");

// Functions annotated with MUST_USE_RESULT require that the user stores the return value, or otherwise
// a warning is printed.
#if _MSC_VER >= 1700
// http://msdn.microsoft.com/en-us/library/jj159529.aspx
#define MUST_USE_RESULT _Check_return_
#elif defined(__clang__) || (defined(__GNUC__) && ((__GNUC__*10000+__GNUC_MINOR*100) >= 30400))
// http://gcc.gnu.org/onlinedocs/gcc-3.4.0/gcc/Function-Attributes.html
#define MUST_USE_RESULT __attribute__((warn_unused_result))
#else
#define MUST_USE_RESULT
#endif

// Looking at disasm, compilers have been seen to be stupid about inlining some single-instruction SIMD intrinsics functions, so use this to force.
#ifdef _DEBUG
#define FORCE_INLINE inline
#elif defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline __attribute__((always_inline))
#endif
