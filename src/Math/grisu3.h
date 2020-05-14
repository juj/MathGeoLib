/* This file is part of an implementation of the "grisu3" double to string
	conversion algorithm described in the research paper

	"Printing Floating-Point Numbers Quickly And Accurately with Integers"
	by Florian Loitsch, available at
	http://www.cs.tufts.edu/~nr/cs257/archive/florian-loitsch/printf.pdf */
#pragma once

#include "../MathBuildConfig.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __EMSCRIPTEN__
int js_double_to_string(double d, char *dst);
#endif

/// Converts the given double-precision floating point number to a string representation.
/** For most inputs, this string representation is the
	shortest such, which deserialized again, returns the same bit
	representation of the double.
	@param v The number to convert.
	@param dst [out] The double-precision floating point number will be written here
		as a null-terminated string. The conversion algorithm will write at most 25 bytes
		to this buffer. (null terminator is included in this count).
		The dst pointer may not be null.
	@return the number of characters written to dst, excluding the null terminator (which
		is always written) is returned here. */
int dtoa_grisu3(double v, char *dst);
#define f64_to_string dtoa_grisu3

int f32_to_string(float v, char *dst);

/// Converts an unsigned 32-bit integer to a string. Longest 32-bit unsigned integer is
/// 4294967295, which is 10 bytes (11 if including \0)
/** @param val The number to convert.
	@param dst [out] The unsigned number will be written here
		as a null-terminated string. The conversion algorithm will write at most 11 bytes
		to this buffer. (null terminator is included in this count).
		The dst pointer may not be null.
	@return the number of characters written to dst, excluding the null terminator (which
		is always written) is returned here. */
int u32_to_string(uint32_t val, char *dst);

/// Converts an signed 32-bit integer to a string. Longest 32-bit signed integer is
/// -2147483648, which is 11 bytes (12 if including \0)
/** @param val The number to convert.
	@param dst [out] The unsigned number will be written here
		as a null-terminated string. The conversion algorithm will write at most 12 bytes
		to this buffer. (null terminator is included in this count).
		The dst pointer may not be null.
	@return the number of characters written to dst, excluding the null terminator (which
		is always written) is returned here. */
int i32_to_string(int i, char *dst);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#if defined(MATH_ENABLE_STL_SUPPORT)
#include <string>
#endif

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT dtoa_grisu3_string(double v);
#endif

#endif
