#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

#include "../src/MathBuildConfig.h"
#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include <cmath>

#include "../src/Math/SSEMath.h"
#include "../src/Math/BitFuncs.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

char int_to_string[64];

// Test if a manually written int->string conversion beats the built-in variants. Results from the benchmarks below:
// U32ToString - 23.488 nsecs
// itoa        - 64.544 nsecs
// sprintf     - 210.107 nsecs
// From 2013 Macbook Pro + Win8.1 + VS2010

int U32ToString(u32 val, char *str)
{
	char *s = str;
	for(;;)
	{
		int ni = val / 10;
		int digit = val - ni*10;
		*s++ = (char)('0' + digit);
		if (ni == 0)
			break;
		val = ni;
	}
	*s = '\0';
	ptrdiff_t len = s - str;
	for(int i = 0; i < len/2; ++i)
	{
		char ch = str[i];
		str[i] = str[len-1-i];
		str[len-1-i] = ch;
	}

	return (int)(s - str);
}

#ifdef MATH_SSE2

FORCE_INLINE __m128i INT_TO_M128(int i)
{
#ifdef _MSC_VER
	__m128i v = _mm_setzero_si128();
	v.m128i_i32[0] = i;
	return v;
#elif defined(__clang__) || defined(__GNUC__)
	return _mm_set1_epi32(i);
#else
	return _mm_loadu_si32(&i);
#endif
}

FORCE_INLINE int M128_TO_INT16(__m128i i)
{
#ifdef _MSC_VER
	return i.m128i_i16[0];
#else
	union m128_to_int16
	{
		__m128i m128i;
		signed short i16[8];
	};
	m128_to_int16 mi;
	mi.m128i = i;
	return (int)mi.i16[0];
#endif
}

static const __m128i one_hundredmth = INT_TO_M128((int)2882303761 /*42.94967295*65536*32*32 */);
static const __m128i hundredm = INT_TO_M128(100000000);
static const __m128i one_tenthousandth = _mm_set1_epi32((int)3518437209 /*429496.7295*8192 */);
static const __m128i tenthousand = _mm_set1_epi32(10000);
static const __m128i one_hundredth = _mm_set1_epi16((short)(unsigned short)41944 /*655.36*64 */);
static const __m128i hundred = _mm_set1_epi16(100);
static const __m128i one_tenth = _mm_set1_epi16((short)(unsigned short)52429 /* == 8 * 6553.6 */);
static const __m128i ten = _mm_set1_epi16(10);
static const __m128i zerochar = _mm_set1_epi8('0');

#ifdef _MSC_VER
#pragma intrinsic(_BitScanForward)
#endif

#ifdef MATH_SSE41
// Warning: This is about 5 times slower than the C version! Better to just use the C version.
int U32ToString_SSE(u32 i, char *str)
{
	// Max u32: 4294967295 ~ 4e9

	// 4e9 / 1e8 -> [ 42 (64), 94967295 (64) ]
	//     / 1e4 -> [ .. (32), 42 (32), 9496 (32), 7295 (32) ]
	//     / 1e2 -> [ (16), (16), (16), 42 (16), 94 (16), 96 (16), 72 (16), 95 (16) ]
	//     / 10  -> [ 

	__m128i v = INT_TO_M128((int)i);

	// 1/100000000 ~~ 42.94967296 / 4294967296
	__m128i hi = _mm_srli_epi64(_mm_mul_epu32(v, one_hundredmth), 26+32);
	__m128i lo = _mm_sub_epi32(v, _mm_mullo_epi32(hi, hundredm));

	__m128i msb = _mm_slli_si128(hi, 6); // [42, 00, 00, 00, 00], or two most significant digits as 16-bit
	v = lo; // 0 - 94967295 in 32-bit
	// 1/10000 ~~ 429496.7296 / 4294967296
	hi = _mm_srli_epi64(_mm_mul_epu32(v, one_tenthousandth), 13+32);
	lo = _mm_sub_epi32(v, _mm_mullo_epi32(hi, tenthousand));

	v = _mm_unpacklo_epi16(hi, lo); // [7295, 9496] in 16-bit

	hi = _mm_srli_epi16(_mm_mulhi_epu16(v, one_hundredth), 6);
	lo = _mm_sub_epi16(v, _mm_mullo_epi16(hi, hundred));

	v = _mm_unpacklo_epi16(hi, lo);  // [95, 72, 96, 94] in 16-bit
	v = _mm_unpacklo_epi64(msb, v); // [94, 96, 72, 95, 42, 00, 00, 00] in 16-bit
	hi = _mm_srli_epi16(_mm_mulhi_epu16(v, one_tenth), 3);
	lo = _mm_sub_epi16(v, _mm_mullo_epi16(hi, ten));

	lo = _mm_or_si128(_mm_slli_si128(lo, 1), hi);

	lo = _mm_add_epi8(lo, zerochar);
	__m128i ones = _mm_cmpgt_epi8(lo, zerochar);
	ones = _mm_or_si128(_mm_slli_si128(ones, 8), ones);
	ones = _mm_or_si128(_mm_slli_si128(ones, 4), ones);
	ones = _mm_or_si128(_mm_slli_si128(ones, 2), ones);
	ones = _mm_or_si128(_mm_slli_si128(ones, 1), ones);
	unsigned int onesBits = (unsigned int)_mm_movemask_epi8(ones);
	unsigned long shift = CountTrailingZeroes32(onesBits);
	_mm_maskmoveu_si128(lo, ones, str-shift);
	int len = 16-shift;
	str[len] = '\0';
	return len;
}

RANDOMIZED_TEST(U32ToString_SSE)
{
	char str[32];
	char str2[32];
	for(int i = 0; i < 100; ++i)
	{
		u32 m = rng.Int(0, 43);
		m *= 100000000U;
		u32 n = rng.Int();
		U32ToString_SSE(n+m, str);
		U32ToString(n+m, str2);
		assert2(!strcmp(str, str2), std::string(str), std::string(str2));
	}
}

BENCHMARK(U32ToString_SSE, "SSE uint -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	U32ToString_SSE((u32)pseudoRandom, int_to_string);
}
BENCHMARK_END;
#endif

#endif

// Tests whether modulus+division is slower than division+multiplication. (doesn't seem to be the case)
int U32ToString_Slow(u32 val, char *str)
{
	char *s = str;
	for(;;)
	{
		int digit = val % 10;
		*s++ = (char)('0' + digit);
		val /= 10;
		if (val == 0)
			break;
	}
	*s = '\0';
	ptrdiff_t len = s - str;
	for(int i = 0; i < len/2; ++i)
	{
		char ch = str[i];
		str[i] = str[len-1-i];
		str[len-1-i] = ch;
	}

	return (int)(s - str);
}

int IntToString(int i, char *str)
{
	if (i < 0)
	{
		if (i == INT_MIN)
		{
			strcpy(str, "-2147483648");
			return 11; // == strlen("-2147483648")
		}
		*str++ = '-';
		return U32ToString((u32)-i, str) + 1;
	}
	return U32ToString((u32)i, str);
}

RANDOMIZED_TEST(IntToString)
{
	const int nums[] = { 0, 1, -1, 10, 11, -10, -11, INT_MIN, INT_MAX, (int)UINT_MAX, 42};

	for(size_t j = 0; j < sizeof(nums)/sizeof(nums[0]); ++j)
	{
		int i = nums[j];
		if (i == 42) i = rng.Int();
		char str[32] = {};
		int len = IntToString(i, str);
		assert(len > 0);
		MARK_UNUSED(len);
		assert(strlen(str) == (size_t)len);
		char str2[32] = {};
		sprintf(str2, "%d", i);
	//	printf("i: %d, str: %s, str2: %s\n", i, str, str2);
		assert(!strcmp(str, str2));
	}
}

BENCHMARK(IntToString, "custom int -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	IntToString((int)pseudoRandom, int_to_string);
}
BENCHMARK_END;

BENCHMARK(IntToString_sprintf, "sprintf int -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	sprintf(int_to_string, "%d", (int)pseudoRandom);
}
BENCHMARK_END;

#ifdef _MSC_VER
BENCHMARK(IntToString_itoa, "itoa int -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	_itoa((int)pseudoRandom, int_to_string, 10);
}
BENCHMARK_END;
#endif

RANDOMIZED_TEST(U32ToString)
{
	const u32 nums[] = { 0, 1, (u32)-1, 10, 11, (u32)-10, (u32)-11, (u32)INT_MIN, INT_MAX, UINT_MAX, 42};

	for(size_t j = 0; j < sizeof(nums)/sizeof(nums[0]); ++j)
	{
		u32 i = nums[j];
		if (i == 42) i = (u32)rng.Int();
		char str[32] = {};
		int len = U32ToString(i, str);
		assert(len > 0);
		MARK_UNUSED(len);
		assert(strlen(str) == (size_t)len);
		char str2[32] = {};
		sprintf(str2, "%lu", (unsigned long)i);
	//	printf("i: %d, str: %s, str2: %s\n", i, str, str2);
		assert(!strcmp(str, str2));
	}
}

BENCHMARK(U32ToString, "custom uint -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	U32ToString((u32)pseudoRandom, int_to_string);
}
BENCHMARK_END;

BENCHMARK(U32ToString_Slow, "custom slow uint -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	U32ToString_Slow((u32)pseudoRandom, int_to_string);
}
BENCHMARK_END;

BENCHMARK(U32ToString_sprintf, "sprintf uint -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	sprintf(int_to_string, "%u", pseudoRandom);
}
BENCHMARK_END;

#ifdef _MSC_VER
BENCHMARK(U32ToString_itoa, "itoa uint -> string conversion")
{
	unsigned int pseudoRandom = (unsigned int)i * 1053928445U;
	_itoa((int)pseudoRandom, int_to_string, 10);
}
BENCHMARK_END;
#endif
