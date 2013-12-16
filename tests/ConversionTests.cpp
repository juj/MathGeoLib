#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include <cmath>

using namespace TestData;

// Test if a manually written int->string conversion beats the built-in variants. Results from the benchmarks below:
// U32ToString - 23.488 nsecs
// itoa        - 64.544 nsecs
// sprintf     - 210.107 nsecs
// From 2013 Macbook Pro + Win8.1 + VS2010

int U32ToString(u32 i, char *str)
{
	char *s = str;
	for(;;)
	{
		int ni = i / 10;
		int digit = i - ni*10;
		*s++ = (char)('0' + digit);
		if (ni == 0)
			break;
		i = ni;
	}
	*s = '\0';
	int len = s - str;
	for(int i = 0; i < len/2; ++i)
	{
		char ch = str[i];
		str[i] = str[len-1-i];
		str[len-1-i] = ch;
	}

	return s - str;
}

// Tests whether modulus+division is slower than division+multiplication. (doesn't seem to be the case)
int U32ToString_Slow(u32 i, char *str)
{
	char *s = str;
	for(;;)
	{
		int digit = i % 10;
		*s++ = (char)('0' + digit);
		i /= 10;
		if (i == 0)
			break;
	}
	*s = '\0';
	int len = s - str;
	for(int i = 0; i < len/2; ++i)
	{
		char ch = str[i];
		str[i] = str[len-1-i];
		str[len-1-i] = ch;
	}

	return s - str;
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

TEST(IntToString)
{
	const int nums[] = { 0, 1, -1, 10, 11, -10, -11, INT_MIN, INT_MAX, (int)UINT_MAX, 42};

	for(int j = 0; j < sizeof(nums)/sizeof(nums[0]); ++j)
	{
		int i = nums[j];
		if (i == 42) i = rng.Int();
		char str[32] = {};
		int len = IntToString(i, str);
		assert(len > 0);
		assert(strlen(str) == (size_t)len);
		char str2[32] = {};
		sprintf(str2, "%d", i);
	//	printf("i: %d, str: %s, str2: %s\n", i, str, str2);
		assert(!strcmp(str, str2));
	}
}

char int_to_string[64];

BENCHMARK(IntToString, "custom int -> string conversion")
{
	IntToString(i*1053928445, int_to_string);
}
BENCHMARK_END;

BENCHMARK(IntToString_sprintf, "sprintf int -> string conversion")
{
	sprintf(int_to_string, "%d", i*1053928445);
}
BENCHMARK_END;

BENCHMARK(IntToString_itoa, "itoa int -> string conversion")
{
	itoa(i*1053928445, int_to_string, 10);
}
BENCHMARK_END;

TEST(U32ToString)
{
	const u32 nums[] = { 0, 1, (u32)-1, 10, 11, (u32)-10, (u32)-11, (u32)INT_MIN, INT_MAX, UINT_MAX, 42};

	for(int j = 0; j < sizeof(nums)/sizeof(nums[0]); ++j)
	{
		u32 i = nums[j];
		if (i == 42) i = (u32)rng.Int();
		char str[32] = {};
		int len = U32ToString(i, str);
		assert(len > 0);
		assert(strlen(str) == (size_t)len);
		char str2[32] = {};
		sprintf(str2, "%u", i);
	//	printf("i: %d, str: %s, str2: %s\n", i, str, str2);
		assert(!strcmp(str, str2));
	}
}

BENCHMARK(U32ToString, "custom uint -> string conversion")
{
	U32ToString(i*1053928445, int_to_string);
}
BENCHMARK_END;

BENCHMARK(U32ToString_Slow, "custom slow uint -> string conversion")
{
	U32ToString_Slow(i*1053928445, int_to_string);
}
BENCHMARK_END;

BENCHMARK(U32ToString_sprintf, "sprintf uint -> string conversion")
{
	sprintf(int_to_string, "%u", i*1053928445);
}
BENCHMARK_END;

BENCHMARK(U32ToString_itoa, "itoa uint -> string conversion")
{
	itoa(i*1053928445, int_to_string, 10);
}
BENCHMARK_END;
