#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/SSEMath.h"

using namespace TestData;

#ifdef MATH_SSE2
void aligned_sse_memcpy(void *dst_, void *src_, size_t size)
{
	size >>= 7; // / 16 for 16-byte registers, and / 8 for doing 8 moves at a time.
	__m128i *src = (__m128i*)src_;
	__m128i *dst = (__m128i*)dst_;
	for(size_t i = 0; i < size; ++i)
	{/*
		_mm_prefetch((const char*)(src+8), _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+64, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+128, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+192, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+256, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+320, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+384, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+448, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+512, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+576, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+640, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+704, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+768, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+832, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+896, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+960, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+1024, _MM_HINT_NTA);*/
		__m128i r1 = _mm_load_si128((__m128i*)src);
		__m128i r2 = _mm_load_si128((__m128i*)src+1);
		__m128i r3 = _mm_load_si128((__m128i*)src+2);
		__m128i r4 = _mm_load_si128((__m128i*)src+3);
		__m128i r5 = _mm_load_si128((__m128i*)src+4);
		__m128i r6 = _mm_load_si128((__m128i*)src+5);
		__m128i r7 = _mm_load_si128((__m128i*)src+6);
		__m128i r8 = _mm_load_si128((__m128i*)src+7);

		_mm_store_si128(dst, r1);
		_mm_store_si128(dst+1, r2);
		_mm_store_si128(dst+2, r3);
		_mm_store_si128(dst+3, r4);
		_mm_store_si128(dst+4, r5);
		_mm_store_si128(dst+5, r6);
		_mm_store_si128(dst+6, r7);
		_mm_store_si128(dst+7, r8);

		src += 8;
		dst += 8;
	}
}
#endif

#ifdef MATH_AVX
void aligned_avx_memcpy(void *dst_, void *src_, size_t size)
{
	size >>= 8; // / 32 for 32-byte registers, and / 8 for doing 8 moves at a time.
	__m256i *src = (__m256i*)src_;
	__m256i *dst = (__m256i*)dst_;
	for(size_t i = 0; i < size; ++i)
	{/*
		_mm_prefetch((const char*)(src+8), _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+64, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+128, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+192, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+256, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+320, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+384, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+448, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+512, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+576, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+640, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+704, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+768, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+832, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+896, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+960, _MM_HINT_NTA);
		_mm_prefetch((const char*)(src+8)+1024, _MM_HINT_NTA);*/
		__m256i r1 = _mm256_load_si256((__m256i*)src);
		__m256i r2 = _mm256_load_si256((__m256i*)src+1);
		__m256i r3 = _mm256_load_si256((__m256i*)src+2);
		__m256i r4 = _mm256_load_si256((__m256i*)src+3);
		__m256i r5 = _mm256_load_si256((__m256i*)src+4);
		__m256i r6 = _mm256_load_si256((__m256i*)src+5);
		__m256i r7 = _mm256_load_si256((__m256i*)src+6);
		__m256i r8 = _mm256_load_si256((__m256i*)src+7);
		_mm256_store_si256(dst, r1);
		_mm256_store_si256(dst+1, r2);
		_mm256_store_si256(dst+2, r3);
		_mm256_store_si256(dst+3, r4);
		_mm256_store_si256(dst+4, r5);
		_mm256_store_si256(dst+5, r6);
		_mm256_store_si256(dst+6, r7);
		_mm256_store_si256(dst+7, r8);

		src += 8;
		dst += 8;
	}
}
#endif

BENCHMARK(memcpy, "memcpy")
{
	memcpy(m2, m, (sizeof(float4x4)*testrunner_numItersPerTest)/100);
}
BENCHMARK_END;

#ifdef MATH_SSE2
BENCHMARK(aligned_sse_memcpy, "test against memcpy")
{
	aligned_sse_memcpy(m2, m, (sizeof(float4x4)*testrunner_numItersPerTest)/100);
}
BENCHMARK_END;
#endif

#ifdef MATH_AVX
BENCHMARK(aligned_avx_memcpy, "test against memcpy")
{
	aligned_avx_memcpy(m2, m, (sizeof(float4x4)*testrunner_numItersPerTest)/100);
}
BENCHMARK_END;
#endif

BENCHMARK(copy_float4x4_opequals, "float4x4 operator =")
{
	m2[i] = m[i];
}
BENCHMARK_END;

BENCHMARK(copy_float4x4_memcpy, "test against copy_float4x4_opequals")
{
	memcpy(&m2[i], &m[i], sizeof(m[i]));
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(copy_float4x4_sse, "test against copy_float4x4_opequals")
{
	m2[i].row[0] = m[i].row[0];
	m2[i].row[1] = m[i].row[1];
	m2[i].row[2] = m[i].row[2];
	m2[i].row[3] = m[i].row[3];
}
BENCHMARK_END;
#endif

#ifdef MATH_AVX
BENCHMARK(copy_float4x4_avx, "test against copy_float4x4_opequals")
{
	m2[i].row2[0] = m[i].row2[0];
	m2[i].row2[1] = m[i].row2[1];
}
BENCHMARK_END;
#endif

BENCHMARK(copy_float4x4_scalar, "test against copy_float4x4_opequals")
{
	m2[i].v[0][0] = m[i].v[0][0];
	m2[i].v[0][1] = m[i].v[0][1];
	m2[i].v[0][2] = m[i].v[0][2];
	m2[i].v[0][3] = m[i].v[0][3];

	m2[i].v[1][0] = m[i].v[1][0];
	m2[i].v[1][1] = m[i].v[1][1];
	m2[i].v[1][2] = m[i].v[1][2];
	m2[i].v[1][3] = m[i].v[1][3];

	m2[i].v[2][0] = m[i].v[2][0];
	m2[i].v[2][1] = m[i].v[2][1];
	m2[i].v[2][2] = m[i].v[2][2];
	m2[i].v[2][3] = m[i].v[2][3];

	m2[i].v[3][0] = m[i].v[3][0];
	m2[i].v[3][1] = m[i].v[3][1];
	m2[i].v[3][2] = m[i].v[3][2];
	m2[i].v[3][3] = m[i].v[3][3];
}
BENCHMARK_END;
