#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"
#include <cmath>

#if __cplusplus >= 201103L
TEST(CXX11StdFinite)
{
	// When using MathGeoLib, users should still be able to invoke C++11 std::isfinite function.
	// http://en.cppreference.com/w/cpp/numeric/math/isfinite
	assert(std::isfinite(5.f));
	assert(std::isfinite(5.0));

	using namespace std;

	assert(isfinite(5.f));
	assert(isfinite(5.0));


#ifndef EMSCRIPTEN // long double is not supported.
	assert(std::isfinite(5.0L));
	assert(isfinite(5.0L));
#endif

}
#endif

TEST(IsFinite)
{
	assert(IsFinite(5));
	assert(IsFinite(5.f));
	assert(IsFinite(5.0));
#ifndef EMSCRIPTEN // long double is not supported.
	assert(IsFinite(5.0L));
#endif

	assert(!IsFinite(FLOAT_NAN));
	assert(!IsFinite(FLOAT_INF));
	assert(IsFinite(FLT_MAX));

	assert(!IsFinite((double)FLOAT_NAN));
	assert(!IsFinite((double)FLOAT_INF));
	assert(IsFinite((double)FLT_MAX));

#ifndef EMSCRIPTEN // long double is not supported.
	assert(!IsFinite((long double)FLOAT_NAN));
	assert(!IsFinite((long double)FLOAT_INF));
	assert(IsFinite((long double)FLT_MAX));
#endif
}

TEST(IsNan)
{
	assert(!IsNan(5.f));
	assert(!IsNan(5.0));
#ifndef EMSCRIPTEN // long double is not supported.
	assert(!IsNan(5.0L));
#endif

	assert(IsNan(FLOAT_NAN));
	assert(!IsNan(FLOAT_INF));
	assert(!IsNan(FLT_MAX));

	assert(IsNan((double)FLOAT_NAN));
	assert(!IsNan((double)FLOAT_INF));
	assert(!IsNan((double)FLT_MAX));

#ifndef EMSCRIPTEN // long double is not supported.
	assert(IsNan((long double)FLOAT_NAN));
	assert(!IsNan((long double)FLOAT_INF));
	assert(!IsNan((long double)FLT_MAX));
#endif
}

TEST(IsInf)
{
	assert(!IsInf(5.f));
	assert(!IsInf(5.0));
#ifndef EMSCRIPTEN // long double is not supported.
	assert(!IsInf(5.0L));
#endif

	assert(!IsInf(FLOAT_NAN));
	assert(IsInf(FLOAT_INF));
	assert(IsInf(-FLOAT_INF));
	assert(!IsInf(FLT_MAX));

	assert(!IsInf((double)FLOAT_NAN));
	assert(IsInf((double)FLOAT_INF));
	assert(IsInf(-(double)FLOAT_INF));
	assert(!IsInf((double)FLT_MAX));

#ifndef EMSCRIPTEN // long double is not supported.
	assert(!IsInf((long double)FLOAT_NAN));
	assert(IsInf((long double)FLOAT_INF));
	assert(IsInf(-(long double)FLOAT_INF));
	assert(!IsInf((long double)FLT_MAX));
#endif
}

TEST(ReinterpretAsInt)
{
	assert(ReinterpretAsInt(0.0f) == 0x00000000);
	assert(ReinterpretAsInt(1.0f) == 0x3F800000);
	assert(ReinterpretAsInt(2.0f) == 0x40000000);
	assert(ReinterpretAsInt(-1.0f) == 0xBF800000);
	assert(ReinterpretAsInt(FLOAT_INF) == 0x7F800000);
}

TEST(ReinterpretAsFloat)
{
	assert(ReinterpretAsFloat(0x00000000) == 0.0f);
	assert(ReinterpretAsFloat(0x3F800000) == 1.0f);
	assert(ReinterpretAsFloat(0x40000000) == 2.0f);
	assert(ReinterpretAsFloat(0xBF800000) == -1.0f);
	assert(ReinterpretAsFloat(0x7F800000) == FLOAT_INF);
	assert(IsNan(ReinterpretAsFloat(0x7F800001)));
}

// Idea: Since approx sqrt is so fast, run through that and do one manual Newton-Rhapson iteration to improve.
float NewtonRhapsonSqrt(float x)
{
	float estimate = SqrtFast(x);
	return estimate - (estimate*estimate - x) * 0.5f / estimate;
}

#ifdef MATH_SSE
float NewtonRhapsonSSESqrt(float x)
{
	__m128 X = FLOAT_TO_M128(x);
	__m128 estimate = _mm_rcp_ss(_mm_rsqrt_ss(X));
	__m128 e2 = _mm_mul_ss(estimate,estimate);
	__m128 half = _mm_set_ss(0.5f);
	__m128 recipEst = _mm_rcp_ss(estimate);

	return M128_TO_FLOAT(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float NewtonRhapsonSSESqrt2(float x)
{
	__m128 X = FLOAT_TO_M128(x);
	__m128 estimate = _mm_rsqrt_ss(_mm_rcp_ss(X));
	__m128 e2 = _mm_mul_ss(estimate,estimate);
	__m128 half = _mm_set_ss(0.5f);
	__m128 recipEst = _mm_rcp_ss(estimate);

	return M128_TO_FLOAT(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float NewtonRhapsonSSESqrt3(float x)
{
	__m128 X = FLOAT_TO_M128(x);
	__m128 estimate = _mm_mul_ss(X, _mm_rsqrt_ss(X));
	__m128 e2 = _mm_mul_ss(estimate,estimate);
	__m128 half = _mm_set_ss(0.5f);
	__m128 recipEst = _mm_rcp_ss(estimate);

	return M128_TO_FLOAT(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float Sqrt_Via_Rcp_RSqrt(float x)
{
	return M128_TO_FLOAT(_mm_rcp_ss(_mm_rsqrt_ss(FLOAT_TO_M128(x))));
}

#endif

float *PosFloatArray()
{
	LCG lcg;
	static float *arr;
	if (!arr)
	{
		arr = new float[testrunner_numItersPerTest+32];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (float*)a;
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = lcg.Float(0.f, 100000.f);
	}
	return arr;
}

float *pf = PosFloatArray();
extern float *f;

UNIQUE_TEST(sqrt_precision)
{
	const int C = 7;
	float maxRelError[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		float f = rng.Float(0.f, 1e20f);
		float x = (float)sqrt((double)f); // best precision of the sqrt.

		float X[C];
		X[0] = Sqrt(f);
		X[1] = SqrtFast(f);
		X[2] = NewtonRhapsonSqrt(f);
#ifdef MATH_SSE
		X[3] = NewtonRhapsonSSESqrt(f);
		X[4] = NewtonRhapsonSSESqrt2(f);
		X[5] = NewtonRhapsonSSESqrt3(f);
		X[6] = Sqrt_Via_Rcp_RSqrt(f);
#endif

		for(int j = 0; j < C; ++j)
			maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
	}

	LOGI("Max relative error with Sqrt: %e", maxRelError[0]);
	assert(maxRelError[0] < 1e-9f);
	LOGI("Max relative error with SqrtFast: %e", maxRelError[1]);
	assert(maxRelError[1] < 1e-3f);
	LOGI("Max relative error with NewtonRhapsonSqrt: %e", maxRelError[2]);
	assert(maxRelError[2] < 1e-6f);
#ifdef MATH_SSE
	LOGI("Max relative error with NewtonRhapsonSSESqrt: %e", maxRelError[3]);
	assert(maxRelError[3] < 1e-6f);

	LOGI("Max relative error with NewtonRhapsonSSESqrt2: %e", maxRelError[4]);
	assert(maxRelError[4] < 1e-6f);

	LOGI("Max relative error with NewtonRhapsonSSESqrt3: %e", maxRelError[5]);
	assert(maxRelError[5] < 1e-6f);

	LOGI("Max relative error with Sqrt_Via_Rcp_RSqrt: %e", maxRelError[6]);
	assert(maxRelError[6] < 1e-3f);
#endif
}

BENCHMARK(sqrt_sqrtf)
{
	TIMER_BEGIN
	{
		f[i] = sqrtf(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_Sqrt)
{
	TIMER_BEGIN
	{
		f[i] = Sqrt(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_SqrtFast)
{
	TIMER_BEGIN
	{
		f[i] = SqrtFast(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_NewtonRhapsonSqrt)
{
	TIMER_BEGIN
	{
		f[i] = NewtonRhapsonSqrt(pf[i]);
	}
	TIMER_END
}

#ifdef MATH_SSE
BENCHMARK(sqrt_NewtonRhapsonSSESqrt)
{
	TIMER_BEGIN
	{
		f[i] = NewtonRhapsonSSESqrt(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_NewtonRhapsonSSESqrt2)
{
	TIMER_BEGIN
	{
		f[i] = NewtonRhapsonSSESqrt2(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_NewtonRhapsonSSESqrt3)
{
	TIMER_BEGIN
	{
		f[i] = NewtonRhapsonSSESqrt3(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_Sqrt_Via_Rcp_RSqrt)
{
	TIMER_BEGIN
	{
		f[i] = Sqrt_Via_Rcp_RSqrt(pf[i]);
	}
	TIMER_END
}

FORCE_INLINE float recip_sqrtf(float x)
{
	return 1.f / sqrtf(x);
}

FORCE_INLINE float sqrtf_recip(float x)
{
	return sqrtf(1.f / x);
}

// Quake Inverse Sqrt, from http://betterexplained.com/articles/understanding-quakes-fast-inverse-square-root/
// Benchmarked and tested for reference, DON'T USE THIS! It's twice as slow, and a magnitude of 1e3 worse in precision
// compared to the function RSqrt!
float QuakeInvSqrt(float x)
{
	float xhalf = 0.5f * x;
	int i = *(int*)&x; // store floating-point bits in integer
	i = 0x5f375a86 - (i >> 1); // A better initial value: http://en.wikipedia.org/wiki/Fast_inverse_square_root#History_and_investigation
	//i = 0x5f3759d5 - (i >> 1); // initial guess for Newton's method
	x = *(float*)&i; // convert new bits into float
	x = x*(1.5f - xhalf*x*x); // One round of Newton's method
	return x;
}

UNIQUE_TEST(sqrt_rsqrt_precision)
{
	const int C = 7;
	float maxRelError[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		float f = rng.Float(1e-5f, 1e20f);
		float x = (float)(1.0 / sqrt((double)f)); // best precision of the sqrt.

		float X[C];
		X[0] = RSqrt(f);
		X[1] = recip_sqrtf(f);
		X[2] = sqrtf_recip(f);
		X[3] = QuakeInvSqrt(f);

		for(int j = 0; j < C; ++j)
			maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
	}

	LOGI("Max relative error with RSqrt: %e", maxRelError[0]);
	assert(maxRelError[0] < 1e-3f);
	LOGI("Max relative error with 1.f/sqrtf: %e", maxRelError[1]);
	assert(maxRelError[1] < 1e-6f);
	LOGI("Max relative error with sqrtf(1.f/x): %e", maxRelError[2]);
	assert(maxRelError[2] < 1e-6f);
	LOGI("Max relative error with Quake InvSqrt: %e", maxRelError[3]);
	assert(maxRelError[3] < 1e-2f);
}

BENCHMARK(sqrt_RSqrt)
{
	TIMER_BEGIN
	{
		f[i] = RSqrt(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_recip_sqrtf)
{
	TIMER_BEGIN
	{
		f[i] = recip_sqrtf(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_sqrtf_recip)
{
	TIMER_BEGIN
	{
		f[i] = sqrtf_recip(pf[i]);
	}
	TIMER_END
}

BENCHMARK(sqrt_QuakeInvSqrt)
{
	TIMER_BEGIN
	{
		f[i] = QuakeInvSqrt(pf[i]);
	}
	TIMER_END
}

#endif
