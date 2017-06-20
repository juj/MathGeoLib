#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include <cmath>
#include <algorithm>

#ifdef MATH_SSE2
#include "../src/Math/simd.h"
#include "../src/Math/sse_mathfun.h"
#endif

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

bool TrueCondition() // This is a function that always returns true, but it is impossible for compiler to know that.
{
	tick_t t1 = Clock::Tick();
	tick_t t2 = Clock::Tick();
	return t1 != 0 || t2 != 10000000;
}

#ifdef FAIL_USING_EXCEPTIONS

#ifndef OPTIMIZED_RELEASE // In OPTIMIZED_RELEASE builds, assume() macro does not exist.

UNIQUE_TEST(assumeWorks)
{
	EXPECT_FAIL("Testing exception throwing");

	if (TrueCondition())
		assume(false && "Testing that assume() macro fails as expected!");
}

#endif

UNIQUE_TEST(assertWorks)
{
	EXPECT_FAIL("Testing exception throwing");

	if (TrueCondition())
		assert(false && "Testing that assert() macro fails as expected!");
}

UNIQUE_TEST(assume1Works)
{
	EXPECT_FAIL("Testing exception throwing");

	int val = 5;
	if (TrueCondition())
		assume1(false && "Testing that assume() macro fails as expected!", val);
}

UNIQUE_TEST(assert1Works)
{
	EXPECT_FAIL("Testing exception throwing");

	int val = 5;
	if (TrueCondition())
		assert1(false && "Testing that assert() macro fails as expected!", val);
}

UNIQUE_TEST(assume4Works)
{
	EXPECT_FAIL("Testing exception throwing");

	int val = 5;
	if (TrueCondition())
		assume4(false && "Testing that assume() macro fails as expected!", val, val, val, val);
}

UNIQUE_TEST(assert4Works)
{
	EXPECT_FAIL("Testing exception throwing");

	int val = 5;
	if (TrueCondition())
		assert4(false && "Testing that assert() macro fails as expected!", val, val, val, val);
}

UNIQUE_TEST(ExceptionGoesThrough)
{
	EXPECT_FAIL("Testing exception throwing");

	LOGI("Testing exception throwing");
	if (TrueCondition())
		throw std::runtime_error("expect failure");
}

UNIQUE_TEST(ExceptionCatchingWorks)
{
	bool gotException = false;
	try
	{
		LOGI("Testing exception throwing.");
		if (TrueCondition())
			throw std::runtime_error("testing exceptions");
	}
	catch(const std::runtime_error &)
	{
		gotException = true;
		LOGI("Exception catching works.");
	}
	if (!gotException)
	{
		LOGE("Exception catching does not work!");
	}
	assert(gotException);
}

UNIQUE_TEST(BaseDerivedExceptionCatchingWorks)
{
	bool gotException = false;
	try
	{
		LOGI("Testing exception throwing.");
		if (TrueCondition())
			throw std::runtime_error("testing exceptions");
	}
	catch(const std::exception &)
	{
		gotException = true;
		LOGI("Exception catching works.");
	}
	if (!gotException)
	{
		LOGE("Exception catching does not work!");
	}
	assert(gotException);
}
#endif

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


#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(std::isfinite(5.0L));
	assert(isfinite(5.0L));
#endif

}
#endif

#if !defined(__EMSCRIPTEN__)
struct U80
{
	union
	{
		u8 data[16];
		long double ld;
	};

	explicit U80(long double d)
	{
		for(int i = 0; i < 16; ++i)
			data[i] = 0;
		ld = d;
		assert(sizeof(long double) <= 16);
		assert(sizeof(d) <= 16);
	}

	std::string ToString() const
	{
		char str[256] = "0x";
		for(int i = sizeof(long double)-1; i >= 0; --i)
		{
			char str2[16];
			sprintf(str2, "%02X", data[i]);
			strcat(str, str2);
		}
		return str;
	}
};
#endif

UNIQUE_TEST(FloatRepresentation)
{
	LOGI("sizeof(float): %d", (int)sizeof(float));
	LOGI("FLOAT_NAN: %f, or 0x%X", FLOAT_NAN, (unsigned int)ReinterpretAsU32(FLOAT_NAN));
	LOGI("FLOAT_INF: %f, or 0x%X", FLOAT_INF, (unsigned int)ReinterpretAsU32(FLOAT_INF));

	LOGI("sizeof(double): %d", (int)sizeof(double));
	LOGI("FLOAT_NAN as double: %f, or 0x%llX", FLOAT_NAN, ReinterpretAsU64((double)FLOAT_NAN));
	LOGI("FLOAT_INF as double: %f, or 0x%llX", FLOAT_INF, ReinterpretAsU64((double)FLOAT_INF));

	// Commented out - Valgrind does not support long double and gives a false positive if the following code is run,
	// so just for simplicity, leave this test out.
	/*
#if !defined(__EMSCRIPTEN__)
	LOGI("sizeof(long double): %d", (int)sizeof(long double));
	LOGI("LDBL_DIG: %d, LDBL_MANT_DIG: %d", (int)LDBL_DIG, (int)LDBL_MANT_DIG);
#if !defined(__MINGW32__) // MinGW doesn't support printing out long doubles
	LOGI("FLOAT_NAN as long double: %Lf, or %s", (long double)FLOAT_NAN, U80((long double)FLOAT_NAN).ToString().c_str());
	LOGI("FLOAT_INF as long double: %Lf, or %s", (long double)FLOAT_INF, U80((long double)FLOAT_INF).ToString().c_str());
#endif

#endif
	*/
}

TEST(IsFinite)
{
	assert(IsFinite(5));
	assert(IsFinite(5.f));
	assert(IsFinite(5.0));
#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(IsFinite(5.0L));
#endif

	assert(!IsFinite(FLOAT_NAN));
	assert(!IsFinite(FLOAT_INF));
	assert(IsFinite(FLT_MAX));

	assert(!IsFinite((double)FLOAT_NAN));
	assert(!IsFinite((double)FLOAT_INF));
	assert(IsFinite((double)FLT_MAX));

#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(!IsFinite((long double)FLOAT_NAN));
	assert(!IsFinite((long double)FLOAT_INF));
	assert(IsFinite((long double)FLT_MAX));
#endif
}

TEST(IsNan)
{
	assert(!IsNan(5.f));
	assert(!IsNan(5.0));
#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(!IsNan(5.0L));
#endif

	assert(IsNan(FLOAT_NAN));
	assert(!IsNan(FLOAT_INF));
	assert(!IsNan(FLT_MAX));

	assert(IsNan((double)FLOAT_NAN));
	assert(!IsNan((double)FLOAT_INF));
	assert(!IsNan((double)FLT_MAX));

#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(IsNan((long double)FLOAT_NAN));
	assert(!IsNan((long double)FLOAT_INF));
	assert(!IsNan((long double)FLT_MAX));
#endif
}

TEST(IsInf)
{
	assert(!IsInf(5.f));
	assert(!IsInf(5.0));
#ifndef __EMSCRIPTEN__ // long double is not supported.
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

#ifndef __EMSCRIPTEN__ // long double is not supported.
	assert(!IsInf((long double)FLOAT_NAN));
	assert(IsInf((long double)FLOAT_INF));
	assert(IsInf(-(long double)FLOAT_INF));
	assert(!IsInf((long double)FLT_MAX));
#endif
}

TEST(ReinterpretAsU32)
{
	assert(ReinterpretAsU32(0.0f) == 0x00000000);
	assert(ReinterpretAsU32(1.0f) == 0x3F800000);
	assert(ReinterpretAsU32(2.0f) == 0x40000000);
	assert(ReinterpretAsU32(-1.0f) == 0xBF800000);
	assert(ReinterpretAsU32(FLOAT_INF) == 0x7F800000);
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

// Idea: Since approx Sqrt is so fast, run through that and do one manual Newton-Rhapson iteration to improve.
float NewtonRhapsonSqrt(float x)
{
	float estimate = SqrtFast(x);
	return estimate - (estimate*estimate - x) * 0.5f / estimate;
}

#ifdef MATH_SSE
float NewtonRhapsonSSESqrt(float x)
{
	simd4f X = setx_ps(x);
	simd4f estimate = _mm_rcp_ss(_mm_rsqrt_ss(X));
	simd4f e2 = _mm_mul_ss(estimate,estimate);
	simd4f half = _mm_set_ss(0.5f);
	simd4f recipEst = _mm_rcp_ss(estimate);

	return s4f_x(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float NewtonRhapsonSSESqrt2(float x)
{
	simd4f X = setx_ps(x);
	simd4f estimate = _mm_rsqrt_ss(_mm_rcp_ss(X));
	simd4f e2 = _mm_mul_ss(estimate,estimate);
	simd4f half = _mm_set_ss(0.5f);
	simd4f recipEst = _mm_rcp_ss(estimate);

	return s4f_x(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float NewtonRhapsonSSESqrt3(float x)
{
	simd4f X = setx_ps(x);
	simd4f estimate = _mm_mul_ss(X, _mm_rsqrt_ss(X));
	simd4f e2 = _mm_mul_ss(estimate,estimate);
	simd4f half = _mm_set_ss(0.5f);
	simd4f recipEst = _mm_rcp_ss(estimate);

	return s4f_x(_mm_sub_ss(estimate, _mm_mul_ss(_mm_mul_ss((_mm_sub_ss(e2, X)), half), recipEst)));
}

float Sqrt_Via_Rcp_RSqrt(float x)
{
	return s4f_x(_mm_rcp_ss(_mm_rsqrt_ss(setx_ps(x))));
}

#endif

// Quake Inverse Sqrt, from http://betterexplained.com/articles/understanding-quakes-fast-inverse-square-root/
// Benchmarked and tested for reference, DON'T USE THIS! It's twice as slow, and a magnitude of 1e3 worse in precision
// compared to the function RSqrt!
float QuakeInvSqrt(float x)
{
	float xhalf = 0.5f * x;
	u32 i = ReinterpretAsU32(x);//*(int*)&x; // store floating-point bits in integer
	i = 0x5f375a86 - (i >> 1); // A better initial value: http://en.wikipedia.org/wiki/Fast_inverse_square_root#History_and_investigation
	//i = 0x5f3759d5 - (i >> 1); // initial guess for Newton's method
	x = ReinterpretAsFloat(i);//*(float*)&i; // convert new bits into float
	x = x*(1.5f - xhalf*x*x); // One round of Newton's method
	return x;
}

float QuakeHackSqrt(float f)
{
	return f * QuakeInvSqrt(f);
}

float AnotherHackSqrt(float f)
{
	int i = 0x1FBD1DF5 + (*(int*)&f >> 1);
	return *(float*)&i;
}

UNIQUE_TEST(SqrtVals)
{
	assert(Sqrt(-0.f) == 0.f);
	assert(Sqrt(0.f) == 0.f);
	assert(Sqrt(1.f) == 1.f);
}

UNIQUE_TEST(sqrt_precision)
{
	const int C = 11;

	const float ranges[] = { 1e3f, 1e6f, 1e9f, 1e15f, 1e20f };

	for(size_t k = 0; k < sizeof(ranges)/sizeof(ranges[0]); ++k)
	{
		float maxRelError[C] = {};
		float X[C] = {};

		LOGI("Within range of [0, %e]:", ranges[k]);
		for(int i = 0; i < 100000; ++i)
		{
			float f = rng.Float(0.f, ranges[k]);
			float x = (float)sqrt((double)f); // best precision of the Sqrt.

			X[0] = Sqrt(f);
			X[1] = SqrtFast(f);
			X[2] = NewtonRhapsonSqrt(f);
#ifdef MATH_SSE
			X[3] = NewtonRhapsonSSESqrt(f);
			X[4] = NewtonRhapsonSSESqrt2(f);
			X[5] = NewtonRhapsonSSESqrt3(f);
			X[6] = Sqrt_Via_Rcp_RSqrt(f);
#endif
#ifdef MATH_SIMD
			X[7] = s4f_x(sqrt_ps(set1_ps(f)));
#endif
			X[8] = sqrtf(f);
			X[9] = QuakeHackSqrt(f);
			X[10] = AnotherHackSqrt(f);

			for(int j = 0; j < C; ++j)
				maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
		}

		LOGI("Max relative error with Sqrt: %e", maxRelError[0]);
		assert(maxRelError[0] < 1e-6f);
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
#ifdef MATH_SIMD
		LOGI("Max relative error with sqrt_ps: %e", maxRelError[7]);
		assert(maxRelError[7] < 1e-6f);
#endif

		LOGI("Max relative error with sqrtf: %e", maxRelError[8]);
		LOGI("Max relative error with QuakeHackSqrt: %e", maxRelError[9]);
		LOGI("Max relative error with AnotherHackSqrt: %e", maxRelError[10]);
	}
}

BENCHMARK(Sqrt, "Sqrt")
{
	f[i] = Sqrt(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sqrtf, "test against Sqrt")
{
	f[i] = sqrtf(pf[i]);
}
BENCHMARK_END;

BENCHMARK(SqrtFast, "SqrtFast")
{
	f[i] = SqrtFast(pf[i]);
}
BENCHMARK_END;

BENCHMARK(NewtonRhapsonSqrt, "test against SqrtFast")
{
	f[i] = NewtonRhapsonSqrt(pf[i]);
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(NewtonRhapsonSSESqrt, "test against SqrtFast")
{
	f[i] = NewtonRhapsonSSESqrt(pf[i]);
}
BENCHMARK_END;

BENCHMARK(NewtonRhapsonSSESqrt2, "test against SqrtFast")
{
	f[i] = NewtonRhapsonSSESqrt2(pf[i]);
}
BENCHMARK_END;

BENCHMARK(NewtonRhapsonSSESqrt3, "test against SqrtFast")
{
	f[i] = NewtonRhapsonSSESqrt3(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Sqrt_Via_Rcp_RSqrt, "test against Sqrt")
{
	f[i] = Sqrt_Via_Rcp_RSqrt(pf[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(QuakeHackSqrt, "test against Sqrt")
{
	f[i] = QuakeHackSqrt(pf[i]);
}
BENCHMARK_END;

BENCHMARK(AnotherHackSqrt, "test against Sqrt")
{
	f[i] = AnotherHackSqrt(pf[i]);
}
BENCHMARK_END;

FORCE_INLINE float recip_sqrtf(float x)
{
	return 1.f / sqrtf(x);
}

FORCE_INLINE float sqrtf_recip(float x)
{
	return sqrtf(1.f / x);
}

UNIQUE_TEST(sqrt_rsqrt_precision)
{
	const int C = 7;
	float maxRelError[C] = {};
	float X[C] = {};

	for(int i = 0; i < 10; ++i)
	{
		float f = rng.Float(1e-6f, 1e20f);
		float x = (float)(1.0 / sqrt((double)f)); // best precision of the rsqrt.

		X[0] = RSqrt(f);
		X[1] = recip_sqrtf(f);
		X[2] = sqrtf_recip(f);
		X[3] = QuakeInvSqrt(f);
		X[4] = RSqrtFast(f);
#ifdef MATH_SIMD
		X[5] = s4f_x(rsqrt_ps(set1_ps(f)));
#endif
		for(int j = 0; j < C; ++j)
			maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
	}
	LOGI("Max relative error with RSqrt: %e", maxRelError[0]);
	assert(maxRelError[0] < 1e-6f);
	LOGI("Max relative error with 1.f/sqrtf: %e", maxRelError[1]);
	assert(maxRelError[1] < 1e-6f);
	LOGI("Max relative error with sqrtf(1.f/x): %e", maxRelError[2]);
	assert(maxRelError[2] < 1e-6f);
	LOGI("Max relative error with Quake InvSqrt: %e", maxRelError[3]);
	assert(maxRelError[3] < 1e-2f);
	LOGI("Max relative error with RSqrtFast: %e", maxRelError[4]);
	assert(maxRelError[4] < 5e-3f);
#ifdef MATH_SIMD
	LOGI("Max relative error with rsqrt_ps: %e", maxRelError[5]);
	assert(maxRelError[5] < 1e-6f);
#endif
}

BENCHMARK(RSqrt, "RSqrt")
{
	f[i] = RSqrt(pf[i]);
}
BENCHMARK_END;

BENCHMARK(recip_sqrtf, "test against RSqrt")
{
	f[i] = recip_sqrtf(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sqrtf_recip, "test against RSqrt")
{
	f[i] = sqrtf_recip(pf[i]);
}
BENCHMARK_END;

BENCHMARK(QuakeInvSqrt, "test against RSqrt")
{
	f[i] = QuakeInvSqrt(pf[i]);
}
BENCHMARK_END;

BENCHMARK(RSqrtFast, "RSqrtFast")
{
	f[i] = RSqrtFast(pf[i]);
}
BENCHMARK_END;

float OneOverX(float x)
{
	return 1.0f / x;
}

#ifdef MATH_SSE
float NewtonRhapsonRecip(float x)
{
	simd4f X = setx_ps(x);
	simd4f e = _mm_rcp_ss(X);
	// 1/x = D
	// f(e) = e^-1 - x
	// f'(e) = -e^-2

	// e_n = e + (e^-1 - x) / e^-2
	// e_n = e + e - x*e^2
	// e_n = 2*e - x*e^2

	// Do one iteration of Newton-Rhapson:
	simd4f e2 = _mm_mul_ss(e,e);
	
	return s4f_x(_mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2)));
}

float NewtonRhapsonRecip2(float x)
{
	simd4f X = setx_ps(x);
	simd4f e = _mm_rcp_ss(X);
	// 1/x = D
	// f(e) = e^-1 - x
	// f'(e) = -e^-2

	// e_n = e + (e^-1 - x) / e^-2
	// e_n = e + e - x*e^2
	// e_n = 2*e - x*e^2

	// Do one iteration of Newton-Rhapson:
	__m128 e2 = _mm_mul_ss(e,e);

	e = _mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2));
	e2 = _mm_mul_ss(e,e);
	return s4f_x(_mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2)));
}

#ifdef MATH_SSE2
static inline double rcp_double(float x)
{
	__m128d X = _mm_cvtps_pd(_mm_set1_ps(x));
	__m128d e = _mm_cvtps_pd(_mm_rcp_ps(_mm_set1_ps(x)));
	e = _mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e)));
	e = _mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e)));
	e = _mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e)));
	e = _mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e)));
	e = _mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e)));
	return _mm_cvtsd_f64(_mm_sub_sd(_mm_add_sd(e,e), _mm_mul_sd(X, _mm_mul_sd(e,e))));
}
#endif

#endif

UNIQUE_TEST(sqrt_recip_precision)
{
	const int C = 7;
	float maxRelError[C] = {};
	float X[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		float f = rng.Float(1e-5f, 1e20f);
		float x = (float)(1.0 / (double)f); // best precision of the reciprocal.

		X[0] = Recip(f);
		X[1] = RecipFast(f);
#ifdef MATH_SSE
		X[2] = NewtonRhapsonRecip(f);
		X[3] = NewtonRhapsonRecip2(f);
#endif
		X[4] = OneOverX(f);
#ifdef MATH_SIMD
		X[5] = s4f_x(rcp_ps(set1_ps(f)));
#endif
#ifdef MATH_SSE2
		X[6] = (float)rcp_double(f);
#endif

		for(int j = 0; j < C; ++j)
			maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
	}

	LOGI("Max relative error with Recip: %e", maxRelError[0]);
	assert(maxRelError[0] < 1e-5f);
	LOGI("Max relative error with RecipFast: %e", maxRelError[1]);
	assert(maxRelError[1] < 5e-3f);
#ifdef MATH_SSE
	LOGI("Max relative error with NewtonRhapsonRecip: %e", maxRelError[2]);
	assert(maxRelError[2] < 1e-5f);
	LOGI("Max relative error with NewtonRhapsonRecip2: %e", maxRelError[3]);
	assert(maxRelError[3] < 1e-5f);
#endif
	LOGI("Max relative error with 1.f/x: %e", maxRelError[4]);
	assert(maxRelError[4] < 1e-6f);
#ifdef MATH_SIMD
	LOGI("Max relative error with rcp_ps: %e", maxRelError[5]);
	assert(maxRelError[5] < 1e-5f);
#endif
#ifdef MATH_SSE2
	LOGI("Max relative error with rcp_double: %e", maxRelError[6]);
	assert(maxRelError[6] < 1e-5f);
#endif
}

BENCHMARK(Recip, "Recip")
{
	f[i] = Recip(pf[i]);
}
BENCHMARK_END;

BENCHMARK(RecipFast, "RecipFast")
{
	f[i] = RecipFast(pf[i]);
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(NewtonRhapsonRecip, "test against RecipFast")
{
	f[i] = NewtonRhapsonRecip(pf[i]);
}
BENCHMARK_END;

BENCHMARK(NewtonRhapsonRecip2, "test against RecipFast")
{
	f[i] = NewtonRhapsonRecip2(pf[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(OneOverX, "rest against Recip")
{
	f[i] = OneOverX(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Sin, "Sin")
{
	f[i] = Sin(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sin, "sin")
{
	f[i] = sin(pf[i]);
}
BENCHMARK_END;

#if defined(_MSC_VER) && !defined(_M_X64) && !defined(WIN8RT)
// For historical purposes, compare how the FSIN instruction does against SIMD.
// On VS2013 32-bit, not very well:
// Benchmark 'Sin': Sin Best : 21.082 nsecs / 35.428 ticks, Avg : 26.472 nsecs, Worst : 29.514 nsecs
// Benchmark 'x87_sin' : FSIN Best : 49.994 nsecs / 84.866 ticks, Avg : 52.873 nsecs, Worst : 57.222 nsecs
double x87_sin(double x)
{
	double sine;
	__asm {
		fld x
		fsin
		fstp sine
	}
	return sine;
}

BENCHMARK(x87_sin, "FSIN")
{
	f[i] = (float)x87_sin((double)pf[i]);
}
BENCHMARK_END;

#endif

// Another interesting comparison: how fast is sin() via precomputed lookup tables
// compared to SIMD implementation?
// Test a lookup table implementation adapted from http://www.flipcode.com/archives/Fast_Trigonometry_Functions_Using_Lookup_Tables.shtml
#define MAX_CIRCLE_ANGLE      65536
#define HALF_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE (MAX_CIRCLE_ANGLE - 1)
#define PI 3.14159265358979323846f
static float fast_cossin_table[MAX_CIRCLE_ANGLE];           // Declare table of fast cosinus and sinus
class Init_fast_cossin_table_
{
public:
	Init_fast_cossin_table_()
	{
		// Build cossin table
		for(int i = 0; i < MAX_CIRCLE_ANGLE; i++)
			fast_cossin_table[i] = (float)sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
	}
};
Init_fast_cossin_table_ static_sincos_initializer;
static inline float sin_lookuptable(float n)
{
	int i = (int)(n * (HALF_MAX_CIRCLE_ANGLE / PI));
	if (i < 0) return fast_cossin_table[MAX_CIRCLE_ANGLE - ((-i)&MASK_MAX_CIRCLE_ANGLE)];
	else return fast_cossin_table[i&MASK_MAX_CIRCLE_ANGLE];
}
BENCHMARK(sin_lookuptable, "sin_lookuptable")
{
	f[i] = sin_lookuptable(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sinf, "sinf")
{
	f[i] = sinf(pf[i]);
}
BENCHMARK_END;

#ifdef MATH_SSE2

BENCHMARK(sin_ps, "sin_ps")
{
	f[i] = s4f_x(sin_ps(setx_ps(pf[i])));
}
BENCHMARK_END;

static const __m128 pi2 = _mm_set1_ps(2.f*pi);

FORCE_INLINE __m128 sin_ps_range_reduced(__m128 fl)
{
	return sin_ps(modf_ps(fl, pi2));
}

BENCHMARK(sin_ps_range_reduced, "sin_ps_range_reduced")
{
	f[i] = s4f_x(sin_ps_range_reduced(_mm_set_ss(pf[i])));
}
BENCHMARK_END;

UNIQUE_TEST(sin_ps_precision)
{
	const int C = 6;

	for(int l = 0; l < 2; ++l)
	{
		float maxRelError[C] = {};
		float maxAbsError[C] = {};
		float maxErrorArg[C] = {};
		float maxErrorVal[C] = {};
		float X[C] = {};

		float maxVal = (l == 0 ? 4.f*pi : 1e6f);
		for(int i = 0; i < 1000000; ++i)
		{
			float f = rng.Float(-maxVal, maxVal);//3.141592654f);
			//f = Mod(f, 2.f*pi);
//			Mod(f, 2.f*pi)
			const double pi_d = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
			float x = (float)sin(fmod((double)f, 2.0*pi_d)); // best precision of the Sin.

			X[0] = s4f_x(sin_ps(setx_ps(f)));
			X[1] = s4f_x(sin_ps_range_reduced(setx_ps(f)));
			X[2] = sinf(f);
			X[3] = Sin(f);
			X[4] = (float)sin((double)f);
			X[5] = sin_lookuptable(f);

			for(int j = 0; j < C; ++j)
			{
				float absError = Abs(x - X[j]);
				if (absError > maxAbsError[j])
				{
					maxAbsError[j] = absError;
					maxErrorArg[j] = f;
					maxErrorVal[j] = X[j];
				}
				maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
			}
		}

		LOGI(" ");
		LOGI("With |x| < %f:", maxVal);
		LOGI("Max absolute error with sin_ps(x): %e, with sin_ps(%f)=%f vs correct %f", maxAbsError[0], maxErrorArg[0], maxErrorVal[0], sin(maxErrorArg[0]));
		LOGI("Max absolute error with sin_ps_range_reduced(x): %e, with sin_ps_range_reduced(%f)=%f vs correct %f", maxAbsError[1], maxErrorArg[1], maxErrorVal[1], sin(maxErrorArg[1]));
		LOGI("Max absolute error with sinf(x): %e, with sinf(%f) = %f vs correct %f", maxAbsError[2], maxErrorArg[2], maxErrorVal[2], sin(maxErrorArg[2]));
		LOGI("Max absolute error with Sin(x): %e, with Sin(%f) = %f vs correct %f", maxAbsError[3], maxErrorArg[3], maxErrorVal[3], sin(maxErrorArg[3]));
		LOGI("Max absolute error with unwrapped sin(x): %e, with unwrapped sin(%f) = %f vs correct %f", maxAbsError[4], maxErrorArg[4], maxErrorVal[4], sin(maxErrorArg[4]));
		LOGI("Max absolute error with sin_lookuptable(x): %e, with sin_lookuptable(%f) = %f vs correct %f", maxAbsError[5], maxErrorArg[5], maxErrorVal[5], sin(maxErrorArg[5]));
		LOGI(" ");

		LOGI("Max relative error with sin_ps(x): %e",  maxRelError[0]);
		LOGI("Max relative error with sin_ps_range_reduced(x): %e",  maxRelError[1]);
		LOGI("Max relative error with sinf(x): %e",  maxRelError[2]);
		LOGI("Max relative error with Sin(x): %e",  maxRelError[3]);
		LOGI("Max relative error with unwrapped sin(x): %e",  maxRelError[4]);
		LOGI("Max relative error with sin_lookuptable(x): %e", maxRelError[5]);
	}
}

#endif

BENCHMARK(Cos, "Cos")
{
	f[i] = Cos(pf[i]);
}
BENCHMARK_END;

BENCHMARK(SinCos, "SinCos")
{
	SinCos(pf[i], pf[i+1], pf[i+2]);
}
BENCHMARK_END;

static float2 Sin_Cos(float angleRadians)
{
	return float2(Sin(angleRadians), Cos(angleRadians));
}

static float2 sinfcosf(float angleRadians)
{
	return float2(sinf(angleRadians), cosf(angleRadians));
}

BENCHMARK(Sin_Cos, "Sin()+Cos()")
{
	fl_2[i] = Sin_Cos(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sinfcosf, "sinfcosf")
{
	fl_2[i] = sinfcosf(pf[i]);
}
BENCHMARK_END;

#ifdef MATH_SSE2
float2 sincos_ps_float2(float angleRadians)
{
	__m128 angle = modf_ps(setx_ps(angleRadians), pi2);
	__m128 sin, cos;
	sincos_ps(angle, &sin, &cos);
	return float2(s4f_x(sin), s4f_x(cos));
}

BENCHMARK(sincos_ps, "sincos_ps")
{
	fl_2[i] = sincos_ps_float2(pf[i]);
}
BENCHMARK_END;

#endif

BENCHMARK(Tan, "Tan")
{
	f[i] = Tan(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Asin, "Asin")
{
	f[i] = Asin(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Acos, "Acos")
{
	f[i] = Acos(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Atan, "Atan")
{
	f[i] = Atan(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Atan2, "Atan2")
{
	f[i] = Atan2(f[i], pf[i]);
}
BENCHMARK_END;

BENCHMARK(Sinh, "Sinh")
{
	f[i] = Sinh(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Cosh, "Cosh")
{
	f[i] = Cosh(pf[i]);
}
BENCHMARK_END;

BENCHMARK(Tanh, "Tanh")
{
	f[i] = Tanh(pf[i]);
}
BENCHMARK_END;

#ifdef min
#undef min
#endif

RANDOMIZED_TEST(Min)
{
	float a = rng.Float(-1e9, 1e9);
	float b = rng.Float(-1e9, 1e9);
	if (rng.Int(0,10) == 0) a = -FLOAT_INF;
	if (rng.Int(0,10) == 0) a = FLOAT_INF;
	if (rng.Int(0,10) == 0) b = -FLOAT_INF;
	if (rng.Int(0,10) == 0) b = FLOAT_INF;
	float correctMin = std::min(a,b);
	float mglMin = Min(a, b);
	assert(correctMin == mglMin);
	MARK_UNUSED(correctMin);
	MARK_UNUSED(mglMin);
}

#ifdef max
#undef max
#endif

RANDOMIZED_TEST(Max)
{
	float a = rng.Float(-1e9, 1e9);
	float b = rng.Float(-1e9, 1e9);
	if (rng.Int(0,10) == 0) a = -FLOAT_INF;
	if (rng.Int(0,10) == 0) a = FLOAT_INF;
	if (rng.Int(0,10) == 0) b = -FLOAT_INF;
	if (rng.Int(0,10) == 0) b = FLOAT_INF;
	float correctMax = std::max(a,b);
	float mglMax = Max(a, b);
	assert(correctMax == mglMax);
	MARK_UNUSED(correctMax);
	MARK_UNUSED(mglMax);
}

BENCHMARK(Min, "Min")
{
	f[i] = Min(pf[i], uf[i]);
	f[i+1] = Min(pf[i+1], uf[i+1]);
	f[i+2] = Min(pf[i+2], uf[i+2]);
	f[i+3] = Min(pf[i+3], uf[i+3]);
	f[i+4] = Min(pf[i+4], uf[i+4]);
	f[i+5] = Min(pf[i+5], uf[i+5]);
	f[i+6] = Min(pf[i+6], uf[i+6]);
	f[i+7] = Min(pf[i+7], uf[i+7]);
}
BENCHMARK_END;

#ifdef MATH_SSE

// Test if explicitly storing the destination gives better codegen.
// (does not seem to be the case)
FORCE_INLINE void Min_SSE_dst(float *dst, float a, float b)
{
	_mm_store_ss(dst, _mm_min_ss(setx_ps(a), setx_ps(b)));
}

BENCHMARK(Min_SSE_dst, "Min SSE with dst pointer")
{
	Min_SSE_dst(f+i, pf[i], uf[i]);
	Min_SSE_dst(f+i+1, pf[i + 1], uf[i + 1]);
	Min_SSE_dst(f+i+2, pf[i + 2], uf[i + 2]);
	Min_SSE_dst(f+i+3, pf[i + 3], uf[i + 3]);
	Min_SSE_dst(f+i+4, pf[i + 4], uf[i + 4]);
	Min_SSE_dst(f+i+5, pf[i + 5], uf[i + 5]);
	Min_SSE_dst(f+i+6, pf[i + 6], uf[i + 6]);
	Min_SSE_dst(f+i+7, pf[i + 7], uf[i + 7]);
}
BENCHMARK_END;

#endif

template<typename T>
T MinWithTemplate(const T &a, const T &b)
{
	return a <= b ? a : b;
}

BENCHMARK(Min_template, "Min with template<>")
{
	f[i] = MinWithTemplate(pf[i], uf[i]);
	f[i+1] = MinWithTemplate(pf[i+1], uf[i+1]);
	f[i+2] = MinWithTemplate(pf[i+2], uf[i+2]);
	f[i+3] = MinWithTemplate(pf[i+3], uf[i+3]);
	f[i+4] = MinWithTemplate(pf[i+4], uf[i+4]);
	f[i+5] = MinWithTemplate(pf[i+5], uf[i+5]);
	f[i+6] = MinWithTemplate(pf[i+6], uf[i+6]);
	f[i+7] = MinWithTemplate(pf[i+7], uf[i+7]);
}
BENCHMARK_END;

#define MinTernary(a,b) ((a) <= (b) ? (a) : (b))
BENCHMARK(Min_ternary, "Min with ternary")
{
	f[i] = MinTernary(pf[i], uf[i]);
	f[i+1] = MinTernary(pf[i+1], uf[i+1]);
	f[i+2] = MinTernary(pf[i+2], uf[i+2]);
	f[i+3] = MinTernary(pf[i+3], uf[i+3]);
	f[i+4] = MinTernary(pf[i+4], uf[i+4]);
	f[i+5] = MinTernary(pf[i+5], uf[i+5]);
	f[i+6] = MinTernary(pf[i+6], uf[i+6]);
	f[i+7] = MinTernary(pf[i+7], uf[i+7]);
}
BENCHMARK_END;

// Test Emscripten-specific effects of a function call.
#ifdef __EMSCRIPTEN__
float __attribute__((noinline)) Min_funccall(float a, float b)
{
	return MinTernary(a, b);
}

BENCHMARK(Min_funccall, "Min with funccall")
{
	f[i] = Min_funccall(pf[i], uf[i]);
	f[i+1] = Min_funccall(pf[i+1], uf[i+1]);
	f[i+2] = Min_funccall(pf[i+2], uf[i+2]);
	f[i+3] = Min_funccall(pf[i+3], uf[i+3]);
	f[i+4] = Min_funccall(pf[i+4], uf[i+4]);
	f[i+5] = Min_funccall(pf[i+5], uf[i+5]);
	f[i+6] = Min_funccall(pf[i+6], uf[i+6]);
	f[i+7] = Min_funccall(pf[i+7], uf[i+7]);
}
BENCHMARK_END;
#endif

#ifndef _MSC_VER
BENCHMARK(Min_fmin, "C99/C++11 fmin")
{
	f[i] = fmin(pf[i], uf[i]);
	f[i+1] = fmin(pf[i+1], uf[i+1]);
	f[i+2] = fmin(pf[i+2], uf[i+2]);
	f[i+3] = fmin(pf[i+3], uf[i+3]);
	f[i+4] = fmin(pf[i+4], uf[i+4]);
	f[i+5] = fmin(pf[i+5], uf[i+5]);
	f[i+6] = fmin(pf[i+6], uf[i+6]);
	f[i+7] = fmin(pf[i+7], uf[i+7]);
}
BENCHMARK_END;
#endif

UNIQUE_TEST(IsPow2)
{
	assert(IsPow2(0));
	assert(IsPow2(1));
	assert(IsPow2(2));
	assert(!IsPow2(3));
	assert(IsPow2(4));
	assert(!IsPow2(6));
	assert(IsPow2(8));
	assert(IsPow2(65536));
	assert(!IsPow2(3423626));
	assert(!IsPow2((u32)-1));
	assert(!IsPow2((u32)0xFFFFFFFFU));
	assert(IsPow2((u32)0x40000000U));
	assert(IsPow2((u32)0x80000000U));
}

UNIQUE_TEST(IsPow2_u64)
{
	assert(IsPow2(0ULL));
	assert(IsPow2(1ULL));
	assert(IsPow2(2ULL));
	assert(!IsPow2(3ULL));
	assert(IsPow2(4ULL));
	assert(!IsPow2(6ULL));
	assert(IsPow2(8ULL));
	assert(IsPow2(65536ULL));
	assert(!IsPow2(3423626ULL));
	assert(!IsPow2((u64)-1));
	assert(!IsPow2(0xFFFFFFFFULL));
	assert(!IsPow2(0xFFFFFFFFFFFFFFFFULL));
	assert(IsPow2(0x40000000ULL));
	assert(IsPow2(0x80000000ULL));
	assert(IsPow2(0x100000000ULL));
	assert(IsPow2(0x4000000000000000ULL));
	assert(IsPow2(0x8000000000000000ULL));
}

UNIQUE_TEST(RoundUpPow2)
{
	asserteq(RoundUpPow2(0), 0);
	asserteq(RoundUpPow2(1), 1);
	asserteq(RoundUpPow2(2), 2);
	asserteq(RoundUpPow2(3), 4);
	asserteq(RoundUpPow2(4), 4);
	asserteq(RoundUpPow2(5), 8);
	asserteq(RoundUpPow2(64241), 65536);
	asserteq(RoundUpPow2((u32)0x7FFFFFFFU), 0x80000000U);
	asserteq(RoundUpPow2((u32)0x80000000U), 0x80000000U);
	asserteq(RoundUpPow2((u32)0x80000001U), 0);
	asserteq(RoundUpPow2((u32)0xFE873210U), 0);
	asserteq(RoundUpPow2((u32)0xFFFFFFFFU), 0);
}

UNIQUE_TEST(RoundUpPow2_u64)
{
	asserteq(RoundUpPow2(0ULL), 0);
	asserteq(RoundUpPow2(1ULL), 1);
	asserteq(RoundUpPow2(2ULL), 2);
	asserteq(RoundUpPow2(3ULL), 4);
	asserteq(RoundUpPow2(4ULL), 4);
	asserteq(RoundUpPow2(5ULL), 8);
	asserteq(RoundUpPow2(64241ULL), 65536);
	asserteq(RoundUpPow2(0x7FFFFFFFULL), 0x80000000U);
	asserteq(RoundUpPow2(0x80000000ULL), 0x80000000U);
	asserteq(RoundUpPow2(0x80000001ULL), 0x100000000ULL);
	asserteq(RoundUpPow2(0xFE873210ULL), 0x100000000ULL);
	asserteq(RoundUpPow2(0xFFFFFFFFULL), 0x100000000ULL);
	asserteq(RoundUpPow2(0x7FFFFFFFFFFFFFFFULL), 0x8000000000000000ULL);
	asserteq(RoundUpPow2(0x8000000000000000ULL), 0x8000000000000000ULL);
	asserteq(RoundUpPow2(0x8000000000000001ULL), 0);
	asserteq(RoundUpPow2(0xFE87321084208274ULL), 0);
	asserteq(RoundUpPow2(0xFFFFFFFFFFFFFFFFULL), 0);
}

UNIQUE_TEST(RoundDownPow2)
{
	asserteq(RoundDownPow2(0), 0);
	asserteq(RoundDownPow2(1), 1);
	asserteq(RoundDownPow2(2), 2);
	asserteq(RoundDownPow2(3), 2);
	asserteq(RoundDownPow2(4), 4);
	asserteq(RoundDownPow2(5), 4);
	asserteq(RoundDownPow2(64241), 32768);
	asserteq(RoundDownPow2((u32)0x7FFFFFFFU), 0x40000000U);
	asserteq(RoundDownPow2((u32)0x80000000U), 0x80000000U);
	asserteq(RoundDownPow2((u32)0x80000001U), 0x80000000U);
	asserteq(RoundDownPow2((u32)0xFE873210U), 0x80000000U);
	asserteq(RoundDownPow2((u32)0xFFFFFFFFU), 0x80000000U);
}

UNIQUE_TEST(RoundDownPow2_u64)
{
	asserteq(RoundDownPow2(0ULL), 0);
	asserteq(RoundDownPow2(1ULL), 1);
	asserteq(RoundDownPow2(2ULL), 2);
	asserteq(RoundDownPow2(3ULL), 2);
	asserteq(RoundDownPow2(4ULL), 4);
	asserteq(RoundDownPow2(5ULL), 4);
	asserteq(RoundDownPow2(64241ULL), 32768);
	asserteq(RoundDownPow2(0x7FFFFFFFULL), 0x40000000U);
	asserteq(RoundDownPow2(0x80000000ULL), 0x80000000U);
	asserteq(RoundDownPow2(0x80000001ULL), 0x80000000ULL);
	asserteq(RoundDownPow2(0xFE873210ULL), 0x80000000ULL);
	asserteq(RoundDownPow2(0xFFFFFFFFULL), 0x80000000ULL);
	asserteq(RoundDownPow2(0x7FFFFFFFFFFFFFFFULL), 0x4000000000000000ULL);
	asserteq(RoundDownPow2(0x8000000000000000ULL), 0x8000000000000000ULL);
	asserteq(RoundDownPow2(0x8000000000000001ULL), 0x8000000000000000ULL);
	asserteq(RoundDownPow2(0xFE87321084208274ULL), 0x8000000000000000ULL);
	asserteq(RoundDownPow2(0xFFFFFFFFFFFFFFFFULL), 0x8000000000000000ULL);
}

UNIQUE_TEST(LSB_U32)
{
	asserteq(LSB((u32)0), 0);
	asserteq(LSB((u32)1), 1);
	asserteq(LSB((u32)2), 3);
	asserteq(LSB((u32)3), 7);
	asserteq(LSB((u32)16), 0xFFFFU);
	asserteq(LSB((u32)31), 0x7FFFFFFFU);
	asserteq(LSB((u32)32), 0xFFFFFFFFU);
}

UNIQUE_TEST(LSB_U64)
{
	asserteq(LSB64((u64)0), 0);
	asserteq(LSB64((u64)1), 1);
	asserteq(LSB64((u64)2), 3);
	asserteq(LSB64((u64)3), 7);
	asserteq(LSB64((u64)16), 0xFFFFULL);
	asserteq(LSB64((u64)31), 0x7FFFFFFFULL);
	asserteq(LSB64((u64)32), 0xFFFFFFFFULL);
	asserteq(LSB64((u64)33), 0x1FFFFFFFFULL);
	asserteq(LSB64((u64)63), 0x7FFFFFFFFFFFFFFFULL);
	asserteq(LSB64((u64)64), 0xFFFFFFFFFFFFFFFFULL);
}

UNIQUE_TEST(Sin_lookuptable)
{
	asserteq(Sin(0.f), 0.f);
	assert1(EqualAbs(Sin(pi), 0.f, 1e-7f), Sin(pi));
	assert1(EqualAbs(Sin(-pi), 0.f, 1e-7f), Sin(-pi));
	assert1(EqualAbs(Sin(-37.6991577f), 0.f, 1e-4f), Sin(-37.6991577f));
	assert1(EqualAbs(Sin(pi/2.f), 1.f, 1e-7f), Sin(pi/2.f));
	assert1(EqualAbs(Sin(-pi/2.f), -1.f, 1e-4f), Sin(-pi/2.f));
	assert1(EqualAbs(Sin(pi/4.f), 1.f/Sqrt(2.f), 1e-4f), Sin(pi/4.f));
}

UNIQUE_TEST(Cos_lookuptable)
{
	asserteq(Cos(0.f), 1.f);
	assert1(EqualAbs(Cos(pi), -1.f, 1e-7f), Cos(pi));
	assert1(EqualAbs(Cos(-pi), -1.f, 1e-7f), Cos(-pi));
	assert1(EqualAbs(Cos(-37.6991577f), 1.f, 1e-4f), Cos(-37.6991577f));
	assert1(EqualAbs(Cos(pi / 2.f), 0.f, 1e-7f), Cos(pi / 2.f));
	assert1(EqualAbs(Cos(-pi / 2.f), -0.f, 1e-4f), Cos(-pi / 2.f));
	assert1(EqualAbs(Cos(pi / 4.f), 1.f/Sqrt(2.f), 1e-4f), Cos(pi / 4.f));
}
