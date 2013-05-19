#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include <cmath>

#ifdef MATH_SSE2
#include "../src/Math/sse_mathfun.h"
#endif

using namespace TestData;

bool TrueCondition() // This is a function that always returns true, but it is impossible for compiler to know that.
{
	tick_t t1 = Clock::Tick();
	tick_t t2 = Clock::Tick();
	return t1 != 0 || t2 != 10000000;
}

#ifdef FAIL_USING_EXCEPTIONS

#ifndef EMSCRIPTEN ///\todo This breaks on Emscripten!
UNIQUE_TEST(ExceptionGoesThrough)
{
	LOGI("Testing exception throwing.");
	if (TrueCondition())
		throw std::runtime_error("expect failure");
}
#endif

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


#ifndef EMSCRIPTEN // long double is not supported.
	assert(std::isfinite(5.0L));
	assert(isfinite(5.0L));
#endif

}
#endif

#if !defined(EMSCRIPTEN)
struct U80
{
	u8 data[16];

	explicit U80(long double d)
	{
		assert(sizeof(long double) <= 16);
		memset(data, 0, sizeof(data));
		memcpy(data, &d, sizeof(d));
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

#if !defined(EMSCRIPTEN)
	LOGI("sizeof(long double): %d", (int)sizeof(long double));
	LOGI("LDBL_DIG: %d, LDBL_MANT_DIG: %d", (int)LDBL_DIG, (int)LDBL_MANT_DIG);
#if !defined(__MINGW32__) // MinGW doesn't support printing out long doubles
	LOGI("FLOAT_NAN as long double: %Lf, or %s", (long double)FLOAT_NAN, U80((long double)FLOAT_NAN).ToString().c_str());
	LOGI("FLOAT_INF as long double: %Lf, or %s", (long double)FLOAT_INF, U80((long double)FLOAT_INF).ToString().c_str());
#endif

#endif
}

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

UNIQUE_TEST(sqrt_precision)
{
	const int C = 7;
	float maxRelError[C] = {};
	float X[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		float f = rng.Float(0.f, 1e20f);
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
	u32 i = ReinterpretAsU32(x);//*(int*)&x; // store floating-point bits in integer
	i = 0x5f375a86 - (i >> 1); // A better initial value: http://en.wikipedia.org/wiki/Fast_inverse_square_root#History_and_investigation
	//i = 0x5f3759d5 - (i >> 1); // initial guess for Newton's method
	x = ReinterpretAsFloat(i);//*(float*)&i; // convert new bits into float
	x = x*(1.5f - xhalf*x*x); // One round of Newton's method
	return x;
}

UNIQUE_TEST(sqrt_rsqrt_precision)
{
	const int C = 7;
	float maxRelError[C] = {};
	float X[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		float f = rng.Float(1e-5f, 1e20f);
		float x = (float)(1.0 / sqrt((double)f)); // best precision of the rsqrt.

		X[0] = RSqrt(f);
		X[1] = recip_sqrtf(f);
		X[2] = sqrtf_recip(f);
		X[3] = QuakeInvSqrt(f);
		X[4] = RSqrtFast(f);

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
	assert(maxRelError[4] < 1e-3f);
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
	__m128 X = FLOAT_TO_M128(x);
	__m128 e = _mm_rcp_ss(X);
	// 1/x = D
	// f(e) = e^-1 - x
	// f'(e) = -e^-2

	// e_n = e + (e^-1 - x) / e^-2
	// e_n = e + e - x*e^2
	// e_n = 2*e - x*e^2

	// Do one iteration of Newton-Rhapson:
	__m128 e2 = _mm_mul_ss(e,e);
	
	return M128_TO_FLOAT(_mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2)));
}

float NewtonRhapsonRecip2(float x)
{
	__m128 X = FLOAT_TO_M128(x);
	__m128 e = _mm_rcp_ss(X);
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
	return M128_TO_FLOAT(_mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2)));
}
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

		for(int j = 0; j < C; ++j)
			maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
	}

	LOGI("Max relative error with Recip: %e", maxRelError[0]);
	assert(maxRelError[0] < 1e-5f);
	LOGI("Max relative error with RecipFast: %e", maxRelError[1]);
	assert(maxRelError[1] < 1e-3f);
#ifdef MATH_SSE
	LOGI("Max relative error with NewtonRhapsonRecip: %e", maxRelError[2]);
	assert(maxRelError[2] < 1e-5f);
	LOGI("Max relative error with NewtonRhapsonRecip2: %e", maxRelError[3]);
	assert(maxRelError[3] < 1e-5f);
#endif
	LOGI("Max relative error with 1.f/x: %e", maxRelError[4]);
	assert(maxRelError[4] < 1e-6f);
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

BENCHMARK(sin, "test against Sin")
{
	f[i] = sin(pf[i]);
}
BENCHMARK_END;

BENCHMARK(sinf, "test against Sin")
{
	f[i] = sinf(pf[i]);
}
BENCHMARK_END;

#ifdef MATH_SSE2

BENCHMARK(sin_ps, "test against Sin")
{
	f[i] = M128_TO_FLOAT(sin_ps(FLOAT_TO_M128(pf[i])));
}
BENCHMARK_END;

UNIQUE_TEST(sin_ps_precision)
{
	const int C = 3;
	float maxRelError[C] = {};
	float maxAbsError[C] = {};
	float X[C] = {};

	for(int l = 0; l < 2; ++l)
	{
		float maxVal = (l == 0 ? 4.f*pi : 1e8f);
		for(int i = 0; i < 1000000; ++i)
		{
			float f = rng.Float(-maxVal, maxVal);//3.141592654f);
			float x = (float)sin((double)f); // best precision of the Sin.

			X[0] = M128_TO_FLOAT(sin_ps(FLOAT_TO_M128(f)));
			X[1] = sinf(f);
			X[2] = Sin(f);

			for(int j = 0; j < C; ++j)
			{
				maxRelError[j] = Max(RelativeError(x, X[j]), maxRelError[j]);
				maxAbsError[j] = Abs(x - X[j]);
			}
		}

		LOGI("With |x| < %f:", maxVal);
		LOGI("Max absolute error with sin_ps(x): %e",  maxAbsError[0]);
		LOGI("Max absolute error with sinf(x): %e",  maxAbsError[1]);
		LOGI("Max absolute error with Sin(x): %e",  maxAbsError[2]);
		LOGI(" ");

		LOGI("Max relative error with sin_ps(x): %e",  maxRelError[0]);
		LOGI("Max relative error with sinf(x): %e",  maxRelError[1]);
		LOGI("Max relative error with Sin(x): %e",  maxRelError[2]);
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
	fl_2[i] = SinCos(pf[i]);
}
BENCHMARK_END;

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
