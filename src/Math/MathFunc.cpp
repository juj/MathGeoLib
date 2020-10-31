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

/** @file MathFunc.cpp
	@author Jukka Jylänki
	@brief Common mathematical functions. */
#include "MathFunc.h"
#include "Swap.h"
#include "SSEMath.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#include <algorithm>
#endif

#include "myassert.h"
#include "float2.h"
#include "float4.h"
#ifdef MATH_WITH_GRISU3
#include "grisu3.h"
#endif

#ifdef WIN32
#include "../Math/InclWindows.h"
#endif

#ifdef MATH_SSE2
#include "sse_mathfun.h"
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/em_math.h>
#endif

MATH_BEGIN_NAMESPACE

bool mathBreakOnAssume =
#ifdef MATH_STARTUP_BREAK_ON_ASSUME
	true;
#else
	false;
#endif

void SetMathBreakOnAssume(bool isEnabled)
{
	mathBreakOnAssume = isEnabled;
}

/// Returns the current state of the math break-on-assume flag.
bool MathBreakOnAssume()
{
	return mathBreakOnAssume;
}

bool AssumeFailed()
{
#ifndef OPTIMIZED_RELEASE
	if (mathBreakOnAssume)
	{
#if defined(WIN32) && !defined(WIN8RT) // Win8 metro apps don't have DebugBreak.
		DebugBreak();
#elif defined(__EMSCRIPTEN__)
		emscripten_debugger();
// TODO: Test locally on Linux GCC and enable
//#elif defined(__clang__) && __has_builtin(__builtin_debugtrap)
//		__builtin_debugtrap();
#endif
	}
#endif
	// If your debugger is breaking in this function, it means that an assume() failure has occurred,
	// or a LOGE()/LOGW() failure has occurred, and building with trap-to-debugger enabled. Navigate
	// up the callstack to find the offending code that raised the error.
	return mathBreakOnAssume;
}

#if defined(__EMSCRIPTEN__) && !defined(MATH_USE_SINCOS_LOOKUPTABLE)
// On Emscripten using lookup tables has been profiled to be significantly faster.
#define MATH_USE_SINCOS_LOOKUPTABLE
#endif

#define MAX_CIRCLE_ANGLE           65536
#define HALF_MAX_CIRCLE_ANGLE     (MAX_CIRCLE_ANGLE/2)
#define QUARTER_MAX_CIRCLE_ANGLE  (MAX_CIRCLE_ANGLE/4)
#define MASK_MAX_CIRCLE_ANGLE     (MAX_CIRCLE_ANGLE - 1)
#define PI                        3.14159265358979323846f

#ifdef MATH_USE_SINCOS_LOOKUPTABLE

// A lookup table implementation adapted from http://www.flipcode.com/archives/Fast_Trigonometry_Functions_Using_Lookup_Tables.shtml
static float fast_cossin_table[MAX_CIRCLE_ANGLE];           // Declare table of fast cosinus and sinus

class Init_fast_cossin_table
{
public:
	Init_fast_cossin_table()
	{
		// Build cossin table
		for(int i = 0; i < MAX_CIRCLE_ANGLE; i++)
#ifdef __EMSCRIPTEN__
			fast_cossin_table[i] = (float)emscripten_math_sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
#else
			fast_cossin_table[i] = (float)sin((double)i * PI / HALF_MAX_CIRCLE_ANGLE);
#endif
	}
};
Init_fast_cossin_table static_initializer;

static inline float sin_lookuptable(float n)
{
	int i = (int)(n * (HALF_MAX_CIRCLE_ANGLE / PI));
	if (i < 0) return -fast_cossin_table[(-i) & MASK_MAX_CIRCLE_ANGLE];
	else return fast_cossin_table[i & MASK_MAX_CIRCLE_ANGLE];
}

static inline float cos_lookuptable(float n)
{
	int i = (int)(n * (HALF_MAX_CIRCLE_ANGLE / PI));
	if (i < 0) return fast_cossin_table[(QUARTER_MAX_CIRCLE_ANGLE - i) & MASK_MAX_CIRCLE_ANGLE];
	else return fast_cossin_table[(QUARTER_MAX_CIRCLE_ANGLE + i) & MASK_MAX_CIRCLE_ANGLE];
}

static inline void sincos_lookuptable(float n, float &sinOut, float &cosOut)
{
	int i = (int)(n * (HALF_MAX_CIRCLE_ANGLE / PI));
	i = (i >= 0) ? (i & MASK_MAX_CIRCLE_ANGLE) : (MAX_CIRCLE_ANGLE - ((-i) & MASK_MAX_CIRCLE_ANGLE));
	sinOut = fast_cossin_table[i];
	cosOut = fast_cossin_table[(QUARTER_MAX_CIRCLE_ANGLE + i) & MASK_MAX_CIRCLE_ANGLE];
}

static inline void sincos_lookuptable_u16ScaledRadians(u16 u16ScaledRadians, float &sinOut, float &cosOut)
{
	sinOut = fast_cossin_table[u16ScaledRadians];
	cosOut = fast_cossin_table[(u16)(QUARTER_MAX_CIRCLE_ANGLE + u16ScaledRadians)];
}

#endif

#ifdef MATH_SSE2
static const __m128 pi2 = _mm_set1_ps(2.f*pi);
#endif

float Sin(float angleRadians)
{
#ifdef MATH_USE_SINCOS_LOOKUPTABLE
	return sin_lookuptable(angleRadians);
#elif defined(MATH_SSE2)
	// Do range reduction by 2pi before calling sin - this enchances precision of sin_ps a lot
	return s4f_x(sin_ps(modf_ps(setx_ps(angleRadians), pi2)));
#else
	return sinf(angleRadians);
#endif
}

float Cos(float angleRadians)
{
#ifdef MATH_USE_SINCOS_LOOKUPTABLE
	return cos_lookuptable(angleRadians);
#elif defined(MATH_SSE2)
	// Do range reduction by 2pi before calling cos - this enchances precision of cos_ps a lot
	return s4f_x(cos_ps(modf_ps(setx_ps(angleRadians), pi2)));
#else
	return cosf(angleRadians);
#endif
}

float Tan(float angleRadians)
{
#ifdef __EMSCRIPTEN__
	// Use Math.tan() for minimal code size
	return emscripten_math_tan(angleRadians);
#else
	return tanf(angleRadians);
#endif
}

void SinCos(float angleRadians, float &outSin, float &outCos)
{
#ifdef MATH_USE_SINCOS_LOOKUPTABLE
	return sincos_lookuptable(angleRadians, outSin, outCos);
#elif defined(MATH_SSE2)
	__m128 angle = modf_ps(setx_ps(angleRadians), pi2);
	__m128 sin, cos;
	sincos_ps(angle, &sin, &cos);
	outSin = s4f_x(sin);
	outCos = s4f_x(cos);
#else
	outSin = Sin(angleRadians);
	outCos = Cos(angleRadians);
#endif
}

void SinCosU16ScaledRadians(u16 u16ScaledRadians, float &outSin, float &outCos)
{
#ifdef MATH_USE_SINCOS_LOOKUPTABLE
	return sincos_lookuptable_u16ScaledRadians(u16ScaledRadians, outSin, outCos);
#elif defined(MATH_SSE2)
	float angleRadians = u16ScaledRadians * (PI / HALF_MAX_CIRCLE_ANGLE);
	__m128 angle = modf_ps(setx_ps(angleRadians), pi2);
	__m128 sin, cos;
	sincos_ps(angle, &sin, &cos);
	outSin = s4f_x(sin);
	outCos = s4f_x(cos);
#else
	float angleRadians = u16ScaledRadians * (PI / HALF_MAX_CIRCLE_ANGLE);
	outSin = Sin(angleRadians);
	outCos = Cos(angleRadians);
#endif
}

void SinCos2(const float4 &angleRadians, float4 &outSin, float4 &outCos)
{
#ifdef MATH_SSE2
	__m128 angle = modf_ps(angleRadians.v, pi2);
	sincos_ps(angle, &outSin.v, &outCos.v);
#else
	SinCos(angleRadians.x, outSin.x, outCos.x);
	SinCos(angleRadians.y, outSin.y, outCos.y);
#endif
}

void SinCos3(const float4 &angleRadians, float4 &outSin, float4 &outCos)
{
#ifdef MATH_SSE2
	__m128 angle = modf_ps(angleRadians.v, pi2);
	sincos_ps(angle, &outSin.v, &outCos.v);
#else
	SinCos(angleRadians.x, outSin.x, outCos.x);
	SinCos(angleRadians.y, outSin.y, outCos.y);
	SinCos(angleRadians.z, outSin.z, outCos.z);
#endif
}

void SinCos4(const float4 &angleRadians, float4 &outSin, float4 &outCos)
{
#ifdef MATH_SSE2
	__m128 angle = modf_ps(angleRadians.v, pi2);
	sincos_ps(angle, &outSin.v, &outCos.v);
#else
	SinCos(angleRadians.x, outSin.x, outCos.x);
	SinCos(angleRadians.y, outSin.y, outCos.y);
	SinCos(angleRadians.z, outSin.z, outCos.z);
	SinCos(angleRadians.w, outSin.w, outCos.w);
#endif
}

float Asin(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.asin() for minimal code size
	return emscripten_math_asin(x);
#else
	return asinf(x);
#endif
}

float Acos(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.acos() for minimal code size
	return emscripten_math_acos(x);
#else
	return acosf(x);
#endif
}

float Atan(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.atan() for minimal code size
	return emscripten_math_atan(x);
#else
	return atanf(x);
#endif
}

float Atan2(float y, float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.atan2() for minimal code size
	return emscripten_math_atan2(y, x);
#else
	return atan2f(y, x);
#endif
}

float Sinh(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.sinh() for minimal code size
	return emscripten_math_sinh(x);
#else
	return sinhf(x);
#endif
}

float Cosh(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.atan() for minimal code size
	return emscripten_math_cosh(x);
#else
	return coshf(x);
#endif
}

float Tanh(float x)
{
#ifdef __EMSCRIPTEN__
	// Use Math.tanh() for minimal code size
	return emscripten_math_tanh(x);
#else
	return tanhf(x);
#endif
}

bool IsPow2(u32 number)
{
	return (number & (number-1)) == 0;
}

bool IsPow2(u64 number)
{
	return (number & (number-1)) == 0;
}

u32 RoundUpPow2(u32 x)
{
	assert(sizeof(u32) == 4);
	--x;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	++x;

	return x;
}

u64 RoundUpPow2(u64 x)
{
	assert(sizeof(u64) == 8);
	--x;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	x |= x >> 32;
	++x;

	return x;
}

u32 RoundDownPow2(u32 x)
{
	assert(sizeof(u32) == 4);
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	return x - (x >> 1);
}

u64 RoundDownPow2(u64 x)
{
	assert(sizeof(u64) == 8);
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	x |= x >> 32;
	return x - (x >> 1);
}

int RoundIntUpToMultipleOfPow2(int x, int n)
{
	assert(IsPow2(n));
	return (x + n-1) & ~(n-1);
}

s64 RoundIntUpToMultipleOfPow2(s64 x, s64 n)
{
	assert(IsPow2(n));
	return (x + n-1) & ~(n-1);
}

float Pow(float base, float exponent)
{
	return pow(base, exponent);
}

float Exp(float exponent)
{
	return exp(exponent);
}

float Log(float base, float value)
{
	return log(value) / log(base);
}

float Log2(float value)
{
	return Log(2.f, value);
}

float Ln(float value)
{
	return log(value);
}

float Log10(float value)
{
	return Log(10.f, value);
}

float Ceil(float x)
{
	return ceilf(x);
}

int CeilInt(float x)
{
	return (int)ceilf(x);
}

float Floor(float x)
{
	return floorf(x);
}

int FloorInt(float x)
{
	return (int)floorf(x);
}

float Round(float x)
{
	return Floor(x+0.5f);
}

int RoundInt(float x)
{
	return (int)Round(x);
}

float Sign(float x)
{
	return x >= 0.f ? 1.f : -1.f;
}

float SignOrZero(float x, float epsilon)
{
	return Abs(x) <= epsilon ? 0.f : Sign(x);
}

float Lerp(float a, float b, float t)
{
	return a + t * (b-a);
}

float LerpMod(float a, float b, float mod, float t)
{
	a = ModPos(a, mod);
	b = ModPos(b, mod);
	if (Abs(b-a) * 2.f <= mod)
		return Lerp(a, b, t);
	else
	{
		if (a < b)
			return ModPos(Lerp(a + mod, b, t), mod);
		else
			return ModPos(Lerp(a, b + mod, t), mod);
	}
}

float InvLerp(float a, float b, float x)
{
	assume(Abs(b-a) > 1e-5f);
	return (x - a) / (b - a);
}

float Step(float y, float x)
{
	return (x >= y) ? 1.f : 0.f;
}

float Ramp(float min, float max, float x)
{
	return x <= min ? 0.f : (x >= max ? 1.f : (x - min) / (max - min));
}

float PingPongMod(float x, float mod)
{
	x = Mod(x, mod * 2.f);
	return x >= mod ? (2.f * mod - x) : x;
}

float Mod(float x, float mod)
{
	return fmod(x, mod);
}

float Mod(float x, int mod)
{
	///@todo Optimize.
	return fmod(x, (float)mod);
}

float ModPos(float x, float mod)
{
	float m = fmod(x, mod);
	return m >= 0.f ? m : (m + mod);
}

float ModPos(float x, int mod)
{
	///@todo Optimize.
	return ModPos(x, (float)mod);
}

float Frac(float x)
{
	return x - Floor(x);
}

/** Uses a recursive approach, not the fastest/brightest method.
	Note that 13! = 6227020800 overflows already.
	@return n! = n * (n-1) * (n-2) * ... * 1. */
int Factorial(int n)
{
	int result = 1;
	for(int i = 2; i <= n; i++)
		result *= i;
	return result;
}

/** @return Binomial coefficients with recursion, i.e. n choose k, C(n,k) or nCk. */
int CombinatorialRec(int n, int k)
{
	/* We could do:
			return factorial(n)/(factorial(n-k)*factorial(k));
		But prefer the recursive approach instead, because it's not so prone
		to numerical overflow. This approach uses the idea of the Pascal triangle. */

	if (k <= 0 || k >= n)
		return 1;
	else
		return CombinatorialRec(n-1,k-1) + CombinatorialRec(n-1,k);
}

/** @return Binomial coefficients by tabulation, i.e. n choose k, C(n,k) or nCk. */
int CombinatorialTab(int n, int k)
{
	if (k == 0 || k == n)
		return 1;
	if (k < 0 || k > n)
		return 0;
	// We use two auxiliary tables, one of size n-2 and one of size n-3.
	int *table = new int[2*(k+1)]; ///@todo We can lower this size.
	for(int i = 0; i < 2*(k+1); ++i)
		table[i] = 1;
	int *t1 = &table[0];
	int *t2 = &table[k+1];
	// Iteratively fill the tables.
	for(int i = 2; i <= n; ++i)
	{
		for(int j = Max(1, i-n+k); j <= Min(k,i-1); ++j)
			t1[j] = t2[j] + t2[j-1];
		Swap(t1, t2);
	}
	int c = t2[k];
	delete[] table;
	return c;
}

float PowUInt(float base, u32 exponent)
{
	// 'Fast Exponentiation': We interpret exponent in base two and calculate the power by
	// squaring and multiplying by base.

	// Find the highest bit that is set.
	u32 e = 0x80000000;
	while((exponent & e) == 0 && e > 0)
		e >>= 1;

	float val = 1.f;
	do
	{
		val *= val; // Shifts the exponent one place left
		val *= (exponent & e) != 0 ? base : 1.f; // Adds a 1 as the LSB of the exponent
		e >>= 1;
	} while(e > 0);

	return val;
}

/** @param base Exponent base value.
	@param exponent Integer exponent to raise base to.
	@return pow(base,exponent) but optimized because we only use integer exponent. */
float PowInt(float base, int exponent)
{
	if (exponent == 0)
		return 1.f;
	else if (exponent < 0)
		return 1.f / PowUInt(base, (u32)-exponent);
	else
		return PowUInt(base, (u32)exponent);
}

char *SerializeFloat(float f, char *dstStr)
{
	if (!IsNan(f))
	{
#ifdef MATH_WITH_GRISU3
		int numChars = dtoa_grisu3((double)f, dstStr);
		return dstStr + numChars;
#else
		return dstStr + sprintf(dstStr, "%.17g", f);
#endif
	}
	else
	{
		u32 u = ReinterpretAsU32(f);
		int numChars = sprintf(dstStr, "NaN(%8X)", (unsigned int)u);
		return dstStr + numChars;
	}
}

float DeserializeFloat(const char *str, const char **outEndStr)
{
	if (!str)
		return FLOAT_NAN;
	while(*str > 0 && *str <= ' ')
		++str;
	if (*str == 0)
		return FLOAT_NAN;
	if (MATH_NEXT_WORD_IS(str, "NaN("))
	{
		str += strlen("NaN("); //MATH_SKIP_WORD(str, "NaN(");
		u32 x;
		int n = sscanf(str, "%X", (unsigned int *)&x);
		if (n != 1)
			return FLOAT_NAN;
		while(*str != 0)
		{
			++str;
			if (*str == ')')
			{
				++str;
				break;
			}
		}
		if (outEndStr)
			*outEndStr = str;
		return ReinterpretAsFloat(x);
	}
	float f;

	if (!strncmp(str, "-inf", 4)) { f = -FLOAT_INF; str += 4; }
	else if (!strncmp(str, "inf", 3)) { f = FLOAT_INF; str += 3; }
	else f = (float)strtod(str, const_cast<char**>(&str));

	while(*str > 0 && *str <= ' ')
		++str;
	if (*str == ',' || *str == ';')
		++str;
	while(*str > 0 && *str <= ' ')
		++str;
	if (outEndStr)
		*outEndStr = str;
	return f;
}

int hexstr_to_u64(const char *str, uint64_t *u)
{
	assert(u);
	*u = 0;
	const char *s = str;
	for(int i = 0; i <= 16; ++i)
	{
		char ch = *s;
		if (ch >= '0' && ch <= '9')
			*u = 16 * *u + ch - '0';
		else if (ch >= 'a' && ch <= 'f')
			*u = 16 * *u + 10 + ch - 'a';
		else if (ch >= 'A' && ch <= 'F')
			*u = 16 * *u + 10 + ch - 'A';
		else
			break;
		++s;
	}
	return (int)(s - str);
}

double DeserializeDouble(const char *str, const char **outEndStr)
{
	if (!str)
		return (double)FLOAT_NAN;
	while(*str > 0 && *str <= ' ')
		++str;
	if (*str == 0)
		return (double)FLOAT_NAN;
	if (MATH_NEXT_WORD_IS(str, "NaN("))
	{
		str += strlen("NaN("); //MATH_SKIP_WORD(str, "NaN(");

		// Read 64-bit unsigned hex representation of the NaN. TODO: Make this more efficient without using sscanf.
		uint64_t u;
		int nChars = hexstr_to_u64(str, &u);
		str += nChars;
		while(*str != 0)
		{
			++str;
			if (*str == ')')
			{
				++str;
				break;
			}
		}
		if (outEndStr)
			*outEndStr = str;
		return ReinterpretAsDouble(u);
	}
	double f;
	
	if (!strncmp(str, "-inf", 4)) { f = (double)-FLOAT_INF; str += 4; }
	else if (!strncmp(str, "inf", 3)) { f = (double)FLOAT_INF; str += 3; }
	else f = strtod(str, const_cast<char**>(&str));

	while(*str > 0 && *str <= ' ')
		++str;
	if (*str == ',' || *str == ';')
		++str;
	while(*str > 0 && *str <= ' ')
		++str;
	if (outEndStr)
		*outEndStr = str;
	return f;
}

MATH_END_NAMESPACE
