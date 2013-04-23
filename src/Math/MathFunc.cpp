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
#include "SSEMath.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#include <algorithm>
#endif

#include "myassert.h"
#include "float2.h"

#ifdef WIN32
#include <Windows.h>
#endif

MATH_BEGIN_NAMESPACE

bool mathBreakOnAssume = false;

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
	if (mathBreakOnAssume)
	{
#if defined(WIN32) && !defined(WIN8RT) // Win8 metro apps don't have DebugBreak.
		DebugBreak();
#endif
	}
	return mathBreakOnAssume;
}

float Sin(float angleRadians)
{
	return sinf(angleRadians);
}

float Cos(float angleRadians)
{
	return cosf(angleRadians);
}

float Tan(float angleRadians)
{
	return tanf(angleRadians);
}

float2 SinCos(float angleRadians)
{
	return float2(sinf(angleRadians), cosf(angleRadians));
}

float Asin(float x)
{
	return asinf(x);
}

float Acos(float x)
{
	return acosf(x);
}

float Atan(float x)
{
	return atanf(x);
}

float Atan2(float y, float x)
{
	return atan2f(y, x);
}

float Sinh(float x)
{
	return sinhf(x);
}

float Cosh(float x)
{
	return coshf(x);
}

float Tanh(float x)
{
	return tanhf(x);
}

bool IsPow2(unsigned int number)
{
	return (number & (number-1)) == 0;
}

unsigned int RoundUpPow2(unsigned int x)
{
	assert(sizeof(unsigned int) <= 4);
	--x;
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	++x;

	return x;
}

unsigned int RoundDownPow2(unsigned int x)
{
	assert(sizeof(unsigned int) <= 4);
	x |= x >> 1;
	x |= x >> 2;
	x |= x >> 4;
	x |= x >> 8;
	x |= x >> 16;
	return x - (x >> 1);
}

int RoundIntUpToMultipleOfPow2(int x, int n)
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
	assume(Abs(b-a) > eps);
	return (x - a) / (b - a);
}

float Step(float y, float x)
{
	return (x >= y) ? 1.f : 0.f;
}

float SmoothStep(float min, float max, float x)
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

MATH_END_NAMESPACE
