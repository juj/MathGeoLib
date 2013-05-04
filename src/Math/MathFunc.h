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

/** @file MathFunc.h
	@author Jukka Jylänki
	@brief Common mathematical functions. */
#pragma once

#include "myassert.h"
#include <math.h>
#include <cmath>
#include <float.h>
#include <string.h>

#include "MathTypes.h"
#include "MathConstants.h"
#include "float3.h"
#include "Reinterpret.h"
#include "SSEMath.h"

#ifdef WIN32
#define Polygon Polygon_unused
#include <Windows.h> // For DebugBreak();
#undef Polygon
#endif

#include "assume.h"

MATH_BEGIN_NAMESPACE

/// Computes the dot product of two 2D vectors, the elements are accessed using array notation.
/// @param v1 A vector of type float2, or a C array of two elements.
/// @param v2 A vector of type float2, or a C array of two elements.
/// @see DOT3(), DOT4().
#define DOT2(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1])

/// Computes the dot product of two 3D vectors, the elements are accessed using array notation.
/// @param v1 A vector of type float3, or a C array of three elements.
/// @param v2 A vector of type float3, or a C array of three elements.
/// @see DOT2(), DOT4(), ABSDOT3(), DOT3_xyz(), DOT3STRIDED().
#define DOT3(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1] + (v1)[2] * (v2)[2])

/// Computes the dot product of two 3D vectors, but takes the absolute value of each element before summation.
/// @param v1 A vector of type float3, or a C array of three elements.
/// @param v2 A vector of type float3, or a C array of three elements.
/// @see DOT3(), DOT3_xyz(), DOT3STRIDED().
#define ABSDOT3(v1, v2) (Abs((v1)[0] * (v2)[0]) + Abs((v1)[1] * (v2)[1]) + Abs((v1)[2] * (v2)[2]))

/// Computes the dot product of a float3 and another vector given by three floats.
/// @param v1 A vector of type float3, or a C array of three elements.
/// @param x The x component of a second vector.
/// @param y The y component of a second vector.
/// @param z The z component of a second vector.
/// @see DOT3(), ABSDOT3(), DOT3STRIDED().
#define DOT3_xyz(v1, x, y, z) ((v1)[0] * (x) + (v1)[1] * (y) + (v1)[2] * (z))

/// Computes the dot product of two 3D vectors, but with the elements of the second vector being scattered noncontiguous in memory.
/// @param v1 The first vector in the dot product. This can either be a C array or a float3.
/// @param v2 The second vector in the dot product. As opposed to the DOT3() macro, which accesses the elements of this vector
///	 as v2[0], v2[1], v2[2], this function accesses the elements as v2[0], v2[stride], v2[2*stride].
/// @param stride The distance between between the subsequent vector elements in the array v2.
/// @see DOT3(), ABSDOT3(), DOT3_xyz(), DOT4STRIDED().
#define DOT3STRIDED(v1, v2, stride) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[stride] + (v1)[2] * (v2)[2*stride])

/// Computes the dot product of two 4D vectors, the elements are accessed using array notation.
/// @param v1 A vector of type float4, or a C array of four elements.
/// @param v2 A vector of type float4, or a C array of four elements.
/// @see DOT2(), DOT3(), DOT4STRIDED(), DOT4POS(), DOT4POS_xyz(), DOT4DIR(), DOT4DIR_xyz().
#define DOT4(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1] + (v1)[2] * (v2)[2] + (v1)[3] * (v2)[3])

/// Computes the dot product of two 4D vectors, but with the elements of the second vector being scattered noncontiguous in memory.
/// @param v1 The first vector in the dot product. This can either be a C array or a float4.
/// @param v2 The second vector in the dot product. As opposed to the DOT4() macro, which accesses the elements of this vector
///	 as v2[0], v2[1], v2[2], v2[3], this function accesses the elements as v2[0], v2[stride], v2[2*stride], v2[3*stride].
/// @param stride The distance between between the subsequent vector elements in the array v2.
/// @see DOT4(), DOT4POS(), DOT4POS_xyz(), DOT4DIR(), DOT4DIR_xyz().
#define DOT4STRIDED(v1, v2, stride) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[stride] + (v1)[2] * (v2)[2*stride] + (v1)[3] * (v2)[3*stride])

/// Computes the dot product of a 4D vector and a 3D position vector (w == 1).
/// @param vec4D The 4D vector in the dot product, or a C array of four elements.
/// @param vecPos The 3D vector in the dot product. This vector is expanded to a 4D vector by setting w == 1.
/// @see DOT4(), DOT4STRIDED(), DOT4POS_xyz(), DOT4DIR(), DOT4DIR_xyz().
#define DOT4POS(vec4D, vecPos) ((vec4D)[0] * (vecPos)[0] + (vec4D)[1] * (vecPos)[1] + (vec4D)[2] * (vecPos)[2] + (vec4D)[3])

/// Computes the dot product of a 4D vector and a position vector (x,y,z,1).
/// @param vec4D The 4D vector in the dot product, or a C array of four elements.
/// @param x The x component of the second vector.
/// @param y The y component of the second vector.
/// @param z The z component of the second vector.
/// @see DOT4(), DOT4STRIDED(), DOT4POS(), DOT4DIR(), DOT4DIR_xyz().
#define DOT4POS_xyz(vec4D, x, y, z) ((vec4D)[0] * (x) + (vec4D)[1] * (y) + (vec4D)[2] * (z) + (vec4D)[3])

/// Computes the dot product of a 4D vector and a direction vector (x,y,z,0).
/// @note This function is only provided for convenience, since this is identical to DOT3.
/// @see DOT3(), DOT4(), DOT4STRIDED(), DOT4POS(), DOT4POS_xyz().
#define DOT4DIR(vec4D, vecDir) DOT3(vec4D, vecDir)

/// Computes the dot product of a 4D vector and a direction vector (x,y,z,0).
/// @note This function is only provided for convenience, since this is identical to DOT3_xyz.
/// @see DOT3_xyz(), DOT4(), DOT4STRIDED(), DOT4POS(), DOT4POS_xyz(), DOT4DIR().
#define DOT4DIR_xyz(vec4D, x, y, z) DOT3_xyz(vec4D, x, y, z)

/// Converts the given amount of degrees into radians.
/// 180 degrees equals pi, 360 degrees is a full circle, and equals 2pi.
inline float3 DegToRad(const float3 &degrees) { return degrees * (pi / 180.f); }
inline float DegToRad(float degrees) { return degrees * (pi / 180.f); }

/// Converts the given amount of radians into degrees.
inline float3 RadToDeg(const float3 &radians) { return radians * (180.f / pi); }
inline float RadToDeg(float radians) { return radians * (180.f / pi); }

/// Computes the function sin(x).
/** @see Cos(), Tan(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Sin(float angleRadians);
/// Computes the function cos(x).
/** @see Sin(), Tan(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Cos(float angleRadians);
/// Computes the function tan(x).
/** @see Sin(), Cos(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Tan(float angleRadians);
/// Simultaneously computes both sin(x) and cos(x), which yields a small performance increase over to
/// computing them separately.
/** @see Sin(), Cos(), Tan(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float2 SinCos(float angleRadians);
/// Computes the function arcsin(x), in radians.
/** @see Sin(), Cos(), Tan(), SinCos(), Acos(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Asin(float x);
/// Computes the function arccos(x), in radians.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Atan(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Acos(float x);
/// Computes the function arctan(x), in radians.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Acos(), Atan2(), Sinh(), Cosh(), Tanh(). */
float Atan(float x);
/// Computes the signed (principal value) arc-tangent of y/x, in radians.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Acos(), Atan(), Sinh(), Cosh(), Tanh(). */
float Atan2(float y, float x);
/// Computes the hyperbolic sine of x.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Cosh(), Tanh(). */
float Sinh(float x);
/// Computes the hyperbolic cosine of x.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Tanh(). */
float Cosh(float x);
/// Computes the hyperbolic tangent of x.
/** @see Sin(), Cos(), Tan(), SinCos(), Asin(), Acos(), Atan(), Atan2(), Sinh(), Cosh(). */
float Tanh(float x);

/// Returns true if the given number is a power of 2.
/** @see RoundUpPow2(), RoundDownPow2(). */
bool IsPow2(unsigned int number);
/// Returns the smallest power-of-2 number (1,2,4,8,16,32,...) greater or equal than the given number.
/** @see IsPow2(), RoundDownPow2(). */
unsigned int RoundUpPow2(unsigned int number);
/// Returns the largest power-of-2 number (1,2,4,8,16,32,...) smaller or equal than the given number.
/** @see IsPow2(), RoundUpPow2(). */
unsigned int RoundDownPow2(unsigned int number);

/// Returns the given number rounded up to the next multiple of n.
/** @param x The number to round up.
	@param n The multiple to round x to. The value n must be a power-of-2. */
int RoundIntUpToMultipleOfPow2(int x, int n);

/// Raises the given base to an integral exponent.
/** @see Pow(), Exp(). */
float PowInt(float base, int exponent);
/// Raises the given base to a float exponent.
/** @see PowInt(), Exp(). */
float Pow(float base, float exponent);
/// Returns e (the constant 2.71828...) raised to the given power.
/** @see PowInt(), Pow(). */
float Exp(float exponent);
/// Computes a logarithm of the given value in the specified base.
/** @see Log2(), Ln(), Log10(). */
float Log(float base, float value);
/// Computes a logarithm in base-2.
/** @see Log(), Ln(), Log10(). */
float Log2(float value);
/// Computes a logarithm in the natural base (using e=2.71828... as the base)
/** @see Log(), Log2(), Log10(). */
float Ln(float value);
/// Computes a logarithm in base-10.
/** @see Log(), Log2(), Ln(). */
float Log10(float value);

/// Returns f rounded up to the next integer, as float.
/** @see CeilInt(), Floor(), FloorInt(), Round(), RoundInt(). */
float Ceil(float f);
/// Returns f rounded up to the next integer, as integer.
/** @see Ceil(), Floor(), FloorInt(), Round(), RoundInt(). */
int CeilInt(float f);
/// Returns f rounded down to the previous integer, as float.
/** @see Ceil(), CeilInt(), FloorInt(), Round(), RoundInt(). */
float Floor(float f);
/// Returns f rounded down to the previous integer, as integer.
/** @see Ceil(), CeilInt(), Floor(), Round(), RoundInt(). */
int FloorInt(float f);
/// Returns f rounded to the nearest integer, as float.
/** @see Ceil(), CeilInt(), Floor(), FloorInt(), RoundInt(). */
float Round(float f);
/// Returns f rounded to the nearest integer, as integer.
/** @see Ceil(), CeilInt(), Floor(), FloorInt(), Round(). */
int RoundInt(float f);

/// Returns -1 or 1 depending on the sign of f.
/** @see SignOrZero(). */
float Sign(float f);
/// Returns 0 if f is zero up to the given epsilon. Otherwise returns -1 or 1 depending on the sign of f.
/** @see Sign(). */
float SignOrZero(float f, float epsilon = 1e-8f);

/// Linearly interpolates between a and b.
/** @param t A value between [0,1].
	@param a The first endpoint to lerp between.
	@param b The second endpoint to lerp between.
	@return This function computes a + t*(b-a). That is, if t==0, this function returns a. If t==1, this function returns b.
		Otherwise, the returned value linearly moves from a to b as t ranges from 0 to 1.
	@see LerpMod(), InvLerp(), Step(), SmoothStep(), PingPongMod(), Mod(), ModPos(), Frac(). */
float Lerp(float a, float b, float t);
/// Linearly interpolates from a to b, under the modulus mod.
/** This function takes evaluates a and b in the range [0, mod] and takes the shorter path to reach from a to b.
	@see Lerp(), InvLerp(), Step(), SmoothStep(), PingPongMod(), Mod(), ModPos(), Frac(). */
float LerpMod(float a, float b, float mod, float t);
/// Computes the lerp factor a and b have to be Lerp()ed to get x.
/** @see Lerp(), LerpMod(), Step(), SmoothStep(), PingPongMod(), Mod(), ModPos(), Frac(). */
float InvLerp(float a, float b, float x);
/// See http://msdn.microsoft.com/en-us/library/bb509665(v=VS.85).aspx
/** @see Lerp(), LerpMod(), InvLerp(), SmoothStep(), PingPongMod(), Mod(), ModPos(), Frac(). */
float Step(float y, float x);
/// See http://msdn.microsoft.com/en-us/library/bb509658(v=vs.85).aspx
/** @see Lerp(), LerpMod(), InvLerp(), Step(), PingPongMod(), Mod(), ModPos(), Frac(). */
float SmoothStep(float min, float max, float x);
/// Limits x to the range [0, mod], but instead of wrapping around from mod to 0, the result will move back
/// from mod to 0 as x goes from mod to 2*mod.
/** @see Lerp(), LerpMod(), InvLerp(), Step(), SmoothStep(), Mod(), ModPos(), Frac(). */
float PingPongMod(float x, float mod);
/// Computes a floating-point modulus.
/// This function returns a value in the range ]-mod, mod[.
/** @see Lerp(), LerpMod(), InvLerp(), Step(), SmoothStep(), PingPongMod(), ModPos(), Frac(). */
float Mod(float x, float mod);
/// Computes a floating-point modulus using an integer as the modulus.
float Mod(float x, int mod);
/// Computes a floating-point modulus, but restricts the output to the range [0, mod[.
/** @see Lerp(), LerpMod(), InvLerp(), Step(), SmoothStep(), PingPongMod(), Mod(), Frac(). */
float ModPos(float x, float mod);
/// Computes a floating-point modulus, but restricts the output to the range [0, mod[.
float ModPos(float x, int mod);
/// Returns the fractional part of x.
/** @see Lerp(), LerpMod(), InvLerp(), Step(), SmoothStep(), PingPongMod(), Mod(), ModPos(). */
float Frac(float x);

/// Returns the square root of x.
FORCE_INLINE float Sqrt(float x)
{
#ifdef MATH_SSE
	return M128_TO_FLOAT(_mm_sqrt_ss(FLOAT_TO_M128(x)));
#else
	return sqrtf(x);
#endif
}

/// Computes a fast approximation of the square root of x.
FORCE_INLINE float SqrtFast(float x)
{
#ifdef MATH_SSE
	__m128 X = FLOAT_TO_M128(x);
	return M128_TO_FLOAT(_mm_mul_ss(X, _mm_rsqrt_ss(X)));
#else
	return sqrtf(x);
#endif
}

/// Returns 1/Sqrt(x). (The reciprocal of the square root of x)
FORCE_INLINE float RSqrt(float x)
{
#ifdef MATH_SSE
	__m128 X = FLOAT_TO_M128(x);
	__m128 e = _mm_rsqrt_ss(X);

	// Do one iteration of Newton-Rhapson:
	// e_n = e + 0.5 * (e - x * e^3)
	__m128 e3 = _mm_mul_ss(_mm_mul_ss(e,e), e);
	__m128 half = _mm_set_ss(0.5f);
	
	return M128_TO_FLOAT(_mm_add_ss(e, _mm_mul_ss(half, _mm_sub_ss(e, _mm_mul_ss(X, e3)))));
#else
	return 1.f / sqrtf(x);
#endif
}

/// SSE implementation of reciprocal square root.
FORCE_INLINE float RSqrtFast(float x)
{
#ifdef MATH_SSE
	return M128_TO_FLOAT(_mm_rsqrt_ss(FLOAT_TO_M128(x)));
#else
	return 1.f / sqrtf(x);
#endif
}

/// Returns 1/x, the reciprocal of x.
FORCE_INLINE float Recip(float x)
{
#ifdef MATH_SSE
	__m128 X = FLOAT_TO_M128(x);
	__m128 e = _mm_rcp_ss(X);
	// Do one iteration of Newton-Rhapson:
	// e_n = 2*e - x*e^2
	__m128 e2 = _mm_mul_ss(e,e);
	return M128_TO_FLOAT(_mm_sub_ss(_mm_add_ss(e, e), _mm_mul_ss(X, e2)));
#else
	return 1.f / x;
#endif
}

/// Returns 1/x, the reciprocal of x, using a fast approximation (SSE rcp instruction).
FORCE_INLINE float RecipFast(float x)
{
#ifdef MATH_SSE
	return M128_TO_FLOAT(_mm_rcp_ss(FLOAT_TO_M128(x)));
#else
	return 1.f / x;
#endif
}

/// Calculates n! at runtime. Use class FactorialT<N> to evaluate factorials at compile-time.
int Factorial(int n);

/// Calculates (N nCr K) at runtime with recursion, running time is exponential to n.
/// Use class Combinatorial<N, K> to evaluate combinatorials at compile-time.
int CombinatorialRec(int n, int k);

/// Calculates (N nCr K) at runtime, running time is proportional to n*k.
/// Use class Combinatorial<N, K> to evaluate combinatorials at compile-time.
int CombinatorialTab(int n, int k);

/// Clamps the given input value to the range [min, max].
/** @see Clamp01(), Min(), Max(). */
template<typename T>
inline T Clamp(const T &val, const T &floor, const T &ceil)
{
	assume(floor <= ceil);
	return val <= ceil ? (val >= floor ? val : floor) : ceil;
}

/// Clamps the given input value to the range [0, 1].
/** @see Clamp(), Min(), Max(). */
template<typename T>
inline T Clamp01(const T &val) { return Clamp(val, T(0), T(1)); }

/// Computes the smaller of two values.
/** @see Clamp(), Clamp01(), Max(). */
template<typename T>
const T Min(const T &a, const T &b)
{
	return a < b ? a : b;
}

/// Computes the larger of two values.
/** @see Clamp(), Clamp01(), Min(). */
template<typename T>
const T Max(const T &a, const T &b)
{
	return a >= b ? a : b;
}

template<>
inline const float Max(const float &a, const float &b)
{
#ifdef MATH_SSE
	return M128_TO_FLOAT(_mm_max_ss(FLOAT_TO_M128(a), FLOAT_TO_M128(b)));
#else
	return a >= b ? a : b;
#endif
}

/// Computes the smallest of three values.
/** @see Clamp(), Clamp01(), Max(). */
template<typename T>
const T Min(const T &a, const T &b, const T &c)
{
	return Min(Min(a, b), c);
}

template<>
inline const float Min(const float &a, const float &b)
{
#ifdef MATH_SSE
	return M128_TO_FLOAT(_mm_min_ss(FLOAT_TO_M128(a), FLOAT_TO_M128(b)));
#else
	return a <= b ? a : b;
#endif
}

/// Computes the largest of three values.
/** @see Clamp(), Clamp01(), Min(). */
template<typename T>
const T Max(const T &a, const T &b, const T &c)
{
	return Max(Max(a, b), c);
}

/// Computes the smallest of four values.
/** @see Clamp(), Clamp01(), Max(). */
template<typename T>
const T Min(const T &a, const T &b, const T &c, const T &d)
{
	return Min(Min(a, b), Min(c, d));
}

/// Computes the largest of four values.
/** @see Clamp(), Clamp01(), Min(). */
template<typename T>
const T Max(const T &a, const T &b, const T &c, const T &d)
{
	return Max(Max(a, b), Max(c, d));
}

/// Swaps the two values.
template<typename T>
void Swap(T &a, T &b)
{
	T temp = a;
	a = b;
	b = temp;
}

/** @return True if a > b. */
template<typename T>
bool GreaterThan(const T &a, const T &b)
{
	return a > b;
}

/** @return True if a < b. */
template<typename T>
bool LessThan(const T &a, const T &b)
{
	return a < b;
}

/** @return The absolute value of a. */
template<typename T>
const T Abs(const T &a)
{
	return a >= 0 ? a : -a;
}

/// @return True if a and b are equal, using operator ==().
template<typename T>
bool Equal(const T &a, const T &b)
{
	return a == b;
}

/** Compares the two values for equality up to a small epsilon. */
template<> bool FORCE_INLINE Equal(const float &a, const float &b) { return Abs(a-b) <= eps; }
template<> bool FORCE_INLINE Equal(const double &a, const double &b) { return Abs(a-b) <= eps; }
#ifndef EMSCRIPTEN // long double is not supported.
template<> bool FORCE_INLINE Equal(const long double &a, const long double &b) { return Abs(a-b) <= eps; }
#endif

/** Compares the two values for equality, allowing the given amount of absolute error. */
bool EqualAbs(float a, float b, float epsilon = 1e-4f);

/// Computes the relative error of the two variables.
float RelativeError(float a, float b);

/** Compares the two values for equality, allowing the given amount of relative error.
	Beware that for values very near 0, the relative error is significant. */
bool EqualRel(float a, float b, float maxRelError = 1e-4f);

/** Compares two floats interpreted as integers, see
	http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
	Warning: This comparison is not safe with NANs or INFs. */
bool EqualUlps(float a, float b, int maxUlps = 10000);

/// Returns true if the given value is not an inf or a nan.
template<typename T> FORCE_INLINE bool IsFinite(T /*value*/) { return true; }

template<> FORCE_INLINE bool IsFinite<float>(float f) { return (ReinterpretAsU32(f) << 1) < 0xFF000000u; }
template<> FORCE_INLINE bool IsFinite<double>(double d) { return (ReinterpretAsU64(d) << 1) < 0xFFE0000000000000ULL; }

/// Returns true if the given value is a not-a-number.
FORCE_INLINE bool IsNan(float f) { return (ReinterpretAsU32(f) << 1) > 0xFF000000u; }
FORCE_INLINE bool IsNan(double d) { return (ReinterpretAsU64(d) << 1) > 0xFFE0000000000000ULL; }

/// Returns true if the given value is +inf or -inf.
FORCE_INLINE bool IsInf(float f) { return (ReinterpretAsU32(f) << 1) == 0xFF000000u; }
FORCE_INLINE bool IsInf(double d) { return (ReinterpretAsU64(d) << 1) == 0xFFE0000000000000ULL; }

#ifdef _MSC_VER
template<> FORCE_INLINE bool IsFinite<long double>(long double value) { return _finite((double)value) != 0; }
FORCE_INLINE bool IsInf(long double value) { return IsInf((double)value); }
FORCE_INLINE bool IsNan(long double value) { return IsNan((double)value); }
#elif !defined(EMSCRIPTEN) // long double is not supported.
//template<> FORCE_INLINE bool IsFinite<long double>(long double value) { asserteq(sizeof(long double), 16); u64 val[2]; memcpy(val, &value, sizeof(u64)*2); return (val[1] & 0x7FFF) != 0x7FFF || val[0] < 0x8000000000000000ULL; }
//FORCE_INLINE bool IsInf(long double value) { asserteq(sizeof(long double), 16); u64 val[2]; memcpy(val, &value, sizeof(u64)*2); return (val[1] & 0x7FFF) == 0x7FFF && val[0] == 0x8000000000000000ULL; }
//FORCE_INLINE bool IsNan(long double value) { asserteq(sizeof(long double), 16); u64 val[2]; memcpy(val, &value, sizeof(u64)*2); return (val[1] & 0x7FFF) == 0x7FFF && val[0] >  0x8000000000000000ULL; }
template<> FORCE_INLINE bool IsFinite<long double>(long double value) { return IsFinite<double>((double)value); }
FORCE_INLINE bool IsInf(long double value) { return IsInf((double)value); }
FORCE_INLINE bool IsNan(long double value) { return IsNan((double)value); }
#endif

MATH_END_NAMESPACE
