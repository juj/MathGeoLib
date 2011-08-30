/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief Common mathematical functions.
*/
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <cassert>
#endif
#include <math.h>
#include <float.h>

#include "Types.h"
#include "MathConstants.h"
#include "Math/float3.h"

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h> // For DebugBreak();
#endif

#include "assume.h"

/// Computes the dot product of two 2D vectors, the elements are accessed using array notation.
#define DOT2(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1])

/// Computes the dot product of two 3D vectors, the elements are accessed using array notation.
#define DOT3(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1] + (v1)[2] * (v2)[2])

/// Computes the dot product of two 3D vectors, but takes the absolute value of each element before summation.
#define ABSDOT3(v1, v2) (Abs((v1)[0] * (v2)[0]) + Abs((v1)[1] * (v2)[1]) + Abs((v1)[2] * (v2)[2]))

#define DOT3_xyz(v1, x, y, z) ((v1)[0] * (x) + (v1)[1] * (y) + (v1)[2] * (z))

#define DOT3STRIDED(v1, v2, stride) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[stride] + (v1)[2] * (v2)[2*stride])

/// Computes the dot product of two 4D vectors, the elements are accessed using array notation.
#define DOT4(v1, v2) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[1] + (v1)[2] * (v2)[2] + (v1)[3] * (v2)[3])

#define DOT4STRIDED(v1, v2, stride) ((v1)[0] * (v2)[0] + (v1)[1] * (v2)[stride] + (v1)[2] * (v2)[2*stride] + (v1)[3] * (v2)[3*stride])

/// Computes the dot product of a 4D vector and a 3D position vector (w == 1).
#define DOT4POS(vec4D, vecPos) ((vec4D)[0] * (vecPos)[0] + (vec4D)[1] * (vecPos)[1] + (vec4D)[2] * (vecPos)[2] + (vec4D)[3])

#define DOT4POS_xyz(vec4D, x, y, z) ((vec4D)[0] * (x) + (vec4D)[1] * (y) + (vec4D)[2] * (z) + (vec4D)[3])

/// Computes the dot product of a 4D vector and a 3D direction vector (w == 1). Provided for convenience, since this is identical to DOT3.
#define DOT4DIR(vec4D, vecDir) ((vec4D)[0] * (vecDir)[0] + (vec4D)[1] * (vecDir)[1] + (vec4D)[2] * (vecDir)[2])

#define DOT4DIR_xyz(vec4D, x, y, z) ((vec4D)[0] * (x) + (vec4D)[1] * (y) + (vec4D)[2] * (z))

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(_MSC_VER)
#include <limits>
#define FLOAT_NAN std::numeric_limits<float>::quiet_NaN()
#define FLOAT_INF std::numeric_limits<float>::infinity()
#define FLOAT_MAX std::numeric_limits<float>::max()
#else
#define FLOAT_MAX FLT_MAX
#define FLOAT_NAN NAN
#define FLOAT_INF INFINITY
#endif
/// Returns the given amount of degrees in radians.
/// 180 degrees equals pi, 360 degrees is a full circle, and equals 2pi.
inline float3 DegToRad(const float3 &degrees) { return degrees * (pi / 180.f); }
inline float DegToRad(float degrees) { return degrees * (pi / 180.f); }

inline float3 RadToDeg(const float3 &radians) { return radians * (180.f / pi); }
inline float RadToDeg(float radians) { return radians * (180.f / pi); }

inline float Cos(float angleRadians) { return cos(angleRadians); }
inline float Sin(float angleRadians) { return sin(angleRadians); }
inline float Sqrt(float v) { return sqrt(v); }
inline float Pow(float base, float exp) { return pow(base, exp); }

/// Integral base to an integral power.
template<u32 Base, u32 Power>
class PowIntT
{
public:
    enum { val = Base * PowIntT<Base,Power-1>::val };
};

/** @cond FULL */

/// End recursion for Base^1.
template<u32 Base>
class PowIntT<Base, 1>
{
public:
    enum { val = Base };
};
/// @endcond

/// Factorial<N> unfolds to N!.
template<int N>
class FactorialT
{
public:
    enum { val = N * FactorialT<N-1>::val };
};

/** @cond FULL */

/// Specialize 0! = 1 to end factorial recursion.
template<>
class FactorialT<0>
{
public:
    enum { val = 1 };
};
/// @endcond

/// Combinatorial<N, K> unfolds to (N nCr K).
template<int N, int K>
class CombinatorialT
{
public:
    enum { val = CombinatorialT<N-1,K-1>::val + CombinatorialT<N-1,K>::val };
};

/** @cond FULL */

/// Specialize (N nCr 0) = 1 to end recursion.
template<int N>
class CombinatorialT<N, 0>
{
public:
    enum { val = 1 };
};

/// Specialize (N nCr N) = 1 to end recursion.
template<int N>
class CombinatorialT<N, N>
{
public:
    enum { val = 1 };
};
/// @endcond

/// Calculates n! at runtime. Use class Factorial<N> to evaluate factorials at compile-time.
int Factorial(int n); 

/// Calculates (N nCr K) at runtime with recursion, running time is exponential to n. 
/// Use class Combinatorial<N, K> to evaluate combinatorials at compile-time.
int CombinatorialRec(int n, int k);

/// Calculates (N nCr K) at runtime, running time is proportional to n*k. 
/// Use class Combinatorial<N, K> to evaluate combinatorials at compile-time.
int CombinatorialTab(int n, int k);

/// Raises a float to an integer power.
float PowInt(float base, int exponent);

/// Returns the given scalar clamped to the range [min, max].
template<typename T>
inline T Clamp(const T &val, const T &floor, const T &ceil)
{
    assert(floor <= ceil);
    return val <= ceil ? (val >= floor ? val : floor) : ceil;
}

template<typename T>
inline T Clamp01(const T &val) { return Clamp(val, T(0), T(1)); }

/** @return The smaller of two values. */
template<typename T>
const T Min(const T &a, const T &b)
{
    return a < b ? a : b;
}

/** @return The larger of two values. */
template<typename T>
const T Max(const T &a, const T &b)
{
    return a >= b ? a : b;
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

template<typename T>
bool Equal(const T &a, const T &b)
{
    return a == b;
}

/** Compares the two values for equality, allowing the given amount of absolute error. */
template<typename T>
bool EqualAbsT(const T &a, const T &b, const T &epsilon)
{
    return Abs(a-b) < epsilon;
}

/** Compares the two values for equality, allowing the given amount of absolute error. */
bool EqualAbs(float a, float b, float epsilon = 1e-6f);

/** Compares the two values for equality, allowing the given amount of relative error. 
    Beware that for values very near 0, the relative error is significant. */
template<typename T>
bool EqualRelT(const T &a, const T &b, const T &maxRelError)
{
    if (a == b) return true; // Handles the special case where a and b are both zero.
    float relativeError = Abs((a-b)/b);
    return relativeError <= maxRelError;
}

/** Compares the two values for equality, allowing the given amount of relative error. 
    Beware that for values very near 0, the relative error is significant. */
bool EqualRel(float a, float b, float maxRelError = 1e-5f);

/** Compares two floats interpreted as integers, see 
    http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm 
    Warning: This comparison is not safe with NANs or INFs. */
bool EqualUlps(float a, float b, int maxUlps = 10000);

#ifndef isfinite
#define isfinite(x) _finite(x)
#endif

template<typename T> inline bool IsFiniteNumber(const T &value) { return true; }
template<> inline bool IsFiniteNumber<float>(const float &value) { return isfinite(value) != 0; }
template<> inline bool IsFiniteNumber<double>(const double &value) { return isfinite(value) != 0; }
template<> inline bool IsFiniteNumber<long double>(const long double &value) { return isfinite(value) != 0; }
