/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float4.cpp
	@author Jukka Jylänki
	@brief */
#include "float4.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#include <utility>
#include <iostream>
#endif

#include <stdlib.h>

#include "float2.h"
#include "float3.h"
#include "../Geometry/AABB.h"
#include "../Geometry/Sphere.h"
#include "../Algorithm/Random/LCG.h"
#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"
#include "MathFunc.h"
#include "SSEMath.h"
#include "float4_sse.h"
#include "float4_neon.h"

MATH_BEGIN_NAMESPACE

using namespace std;

float4::float4(float x_, float y_, float z_, float w_)
#if !defined(MATH_AUTOMATIC_SSE)
	// Best: 8.449 nsecs / 23.376 ticks, Avg: 8.837 nsecs, Worst: 9.601 nsecs
:x(x_), y(y_), z(z_), w(w_)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	// Best: 1.536 nsecs / 4.2 ticks, Avg: 1.609 nsecs, Worst: 1.920 nsecs
	v = set_ps(w_, z_, y_, x_);
#endif
}

float4::float4(const float3 &xyz, float w_)
#if !defined(MATH_AUTOMATIC_SSE) || !defined(MATH_SSE)
// Best: 5.761 nsecs / 15.872 ticks, Avg: 6.237 nsecs, Worst: 7.681 nsecs
:x(xyz.x), y(xyz.y), z(xyz.z), w(w_)
#endif
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 1.536 nsecs / 4.032 ticks, Avg: 1.540 nsecs, Worst: 1.920 nsecs
	v = load_vec3(xyz.ptr(), w_);
#endif
}

float4::float4(float x_, float y_, const float2 &zw)
#if !defined(MATH_AUTOMATIC_SSE)
:x(x_), y(y_), z(zw.x), w(zw.y)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	v = set_ps(zw.y, zw.x, y_, x_);
#endif
}

float4::float4(float x_, const float2 &yz, float w_)
#if !defined(MATH_AUTOMATIC_SSE)
:x(x_), y(yz.x), z(yz.y), w(w_)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	v = set_ps(w_, yz.y, yz.x, x_);
#endif
}

float4::float4(float x_, const float3 &yzw)
#if !defined(MATH_AUTOMATIC_SSE)
:x(x_), y(yzw.x), z(yzw.y), w(yzw.z)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	v = set_ps(yzw.z, yzw.y, yzw.x, x_);
#endif
}

float4::float4(const float2 &xy, float z_, float w_)
#if !defined(MATH_AUTOMATIC_SSE)
:x(xy.x), y(xy.y), z(z_), w(w_)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	v = set_ps(w_, z_, xy.y, xy.x);
#endif
}

float4::float4(const float2 &xy, const float2 &zw)
#if !defined(MATH_AUTOMATIC_SSE)
:x(xy.x), y(xy.y), z(zw.x), w(zw.y)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	v = set_ps(zw.y, zw.x, xy.y, xy.x);
#endif
}

float4::float4(const float *data)
{
	assume(data);
#if defined(MATH_AUTOMATIC_SSE)
	v = loadu_ps(data);
#else
	x = data[0];
	y = data[1];
	z = data[2];
	w = data[3];
#endif
}

float2 float4::xy() const
{
	return float2(x, y);
}

float3 float4::xyz() const
{
	return float3(x, y, z);
}

float2 float4::Swizzled(int i, int j) const
{
	return float2(At(i), At(j));
}

float3 float4::Swizzled(int i, int j, int k) const
{
	return float3(At(i), At(j), At(k));
}

float4 float4::Swizzled(int i, int j, int k, int l) const
{
#if defined(MATH_AVX) && defined MATH_AUTOMATIC_SSE
	return vec4_permute(v, i, j, k, l);
	///\todo How to perform an efficient swizzle if AVX is not available?
	///      We need a dynamic runtime shuffle operation, so _mm_shuffle_ps
	///      cannot be used.
#else
	return float4(At(i), At(j), At(k), At(l));
#endif
}

float4 float4::xxxx() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return xxxx_ps(v);
#else
	return float4::FromScalar(x);
#endif
}

float4 float4::yyyy() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return yyyy_ps(v);
#else
	return float4::FromScalar(x);
#endif
}

float4 float4::zzzz() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return zzzz_ps(v);
#else
	return float4::FromScalar(x);
#endif
}

float4 float4::xxxw() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return shuffle1_ps(v, _MM_SHUFFLE(3, 0, 0, 0));
#else
	return float4(x, x, x, w);
#endif
}

float4 float4::yyyw() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return shuffle1_ps(v, _MM_SHUFFLE(3, 1, 1, 1));
#else
	return float4(y, y, y, w);
#endif
}

float4 float4::zzzw() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return shuffle1_ps(v, _MM_SHUFFLE(3, 2, 2, 2));
#else
	return float4(z, z, z, w);
#endif
}

float4 float4::wwww() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return wwww_ps(v);
#else
	return float4::FromScalar(x);
#endif
}

#ifdef MATH_SIMD

/// The returned vector contains the squared length of the float3 part in the lowest channel of the vector.
///\todo Delete this function.
simd4f float4::LengthSq3_SSE() const
{
	return dot3_ps3(v, v);
}

/// The returned vector contains the length of the float3 part in each channel of the vector.
///\todo Delete this function.
simd4f float4::Length3_SSE() const
{
	return sqrt_ps(dot3_ps(v, v));
}

/// The returned vector contains the squared length of the float4 in each channel of the vector.
simd4f float4::LengthSq4_SSE() const
{
	return dot4_ps(v, v);
}

/// The returned vector contains the length of the float4 in each channel of the vector.
simd4f float4::Length4_SSE() const
{
	return sqrt_ps(dot4_ps(v, v));
}

void float4::Normalize3_Fast_SSE()
{
	simd4f len = Length3_SSE();
	simd4f normalized = div_ps(v, len); // Normalize.
	v = cmov_ps(v, normalized, sseMaskXYZ); // Return the original .w component to the vector (this function is supposed to preserve original .w).
}

simd4f float4::Normalize4_SSE()
{
	simd4f len = Length4_SSE();
	simd4f isZero = cmplt_ps(len, simd4fEpsilon); // Was the length zero?
	simd4f normalized = div_ps(v, len); // Normalize.
	v = cmov_ps(normalized, float4::unitX.v, isZero); // If length == 0, output the vector (1,0,0,0).
	return len;
}

void float4::Normalize4_Fast_SSE()
{
	simd4f recipLen = rsqrt_ps(dot4_ps(v, v));
	v = mul_ps(v, recipLen);
}

void float4::NormalizeW_SSE()
{
	v = div_ps(v, wwww_ps(v));
}

#endif

float float4::LengthSq3() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec3_length_sq_float(v);
#else
	return x*x + y*y + z*z;
#endif
}

float float4::Length3() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec3_length_float(v);
#else
	return Sqrt(x*x + y*y + z*z);
#endif
}

float float4::LengthSq4() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_length_sq_float(v);
#else
	return x*x + y*y + z*z + w*w;
#endif
}

float float4::Length4() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_length_float(v);
#else
	return Sqrt(x*x + y*y + z*z + w*w);
#endif
}

float float4::Normalize3()
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	simd4f origLength;
	v = vec4_safe_normalize3(v, origLength);
	return s4f_x(origLength);
#else
	assume(IsFinite());
	float lengthSq = LengthSq3();
	if (lengthSq > 1e-6f)
	{
		float length = Sqrt(lengthSq);
		float invLength = 1.f / length;
		x *= invLength;
		y *= invLength;
		z *= invLength;
		return length;
	}
	else
	{
		Set(1.f, 0.f, 0.f, w); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
#endif
}

float4 float4::Normalized3() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	simd4f origLength;
	return vec4_safe_normalize3(v, origLength);
#else
	float4 copy = *this;
	float length = copy.Normalize3();
	assume(length > 0);
	MARK_UNUSED(length);
	return copy;
#endif
}

float float4::Normalize4()
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	simd4f len = Normalize4_SSE();
	return s4f_x(len);
#else
	assume(IsFinite());
	float lengthSq = LengthSq4();
	if (lengthSq > 1e-6f)
	{
		float length = Sqrt(lengthSq);
		*this *= 1.f / length;
		return length;
	}
	else
	{
		Set(1.f, 0.f, 0.f, 0.f); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
#endif
}

float4 float4::Normalized4() const
{
	float4 copy = *this;
	float length = copy.Normalize4();
	assume(length > 0);
	MARK_UNUSED(length);
	return copy;
}

void float4::NormalizeW()
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	NormalizeW_SSE();
#else
	if (MATH_NS::Abs(w) > 1e-6f)
	{
		float invW = 1.f / w;
		x *= invW;
		y *= invW;
		z *= invW;
		w = 1.f;
	}
#endif
}

bool float4::IsWZeroOrOne(float epsilon) const
{
	return EqualAbs(w, 0.f, epsilon) || EqualAbs(w, 1.f, epsilon);
}

bool float4::IsZero4(float epsilonSq) const
{
	return LengthSq4() <= epsilonSq;
}

bool float4::IsZero3(float epsilonSq) const
{
	return LengthSq3() <= epsilonSq;
}

bool float4::IsNormalized4(float epsilonSq) const
{
	return MATH_NS::Abs(LengthSq4()-1.f) <= epsilonSq;
}

bool float4::IsNormalized3(float epsilonSq) const
{
	return MATH_NS::Abs(LengthSq3()-1.f) <= epsilonSq;
}

void float4::Scale3(float scalar)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	simd4f scale = setx_ps(scalar);
	scale = _mm_shuffle_ps(scale, simd4fOne, _MM_SHUFFLE(0,0,0,0)); // scale = (1 1 s s)
	scale = shuffle1_ps(scale, _MM_SHUFFLE(3,0,0,0)); // scale = (1 s s s)
	v = mul_ps(v, scale);
#else
	x *= scalar;
	y *= scalar;
	z *= scalar;
#endif
}

float float4::ScaleToLength3(float newLength)
{
	///\todo Add SSE-enabled version.
	///\todo Add ClampToLength3.
	float length = LengthSq3();
	if (length < 1e-6f)
		return 0.f;

	length = Sqrt(length);
	float scalar = newLength / length;
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return length;
}

float4 float4::ScaledToLength3(float newLength) const
{
	assume(!IsZero3());

	float4 vtx = *this;
	vtx.ScaleToLength3(newLength);
	return vtx;
}

float float4::ScaleToLength(float newLength)
{
	float length = Length();
	float scalar = newLength / length;
	*this *= scalar;
	return length;
}

float4 float4::ScaledToLength(float newLength) const
{
	float4 vtx = *this;
	vtx.ScaleToLength(newLength);
	return vtx;
}

bool float4::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y) && MATH_NS::IsFinite(z) && MATH_NS::IsFinite(w);
}

bool float4::IsPerpendicular3(const float4 &other, float epsilonSq) const
{
	float dot = Dot3(other);
	return dot*dot <= epsilonSq * LengthSq() * other.LengthSq();
}

bool float4::IsPerpendicular(const float4 &other, float epsilonSq) const
{
	float dot = Dot(other);
	return dot*dot <= epsilonSq * LengthSq() * other.LengthSq();
}

bool IsNeutralCLocale();

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float4::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f, %.3f)", x, y, z, w);
	return str;
}

StringT float4::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(x, str); *s = ','; ++s;
	s = SerializeFloat(y, s); *s = ','; ++s;
	s = SerializeFloat(z, s); *s = ','; ++s;
	s = SerializeFloat(w, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT float4::SerializeToCodeString() const
{
	return "float4(" + SerializeToString() + ")";
}
#endif

float4 float4::FromString(const char *str, const char **outEndStr)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return float4::nan;
	MATH_SKIP_WORD(str, "float4");
	MATH_SKIP_WORD(str, "(");
	float4 f;
	f.x = DeserializeFloat(str, &str);
	f.y = DeserializeFloat(str, &str);
	f.z = DeserializeFloat(str, &str);
	f.w = DeserializeFloat(str, &str);
	if (*str == ')')
		++str;
	if (*str == ',')
		++str;
	if (outEndStr)
		*outEndStr = str;
	return f;
}

float float4::SumOfElements() const
{
#ifdef MATH_AUTOMATIC_SSE
	return sum_xyzw_float(v);
#else
	return x + y + z + w;
#endif
}

float float4::ProductOfElements() const
{
#ifdef MATH_AUTOMATIC_SSE
	return mul_xyzw_float(v);
#else
	return x * y * z * w;
#endif
}

float float4::AverageOfElements() const
{
	return 0.25f * SumOfElements();
}

float float4::MinElement() const
{
	return MATH_NS::Min(MATH_NS::Min(x, y), MATH_NS::Min(z, w));
}

int float4::MinElementIndex() const
{
	if (x < y)
	{
		if (z < w)
			return (x < z) ? 0 : 2;
		else
			return (x < w) ? 0 : 3;
	}
	else
	{
		if (z < w)
			return (y < z) ? 1 : 2;
		else
			return (y < w) ? 1 : 3;
	}
}

float float4::MaxElement() const
{
	return MATH_NS::Max(MATH_NS::Max(x, y), MATH_NS::Min(z, w));
}

int float4::MaxElementIndex() const
{
	if (x > y)
	{
		if (z > w)
			return (x > z) ? 0 : 2;
		else
			return (x > w) ? 0 : 3;
	}
	else
	{
		if (z > w)
			return (y > z) ? 1 : 2;
		else
			return (y > w) ? 1 : 3;
	}
}

float4 float4::Abs() const
{
#ifdef MATH_AUTOMATIC_SSE
	return abs_ps(v);
#else
	return float4(MATH_NS::Abs(x), MATH_NS::Abs(y), MATH_NS::Abs(z), MATH_NS::Abs(w));
#endif
}

float4 float4::Neg3() const
{
#ifdef MATH_AUTOMATIC_SSE
	return neg3_ps(v);
#else
	return float4(-x, -y, -z, w);
#endif
}

float4 float4::Neg4() const
{
#ifdef MATH_AUTOMATIC_SSE
	return neg_ps(v);
#else
	return float4(-x, -y, -z, -w);
#endif
}

float4 float4::Recip3() const
{
	///\todo SSE.
	return float4(1.f/x, 1.f/y, 1.f/z, w);
}

float4 float4::Recip4() const
{
#ifdef MATH_AUTOMATIC_SSE
	return rcp_ps(v);
#else
	return float4(1.f/x, 1.f/y, 1.f/z, 1.f/w);
#endif
}

float4 float4::RecipFast4() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return float4(rcp_ps(v));
#else
	return float4(1.f/x, 1.f/y, 1.f/z, 1.f/w);
#endif
}

float4 float4::Min(float ceil) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return float4(min_ps(v, set1_ps(ceil)));
#else
	return float4(MATH_NS::Min(x, ceil), MATH_NS::Min(y, ceil), MATH_NS::Min(z, ceil), MATH_NS::Min(w, ceil));
#endif
}

float4 float4::Min(const float4 &ceil) const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(min_ps(v, ceil.v));
#else
	return float4(MATH_NS::Min(x, ceil.x), MATH_NS::Min(y, ceil.y), MATH_NS::Min(z, ceil.z), MATH_NS::Min(w, ceil.w));
#endif
}

float4 float4::Max(float floor) const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(max_ps(v, set1_ps(floor)));
#else
	return float4(MATH_NS::Max(x, floor), MATH_NS::Max(y, floor), MATH_NS::Max(z, floor), MATH_NS::Max(w, floor));
#endif
}

float4 float4::Max(const float4 &floor) const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(max_ps(v, floor.v));
#else
	return float4(MATH_NS::Max(x, floor.x), MATH_NS::Max(y, floor.y), MATH_NS::Max(z, floor.z), MATH_NS::Max(w, floor.w));
#endif
}

float4 float4::Clamp(const float4 &floor, const float4 &ceil) const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(max_ps(min_ps(v, ceil.v), floor.v));
#else
	return float4(MATH_NS::Clamp(x, floor.x, ceil.x),
				  MATH_NS::Clamp(y, floor.y, ceil.y),
				  MATH_NS::Clamp(z, floor.z, ceil.z),
				  MATH_NS::Clamp(w, floor.w, ceil.w));
#endif
}

float4 float4::Clamp01() const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(max_ps(min_ps(v, simd4fOne), simd4fZero));
#else
	return float4(MATH_NS::Clamp(x, 0.f, 1.f),
				  MATH_NS::Clamp(y, 0.f, 1.f),
				  MATH_NS::Clamp(z, 0.f, 1.f),
				  MATH_NS::Clamp(w, 0.f, 1.f));
#endif
}

float4 float4::Clamp(float floor, float ceil) const
{
#ifdef MATH_AUTOMATIC_SSE
	return float4(max_ps(min_ps(v, set1_ps(ceil)), set1_ps(floor)));
#else
	return float4(MATH_NS::Clamp(x, floor, ceil),
				  MATH_NS::Clamp(y, floor, ceil),
				  MATH_NS::Clamp(z, floor, ceil),
				  MATH_NS::Clamp(w, floor, ceil));
#endif
}

float float4::Distance3Sq(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return vec3_length_sq_float(sub_ps(v, rhs.v));
#else
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	float dz = z - rhs.z;
	return dx*dx + dy*dy + dz*dz;
#endif
}

float float4::Distance3(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return vec3_length_float(sub_ps(v, rhs.v));
#else
	return Sqrt(Distance3Sq(rhs));
#endif
}

float float4::Distance4Sq(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return vec4_length_sq_float(sub_ps(v, rhs.v));
#else
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	float dz = z - rhs.z;
	float dw = w - rhs.w;
	return dx*dx + dy*dy + dz*dz + dw*dw;
#endif
}

float float4::Distance4(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return vec4_length_float(sub_ps(v, rhs.v));
#else
	return Sqrt(Distance4Sq(rhs));
#endif
}

float float4::Dot3(const float3 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return dot3_float(v, float4(rhs, 0.f));
#else
	return x * rhs.x + y * rhs.y + z * rhs.z;
#endif
}

float float4::Dot3(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return dot3_float(v, rhs.v);
#else
	return x * rhs.x + y * rhs.y + z * rhs.z;
#endif
}

float float4::Dot4(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return dot4_float(v, rhs.v);
#else
	return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
#endif
}

/** dst = A x B - Apply the diagonal rule to derive the standard cross product formula:
\code
	    |a cross b| = |a||b|sin(alpha)

	    i            j            k            i            j            k        units (correspond to x,y,z)
	    a.x          a.y          a.z          a.x          a.y          a.z      vector a (this)
	    b.x          b.y          b.z          b.x          b.y          b.z      vector b
	-a.z*b.y*i   -a.x*b.z*j   -a.y*b.x*k    a.y*b.z*i    a.z*b.x*j    a.x*b.y*k   result

	Add up the results:
	    x = a.y*b.z - a.z*b.y
	    y = a.z*b.x - a.x*b.z
	    z = a.x*b.y - a.y*b.x
\endcode

Cross product is anti-commutative, i.e. a x b == -b x a.
It distributes over addition, meaning that a x (b + c) == a x b + a x c,
and combines with scalar multiplication: (sa) x b == a x (sb).
i x j == -(j x i) == k,
(j x k) == -(k x j) == i,
(k x i) == -(i x k) == j. */
float4 float4::Cross3(const float3 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return float4(cross_ps(v, load_vec3(rhs.ptr(), 0.f)));
#else
	float4 dst;
	dst.x = y * rhs.z - z * rhs.y;
	dst.y = z * rhs.x - x * rhs.z;
	dst.z = x * rhs.y - y * rhs.x;
	dst.w = 0.f;
	return dst;
#endif
}

float4 float4::Cross3(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	assert((((uintptr_t)&rhs) & 15) == 0); // For SSE ops we must be 16-byte aligned.
	assert((((uintptr_t)this) & 15) == 0);
	return float4(cross_ps(v, rhs.v));
#else
	return Cross3(rhs.xyz());
#endif
}

float4x4 float4::OuterProduct(const float4 &rhs) const
{
	const float4 &i = *this;
	const float4 &j = rhs;
	return float4x4(i[0]*j[0], i[0]*j[1], i[0]*j[2], i[0]*j[3],
					i[1]*j[0], i[1]*j[1], i[1]*j[2], i[1]*j[3],
					i[2]*j[0], i[2]*j[1], i[2]*j[2], i[2]*j[3],
					i[3]*j[0], i[3]*j[1], i[3]*j[2], i[3]*j[3]);
}

float4 float4::Perpendicular3(const float3 &hint, const float3 &hint2) const
{
	assume(!this->IsZero3());
	assume(EqualAbs(w, 0));
	assume(hint.IsNormalized());
	assume(hint2.IsNormalized());
	float3 perp = this->Cross3(hint).xyz();
	float len = perp.Normalize();
	if (len == 0)
		return float4(hint2, 0);
	else
		return float4(perp, 0);
}

float4 float4::Perpendicular(const float4 &hint, const float4 &hint2) const
{
	assume(!this->IsZero3());
	assume(EqualAbs(w, 0));
	assume(hint.IsNormalized());
	assume(hint2.IsNormalized());
	float4 perp = this->Cross(hint);
	float len = perp.Normalize();
	if (len == 0)
		return hint2;
	else
		return perp;
}

float4 float4::AnotherPerpendicular3(const float3 &hint, const float3 &hint2) const
{
	float4 firstPerpendicular = Perpendicular3(hint, hint2);
	float4 perp = this->Cross3(firstPerpendicular);
	return perp.Normalized3();
}

float4 float4::AnotherPerpendicular(const float4 &hint, const float4 &hint2) const
{
	float4 firstPerpendicular = Perpendicular(hint, hint2);
	float4 perp = this->Cross(firstPerpendicular);
	return perp.Normalized();
}

void float4::PerpendicularBasis(float4 &outB, float4 &outC) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Benchmark 'float4_PerpendicularBasis': float4::PerpendicularBasis
	//   Best: 17.468 nsecs / 29.418 ticks, Avg: 17.703 nsecs, Worst: 19.275 nsecs
	basis_ps(this->v, &outB.v, &outC.v);
#else
	float3 b, c;
	Float3Part().PerpendicularBasis(b, c);
	outB = float4(b, 0.f);
	outC = float4(c, 0.f);
#endif
}

float4 float4::RandomPerpendicular(LCG &rng) const
{
	return Perpendicular(RandomDir(rng));
}

float4 float4::Reflect3(const float3 &normal) const
{
	assume2(normal.IsNormalized(), normal.SerializeToCodeString(), normal.Length());
	assume(EqualAbs(w, 0));
	return 2.f * this->ProjectToNorm3(normal) - *this;
}

float4 float4::Reflect(const float4 &normal) const
{
	assume2(normal.IsNormalized(), normal.SerializeToCodeString(), normal.Length());
	assume(EqualAbs(w, 0));
	return 2.f * this->ProjectToNorm(normal) - *this;
}

/// Implementation from http://www.flipcode.com/archives/reflection_transmission.pdf .
float4 float4::Refract(const float4 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const
{
	// This code is duplicated in float2::Refract.
	float n = negativeSideRefractionIndex / positiveSideRefractionIndex;
	float cosI = this->Dot(normal);
	float sinT2 = n*n*(1.f - cosI*cosI);
	if (sinT2 > 1.f) // Total internal reflection occurs?
		return (-*this).Reflect(normal);
	return n * *this - (n + Sqrt(1.f - sinT2)) * normal;
}

float float4::AngleBetween3(const float4 &other) const
{
	float cosa = Dot3(other) / Sqrt(LengthSq3() * other.LengthSq3());
	if (cosa >= 1.f)
		return 0.f;
	else if (cosa <= -1.f)
		return pi;
	else
		return acos(cosa);
}

float float4::AngleBetweenNorm3(const float4 &other) const
{
	assume(this->IsNormalized3());
	assume(other.IsNormalized3());
	return acos(Dot3(other));
}

float float4::AngleBetween4(const float4 &other) const
{
	float cosa = Dot4(other) / Sqrt(LengthSq4() * other.LengthSq4());
	if (cosa >= 1.f)
		return 0.f;
	else if (cosa <= -1.f)
		return pi;
	else
		return acos(cosa);
}

float float4::AngleBetweenNorm4(const float4 &other) const
{
	assume(this->IsNormalized4());
	assume(other.IsNormalized4());
	return acos(Dot4(other));
}

float4 float4::ProjectTo3(const float3 &target) const
{
	assume(!target.IsZero());
	assume(this->IsWZeroOrOne());
	return float4(target * MATH_NS::Dot(xyz(), target) / target.LengthSq(), w);
}

float4 float4::ProjectTo(const float4 &target) const
{
	assume(!target.IsZero());
	assume(this->IsWZeroOrOne());
	return target * (this->Dot(target) / target.LengthSq());
}

float4 float4::ProjectToNorm3(const float3 &target) const
{
	assume(target.IsNormalized());
	assume(this->IsWZeroOrOne());
	return float4(target * MATH_NS::Dot(xyz(), target), w);
}

float4 float4::ProjectToNorm(const float4 &target) const
{
	assume(target.IsNormalized());
	assume(this->IsWZeroOrOne());
	return target * this->Dot(target);
}

bool MUST_USE_RESULT float4::AreCollinear(const float4 &p1, const float4 &p2, const float4 &p3, float epsilon)
{
	return (p2 - p1).Cross(p3 - p1).LengthSq() <= epsilon;
}

float4 float4::Lerp(const float4 &b, float t) const
{
	assume(EqualAbs(this->w, b.w));
	assume(0.f <= t && t <= 1.f);
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return vec4_lerp(v, b.v, t);
#else
	return (1.f - t) * *this + t * b;
#endif
}

float4 float4::Lerp(const float4 &a, const float4 &b, float t)
{
	return a.Lerp(b, t);
}

bool MUST_USE_RESULT float4::AreOrthogonal(const float4 &a, const float4 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon);
}

bool MUST_USE_RESULT float4::AreOrthogonal(const float4 &a, const float4 &b, const float4 &c, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) &&
	       a.IsPerpendicular(c, epsilon) &&
	       b.IsPerpendicular(c, epsilon);
}

void float4::Orthonormalize(float4 &a, float4 &b)
{
	assume(!a.IsZero());
	assume(!b.IsZero());
	a.Normalize();
	b -= b.ProjectToNorm(a);
	b.Normalize();
}

void float4::Orthonormalize(float4 &a, float4 &b, float4 &c)
{
	assume(!a.IsZero());
	a.Normalize();
	b -= b.ProjectToNorm(a);
	assume(!b.IsZero());
	b.Normalize();
	c -= c.ProjectToNorm(a);
	c -= c.ProjectToNorm(b);
	assume(!c.IsZero());
	c.Normalize();
}

bool MUST_USE_RESULT float4::AreOrthonormal(const float4 &a, const float4 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) && a.IsNormalized(epsilon*epsilon) && b.IsNormalized(epsilon*epsilon);
}

bool MUST_USE_RESULT float4::AreOrthonormal(const float4 &a, const float4 &b, const float4 &c, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) &&
		a.IsPerpendicular(c, epsilon) &&
		b.IsPerpendicular(c, epsilon) &&
		a.IsNormalized(epsilon*epsilon) &&
		b.IsNormalized(epsilon*epsilon) &&
		c.IsNormalized(epsilon*epsilon);
}

float4 float4::FromScalar(float scalar)
{
#ifdef MATH_AUTOMATIC_SSE
	return set1_ps(scalar);
#else
	return float4(scalar, scalar, scalar, scalar);
#endif
}

float4 float4::FromScalar(float scalar, float w_)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	simd4f s = set1_ps(scalar);
	simd4f highPart = _mm_unpacklo_ps(s, _mm_set_ss(w_)); // [_ _ w s]
	return _mm_movelh_ps(s, highPart); // [w s s s]
#else
	return float4(scalar, scalar, scalar, w_);
#endif
}

void float4::SetFromScalar(float scalar)
{
#ifdef MATH_AUTOMATIC_SSE
	v = set1_ps(scalar);
#else
	x = scalar;
	y = scalar;
	z = scalar;
	w = scalar;
#endif
}

void float4::Set(float x_, float y_, float z_, float w_)
{
#ifdef MATH_AUTOMATIC_SSE
	v = set_ps(w_, z_, y_, x_);
#else
	x = x_;
	y = y_;
	z = z_;
	w = w_;
#endif
}

void float4::Set(const float4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	v = rhs.v;
#else
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	w = rhs.w;
#endif
}

void float4::SetFromScalar(float scalar, float w_)
{
#ifdef MATH_AUTOMATIC_SSE
#ifdef MATH_SSE
	simd4f s = set1_ps(scalar);
	simd4f highPart = _mm_unpacklo_ps(s, _mm_set_ss(w_)); // [_ _ w s]
	v = _mm_movelh_ps(s, highPart); // [w s s s]
#else
	v = set_ps(w_, scalar, scalar, scalar);
#endif
#else
	x = scalar;
	y = scalar;
	z = scalar;
	w = w_;
#endif
}

void float4::SetFromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float cx = Cos(inclination);
	float sin, cos;
	SinCos(azimuth, sin, cos);
	x = cx * sin * radius;
	y = -Sin(inclination) * radius;
	z = cx * cos * radius;
	w = 0.f;
}

float4 MUST_USE_RESULT float4::FromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float4 v;
	v.SetFromSphericalCoordinates(azimuth, inclination, radius);
	return v;
}

void float4::SetFromSphericalCoordinates(float azimuth, float inclination)
{
	float4 vec, s, c;
	vec.x = inclination;
	vec.y = azimuth;
	SinCos2(vec, s, c);
	x = c.x * s.y;
	y = -s.x;
	z = c.x * c.y;
	w = 0.f;
}

float4 MUST_USE_RESULT float4::FromSphericalCoordinates(float azimuth, float inclination)
{
	float4 v;
	v.SetFromSphericalCoordinates(azimuth, inclination);
	return v;
}

float3 float4::ToSphericalCoordinates() const
{
	// R_y * R_x * (0,0,length) = (cosx*siny, -sinx, cosx*cosy).
	float4 vec = *this;
	float len = vec.Normalize();
	if (len <= 1e-5f)
		return float3::zero;
	float azimuth = atan2(vec.x, vec.z);
	float inclination = asin(-vec.y);
	return float3(azimuth, inclination, len);
}

float2 float4::ToSphericalCoordinatesNormalized() const
{
	assume(IsNormalized());
	float azimuth = atan2(x, z);
	float inclination = asin(-y);
	return float2(azimuth, inclination);
}

bool float4::Equals(const float4 &other, float epsilon) const
{
	return MATH_NS::Abs(x - other.x) < epsilon &&
	       MATH_NS::Abs(y - other.y) < epsilon &&
	       MATH_NS::Abs(z - other.z) < epsilon &&
	       MATH_NS::Abs(w - other.w) < epsilon;
}

bool float4::Equals(float x_, float y_, float z_, float w_, float epsilon) const
{
	return MATH_NS::Abs(x - x_) < epsilon &&
	       MATH_NS::Abs(y - y_) < epsilon &&
	       MATH_NS::Abs(z - z_) < epsilon &&
	       MATH_NS::Abs(w - w_) < epsilon;
}

bool float4::BitEquals(const float4 &other) const
{
	return ReinterpretAsU32(x) == ReinterpretAsU32(other.x) &&
		ReinterpretAsU32(y) == ReinterpretAsU32(other.y) &&
		ReinterpretAsU32(z) == ReinterpretAsU32(other.z) &&
		ReinterpretAsU32(w) == ReinterpretAsU32(other.w);
}

float4 MUST_USE_RESULT float4::RandomDir(LCG &lcg, float length)
{
	return DIR_TO_FLOAT4(Sphere(POINT_VEC_SCALAR(0.f), length).RandomPointOnSurface(lcg) - vec(POINT_VEC_SCALAR(0.f)));
}

float4 MUST_USE_RESULT float4::RandomSphere(LCG &lcg, const float4 &center, float radius)
{
	return POINT_TO_FLOAT4(Sphere(FLOAT4_TO_POINT(center), radius).RandomPointInside(lcg));
}

float4 MUST_USE_RESULT float4::RandomBox(LCG &lcg, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
	return RandomBox(lcg, float4(xmin, ymin, zmin, 1.f), float4(xmax, ymax, zmax, 1.f));
}

float4 MUST_USE_RESULT float4::RandomBox(LCG &lcg, float minElem, float maxElem)
{
	return RandomBox(lcg, float4(minElem, minElem, minElem, 1.f), float4(maxElem, maxElem, maxElem, 1.f));
}

float4 MUST_USE_RESULT float4::RandomBox(LCG &lcg, const float4 &minValues, const float4 &maxValues)
{
	return POINT_TO_FLOAT4(AABB(FLOAT4_TO_POINT(minValues), FLOAT4_TO_POINT(maxValues)).RandomPointInside(lcg));
}

float4 float4::RandomGeneral(LCG &lcg, float minElem, float maxElem)
{
	return float4(lcg.Float(minElem, maxElem), lcg.Float(minElem, maxElem), lcg.Float(minElem, maxElem), lcg.Float(minElem, maxElem));
}

float4 float4::operator +(const float4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return add_ps(v, rhs.v);
#else
	return float4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
#endif
}

float4 float4::operator -(const float4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return sub_ps(v, rhs.v);
#else
	return float4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
#endif
}

float4 float4::operator -() const
{
#ifdef MATH_AUTOMATIC_SSE
	return neg_ps(v);
#else
	return float4(-x, -y, -z, -w);
#endif
}

float4 float4::operator *(float scalar) const
{
#ifdef MATH_AUTOMATIC_SSE
	return muls_ps(v, scalar);
#else
	return float4(x * scalar, y * scalar, z * scalar, w * scalar);
#endif
}

float4 operator *(float scalar, const float4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	return muls_ps(rhs.v, scalar);
#else
	return float4(scalar * rhs.x, scalar * rhs.y, scalar * rhs.z, scalar * rhs.w);
#endif
}

float4 float4::operator /(float scalar) const
{
#ifdef MATH_AUTOMATIC_SSE
	return div_ps(v, set1_ps(scalar));
#else
	float invScalar = 1.f / scalar;
	return float4(x * invScalar, y * invScalar, z * invScalar, w * invScalar);
#endif
}

float4 &float4::operator =(const float4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	v = rhs.v;
#else
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	w = rhs.w;
#endif
	
	return *this;
}

float4 &float4::operator +=(const float4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	v = add_ps(v, rhs.v);
#else
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	w += rhs.w;
#endif

	return *this;
}

float4 &float4::operator -=(const float4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	v = sub_ps(v, rhs.v);
#else
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	w -= rhs.w;
#endif

	return *this;
}

float4 &float4::operator *=(float scalar)
{
#ifdef MATH_AUTOMATIC_SSE
	v = muls_ps(v, scalar);
#else
	x *= scalar;
	y *= scalar;
	z *= scalar;
	w *= scalar;
#endif

	return *this;
}

float4 &float4::operator /=(float scalar)
{
#ifdef MATH_AUTOMATIC_SSE
	v = div_ps(v, set1_ps(scalar));
#else
	float invScalar = 1.f / scalar;
	x *= invScalar;
	y *= invScalar;
	z *= invScalar;
	w *= invScalar;
#endif

	return *this;
}

float4 float4::Add(float s) const
{
#ifdef MATH_AUTOMATIC_SSE
	return add_ps(v, set1_ps(s));
#else
	return float4(x + s, y + s, z + s, w + s);
#endif
}

float4 float4::Sub(float s) const
{
#ifdef MATH_AUTOMATIC_SSE
	return sub_ps(v, set1_ps(s));
#else
	return float4(x - s, y - s, z - s, w - s);
#endif
}

float4 float4::SubLeft(float s) const
{
#ifdef MATH_AUTOMATIC_SSE
	return sub_ps(set1_ps(s), v);
#else
	return float4(s - x, s - y, s - z, s - w);
#endif
}

float4 float4::DivLeft(float s) const
{
#ifdef MATH_AUTOMATIC_SSE
	return div_ps(set1_ps(s), v);
#else
	return float4(s / x, s / y, s / z, s / w);
#endif
}

float4 float4::Mul(const float4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return mul_ps(v, rhs.v);
#else
	return float4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
#endif
}

float4 float4::Div(const float4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return div_ps(v, rhs.v);
#else
	return float4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
#endif
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float4 &rhs)
{
	std::string str = rhs.ToString();
	out << str;
	return out;
}
#endif

float4 Perp2D(const float4 &v) { return float4(v.xy().Perp(), v.z, v.w); }
float4 Mul2D(const float3x3 &transform, const float4 &v) { return transform.Mul(v); }
float4 MulPos2D(const float3x4 &transform, const float4 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 1.f)); }
float4 MulPos2D(const float4x4 &transform, const float4 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 1.f)); }
float4 MulDir2D(const float3x4 &transform, const float4 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 0.f)); }
float4 MulDir2D(const float4x4 &transform, const float4 &v) { return transform.Transform(float4(v.x, v.y, 0.f, 0.f)); }

const float4 float4::zero = float4(0, 0, 0, 0);
const float4 float4::one = float4(1, 1, 1, 1);
const float4 float4::unitX = float4(1, 0, 0, 0);
const float4 float4::unitY = float4(0, 1, 0, 0);
const float4 float4::unitZ = float4(0, 0, 1, 0);
const float4 float4::unitW = float4(0, 0, 0, 1);
const float4 float4::nan = float4(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);
const float4 float4::inf = float4(FLOAT_INF, FLOAT_INF, FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
