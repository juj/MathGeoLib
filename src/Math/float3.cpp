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

/** @file float3.cpp
	@author Jukka Jylänki
	@brief */
#include "float3.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include "myassert.h"
#include <utility>
#endif
#include <stdlib.h>

#include "float2.h"
#include "float4.h"
#include "float3x3.h"
#include "../Geometry/Line.h"
#include "../Geometry/Ray.h"
#include "../Geometry/LineSegment.h"
#include "../Geometry/Sphere.h"
#include "../Geometry/AABB.h"
#include "../Geometry/OBB.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Triangle.h"
#include "../Geometry/Capsule.h"
#include "MathFunc.h"

#ifdef MATH_SIMD
#include "simd.h"
#include "float4_sse.h"
#include "float4_neon.h"
#endif

MATH_BEGIN_NAMESPACE

using namespace std;

float3::float3(float x_, float y_, float z_)
:x(x_), y(y_), z(z_)
{
}

float3::float3(float scalar)
:x(scalar), y(scalar), z(scalar)
{
}

float3::float3(const float2 &xy, float z_)
:x(xy.x), y(xy.y), z(z_)
{
}

float3::float3(const float *data)
{
	assume(data);
	x = data[0];
	y = data[1];
	z = data[2];
}

float2 float3::xx() const { return float2(x,x); }
float2 float3::xy() const { return float2(x,y); }
float2 float3::xz() const { return float2(x,z); }
float2 float3::yx() const { return float2(y,x); }
float2 float3::yy() const { return float2(y,y); }
float2 float3::yz() const { return float2(y,z); }
float2 float3::zx() const { return float2(z,x); }
float2 float3::zy() const { return float2(z,y); }
float2 float3::zz() const { return float2(z,z); }

float2 float3::Swizzled(int i, int j) const
{
	return float2(At(i), At(j));
}

float3 float3::Swizzled(int i, int j, int k) const
{
	return float3(At(i), At(j), At(k));
}

float4 float3::Swizzled(int i, int j, int k, int l) const
{
	return float4(At(i), At(j), At(k), At(l));
}

float float3::LengthSq() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	simd4f v = load_vec3(ptr(), 0.f);
	return sum_xyz_float(mul_ps(v, v));
#else
	return x*x + y*y + z*z;
#endif
}

float float3::Length() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	simd4f v = load_vec3(ptr(), 0.f);
	return s4f_x(sqrt_ps(sum_xyz_ps(mul_ps(v, v))));
#else
	return Sqrt(LengthSq());
#endif
}

float float3::Normalize()
{
#if defined(MATH_AUTOMATIC_SIMD_FLOAT3) && defined(MATH_SSE)
	simd4f origLength;
	simd4f normalized = vec4_safe_normalize3(load_vec3(ptr(), 0.f), origLength);
	store_vec3(ptr(), normalized);
	return s4f_x(origLength);
#else
	assume(IsFinite());
	float length = Length();
	if (length > 1e-6f)
	{
		*this *= 1.f / length;
		return length;
	}
	else
	{
		Set(1.f, 0.f, 0.f); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
#endif
}

float3 float3::Normalized() const
{
#if defined(MATH_AUTOMATIC_SIMD_FLOAT3) && defined(MATH_SSE)
	simd4f origLength;
	simd4f normalized = vec4_safe_normalize3(load_vec3(ptr(), 0.f), origLength);
	float3 copy;
	store_vec3(copy.ptr(), normalized);
	return copy;
#else
	float3 copy = *this;
	float oldLength = copy.Normalize();
	assume(oldLength > 0.f && "float3::Normalized() failed!");
	MARK_UNUSED(oldLength);
	return copy;
#endif
}

float float3::ScaleToLength(float newLength)
{
	float length = LengthSq();
	if (length < 1e-6f)
	{
		Set(newLength, 0, 0); // Will always produce a vector of the requested length.
		return 0.f;
	}

	length = Sqrt(length);
	float scalar = newLength / length;
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return length;
}

float3 float3::ScaledToLength(float newLength) const
{
	assume(!IsZero());

	float3 v = *this;
	v.ScaleToLength(newLength);
	return v;
}

bool float3::IsNormalized(float epsilonSq) const
{
	return MATH_NS::Abs(LengthSq()-1.f) <= epsilonSq;
}

bool float3::IsZero(float epsilonSq) const
{
	return LengthSq() <= epsilonSq;
}

bool float3::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y) && MATH_NS::IsFinite(z);
}

bool float3::IsPerpendicular(const float3 &other, float epsilonSq) const
{
	float dot = Dot(other);
	return dot*dot <= epsilonSq * LengthSq() * other.LengthSq();
}

bool MUST_USE_RESULT float3::AreCollinear(const float3 &p1, const float3 &p2, const float3 &p3, float epsilonSq)
{
	return (p2-p1).Cross(p3-p1).LengthSq() <= epsilonSq;
}

bool IsNeutralCLocale();

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float3::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f)", x, y, z);
	return str;
}

StringT float3::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(x, str); *s = ','; ++s;
	s = SerializeFloat(y, s); *s = ','; ++s;
	s = SerializeFloat(z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT float3::SerializeToCodeString() const
{
	return "float3(" + SerializeToString() + ")";
}
#endif

float3 MUST_USE_RESULT float3::FromString(const char *str, const char **outEndStr)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return float3::nan;
	MATH_SKIP_WORD(str, "float3");
	MATH_SKIP_WORD(str, "(");
	float3 f;
	f.x = DeserializeFloat(str, &str);
	f.y = DeserializeFloat(str, &str);
	f.z = DeserializeFloat(str, &str);
	if (*str == ')')
		++str;
	if (*str == ',')
		++str;
	if (outEndStr)
		*outEndStr = str;
	return f;
}

int CountCommas(const char *start, const char *end)
{
	int commas = 0;
	while(start != end)
		if (*start++ == ',')
			++commas;
	return commas;
}

const char *FindNext(const char *str, char ch)
{
	if (!str)
		return str;
	while(*str)
	{
		if (*str == ch)
			break;
		++str;
	}
	return str;
}

vec PointVecFromString(const char *str, const char **outEndStr)
{
	if (!str)
		return vec::nan;
	if (MATH_NEXT_WORD_IS(str, "float3"))
		return POINT_VEC(float3::FromString(str, outEndStr));
	if (MATH_NEXT_WORD_IS(str, "float4"))
		return FLOAT4_TO_POINT(float4::FromString(str, outEndStr));
	while(*str == ' ')
		++str;
	if (*str == '(')
	{
		const char *end = FindNext(str, ')');
		int numFields = CountCommas(str, end) + 1;
		assume1(numFields == 3 || numFields == 4, numFields);
		if (numFields == 4)
			return FLOAT4_TO_POINT(float4::FromString(str, outEndStr));
	}
	// Default to assuming that the shorter form of serialization was used and there is three floats.
	return POINT_VEC(float3::FromString(str, outEndStr));
}

vec DirVecFromString(const char *str, const char **outEndStr)
{
	if (!str)
		return vec::nan;
	if (MATH_NEXT_WORD_IS(str, "float3"))
		return DIR_VEC(float3::FromString(str, outEndStr));
	if (MATH_NEXT_WORD_IS(str, "float4"))
		return FLOAT4_TO_DIR(float4::FromString(str, outEndStr));
	while(*str == ' ')
		++str;
	if (*str == '(')
	{
		const char *end = FindNext(str, ')');
		int numFields = CountCommas(str, end) + 1;
		assume1(numFields == 3 || numFields == 4, numFields);
		if (numFields == 4)
			return FLOAT4_TO_DIR(float4::FromString(str, outEndStr));
	}
	// Default to assuming that the shorter form of serialization was used and there is three floats.
	return DIR_VEC(float3::FromString(str, outEndStr));
}

float float3::SumOfElements() const
{
	return x + y + z;
}

float float3::ProductOfElements() const
{
	return x * y * z;
}

float float3::AverageOfElements() const
{
	return (x + y + z) / 3.f;
}

float float3::MinElement() const
{
	return MATH_NS::Min(MATH_NS::Min(x, y), z);
}

int float3::MinElementIndex() const
{
	if (x <= y && x <= z)
		return 0;
	else
		return (y <= z) ? 1 : 2;
}

float float3::MaxElement() const
{
	return MATH_NS::Max(MATH_NS::Max(x, y), z);
}

int float3::MaxElementIndex() const
{
	if (x >= y && x >= z)
		return 0;
	else
		return (y >= z) ? 1 : 2;
}

float3 float3::Abs() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), abs_ps(load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(MATH_NS::Abs(x), MATH_NS::Abs(y), MATH_NS::Abs(z));
#endif
}

float3 float3::Neg() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), neg_ps(load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(-x, -y, -z);
#endif
}

float3 float3::Recip() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), rcp_ps(load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(1.f/x, 1.f/y, 1.f/z);
#endif
}

float3 float3::Min(float ceil) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), min_ps(load_vec3(ptr(), 0.f), set1_ps(ceil)));
	return copy;
#else
	return float3(MATH_NS::Min(x, ceil), MATH_NS::Min(y, ceil), MATH_NS::Min(z, ceil));
#endif
}

float3 float3::Min(const float3 &ceil) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), min_ps(load_vec3(ptr(), 0.f), load_vec3(ceil.ptr(), 0.f)));
	return copy;
#else
	return float3(MATH_NS::Min(x, ceil.x), MATH_NS::Min(y, ceil.y), MATH_NS::Min(z, ceil.z));
#endif
}

float3 float3::Max(float floor) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), max_ps(load_vec3(ptr(), 0.f), set1_ps(floor)));
	return copy;
#else
	return float3(MATH_NS::Max(x, floor), MATH_NS::Max(y, floor), MATH_NS::Max(z, floor));
#endif
}

float3 float3::Max(const float3 &floor) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), max_ps(load_vec3(ptr(), 0.f), load_vec3(floor.ptr(), 0.f)));
	return copy;
#else
	return float3(MATH_NS::Max(x, floor.x), MATH_NS::Max(y, floor.y), MATH_NS::Max(z, floor.z));
#endif
}

float3 float3::Clamp(const float3 &floor, const float3 &ceil) const
{
	return Min(ceil).Max(floor);
}

float3 float3::Clamp01() const
{
	return Clamp(0.f, 1.f);
}

float3 float3::Clamp(float floor, float ceil) const
{
	return Min(ceil).Max(floor);
}

float3 float3::ClampLength(float maxLength) const
{
	float lenSq = LengthSq();
	if (lenSq > maxLength*maxLength)
		return *this * (maxLength / Sqrt(lenSq));
	else
		return *this;
}

float3 float3::ClampLength(float minLength, float maxLength) const
{
	float lenSq = LengthSq();
	if (lenSq > maxLength*maxLength)
		return *this * (maxLength / Sqrt(lenSq));
	else if (lenSq < minLength*minLength)
		return *this * (minLength / Sqrt(lenSq));
	else
		return *this;
}

float float3::DistanceSq(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	simd4f d = sub_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f));
	return s4f_x(mul_ps(d, d));
#else
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	float dz = z - rhs.z;
	return dx*dx + dy*dy + dz*dz;
#endif
}

double float3::DistanceSqD(const float3 &rhs) const
{
	double dx = x - rhs.x;
	double dy = y - rhs.y;
	double dz = z - rhs.z;
	return dx*dx + dy*dy + dz*dz;
}

float float3::Distance(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	simd4f d = sub_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f));
	return s4f_x(sqrt_ps(mul_ps(d, d)));
#else
	return Sqrt(DistanceSq(rhs));
#endif
}

float float3::Distance(const Line &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const Ray &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const LineSegment &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const Plane &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const Triangle &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const AABB &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const OBB &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const Sphere &rhs) const { return rhs.Distance(POINT_VEC(*this)); }
float float3::Distance(const Capsule &rhs) const { return rhs.Distance(POINT_VEC(*this)); }

float float3::Dot(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	return dot3_float(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f));
#else
	return x * rhs.x + y * rhs.y + z * rhs.z;
#endif
}

/** dst = A x B - The standard cross product:
\code
		|a cross b| = |a||b|sin(alpha)
	
		i		j		k		i		j		k		units (correspond to x,y,z)
		a		b		c		a		b		c		this vector
		d		e		f		d		e		f		vector v
		-cei	-afj	-bdk	bfi	cdj	aek	result
	
		x = bfi - cei = (bf-ce)i;
		y = cdj - afj = (cd-af)j;
		z - aek - bdk = (ae-bd)k;
\endcode

Cross product is anti-commutative, i.e. a x b == -b x a.
It distributes over addition, meaning that a x (b + c) == a x b + a x c,
and combines with scalar multiplication: (sa) x b == a x (sb).
i x j == -(j x i) == k,
(j x k) == -(k x j) == i,
(k x i) == -(i x k) == j. */
float3 float3::Cross(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), cross_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
	return copy;
#else
	return float3(y * rhs.z - z * rhs.y,
	              z * rhs.x - x * rhs.z,
	              x * rhs.y - y * rhs.x);
#endif
}

float3x3 float3::OuterProduct(const float3 &rhs) const
{
	const float3 &u = *this;
	const float3 &v = rhs;
	return float3x3(u[0]*v[0], u[0]*v[1], u[0]*v[2],
	                u[1]*v[0], u[1]*v[1], u[1]*v[2],
	                u[2]*v[0], u[2]*v[1], u[2]*v[2]);
}

float3 float3::Perpendicular(const float3 &hint, const float3 &hint2) const
{
	assume(!this->IsZero());
	assume(hint.IsNormalized());
	assume(hint2.IsNormalized());
	float3 v = this->Cross(hint);
	float len = v.Normalize();
	if (len == 0)
		return hint2;
	else
		return v;
}

float3 float3::AnotherPerpendicular(const float3 &hint, const float3 &hint2) const
{
	assume(!this->IsZero());
	assume(hint.IsNormalized());
	assume(hint2.IsNormalized());
	float3 firstPerpendicular = Perpendicular(hint, hint2);
	float3 v = this->Cross(firstPerpendicular);
	return v.Normalized();
}

void float3::PerpendicularBasis(float3 &outB, float3 &outC) const
{
#if defined(MATH_AUTOMATIC_SIMD_FLOAT3) && defined(MATH_SSE)
	simd4f v = load_vec3(&x, 0.f);
	simd4f out1, out2;
	basis_ps(v, &out1, &out2);
	store_vec3(&outB.x, out1);
	store_vec3(&outC.x, out2);
#else
	// Pixar orthonormal basis code: https://graphics.pixar.com/library/OrthonormalB/paper.pdf
	float sign = copysignf(1.0f, z);
	const float a = -1.0f / (sign + z);
	const float b = x * y * a;
	outB = float3(1.0f + sign * x * x * a, sign * b,             -sign * x);
	outC = float3(                      b, sign + y * y * a,            -y);
#endif
}

float3 float3::RandomPerpendicular(LCG &rng) const
{
	return Perpendicular(RandomDir(rng));
}

float MUST_USE_RESULT float3::ScalarTripleProduct(const float3 &u, const float3 &v, const float3 &w)
{
	return u.Cross(v).Dot(w);
}

float3 float3::Reflect(const float3 &normal) const
{
	assume2(normal.IsNormalized(), normal.SerializeToCodeString(), normal.Length());
	return 2.f * this->ProjectToNorm(normal) - *this;
}

/// Implementation from http://www.flipcode.com/archives/reflection_transmission.pdf .
float3 float3::Refract(const float3 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const
{
	// This code is duplicated in float2::Refract.
	float n = negativeSideRefractionIndex / positiveSideRefractionIndex;
	float cosI = this->Dot(normal);
	float sinT2 = n*n*(1.f - cosI*cosI);
	if (sinT2 > 1.f) // Total internal reflection occurs?
		return (-*this).Reflect(normal);
	return n * *this - (n + Sqrt(1.f - sinT2)) * normal;
}

float3 float3::ProjectTo(const float3 &direction) const
{
	assume(!direction.IsZero());
	return direction * this->Dot(direction) / direction.LengthSq();
}

float3 float3::ProjectToNorm(const float3 &direction) const
{
	assume(direction.IsNormalized());
	return direction * this->Dot(direction);
}

float float3::AngleBetween(const float3 &other) const
{
	float cosa = Dot(other) / Sqrt(LengthSq() * other.LengthSq());
	if (cosa >= 1.f)
		return 0.f;
	else if (cosa <= -1.f)
		return pi;
	else
		return acos(cosa);
}

float float3::AngleBetweenNorm(const float3 &other) const
{
	assume(this->IsNormalized());
	assume(other.IsNormalized());
	float cosa = Dot(other);
	if (cosa >= 1.f)
		return 0.f;
	else if (cosa <= -1.f)
		return pi;
	else
		return acos(cosa);
}

void float3::Decompose(const float3 &direction, float3 &outParallel, float3 &outPerpendicular) const
{
	assume(direction.IsNormalized());
	outParallel = this->ProjectToNorm(direction);
	outPerpendicular = *this - outParallel;
}

float3 float3::Lerp(const float3 &b, float t) const
{
	assume(0.f <= t && t <= 1.f);
	return (1.f - t) * *this + t * b;
}

float3 MUST_USE_RESULT float3::Lerp(const float3 &a, const float3 &b, float t)
{
	return a.Lerp(b, t);
}

void float3::Orthogonalize(const float3 &a, float3 &b)
{
	if (!a.IsZero())
		b -= b.ProjectTo(a);
}

void float3::Orthogonalize(const float3 &a, float3 &b, float3 &c)
{
	if (!a.IsZero())
	{
		b -= b.ProjectTo(a);
		c -= c.ProjectTo(a);
	}
	if (!b.IsZero())
		c -= c.ProjectTo(b);
}

bool MUST_USE_RESULT float3::AreOrthogonal(const float3 &a, const float3 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon);
}

bool MUST_USE_RESULT float3::AreOrthogonal(const float3 &a, const float3 &b, const float3 &c, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) &&
	       a.IsPerpendicular(c, epsilon) &&
	       b.IsPerpendicular(c, epsilon);
}

void float3::Orthonormalize(float3 &a, float3 &b)
{
	assume(!a.IsZero());
	assume(!b.IsZero());
	a.Normalize();
	b -= b.ProjectToNorm(a);
	b.Normalize();
}

void float3::Orthonormalize(float3 &a, float3 &b, float3 &c)
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

bool MUST_USE_RESULT float3::AreOrthonormal(const float3 &a, const float3 &b, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) && a.IsNormalized(epsilon*epsilon) && b.IsNormalized(epsilon*epsilon);
}

bool MUST_USE_RESULT float3::AreOrthonormal(const float3 &a, const float3 &b, const float3 &c, float epsilon)
{
	return a.IsPerpendicular(b, epsilon) &&
	       a.IsPerpendicular(c, epsilon) &&
	       b.IsPerpendicular(c, epsilon) &&
	       a.IsNormalized(epsilon*epsilon) &&
	       b.IsNormalized(epsilon*epsilon) &&
	       c.IsNormalized(epsilon*epsilon);
}

float3 MUST_USE_RESULT float3::FromScalar(float scalar)
{
	return float3(scalar, scalar, scalar);
}

void float3::SetFromScalar(float scalar)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), set1_ps(scalar));
#else
	x = scalar;
	y = scalar;
	z = scalar;
#endif
}

void float3::Set(float x_, float y_, float z_)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), set_ps(0.f, z_, y_, x_));
#else
	x = x_;
	y = y_;
	z = z_;
#endif
}

void float3::SetFromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float cx = Cos(inclination);
	float sin, cos;
	SinCos(azimuth, sin, cos);
	x = cx * sin * radius;
	y = -Sin(inclination) * radius;
	z = cx * cos * radius;
}

float3 MUST_USE_RESULT float3::FromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float3 v;
	v.SetFromSphericalCoordinates(azimuth, inclination, radius);
	return v;
}

void float3::SetFromSphericalCoordinates(float azimuth, float inclination)
{
	float4 v, s, c;
	v.x = inclination;
	v.y = azimuth;
	SinCos2(v, s, c);
	x = c.x * s.y;
	y = -s.x;
	z = c.x * c.y;
}

float3 MUST_USE_RESULT float3::FromSphericalCoordinates(float azimuth, float inclination)
{
	float3 v;
	v.SetFromSphericalCoordinates(azimuth, inclination);
	return v;
}

float3 float3::ToSphericalCoordinates() const
{
	// R_y * R_x * (0,0,length) = (cosx*siny, -sinx, cosx*cosy).
	float3 v = *this;
	float len = v.Normalize();
	if (len <= 1e-5f)
		return float3::zero;
	float azimuth = atan2(v.x, v.z);
	float inclination = asin(-v.y);
	return float3(azimuth, inclination, len);
}

float2 float3::ToSphericalCoordinatesNormalized() const
{
	assume(IsNormalized());
	float azimuth = atan2(x, z);
	float inclination = asin(-y);
	return float2(azimuth, inclination);
}

bool float3::Equals(const float3 &other, float epsilon) const
{
	return MATH_NS::Abs(x - other.x) < epsilon &&
	       MATH_NS::Abs(y - other.y) < epsilon &&
	       MATH_NS::Abs(z - other.z) < epsilon;
}

bool float3::Equals(float x_, float y_, float z_, float epsilon) const
{
	return MATH_NS::Abs(x - x_) < epsilon &&
	       MATH_NS::Abs(y - y_) < epsilon &&
	       MATH_NS::Abs(z - z_) < epsilon;
}

bool float3::BitEquals(const float3 &other) const
{
	return ReinterpretAsU32(x) == ReinterpretAsU32(other.x) &&
		ReinterpretAsU32(y) == ReinterpretAsU32(other.y) && 
		ReinterpretAsU32(z) == ReinterpretAsU32(other.z);
}

float4 float3::ToPos4() const
{
	return float4(*this, 1.f);
}

float4 float3::ToDir4() const
{
	return float4(*this, 0.f);
}

float3 MUST_USE_RESULT float3::RandomDir(LCG &lcg, float length)
{
	return POINT_TO_FLOAT3(Sphere(POINT_VEC_SCALAR(0.f), length).RandomPointOnSurface(lcg));
}

float3 MUST_USE_RESULT float3::RandomSphere(LCG &lcg, const float3 &center, float radius)
{
	return POINT_TO_FLOAT3(Sphere(POINT_VEC(center), radius).RandomPointInside(lcg));
}

float3 MUST_USE_RESULT float3::RandomBox(LCG &lcg, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
	return RandomBox(lcg, float3(xmin, ymin, zmin), float3(xmax, ymax, zmax));
}

float3 MUST_USE_RESULT float3::RandomBox(LCG &lcg, float minElem, float maxElem)
{
	return RandomBox(lcg, float3(minElem, minElem, minElem), float3(maxElem, maxElem, maxElem));
}

float3 MUST_USE_RESULT float3::RandomBox(LCG &lcg, const float3 &minValues, const float3 &maxValues)
{
	return POINT_TO_FLOAT3(AABB(POINT_VEC(minValues), POINT_VEC(maxValues)).RandomPointInside(lcg));
}

float3 float3::operator +(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), add_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
	return copy;
#else
	return float3(x + rhs.x, y + rhs.y, z + rhs.z);
#endif
}

float3 float3::operator -(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), sub_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
	return copy;
#else
	return float3(x - rhs.x, y - rhs.y, z - rhs.z);
#endif
}

float3 float3::operator -() const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), neg_ps(load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(-x, -y, -z);
#endif
}

float3 float3::operator *(float scalar) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), mul_ps(load_vec3(ptr(), 0.f), set1_ps(scalar)));
	return copy;
#else
	return float3(x * scalar, y * scalar, z * scalar);
#endif
}

float3 operator *(float scalar, const float3 &rhs)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), mul_ps(load_vec3(rhs.ptr(), 0.f), set1_ps(scalar)));
	return copy;
#else
	return float3(scalar * rhs.x, scalar * rhs.y, scalar * rhs.z);
#endif
}

float3 float3::operator /(float scalar) const
{
	float invScalar = 1.f / scalar;
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), mul_ps(load_vec3(ptr(), 0.f), set1_ps(invScalar)));
	return copy;
#else
	return float3(x * invScalar, y * invScalar, z * invScalar);
#endif
}

float3 &float3::operator =(const float3 &rhs)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), load_vec3(rhs.ptr(), 0.f));
#else
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
#endif
	return *this;
}

float3 &float3::operator +=(const float3 &rhs)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), add_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
#else
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
#endif
	return *this;
}

float3 &float3::operator -=(const float3 &rhs)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), sub_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
#else
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
#endif

	return *this;
}

float3 &float3::operator *=(float scalar)
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), mul_ps(load_vec3(ptr(), 0.f), set1_ps(scalar)));
#else
	x *= scalar;
	y *= scalar;
	z *= scalar;
#endif

	return *this;
}

float3 float3::Add(float scalar) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), add_ps(load_vec3(ptr(), 0.f), set1_ps(scalar)));
	return copy;
#else
	return float3(x + scalar, y + scalar, z + scalar);
#endif
}

float3 float3::Sub(float scalar) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), sub_ps(load_vec3(ptr(), 0.f), set1_ps(scalar)));
	return copy;
#else
	return float3(x - scalar, y - scalar, z - scalar);
#endif
}

float3 float3::SubLeft(float scalar) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), sub_ps(set1_ps(scalar), load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(scalar - x, scalar - y, scalar - z);
#endif
}

float3 float3::Mul(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), mul_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
	return copy;
#else
	return float3(x * rhs.x, y * rhs.y, z * rhs.z);
#endif
}

float3 float3::Div(const float3 &rhs) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), div_ps(load_vec3(ptr(), 0.f), load_vec3(rhs.ptr(), 0.f)));
	return copy;
#else
	return float3(x / rhs.x, y / rhs.y, z / rhs.z);
#endif
}

float3 float3::DivLeft(float scalar) const
{
#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	float3 copy;
	store_vec3(copy.ptr(), div_ps(set1_ps(scalar), load_vec3(ptr(), 0.f)));
	return copy;
#else
	return float3(scalar / x, scalar / y, scalar / z);
#endif
}

float3 &float3::operator /=(float scalar)
{
	float invScalar = 1.f / scalar;

#ifdef MATH_AUTOMATIC_SIMD_FLOAT3
	store_vec3(ptr(), mul_ps(load_vec3(ptr(), 0.f), set1_ps(invScalar)));
#else
	x *= invScalar;
	y *= invScalar;
	z *= invScalar;
#endif

	return *this;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3 &rhs)
{
	std::string str = rhs.ToString();
	out << str;
	return out;
}
#endif

const float3 float3::zero = float3(0, 0, 0);
const float3 float3::one = float3(1, 1, 1);
const float3 float3::unitX = float3(1, 0, 0);
const float3 float3::unitY = float3(0, 1, 0);
const float3 float3::unitZ = float3(0, 0, 1);
const float3 float3::nan = float3(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);
const float3 float3::inf = float3(FLOAT_INF, FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
