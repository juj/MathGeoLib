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
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!data)
		return;
#endif
	x = data[0];
	y = data[1];
	z = data[2];
}

float *float3::ptr()
{
	return &x;
}

const float *float3::ptr() const
{
	return &x;
}

CONST_WIN32 float float3::At(int index) const
{
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return FLOAT_NAN;
#endif
	return ptr()[index];
}

float &float3::At(int index)
{
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return ptr()[0];
#endif
	return ptr()[index];
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
	return x*x + y*y + z*z;
}

float float3::Length() const
{
	return Sqrt(LengthSq());
}

float float3::Normalize()
{
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
}

float3 float3::Normalized() const
{
	float3 copy = *this;
	float oldLength = copy.Normalize();
	assume(oldLength > 0.f && "float3::Normalized() failed!");
	MARK_UNUSED(oldLength);
	return copy;
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
	return fabs(LengthSq()-1.f) <= epsilonSq;
}

bool float3::IsZero(float epsilonSq) const
{
	return fabs(LengthSq()) <= epsilonSq;
}

bool float3::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y) && MATH_NS::IsFinite(z);
}

bool float3::IsPerpendicular(const float3 &other, float epsilon) const
{
	return fabs(Dot(other)) <= epsilon * Length() * other.Length();
}

bool MUST_USE_RESULT float3::AreCollinear(const float3 &p1, const float3 &p2, const float3 &p3, float epsilon)
{
	return (p2-p1).Cross(p3-p1).LengthSq() <= epsilon;
}

bool IsNeutralCLocale();

#ifdef MATH_ENABLE_STL_SUPPORT
std::string float3::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f)", x, y, z);
	return std::string(str);
}

std::string float3::SerializeToString() const
{
	assert(IsNeutralCLocale());
	char str[256];
	sprintf(str, "%f %f %f", x, y, z);
	return std::string(str);
}
#endif

float3 MUST_USE_RESULT float3::FromString(const char *str)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return float3::nan;
	if (*str == '(')
		++str;
	float3 f;
	f.x = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t') ///\todo Propagate this to other FromString functions.
		++str;
	if (*str == ',' || *str == ';')
		++str;
	f.y = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t') ///\todo Propagate this to other FromString functions.
		++str;
	if (*str == ',' || *str == ';')
		++str;
	f.z = (float)strtod(str, const_cast<char**>(&str));
	return f;
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
	return float3(fabs(x), fabs(y), fabs(z));
}

float3 float3::Neg() const
{
	return float3(-x, -y, -z);
}

float3 float3::Recip() const
{
	return float3(1.f/x, 1.f/y, 1.f/z);
}

float3 float3::Min(float ceil) const
{
	return float3(MATH_NS::Min(x, ceil), MATH_NS::Min(y, ceil), MATH_NS::Min(z, ceil));
}

float3 float3::Min(const float3 &ceil) const
{
	return float3(MATH_NS::Min(x, ceil.x), MATH_NS::Min(y, ceil.y), MATH_NS::Min(z, ceil.z));
}

float3 float3::Max(float floor) const
{
	return float3(MATH_NS::Max(x, floor), MATH_NS::Max(y, floor), MATH_NS::Max(z, floor));
}

float3 float3::Max(const float3 &floor) const
{
	return float3(MATH_NS::Max(x, floor.x), MATH_NS::Max(y, floor.y), MATH_NS::Max(z, floor.z));
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
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	float dz = z - rhs.z;
	return dx*dx + dy*dy + dz*dz;
}

float float3::Distance(const float3 &rhs) const
{
	return Sqrt(DistanceSq(rhs));
}

float float3::Distance(const Line &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const Ray &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const LineSegment &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const Plane &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const Triangle &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const AABB &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const OBB &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const Sphere &rhs) const { return rhs.Distance(*this); }
float float3::Distance(const Capsule &rhs) const { return rhs.Distance(*this); }

float float3::Dot(const float3 &rhs) const
{
	return x * rhs.x + y * rhs.y + z * rhs.z;
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
	return float3(y * rhs.z - z * rhs.y,
	              z * rhs.x - x * rhs.z,
	              x * rhs.y - y * rhs.x);
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
	assume(normal.IsNormalized());
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
	x = scalar;
	y = scalar;
	z = scalar;
}

void float3::Set(float x_, float y_, float z_)
{
	x = x_;
	y = y_;
	z = z_;
}

void float3::SetFromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float cx = Cos(inclination);
	x = cx * Sin(azimuth) * radius;
	y = -Sin(inclination) * radius;
	z = cx * Cos(azimuth) * radius;
}

float3 MUST_USE_RESULT float3::FromSphericalCoordinates(float azimuth, float inclination, float radius)
{
	float3 v;
	v.SetFromSphericalCoordinates(azimuth, inclination, radius);
	return v;
}

void float3::SetFromSphericalCoordinates(float azimuth, float inclination)
{
	float cx = Cos(inclination);
	x = cx * Sin(azimuth);
	y = -Sin(inclination);
	z = cx * Cos(azimuth);
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
	return fabs(x - other.x) < epsilon &&
	       fabs(y - other.y) < epsilon &&
	       fabs(z - other.z) < epsilon;
}

bool float3::Equals(float x_, float y_, float z_, float epsilon) const
{
	return fabs(x - x_) < epsilon &&
	       fabs(y - y_) < epsilon &&
	       fabs(z - z_) < epsilon;
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
	return Sphere(float3(0,0,0), length).RandomPointOnSurface(lcg);
}

float3 MUST_USE_RESULT float3::RandomSphere(LCG &lcg, const float3 &center, float radius)
{
	return Sphere(center, radius).RandomPointInside(lcg);
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
	return AABB(minValues, maxValues).RandomPointInside(lcg);
}

float3 float3::operator +(const float3 &rhs) const
{
	return float3(x + rhs.x, y + rhs.y, z + rhs.z);
}

float3 float3::operator -(const float3 &rhs) const
{
	return float3(x - rhs.x, y - rhs.y, z - rhs.z);
}

float3 float3::operator -() const
{
	return float3(-x, -y, -z);
}

float3 float3::operator *(float scalar) const
{
	return float3(x * scalar, y * scalar, z * scalar);
}

float3 operator *(float scalar, const float3 &rhs)
{
	return float3(scalar * rhs.x, scalar * rhs.y, scalar * rhs.z);
}

float3 float3::operator /(float scalar) const
{
	float invScalar = 1.f / scalar;
	return float3(x * invScalar, y * invScalar, z * invScalar);
}

float3 operator /(float scalar, const float3 &rhs)
{
	return float3(scalar / rhs.x, scalar / rhs.y, scalar / rhs.z);
}

float3 &float3::operator +=(const float3 &rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;

	return *this;
}

float3 &float3::operator -=(const float3 &rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;

	return *this;
}

float3 &float3::operator *=(float scalar)
{
	x *= scalar;
	y *= scalar;
	z *= scalar;

	return *this;
}

float3 float3::Add(float scalar) const
{
	return float3(x + scalar, y + scalar, z + scalar);
}

float3 float3::Sub(float scalar) const
{
	return float3(x - scalar, y - scalar, z - scalar);
}

float3 float3::SubLeft(float scalar) const
{
	return float3(scalar - x, scalar - y, scalar - z);
}

float3 float3::Mul(const float3 &rhs) const
{
	return float3(x * rhs.x, y * rhs.y, z * rhs.z);
}

float3 float3::Div(const float3 &rhs) const
{
	return float3(x / rhs.x, y / rhs.y, z / rhs.z);
}

float3 float3::DivLeft(float scalar) const
{
	return float3(scalar / x, scalar / y, scalar / z);
}

float3 &float3::operator /=(float scalar)
{
	float invScalar = 1.f / scalar;
	x *= invScalar;
	y *= invScalar;
	z *= invScalar;

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
