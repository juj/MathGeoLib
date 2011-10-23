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
#ifdef MATH_ENABLE_STL_SUPPORT
#include <cassert>
#include <utility>
#endif

#include <stdlib.h>

#include "Math/float2.h"
#include "Math/float3.h"
#include "Math/float4.h"
#include "Geometry/Sphere.h"
#include "Algorithm/Random/LCG.h"
#include "Math/float4x4.h"
#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

using namespace std;

float4::float4(float x_, float y_, float z_, float w_)
:x(x_), y(y_), z(z_), w(w_)
{
}

float4::float4(const float3 &xyz, float w_)
:x(xyz.x), y(xyz.y), z(xyz.z), w(w_)
{
}

float4::float4(const float2 &xy, float z_, float w_)
:x(xy.x), y(xy.y), z(z_), w(w_)
{
}

float4::float4(const float *data)
{
	assume(data);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!data)
		return;
#endif
	x = data[0];
	y = data[1];
	z = data[2];
	w = data[3];
}

float *float4::ptr()
{ 
	return &x;
} 

const float *float4::ptr() const
{ 
	return &x;
} 

CONST_WIN32 float float4::At(int index) const
{ 
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return FLOAT_NAN;
#endif
	return ptr()[index];
}

float &float4::At(int index)
{ 
	assume(index >= 0);
	assume(index < Size);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (index < 0 || index >= Size)
		return ptr()[0];
#endif
	return ptr()[index];
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
	return float4(At(i), At(j), At(k), At(l));
}

float float4::LengthSq3() const
{ 
	return x*x + y*y + z*z;
}

float float4::Length3() const
{ 
	return sqrtf(LengthSq3());
}

float float4::LengthSq4() const
{ 
	return x*x + y*y + z*z + w*w;
}

float float4::Length4() const
{ 
	return sqrtf(LengthSq4());
}

float float4::Normalize3()
{ 
	assume(IsFinite());
	float lengthSq = LengthSq3();
	if (lengthSq > 1e-6f)
	{
		float length = sqrtf(lengthSq);
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
}

float4 float4::Normalized3() const
{
	float4 copy = *this;
	float length = copy.Normalize3();
	assume(length > 0);
	return copy;
}

float float4::Normalize4()
{ 
	assume(IsFinite());
	float lengthSq = LengthSq4();
	if (lengthSq > 1e-6f)
	{
		float length = sqrtf(lengthSq);
		*this *= 1.f / length;
		return length;
	}
	else
	{
		Set(1.f, 0.f, 0.f, 0.f); // We will always produce a normalized vector.
		return 0; // But signal failure, so user knows we have generated an arbitrary normalization.
	}
}

float4 float4::Normalized4() const
{
	float4 copy = *this;
	float length = copy.Normalize4();
	assume(length > 0);
	return copy;
}

bool float4::NormalizeW()
{
	if (fabs(w) > 1e-6f)
	{
		float invW = 1.f / w;
		x *= invW;
		y *= invW;
		z *= invW;
		w = 1.f;
		return true;
	}
	else
		return false;
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
	return fabs(LengthSq4()-1.f) <= epsilonSq;
}

bool float4::IsNormalized3(float epsilonSq) const
{
	return fabs(LengthSq3()-1.f) <= epsilonSq;
}

void float4::Scale3(float scalar)
{
	x *= scalar;
	y *= scalar;
	z *= scalar;
}

float float4::ScaleToLength3(float newLength)
{
	float length = LengthSq3();
	if (length < 1e-6f)
		return 0.f;

	length = sqrtf(length);
	float scalar = newLength / length;
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return length;
}

float4 float4::ScaledToLength3(float newLength) const
{
	assume(!IsZero3());

	float4 v = *this;
	v.ScaleToLength3(newLength);
	return v;
}

bool float4::IsFinite() const
{
	return isfinite(x) && isfinite(y) && isfinite(z) && isfinite(w);
}

bool float4::IsPerpendicular3(const float4 &other, float epsilon) const
{
	return fabs(this->Dot3(other)) < epsilon;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string float4::ToString() const
{ 
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f, %.3f)", x, y, z, w);
	return std::string(str);
}

std::string float4::SerializeToString() const
{ 
	char str[256];
	sprintf(str, "%f %f %f %f", x, y, z, w);
	return std::string(str);
}
#endif

float4 float4::FromString(const char *str)
{
	assume(str);
	if (!str)
		return float4::nan;
	if (*str == '(')
		++str;
	float4 f;
	f.x = (float)strtod(str, const_cast<char**>(&str));
	if (*str == ',' || *str == ';')
		++str;
	f.y = (float)strtod(str, const_cast<char**>(&str));
	if (*str == ',' || *str == ';')
		++str;
	f.z = (float)strtod(str, const_cast<char**>(&str));
	if (*str == ',' || *str == ';')
		++str;
	f.w = (float)strtod(str, const_cast<char**>(&str));
	return f;
}

float float4::SumOfElements() const
{
	return x + y + z + w;
}

float float4::ProductOfElements() const
{
	return x * y * z * w;
}

float float4::AverageOfElements() const
{
	return (x + y + z + w) / 4.f;
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
	return float4(fabs(x), fabs(y), fabs(z), fabs(w));
}

float4 float4::Neg3() const
{
	return float4(-x, -y, -z, w);
}

float4 float4::Neg4() const
{
	return float4(-x, -y, -z, -w);
}

float4 float4::Recip3() const
{
	return float4(1.f/x, 1.f/y, 1.f/z, w);
}

float4 float4::Recip4() const
{
	return float4(1.f/x, 1.f/y, 1.f/z, 1.f/w);
}

float4 float4::Min(float ceil) const
{
	return float4(MATH_NS::Min(x, ceil), MATH_NS::Min(y, ceil), MATH_NS::Min(z, ceil), MATH_NS::Min(w, ceil));
}

float4 float4::Min(const float4 &ceil) const
{
	return float4(MATH_NS::Min(x, ceil.x), MATH_NS::Min(y, ceil.y), MATH_NS::Min(z, ceil.z), MATH_NS::Min(w, ceil.w));
}

float4 float4::Max(float floor) const
{
	return float4(MATH_NS::Max(x, floor), MATH_NS::Max(y, floor), MATH_NS::Max(z, floor), MATH_NS::Max(w, floor));
}

float4 float4::Max(const float4 &floor) const
{
	return float4(MATH_NS::Max(x, floor.x), MATH_NS::Max(y, floor.y), MATH_NS::Max(z, floor.z), MATH_NS::Max(w, floor.w));
}

float4 float4::Clamp(const float4 &floor, const float4 &ceil) const
{
	return float4(MATH_NS::Clamp(x, floor.x, ceil.x),
				  MATH_NS::Clamp(y, floor.y, ceil.y),
				  MATH_NS::Clamp(z, floor.z, ceil.z),
				  MATH_NS::Clamp(w, floor.w, ceil.w));
}

float4 float4::Clamp01() const
{
	return float4(MATH_NS::Clamp(x, 0.f, 1.f),
				  MATH_NS::Clamp(y, 0.f, 1.f),
				  MATH_NS::Clamp(z, 0.f, 1.f),
				  MATH_NS::Clamp(w, 0.f, 1.f));
}

float4 float4::Clamp(float floor, float ceil) const
{
	return float4(MATH_NS::Clamp(x, floor, ceil),
				  MATH_NS::Clamp(y, floor, ceil),
				  MATH_NS::Clamp(z, floor, ceil),
				  MATH_NS::Clamp(w, floor, ceil));
}

float float4::Distance3Sq(const float4 &rhs) const
{
	float dx = x - rhs.x;
	float dy = y - rhs.y;
	float dz = z - rhs.z;
	return dx*dx + dy*dy + dz*dz;
}

float float4::Distance3(const float4 &rhs) const
{
	return sqrtf(Distance3Sq(rhs));
}

float float4::Dot3(const float3 &rhs) const
{
	return x * rhs.x + y * rhs.y + z * rhs.z;
}

float float4::Dot3(const float4 &rhs) const
{
	return x * rhs.x + y * rhs.y + z * rhs.z;
}

float float4::Dot4(const float4 &rhs) const
{
	return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
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
float4 float4::Cross3(const float3 &rhs) const
{
	float4 dst;
	dst.x = y * rhs.z - z * rhs.y;
	dst.y = z * rhs.x - x * rhs.z;
	dst.z = x * rhs.y - y * rhs.x;
	dst.w = 0.f;
	return dst;
}

float4 float4::Cross3(const float4 &rhs) const
{
	return Cross3(rhs.xyz());
}

float4x4 float4::OuterProduct(const float4 &rhs) const
{
	const float4 &u = *this;
	const float4 &v = rhs;
	return float4x4(u[0]*v[0], u[0]*v[1], u[0]*v[2], u[0]*v[3],
					u[1]*v[0], u[1]*v[1], u[1]*v[2], u[1]*v[3],
					u[2]*v[0], u[2]*v[1], u[2]*v[2], u[2]*v[3],
					u[3]*v[0], u[3]*v[1], u[3]*v[2], u[3]*v[3]);
}

float4 float4::Perpendicular3(const float3 &hint, const float3 &hint2) const
{
	assume(!this->IsZero3());
	assume(EqualAbs(w, 0));
	assume(hint.IsNormalized());
	assume(hint2.IsNormalized());
	float3 v = this->Cross3(hint).xyz();
	float len = v.Normalize();
	if (len == 0)
		return float4(hint2, 0);
	else
		return float4(v, 0);
}

float4 float4::AnotherPerpendicular3(const float3 &hint, const float3 &hint2) const
{
	float4 firstPerpendicular = Perpendicular3(hint, hint2);
	float4 v = this->Cross3(firstPerpendicular);
	return v.Normalized3();
}

float4 float4::Reflect3(const float3 &normal) const
{
	assume(normal.IsNormalized());
	assume(EqualAbs(w, 0));
	return 2.f * this->ProjectToNorm3(normal) - *this;
}

float float4::AngleBetween3(const float4 &other) const
{
	return acos(Dot3(other)) / sqrt(LengthSq3() * other.LengthSq3());
}

float float4::AngleBetweenNorm3(const float4 &other) const
{
	assume(this->IsNormalized3());
	assume(other.IsNormalized3());
	return acos(Dot3(other));
}

float float4::AngleBetween4(const float4 &other) const
{
	return acos(Dot4(other)) / sqrt(LengthSq3() * other.LengthSq3());
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
	return float4(target * Dot(xyz(), target) / target.LengthSq(), w);
}

float4 float4::ProjectToNorm3(const float3 &target) const
{
	assume(target.IsNormalized());
	assume(this->IsWZeroOrOne());
	return float4(target * Dot(xyz(), target), w);
}

float4 float4::Lerp(const float4 &b, float t) const
{
	assume(EqualAbs(this->w, b.w));
	assume(0.f <= t && t <= 1.f);
	return (1.f - t) * *this + t * b;
}

float4 float4::Lerp(const float4 &a, const float4 &b, float t)
{
	return a.Lerp(b, t);
}

float4 float4::FromScalar(float scalar)
{ 
	return float4(scalar, scalar, scalar, scalar);
}

float4 float4::FromScalar(float scalar, float w)
{ 
	return float4(scalar, scalar, scalar, w);
}

void float4::SetFromScalar(float scalar)
{
	x = scalar;
	y = scalar;
	z = scalar;
	w = scalar;
}

void float4::Set(float x_, float y_, float z_, float w_)
{
	x = x_;
	y = y_;
	z = z_;
	w = w_;
}

void float4::SetFromScalar(float scalar, float w_)
{
	x = scalar;
	y = scalar;
	z = scalar;
	w = w_;
}

bool float4::Equals(const float4 &other, float epsilon) const
{
	return fabs(x - other.x) < epsilon &&
		   fabs(y - other.y) < epsilon &&
		   fabs(z - other.z) < epsilon &&
		   fabs(w - other.w) < epsilon;
}

bool float4::Equals(float x_, float y_, float z_, float w_, float epsilon) const
{
	return fabs(x - x_) < epsilon &&
		   fabs(y - y_) < epsilon &&
		   fabs(z - z_) < epsilon &&
		   fabs(w - w_) < epsilon;
}

float4 float4::RandomDir(LCG &lcg, float length)
{
	return float4(Sphere(float3(0,0,0), length).RandomPointOnSurface(lcg), 0.f);
}

float4 float4::operator +(const float4 &rhs) const
{
	return float4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

float4 float4::operator -(const float4 &rhs) const
{
	return float4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
}

float4 float4::operator -() const
{
	return float4(-x, -y, -z, -w);
}

float4 float4::operator *(float scalar) const
{
	return float4(x * scalar, y * scalar, z * scalar, w * scalar);
}

float4 operator *(float scalar, const float4 &rhs)
{
	return float4(scalar * rhs.x, scalar * rhs.y, scalar * rhs.z, scalar * rhs.w);
}

float4 float4::operator /(float scalar) const
{
	float invScalar = 1.f / scalar;
	return float4(x * invScalar, y * invScalar, z * invScalar, w * invScalar);
}

float4 &float4::operator +=(const float4 &rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	w += rhs.w;

	return *this;
}

float4 &float4::operator -=(const float4 &rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	w -= rhs.w;

	return *this;
}

float4 &float4::operator *=(float scalar)
{
	x *= scalar;
	y *= scalar;
	z *= scalar;
	w *= scalar;

	return *this;
}

float4 float4::Add(float s) const
{
	return float4(x + s, y + s, z + s, w + s);
}

float4 float4::Sub(float s) const
{
	return float4(x - s, y - s, z - s, w - s);
}

float4 float4::SubLeft(float s) const
{
	return float4(s - x, s - y, s - z, s - w);
}

float4 float4::DivLeft(float s) const
{
	return float4(s / x, s / y, s / z, s / w);
}

float4 float4::Mul(const float4 &rhs) const
{
	return float4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
}

float4 float4::Div(const float4 &rhs) const
{
	return float4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
}

float4 &float4::operator /=(float scalar)
{
	float invScalar = 1.f / scalar;
	x *= invScalar;
	y *= invScalar;
	z *= invScalar;
	w *= invScalar;

	return *this;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float4 &rhs)
{
	std::string str = rhs.ToString();
	out << str;
	return out;
}
#endif

const float4 float4::zero = float4(0, 0, 0, 0);
const float4 float4::one = float4(1, 1, 1, 1);
const float4 float4::unitX = float4(1, 0, 0, 0);
const float4 float4::unitY = float4(0, 1, 0, 0);
const float4 float4::unitZ = float4(0, 0, 1, 0);
const float4 float4::unitW = float4(0, 0, 0, 1);
const float4 float4::nan = float4(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);
const float4 float4::inf = float4(FLOAT_INF, FLOAT_INF, FLOAT_INF, FLOAT_INF);

MATH_END_NAMESPACE
