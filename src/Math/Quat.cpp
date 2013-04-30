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

/** @file Quat.cpp
	@author Jukka Jylänki
	@brief */
#include "Quat.h"
#include <stdlib.h>
#include "float3.h"
#include "float4.h"
#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"
#include "../Algorithm/Random/LCG.h"
#include "assume.h"
#include "MathFunc.h"
#include "SSEMath.h"
#include "float4x4_sse.h"
#include "quat_simd.h"
#include "float4_neon.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Quat::Quat(const float *data)
:x(data[0]),
y(data[1]),
z(data[2]),
w(data[3])
{
}

Quat::Quat(const float3x3 &rotationMatrix)
{
	Set(rotationMatrix);
}

Quat::Quat(const float3x4 &rotationMatrix)
{
	Set(rotationMatrix);
}

Quat::Quat(const float4x4 &rotationMatrix)
{
	Set(rotationMatrix);
}

Quat::Quat(float x_, float y_, float z_, float w_)
:x(x_), y(y_), z(z_), w(w_)
{
}

Quat::Quat(const float3 &rotationAxis, float rotationAngle)
{
	SetFromAxisAngle(rotationAxis, rotationAngle);
}

float3 Quat::WorldX() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return float4(quat_transform_vec4(q, float4::unitX)).xyz();
#else
	return this->Transform(1.f, 0.f, 0.f);
#endif
}

float3 Quat::WorldY() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return float4(quat_transform_vec4(q, float4::unitY)).xyz();
#else
	return this->Transform(0.f, 1.f, 0.f);
#endif
}

float3 Quat::WorldZ() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return float4(quat_transform_vec4(q, float4::unitZ)).xyz();
#else
	return this->Transform(0.f, 0.f, 1.f);
#endif
}

float3 Quat::Axis() const
{
	float3 axis;
	float angle;
	ToAxisAngle(axis, angle);
	return axis;
}

float Quat::Angle() const
{
	return Acos(w) * 2.f;
}

float Quat::Dot(const Quat &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return dot4_float(q, rhs.q);
#else
	return x*rhs.x + y*rhs.y + z*rhs.z + w*rhs.w;
#endif
}

float Quat::LengthSq() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_length_sq_float(q);
#else
	return x*x + y*y + z*z + w*w;
#endif
}

float Quat::Length() const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_length_float(q);
#else
	return Sqrt(LengthSq());
#endif
}

float Quat::Normalize()
{
	///\todo SSE.
	float length = Length();
	if (length < 1e-4f)
		return 0.f;
	float rcpLength = 1.f / length;
	x *= rcpLength;
	y *= rcpLength;
	z *= rcpLength;
	w *= rcpLength;
	return length;
}

Quat Quat::Normalized() const
{
	Quat copy = *this;
	float success = copy.Normalize();
	assume(success > 0 && "Quat::Normalized failed!");
	MARK_UNUSED(success);
	return copy;
}

bool Quat::IsNormalized(float epsilon) const
{
	return EqualAbs(LengthSq(), 1.f, epsilon);
}

bool Quat::IsInvertible(float epsilon) const
{
	return LengthSq() > epsilon && IsFinite();
}

bool Quat::IsFinite() const
{
	return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y) && MATH_NS::IsFinite(z) && MATH_NS::IsFinite(w);
}

bool Quat::Equals(const Quat &rhs, float epsilon) const
{
	return EqualAbs(x, rhs.x, epsilon) && EqualAbs(y, rhs.y, epsilon) && EqualAbs(z, rhs.z, epsilon) && EqualAbs(w, rhs.w, epsilon);
}

float *Quat::ptr()
{
	return &x;
}

const float *Quat::ptr() const
{
	return &x;
}

void Quat::Inverse()
{
	assume(IsNormalized());
	assume(IsInvertible());
	Conjugate();
}

Quat MUST_USE_RESULT Quat::Inverted() const
{
	assume(IsNormalized());
	assume(IsInvertible());
	return Conjugated();
}

float Quat::InverseAndNormalize()
{
	Conjugate();
	return Normalize();
}

void Quat::Conjugate()
{
#ifdef MATH_AUTOMATIC_SSE
	q = negate3_ps(q);
#else
	x = -x;
	y = -y;
	z = -z;
#endif
}

Quat MUST_USE_RESULT Quat::Conjugated() const
{
#ifdef MATH_AUTOMATIC_SSE
	return negate3_ps(q);
#else
	return Quat(-x, -y, -z, w);
#endif
}

float3 MUST_USE_RESULT Quat::Transform(const float3 &vec) const
{
	assume(this->IsNormalized());
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	///\todo Check the generation of temporaries here!
	return float4(quat_transform_vec4(q, float4(vec,0.f).v)).xyz();
#else
	///\todo Optimize/benchmark the scalar path not to generate a matrix!
	float3x3 mat = this->ToFloat3x3();
	return mat * vec;
#endif
}

float3 MUST_USE_RESULT Quat::Transform(float x, float y, float z) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	///\todo Check the generation of temporaries here!
	return float4(quat_transform_vec4(q, float4(x,y,z,0.f).v)).xyz();
#else
	return Transform(float3(x, y, z));
#endif
}

float4 MUST_USE_RESULT Quat::Transform(const float4 &vec) const
{
	assume(vec.IsWZeroOrOne());

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return quat_transform_vec4(q, vec);
#else
	return float4(Transform(vec.x, vec.y, vec.z), vec.w);
#endif
}

Quat MUST_USE_RESULT Quat::Lerp(const Quat &b, float t) const
{
	assume(0.f <= t && t <= 1.f);

#ifdef MATH_AUTOMATIC_SSE
	return vec4_lerp(q, b.q, t);
#else
	return *this * (1.f - t) + b * t;
#endif
}

Quat MUST_USE_RESULT Quat::Lerp(const Quat &a, const Quat &b, float t)
{
	return a.Lerp(b, t);
}

/** Implementation based on the math in the book Watt, Policarpo. 3D Games: Real-time rendering and Software Technology, pp. 383-386. */
Quat MUST_USE_RESULT Quat::Slerp(const Quat &q2, float t) const
{
	///\todo SSE.
	assume(0.f <= t && t <= 1.f);
	assume(IsNormalized());
	assume(q2.IsNormalized());

	float angle = this->Dot(q2);
	float sign = 1.f; // Multiply by a sign of +/-1 to guarantee we rotate the shorter arc.
	if (angle < 0.f)
	{
		angle = -angle;
		sign = -1.f;
	}

	float a;
	float b;
	if (angle <= 0.97f) // perform spherical linear interpolation.
	{
		angle = Acos(angle); // After this, angle is in the range pi/2 -> 0 as the original angle variable ranged from 0 -> 1.

		float c = 1.f / Sin(angle);
		a = Sin((1.f - t) * angle) * c;
		b = Sin(angle * t) * c;
	}
	else // If angle is close to taking the denominator to zero, resort to linear interpolation (and normalization).
	{
		a = 1.f - t;
		b = t;
	}
	
	return (*this * (a * sign) + q2 * b).Normalized();
}

Quat MUST_USE_RESULT Quat::Slerp(const Quat &a, const Quat &b, float t)
{
	return a.Slerp(b, t);
}

Quat Lerp(const Quat &a, const Quat &b, float t)
{
	return a.Lerp(b, t);
}

Quat Slerp(const Quat &a, const Quat &b, float t)
{
	return a.Slerp(b, t);
}

float3 MUST_USE_RESULT Quat::SlerpVector(const float3 &from, const float3 &to, float t)
{
	if (t <= 0.f)
		return from;
	if (t >= 1.f)
		return to;
	///\todo The following chain can be greatly optimized.
	Quat q = Quat::RotateFromTo(from, to);
	q = Slerp(Quat::identity, q, t);
	return q.Transform(from);
}

float3 MUST_USE_RESULT Quat::SlerpVectorAbs(const float3 &from, const float3 &to, float angleRadians)
{
	if (angleRadians <= 0.f)
		return from;
	Quat q = Quat::RotateFromTo(from, to);
	float a = q.Angle();
	if (a <= angleRadians)
		return to;
	float t = angleRadians / a;
	q = Slerp(Quat::identity, q, t);
	return q.Transform(from);
}

float MUST_USE_RESULT Quat::AngleBetween(const Quat &target) const
{
	assume(this->IsInvertible());
	Quat q = target / *this;
	return q.Angle();
}

float3 MUST_USE_RESULT Quat::AxisFromTo(const Quat &target) const
{
	assume(this->IsInvertible());
	Quat q = target / *this;
	return q.Axis();
}

void Quat::ToAxisAngle(float3 &axis, float &angle) const
{
	angle = Acos(w) * 2.f;
	float sinz = Sin(angle/2.f);
	if (fabs(sinz) > 1e-4f)
	{
		sinz = 1.f / sinz;
		axis = float3(x * sinz, y * sinz, z * sinz);
	}
	else
	{
		// The quaternion does not produce any rotation. Still, explicitly
		// set the axis so that the user gets a valid normalized vector back.
		angle = 0.f;
		axis = float3(1.f, 0.f, 0.f);
	}
}

void Quat::SetFromAxisAngle(const float3 &axis, float angle)
{
	assume(axis.IsNormalized());
	assume(MATH_NS::IsFinite(angle));
	float cosz = Cos(angle/2.f);
	float sinz = Sin(angle/2.f);
	x = axis.x * sinz;
	y = axis.y * sinz;
	z = axis.z * sinz;
	w = cosz;
}

/// See Schneider, Eberly. Geometric Tools for Computer Graphics, p. 861.
template<typename M>
void SetQuatFrom(Quat &q, const M &m)
{
	// The rotation matrix is of form: (Eric Lengyel's Mathematics for 3D Game Programming and Computer Graphics 2nd ed., p. 92)
	// 1 - 2y^2 - 2z^2        2xy - 2wz            2xz + 2wy
	//    2xy + 2wz        1 - 2x^2 - 2z^2         2yz - 2wx
	//    2xz - 2wy           2yz + 2wx         1 - 2x^2 - 2y^2

	float r = m[0][0] + m[1][1] + m[2][2]; // The element w is easiest picked up as a sum of the diagonals.
	// Above, r == 3 - 4(x^2+y^2+z^2) == 4(1-x^2-y^2-z^2) - 1 == 4*w^2 - 1.
	if (r > 0) // In this case, |w| > 1/2.
	{
		q.w = Sqrt(r + 1.f) * 0.5f; // We have two choices for the sign of w, arbitrarily pick the positive.
		float inv4w = 1.f / (4.f * q.w);
		q.x = (m[2][1] - m[1][2]) * inv4w;
		q.y = (m[0][2] - m[2][0]) * inv4w;
		q.z = (m[1][0] - m[0][1]) * inv4w;
	}
	else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) // If |q.x| is larger than |q.y| and |q.z|, extract it first. This gives
	{                                                // best stability, and we know below x can't be zero.
		q.x = Sqrt(1.f + m[0][0] - m[1][1] - m[2][2]) * 0.5f; // We have two choices for the sign of x, arbitrarily pick the positive.
		const float x4 = 1.f / (4.f * q.x);
		q.y = (m[0][1] + m[1][0]) * x4;
		q.z = (m[0][2] + m[2][0]) * x4;
		q.w = (m[2][1] - m[1][2]) * x4;
	}
	else if (m[1][1] > m[2][2]) // |q.y| is larger than |q.x| and |q.z|
	{
		q.y = Sqrt(1.f + m[1][1] - m[0][0] - m[2][2]) * 0.5f; // We have two choices for the sign of y, arbitrarily pick the positive.
		const float y4 = 1.f / (4.f * q.y);
		q.x = (m[0][1] + m[1][0]) * y4;
		q.z = (m[1][2] + m[2][1]) * y4;
		q.w = (m[0][2] - m[2][0]) * y4;
	}
	else // |q.z| is larger than |q.x| or |q.y|
	{
		q.z = Sqrt(1.f + m[2][2] - m[0][0] - m[1][1]) * 0.5f; // We have two choices for the sign of z, arbitrarily pick the positive.
		const float z4 = 1.f / (4.f * q.z);
		q.x = (m[0][2] + m[2][0]) * z4;
		q.y = (m[1][2] + m[2][1]) * z4;
		q.w = (m[1][0] - m[0][1]) * z4;
	}
	float oldLength = q.Normalize();
	assume(oldLength > 0.f);
	MARK_UNUSED(oldLength);
}

void Quat::Set(const float3x3 &m)
{
	assume(m.IsColOrthogonal());
	assume(m.HasUnitaryScale());
	assume(!m.HasNegativeScale());
	SetQuatFrom(*this, m);

#ifdef MATH_ASSERT_CORRECTNESS
	// Test that the conversion float3x3->Quat->float3x3 is correct.
	mathassert(this->ToFloat3x3().Equals(m, 0.01f));
#endif
}

void Quat::Set(const float3x4 &m)
{
	assume(m.IsColOrthogonal());
	assume(m.HasUnitaryScale());
	assume(!m.HasNegativeScale());
	SetQuatFrom(*this, m);

#ifdef MATH_ASSERT_CORRECTNESS
	// Test that the conversion float3x3->Quat->float3x3 is correct.
	mathassert(this->ToFloat3x3().Equals(m.Float3x3Part(), 0.01f));
#endif
}

void Quat::Set(const float4x4 &m)
{
	assume(m.IsColOrthogonal3());
	assume(m.HasUnitaryScale());
	assume(!m.HasNegativeScale());
	assume(m.Row(3).Equals(0,0,0,1));
	SetQuatFrom(*this, m);

#ifdef MATH_ASSERT_CORRECTNESS
	// Test that the conversion float3x3->Quat->float3x3 is correct.
	mathassert(this->ToFloat3x3().Equals(m.Float3x3Part(), 0.01f));
#endif
}

void Quat::Set(float x_, float y_, float z_, float w_)
{
	x = x_;
	y = y_;
	z = z_;
	w = w_;
}

Quat MUST_USE_RESULT Quat::LookAt(const float3 &localForward, const float3 &targetDirection, const float3 &localUp, const float3 &worldUp)
{
	return float3x3::LookAt(localForward, targetDirection, localUp, worldUp).ToQuat();
}

Quat MUST_USE_RESULT Quat::RotateX(float angle)
{
	return Quat(float3(1,0,0), angle);
}

Quat MUST_USE_RESULT Quat::RotateY(float angle)
{
	return Quat(float3(0,1,0), angle);
}

Quat MUST_USE_RESULT Quat::RotateZ(float angle)
{
	return Quat(float3(0,0,1), angle);
}

Quat MUST_USE_RESULT Quat::RotateAxisAngle(const float3 &axis, float angle)
{
	return Quat(axis, angle);
}

Quat MUST_USE_RESULT Quat::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection)
{
	assume(sourceDirection.IsNormalized());
	assume(targetDirection.IsNormalized());
	float angle = sourceDirection.AngleBetweenNorm(targetDirection);
	assume(angle >= 0.f);
	// If sourceDirection == targetDirection, the cross product comes out zero, and normalization would fail. In that case, pick an arbitrary axis.
	float3 axis = sourceDirection.Cross(targetDirection);
	float oldLength = axis.Normalize();
	if (oldLength == 0)
		axis = float3(1, 0, 0);
	return Quat(axis, angle);
}

Quat MUST_USE_RESULT Quat::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection,
	const float3 &sourceDirection2, const float3 &targetDirection2)
{
	return LookAt(sourceDirection, targetDirection, sourceDirection2, targetDirection2);
}

Quat MUST_USE_RESULT Quat::FromEulerXYX(float x2, float y, float x) { return (Quat::RotateX(x2) * Quat::RotateY(y) * Quat::RotateX(x)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerXZX(float x2, float z, float x) { return (Quat::RotateX(x2) * Quat::RotateZ(z) * Quat::RotateX(x)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerYXY(float y2, float x, float y) { return (Quat::RotateY(y2) * Quat::RotateX(x) * Quat::RotateY(y)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerYZY(float y2, float z, float y) { return (Quat::RotateY(y2) * Quat::RotateZ(z) * Quat::RotateY(y)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerZXZ(float z2, float x, float z) { return (Quat::RotateZ(z2) * Quat::RotateX(x) * Quat::RotateZ(z)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerZYZ(float z2, float y, float z) { return (Quat::RotateZ(z2) * Quat::RotateY(y) * Quat::RotateZ(z)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerXYZ(float x, float y, float z) { return (Quat::RotateX(x) * Quat::RotateY(y) * Quat::RotateZ(z)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerXZY(float x, float z, float y) { return (Quat::RotateX(x) * Quat::RotateZ(z) * Quat::RotateY(y)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerYXZ(float y, float x, float z) { return (Quat::RotateY(y) * Quat::RotateX(x) * Quat::RotateZ(z)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerYZX(float y, float z, float x) { return (Quat::RotateY(y) * Quat::RotateZ(z) * Quat::RotateX(x)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerZXY(float z, float x, float y) { return (Quat::RotateZ(z) * Quat::RotateX(x) * Quat::RotateY(y)).Normalized(); }
Quat MUST_USE_RESULT Quat::FromEulerZYX(float z, float y, float x) { return (Quat::RotateZ(z) * Quat::RotateY(y) * Quat::RotateX(x)).Normalized(); }

Quat MUST_USE_RESULT Quat::RandomRotation(LCG &lcg)
{
	// Generate a random point on the 4D unitary hypersphere using the rejection sampling method.
	for(int i = 0; i < 1000; ++i)
	{
		float x = lcg.Float(-1, 1);
		float y = lcg.Float(-1, 1);
		float z = lcg.Float(-1, 1);
		float w = lcg.Float(-1, 1);
		float lenSq = x*x + y*y + z*z + w*w;
		if (lenSq >= 1e-6f && lenSq <= 1.f)
			return Quat(x, y, z, w) / Sqrt(lenSq);
	}
	assume(false && "Quat::RandomRotation failed!");
	return Quat::identity;
}

///@todo the following could be heavily optimized. Don't route through float3x3 conversion.

float3 MUST_USE_RESULT Quat::ToEulerXYX() const { return ToFloat3x3().ToEulerXYX(); }
float3 MUST_USE_RESULT Quat::ToEulerXZX() const { return ToFloat3x3().ToEulerXZX(); }
float3 MUST_USE_RESULT Quat::ToEulerYXY() const { return ToFloat3x3().ToEulerYXY(); }
float3 MUST_USE_RESULT Quat::ToEulerYZY() const { return ToFloat3x3().ToEulerYZY(); }
float3 MUST_USE_RESULT Quat::ToEulerZXZ() const { return ToFloat3x3().ToEulerZXZ(); }
float3 MUST_USE_RESULT Quat::ToEulerZYZ() const { return ToFloat3x3().ToEulerZYZ(); }
float3 MUST_USE_RESULT Quat::ToEulerXYZ() const { return ToFloat3x3().ToEulerXYZ(); }
float3 MUST_USE_RESULT Quat::ToEulerXZY() const { return ToFloat3x3().ToEulerXZY(); }
float3 MUST_USE_RESULT Quat::ToEulerYXZ() const { return ToFloat3x3().ToEulerYXZ(); }
float3 MUST_USE_RESULT Quat::ToEulerYZX() const { return ToFloat3x3().ToEulerYZX(); }
float3 MUST_USE_RESULT Quat::ToEulerZXY() const { return ToFloat3x3().ToEulerZXY(); }
float3 MUST_USE_RESULT Quat::ToEulerZYX() const { return ToFloat3x3().ToEulerZYX(); }

float3x3 MUST_USE_RESULT Quat::ToFloat3x3() const
{
	return float3x3(*this);
}

float3x4 MUST_USE_RESULT Quat::ToFloat3x4() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	float3x4 m;
	quat_to_mat3x4(q, _mm_set_ps(1,0,0,0), m.row);
	return m;
#else
	return float3x4(*this);
#endif
}

float4x4 MUST_USE_RESULT Quat::ToFloat4x4() const
{
	assume(IsNormalized());
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	float4x4 m;
	quat_to_mat4x4(q, _mm_set_ps(1,0,0,0), m.row);
	return m;
#else
	return float4x4(*this);
#endif
}

float4x4 MUST_USE_RESULT Quat::ToFloat4x4(const float4 &translation) const
{
	assume(IsNormalized());
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	float4x4 m;
	quat_to_mat4x4(q, translation.v, m.row);
	return m;
#else
	return float4x4(*this, translation.xyz());
#endif
}

bool IsNeutralCLocale();

#ifdef MATH_ENABLE_STL_SUPPORT
std::string MUST_USE_RESULT Quat::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f, %.3f)", x, y, z, w);
	return str;
}

std::string MUST_USE_RESULT Quat::ToString2() const
{
	float3 axis;
	float angle;
	ToAxisAngle(axis, angle);
	char str[256];
	sprintf(str, "Quat(axis:(%.2f,%.2f,%.2f) angle:%2.f)", axis.x, axis.y, axis.z, RadToDeg(angle));
	return str;
}

std::string MUST_USE_RESULT Quat::SerializeToString() const
{
	assert(IsNeutralCLocale());
	char str[256];
	sprintf(str, "%f %f %f %f", x, y, z, w);
	return std::string(str);
}
#endif

Quat MUST_USE_RESULT Quat::FromString(const char *str)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return Quat();
	if (*str == '(')
		++str;
	Quat q;
	q.x = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t') ///\todo Propagate this to other FromString functions.
		++str;
	if (*str == ',' || *str == ';')
		++str;
	q.y = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t')
		++str;
	if (*str == ',' || *str == ';')
		++str;
	q.z = (float)strtod(str, const_cast<char**>(&str));
	while(*str == ' ' || *str == '\t')
		++str;
	if (*str == ',' || *str == ';')
		++str;
	q.w = (float)strtod(str, const_cast<char**>(&str));
	return q;
}

Quat Quat::operator +(const Quat &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return add_ps(q, rhs.q);
#else
	return Quat(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
#endif
}

Quat Quat::operator -(const Quat &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	return sub_ps(q, rhs.q);
#else
	return Quat(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
#endif
}

Quat Quat::operator -() const
{
#ifdef MATH_AUTOMATIC_SSE
	return negate_ps(q);
#else
	return Quat(-x, -y, -z, -w);
#endif
}

Quat Quat::operator *(float scalar) const
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_mul_float(q, scalar);
#else
	return Quat(x * scalar, y * scalar, z * scalar, w * scalar);
#endif
}

float3 Quat::operator *(const float3 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return float4(quat_transform_vec4(q, float4(rhs, 0.f).v)).xyz();
#else
	return Transform(rhs);
#endif
}

float4 Quat::operator *(const float4 &rhs) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return quat_transform_vec4(q, rhs);
#else
	return Transform(rhs);
#endif
}

Quat Quat::operator /(float scalar) const
{
	assume(!EqualAbs(scalar, 0.f));

#ifdef MATH_AUTOMATIC_SSE
	return vec4_div_float(q, scalar);
#else
	return *this * (1.f / scalar);
#endif
}

Quat Quat::operator *(const Quat &r) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return quat_mul_quat(q, r.q);
#else
	return Quat(x*r.w + y*r.z - z*r.y + w*r.x,
	           -x*r.z + y*r.w + z*r.x + w*r.y,
	            x*r.y - y*r.x + z*r.w + w*r.z,
	           -x*r.x - y*r.y - z*r.z + w*r.w);
#endif
}

Quat Quat::operator /(const Quat &r) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return quat_div_quat(q, r.q);
#else
	return Quat(x*r.w - y*r.z + z*r.y - w*r.x,
	            x*r.z + y*r.w - z*r.x - w*r.y,
	           -x*r.y + y*r.x + z*r.w - w*r.z,
	            x*r.x + y*r.y + z*r.z + w*r.w);
#endif
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const Quat &rhs)
{
	out << rhs.ToString();
	return out;
}
#endif

Quat MUST_USE_RESULT Quat::Mul(const Quat &rhs) const { return *this * rhs; }
Quat MUST_USE_RESULT Quat::Mul(const float3x3 &rhs) const { return *this * Quat(rhs); }
float3 MUST_USE_RESULT Quat::Mul(const float3 &vector) const { return this->Transform(vector); }
float4 MUST_USE_RESULT Quat::Mul(const float4 &vector) const { return this->Transform(vector); }

const Quat Quat::identity = Quat(0.f, 0.f, 0.f, 1.f);
const Quat Quat::nan = Quat(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);

MATH_END_NAMESPACE
