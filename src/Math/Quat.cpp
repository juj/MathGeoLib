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

#ifdef MATH_AUTOMATIC_SSE
#include "sse_mathfun.h"
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Quat::Quat(const float *data)
{
	assume(data);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!data)
		return;
#endif
#if defined(MATH_AUTOMATIC_SSE)
	q = loadu_ps(data);
#else
	x = data[0];
	y = data[1];
	z = data[2];
	w = data[3];
#endif
}

Quat::Quat(float x_, float y_, float z_, float w_)
#if !defined(MATH_AUTOMATIC_SSE)
:x(x_), y(y_), z(z_), w(w_)
#endif
{
#if defined(MATH_AUTOMATIC_SSE)
	q = set_ps(w_, z_, y_, x_);
#endif
}

vec Quat::WorldX() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return FLOAT4_TO_DIR(quat_transform_vec4(q, float4::unitX));
#else
	return DIR_VEC(this->Transform(1.f, 0.f, 0.f));
#endif
}

vec Quat::WorldY() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return FLOAT4_TO_DIR(quat_transform_vec4(q, float4::unitY));
#else
	return DIR_VEC(this->Transform(0.f, 1.f, 0.f));
#endif
}

vec Quat::WorldZ() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return FLOAT4_TO_DIR(quat_transform_vec4(q, float4::unitZ));
#else
	return DIR_VEC(this->Transform(0.f, 0.f, 1.f));
#endif
}

vec Quat::Axis() const
{
	assume2(this->IsNormalized(), *this, this->Length());
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 6.145 nsecs / 16.88 ticks, Avg: 6.367 nsecs, Worst: 6.529 nsecs
	assume2(this->IsNormalized(), *this, this->Length());
	simd4f cosAngle = wwww_ps(q);
	simd4f rcpSinAngle = rsqrt_ps(sub_ps(set1_ps(1.f), mul_ps(cosAngle, cosAngle)));
	simd4f a = mul_ps(q, rcpSinAngle);

	// Set the w component to zero.
	simd4f highPart = _mm_unpackhi_ps(a, zero_ps()); // [_ _ 0 z]
	a = _mm_movelh_ps(a, highPart); // [0 z y x]
	return FLOAT4_TO_DIR(a);
#else
	// Best: 6.529 nsecs / 18.152 ticks, Avg: 6.851 nsecs, Worst: 8.065 nsecs

	// Convert cos to sin via the identity sin^2 + cos^2 = 1, and fuse reciprocal and square root to the same instruction,
	// since we are about to divide by it.
	float rcpSinAngle = RSqrt(1.f - w*w);
	return DIR_VEC(x, y, z) * rcpSinAngle;
#endif
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
#ifdef MATH_AUTOMATIC_SSE
	simd4f lenSq = vec4_length_sq_ps(q);
	simd4f len = rsqrt_ps(lenSq);
	simd4f isZero = cmplt_ps(lenSq, simd4fEpsilon); // Was the length zero?
	simd4f normalized = mul_ps(q, len); // Normalize.
	q = cmov_ps(normalized, float4::unitX.v, isZero); // If length == 0, output the vector (1,0,0,0).
	len = cmov_ps(len, zero_ps(), isZero); // If length == 0, output zero as length.
	return s4f_x(len);
#else
	float length = Length();
	if (length < 1e-4f)
		return 0.f;
	float rcpLength = 1.f / length;
	x *= rcpLength;
	y *= rcpLength;
	z *= rcpLength;
	w *= rcpLength;
	return length;
#endif
}

Quat Quat::Normalized() const
{
#ifdef MATH_AUTOMATIC_SSE
	return Quat(vec4_normalize(q));
#else
	Quat copy = *this;
	float success = copy.Normalize();
	assume(success > 0 && "Quat::Normalized failed!");
	MARK_UNUSED(success);
	return copy;
#endif
}

bool Quat::IsNormalized(float epsilonSq) const
{
	return EqualAbs(LengthSq(), 1.f, epsilonSq);
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

bool Quat::BitEquals(const Quat &other) const
{
	return ReinterpretAsU32(x) == ReinterpretAsU32(other.x) &&
		ReinterpretAsU32(y) == ReinterpretAsU32(other.y) &&
		ReinterpretAsU32(z) == ReinterpretAsU32(other.z) &&
		ReinterpretAsU32(w) == ReinterpretAsU32(other.w);
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
	q = neg3_ps(q);
#else
	x = -x;
	y = -y;
	z = -z;
#endif
}

Quat MUST_USE_RESULT Quat::Conjugated() const
{
#ifdef MATH_AUTOMATIC_SSE
	return neg3_ps(q);
#else
	return Quat(-x, -y, -z, w);
#endif
}

float3 MUST_USE_RESULT Quat::Transform(const float3 &vec) const
{
	assume2(this->IsNormalized(), *this, this->LengthSq());
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return float4(quat_transform_vec4(q, load_vec3(vec.ptr(), 0.f))).xyz();
#else
	///\todo Optimize/benchmark the scalar path not to generate a matrix!
	float3x3 mat = this->ToFloat3x3();
	return mat * vec;
#endif
}

float3 MUST_USE_RESULT Quat::Transform(float X, float Y, float Z) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return float4(quat_transform_vec4(q, set_ps(0.f, Z, Y, X))).xyz();
#else
	return Transform(float3(X, Y, Z));
#endif
}

float4 MUST_USE_RESULT Quat::Transform(const float4 &vec) const
{
	assume(vec.IsWZeroOrOne());

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return quat_transform_vec4(q, vec);
#else
	return float4(Transform(vec.x, vec.y, vec.z), vec.w);
#endif
}

Quat MUST_USE_RESULT Quat::Lerp(const Quat &b, float t) const
{
	assume(0.f <= t && t <= 1.f);

	// TODO: SSE
	float angle = this->Dot(b);
	if (angle >= 0.f) // Make sure we rotate the shorter arc.
		return (*this * (1.f - t) + b * t).Normalized();
	else
		return (*this * (t - 1.f) + b * t).Normalized();
}

Quat MUST_USE_RESULT Quat::Slerp(const Quat &q2, float t) const
{
	assume(0.f <= t && t <= 1.f);
	assume(IsNormalized());
	assume(q2.IsNormalized());

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	simd4f angle = dot4_ps(q, q2.q); // <q, q2.q>
	simd4f neg = cmplt_ps(angle, zero_ps()); // angle < 0?
	neg = and_ps(neg, set1_ps_hex(0x80000000)); // Convert 0/0xFFFFFFFF mask to a 0x/0x80000000 mask.
//	neg = s4i_to_s4f(_mm_slli_epi32(s4f_to_s4i(neg), 31)); // A SSE2-esque way to achieve the above would be this, but this seems to clock slower (12.04 clocks vs 11.97 clocks)
	angle = xor_ps(angle, neg); // if angle was negative, make it positive.
	simd4f one = set1_ps(1.f);
	angle = min_ps(angle, one); // If user passed t > 1 or t < -1, clamp the range.

	// Compute a fast polynomial approximation to arccos(angle).
	// arccos(x): (-0.69813170079773212f * x * x - 0.87266462599716477f) * x + 1.5707963267948966f;
	angle = madd_ps(msub_ps(mul_ps(set1_ps(-0.69813170079773212f), angle), angle, set1_ps(0.87266462599716477f)), angle, set1_ps(1.5707963267948966f));

	// Shuffle an appropriate vector from 't' and 'angle' for computing two sines in one go.
	simd4f T = _mm_set_ss(t); // (.., t)
	simd4f oneSubT = sub_ps(one, T); // (.., 1-t)
	T = _mm_movelh_ps(T, oneSubT); // (.., 1-t, .., t)
	angle = mul_ps(angle, T); // (.., (1-t)*angle, .., t*angle)

	// Compute a fast polynomial approximation to sin(t*angle) and sin((1-t)*angle).
	// Here could use "angle = sin_ps(angle);" for precision, but favor speed instead with the following polynomial expansion:
	// sin(x): ((5.64311797634681035370e-03 * x * x - 1.55271410633428644799e-01) * x * x + 9.87862135574673806965e-01) * x
	simd4f angle2 = mul_ps(angle, angle);
	angle = mul_ps(angle, madd_ps(madd_ps(angle2, set1_ps(5.64311797634681035370e-03f), set1_ps(-1.55271410633428644799e-01f)), angle2, set1_ps(9.87862135574673806965e-01f)));

	// Compute the final lerp factors a and b to scale q and q2.
	simd4f a = zzzz_ps(angle);
	simd4f b = xxxx_ps(angle);
	a = xor_ps(a, neg);
	a = mul_ps(q, a);
	a = madd_ps(q2, b, a);

	// The lerp above generates an unnormalized quaternion which needs to be renormalized.
	return mul_ps(a, rsqrt_ps(dot4_ps(a, a)));
#else
	float angle = this->Dot(q2);
	float sign = 1.f; // Multiply by a sign of +/-1 to guarantee we rotate the shorter arc.
	if (angle < 0.f)
	{
		angle = -angle;
		sign = -1.f;
	}

	float a;
	float b;
	if (angle < 0.999) // perform spherical linear interpolation.
	{
		// angle = Acos(angle); // After this, angle is in the range pi/2 -> 0 as the original angle variable ranged from 0 -> 1.
		angle = (-0.69813170079773212f * angle * angle - 0.87266462599716477f) * angle + 1.5707963267948966f;

		float ta = t*angle;
#ifdef MATH_USE_SINCOS_LOOKUPTABLE
		// If Sin() is based on a lookup table, prefer that over polynomial approximation.
		a = Sin(angle - ta);
		b = Sin(ta);
#else
		// Not using a lookup table, manually compute the two sines by using a very rough approximation.
		float ta2 = ta*ta;
		b = ((5.64311797634681035370e-03f * ta2 - 1.55271410633428644799e-01f) * ta2 + 9.87862135574673806965e-01f) * ta;
		a = angle - ta;
		float a2 = a*a;
		a = ((5.64311797634681035370e-03f * a2 - 1.55271410633428644799e-01f) * a2 + 9.87862135574673806965e-01f) * a;
#endif
	}
	else // If angle is close to taking the denominator to zero, resort to linear interpolation (and normalization).
	{
		a = 1.f - t;
		b = t;
	}
	// Lerp and renormalize.
	return (*this * (a * sign) + q2 * b).Normalized();
#endif
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
	Quat delta = target / *this;
	delta.Normalize();
	return delta.Angle();
}

vec MUST_USE_RESULT Quat::AxisFromTo(const Quat &target) const
{
	assume(this->IsInvertible());
	Quat delta = target / *this;
	delta.Normalize();
	return delta.Axis();
}

void Quat::ToAxisAngle(float3 &axis, float &angle) const
{
	// Best: 36.868 nsecs / 98.752 ticks, Avg: 37.095 nsecs, Worst: 37.636 nsecs
	assume2(this->IsNormalized(), *this, this->Length());
	float halfAngle = Acos(w);
	angle = halfAngle * 2.f;
	// Convert cos to sin via the identity sin^2 + cos^2 = 1, and fuse reciprocal and square root to the same instruction,
	// since we are about to divide by it.
	float rcpSinAngle = RSqrt(1.f - w*w);
	axis.x = x * rcpSinAngle;
	axis.y = y * rcpSinAngle;
	axis.z = z * rcpSinAngle;
}

void Quat::ToAxisAngle(float4 &axis, float &angle) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 35.332 nsecs / 94.328 ticks, Avg: 35.870 nsecs, Worst: 57.607 nsecs
	assume2(this->IsNormalized(), *this, this->Length());
	simd4f cosAngle = wwww_ps(q);
	simd4f rcpSinAngle = rsqrt_ps(sub_ps(set1_ps(1.f), mul_ps(cosAngle, cosAngle)));
	angle = Acos(s4f_x(cosAngle)) * 2.f;
	simd4f a = mul_ps(q, rcpSinAngle);

	// Set the w component to zero.
	simd4f highPart = _mm_unpackhi_ps(a, zero_ps()); // [_ _ 0 z]
	axis.v = _mm_movelh_ps(a, highPart); // [0 z y x]
#else
	// Best: 85.258 nsecs / 227.656 ticks, Avg: 85.492 nsecs, Worst: 86.410 nsecs
	ToAxisAngle(reinterpret_cast<float3&>(axis), angle);
	axis.w = 0.f;
#endif
}

void Quat::SetFromAxisAngle(const float3 &axis, float angle)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE2)
	SetFromAxisAngle(load_vec3(axis.ptr(), 0.f), angle);
#else
	assume1(axis.IsNormalized(), axis);
	assume1(MATH_NS::IsFinite(angle), angle);
	float sinz, cosz;
	SinCos(angle*0.5f, sinz, cosz);
	x = axis.x * sinz;
	y = axis.y * sinz;
	z = axis.z * sinz;
	w = cosz;
#endif
}

void Quat::SetFromAxisAngle(const float4 &axis, float angle)
{
	assume1(EqualAbs(axis.w, 0.f), axis);
	assume2(axis.IsNormalized(1e-4f), axis, axis.Length4());
	assume1(MATH_NS::IsFinite(angle), angle);

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE2)
	// Best: 26.499 nsecs / 71.024 ticks, Avg: 26.856 nsecs, Worst: 27.651 nsecs
	simd4f halfAngle = set1_ps(0.5f*angle);
	simd4f sinAngle, cosAngle;
	sincos_ps(halfAngle, &sinAngle, &cosAngle);
	simd4f quat = mul_ps(axis, sinAngle);

	// Set the w component to cosAngle.
	simd4f highPart = _mm_unpackhi_ps(quat, cosAngle); // [_ _ 1 z]
	q = _mm_movelh_ps(quat, highPart); // [1 z y x]
#else
	// Best: 36.868 nsecs / 98.312 ticks, Avg: 36.980 nsecs, Worst: 41.477 nsecs
	SetFromAxisAngle(axis.xyz(), angle);
#endif
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
#ifdef MATH_AUTOMATIC_SSE
	q = set_ps(w_, z_, y_, x_);
#else
	x = x_;
	y = y_;
	z = z_;
	w = w_;
#endif
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
	// If sourceDirection == targetDirection, the cross product comes out zero, and normalization would fail. In that case, pick an arbitrary axis.
	float3 axis = sourceDirection.Cross(targetDirection);
	float oldLength = axis.Normalize();
	if (oldLength != 0.f)
	{
		float halfCosAngle = 0.5f*sourceDirection.Dot(targetDirection);
		float cosHalfAngle = Sqrt(0.5f + halfCosAngle);
		float sinHalfAngle = Sqrt(0.5f - halfCosAngle);
		return Quat(axis.x * sinHalfAngle, axis.y * sinHalfAngle, axis.z * sinHalfAngle, cosHalfAngle);
	}
	else
		return Quat(1.f, 0.f, 0.f, 0.f);

}

Quat MUST_USE_RESULT Quat::RotateFromTo(const float4 &sourceDirection, const float4 &targetDirection)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 12.289 nsecs / 33.144 ticks, Avg: 12.489 nsecs, Worst: 14.210 nsecs
	simd4f cosAngle = dot4_ps(sourceDirection.v, targetDirection.v);
	cosAngle = neg3_ps(cosAngle); // [+ - - -]
	// XYZ channels use the trigonometric formula sin(x/2) = +/-sqrt(0.5-0.5*cosx))
	// The W channel uses the trigonometric formula cos(x/2) = +/-sqrt(0.5+0.5*cosx))
	simd4f half = set1_ps(0.5f);
	simd4f cosSinHalfAngle = sqrt_ps(madd_ps(half, cosAngle, half)); // [cos(x/2), sin(x/2), sin(x/2), sin(x/2)]
	simd4f axis = cross_ps(sourceDirection.v, targetDirection.v);
	simd4f recipLen = rsqrt_ps(dot4_ps(axis, axis));
	axis = mul_ps(axis, recipLen); // [0 z y x]
	// Set the w component to one.
	simd4f one = add_ps(half, half); // [1 1 1 1]
	simd4f highPart = _mm_unpackhi_ps(axis, one); // [_ _ 1 z]
	axis = _mm_movelh_ps(axis, highPart); // [1 z y x]
	Quat q;
	q.q = mul_ps(axis, cosSinHalfAngle);
	return q;
#else
	// Best: 19.970 nsecs / 53.632 ticks, Avg: 20.197 nsecs, Worst: 21.122 nsecs
	assume(EqualAbs(sourceDirection.w, 0.f));
	assume(EqualAbs(targetDirection.w, 0.f));
	return Quat::RotateFromTo(sourceDirection.xyz(), targetDirection.xyz());
#endif
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

float4x4 MUST_USE_RESULT Quat::ToFloat4x4(const float3 &translation) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return ToFloat4x4(float4(translation, 1.f));
#else
	return float4x4(*this, translation);
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
	char str[256];
	char *s = SerializeFloat(x, str); *s = ','; ++s;
	s = SerializeFloat(y, s); *s = ','; ++s;
	s = SerializeFloat(z, s); *s = ','; ++s;
	s = SerializeFloat(w, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string Quat::SerializeToCodeString() const
{
	return "Quat(" + SerializeToString() + ")";
}
#endif

Quat MUST_USE_RESULT Quat::FromString(const char *str, const char **outEndStr)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return Quat::nan;
	MATH_SKIP_WORD(str, "Quat");
	MATH_SKIP_WORD(str, "(");
	Quat f;
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
	return neg_ps(q);
#else
	return Quat(-x, -y, -z, -w);
#endif
}

Quat Quat::operator *(float scalar) const
{
#ifdef MATH_AUTOMATIC_SSE
	return muls_ps(q, scalar);
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
	return div_ps(q, set1_ps(scalar));
#else
	return *this * (1.f / scalar);
#endif
}

Quat Quat::operator *(const Quat &r) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 3.456 nsecs / 9.752 ticks, Avg: 3.721 nsecs, Worst: 3.840 nsecs
	return quat_mul_quat(q, r.q);
#else
	// Best: 12.289 nsecs / 33.216 ticks, Avg: 12.585 nsecs, Worst: 13.442 nsecs
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
