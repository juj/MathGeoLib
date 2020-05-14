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

/** @file TransformOps.cpp
	@author Jukka Jylänki
	@brief */
#include "TransformOps.h"
#include "MathFunc.h"
#include "float2.h"
#include "float4.h"
#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"

MATH_BEGIN_NAMESPACE

TranslateOp::TranslateOp(float tx, float ty, float tz)
:offset(DIR_VEC(tx, ty, tz))
{
}

TranslateOp::TranslateOp(const float3 &trans)
:offset(DIR_VEC(trans))
{
}

TranslateOp::operator float3x4() const
{
	return ToFloat3x4();
}

TranslateOp::operator float4x4() const
{
	return ToFloat4x4();
}


float3x4 TranslateOp::ToFloat3x4() const
{
	float3x4 m;
	m.SetRow(0, 1, 0, 0, offset.x);
	m.SetRow(1, 0, 1, 0, offset.y);
	m.SetRow(2, 0, 0, 1, offset.z);
	return m;
}

float4x4 TranslateOp::ToFloat4x4() const
{
	float4x4 m;
	m.SetRow(0, 1, 0, 0, offset.x);
	m.SetRow(1, 0, 1, 0, offset.y);
	m.SetRow(2, 0, 0, 1, offset.z);
	m.SetRow(3, 0, 0, 0, 1.f);
	return m;
}

vec TranslateOp::Offset() const
{
	return offset;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT TranslateOp::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f)", offset.x, offset.y, offset.z);
	return str;
}
#endif

float3x4 operator *(const TranslateOp &lhs, const float3x4 &rhs)
{
	float3x4 r = rhs;
	r.SetTranslatePart(r.TranslatePart() + DIR_TO_FLOAT3(lhs.Offset()));

	// Our optimized form of multiplication must be the same as this.
	mathassert(r.Equals((float3x4)lhs * rhs));
	return r;
}

float3x4 operator *(const float3x4 &lhs, const TranslateOp &rhs)
{
	float3x4 r = lhs;
	r.SetTranslatePart(lhs.TransformPos(DIR_TO_FLOAT3(rhs.Offset())));

	// Our optimized form of multiplication must be the same as this.
	assume4(r.Equals(lhs * (float3x4)rhs), lhs, rhs, r, lhs * (float3x4)rhs);
	return r;
}

float4x4 operator *(const TranslateOp &lhs, const float4x4 &rhs)
{
	// This function is based on the optimized assumption that the last row of rhs is [0,0,0,1].
	// If this does not hold and you are hitting the check below, explicitly cast TranslateOp lhs to float4x4 before multiplication!
	assume(rhs.Row(3).Equals(0.f, 0.f, 0.f, 1.f));

	float4x4 r = rhs;
	r.SetTranslatePart(r.TranslatePart() + DIR_TO_FLOAT3(lhs.Offset()));
	return r;
}

float4x4 operator *(const float4x4 &lhs, const TranslateOp &rhs)
{
	float4x4 r = lhs;
	r.SetTranslatePart(lhs.TransformPos(DIR_TO_FLOAT3(rhs.Offset())));
	return r;
}

ScaleOp::ScaleOp(const float2 &scaleXY, float scaleZ)
:scale(DIR_VEC(scaleXY.x, scaleXY.y, scaleZ))
{
}

ScaleOp::ScaleOp(float sx, float sy, float sz)
:scale(DIR_VEC(sx, sy, sz))
{
}

ScaleOp::ScaleOp(const float3 &scale)
:scale(DIR_VEC(scale))
{
}

ScaleOp::ScaleOp(const float4 &scale)
:scale(FLOAT4_TO_DIR(scale))
{
}

ScaleOp::operator float3x3() const
{
	return ToFloat3x3();
}

ScaleOp::operator float3x4() const
{
	return ToFloat3x4();
}

ScaleOp::operator float4x4() const
{
	return ToFloat4x4();
}

float3x3 ScaleOp::ToFloat3x3() const
{
	float3x3 m;
	m.SetRow(0, scale.x, 0, 0);
	m.SetRow(1, 0, scale.y, 0);
	m.SetRow(2, 0, 0, scale.z);
	return m;
}

float3x4 ScaleOp::ToFloat3x4() const
{
	float3x4 m;
	m.SetRow(0, scale.x, 0, 0, 0);
	m.SetRow(1, 0, scale.y, 0, 0);
	m.SetRow(2, 0, 0, scale.z, 0);
	return m;
}

float4x4 ScaleOp::ToFloat4x4() const
{
	float4x4 m;
	m.SetRow(0, scale.x, 0, 0, 0);
	m.SetRow(1, 0, scale.y, 0, 0);
	m.SetRow(2, 0, 0, scale.z, 0);
	m.SetRow(3, 0, 0, 0, 1.f);
	return m;
}

float3x3 operator *(const ScaleOp &lhs, const float3x3 &rhs)
{
	float3x3 ret = rhs;
	ret.ScaleRow(0, lhs.scale.x);
	ret.ScaleRow(1, lhs.scale.y);
	ret.ScaleRow(2, lhs.scale.z);

	// Our optimized form of multiplication must be the same as this.
	mathassert(ret.Equals((float3x3)lhs * rhs));
	return ret;
}

float3x3 operator *(const float3x3 &lhs, const ScaleOp &rhs)
{
	float3x3 ret = lhs;
	ret.ScaleCol(0, rhs.scale.x);
	ret.ScaleCol(1, rhs.scale.y);
	ret.ScaleCol(2, rhs.scale.z);

	// Our optimized form of multiplication must be the same as this.
	mathassert(ret.Equals(lhs * (float3x3)rhs));
	return ret;
}

float3x4 operator *(const ScaleOp &lhs, const float3x4 &rhs)
{
	float3x4 ret;
	ret[0][0] = rhs[0][0] * lhs.scale.x; ret[0][1] = rhs[0][1] * lhs.scale.x; ret[0][2] = rhs[0][2] * lhs.scale.x; ret[0][3] = rhs[0][3] * lhs.scale.x;
	ret[1][0] = rhs[1][0] * lhs.scale.y; ret[1][1] = rhs[1][1] * lhs.scale.y; ret[1][2] = rhs[1][2] * lhs.scale.y; ret[1][3] = rhs[1][3] * lhs.scale.y;
	ret[2][0] = rhs[2][0] * lhs.scale.z; ret[2][1] = rhs[2][1] * lhs.scale.z; ret[2][2] = rhs[2][2] * lhs.scale.z; ret[2][3] = rhs[2][3] * lhs.scale.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

float3x4 operator *(const float3x4 &lhs, const ScaleOp &rhs)
{
	float3x4 ret;
	ret[0][0] = lhs[0][0] * rhs.scale.x; ret[0][1] = lhs[0][1] * rhs.scale.y; ret[0][2] = lhs[0][2] * rhs.scale.z; ret[0][3] = lhs[0][3];
	ret[1][0] = lhs[1][0] * rhs.scale.x; ret[1][1] = lhs[1][1] * rhs.scale.y; ret[1][2] = lhs[1][2] * rhs.scale.z; ret[1][3] = lhs[1][3];
	ret[2][0] = lhs[2][0] * rhs.scale.x; ret[2][1] = lhs[2][1] * rhs.scale.y; ret[2][2] = lhs[2][2] * rhs.scale.z; ret[2][3] = lhs[2][3];

	mathassert(ret.Equals(lhs * rhs.ToFloat3x4()));
	return ret;
}

float4x4 operator *(const ScaleOp &lhs, const float4x4 &rhs)
{
	float4x4 ret;
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	simd4f x = xxxx_ps(lhs.scale.v);
	simd4f y = yyyy_ps(lhs.scale.v);
	simd4f z = zzzz_ps(lhs.scale.v);
	ret.row[0] = mul_ps(rhs.row[0], x);
	ret.row[1] = mul_ps(rhs.row[1], y);
	ret.row[2] = mul_ps(rhs.row[2], z);
	ret.row[3] = rhs.row[3];
#else
	ret[0][0] = rhs[0][0] * lhs.scale.x; ret[0][1] = rhs[0][1] * lhs.scale.x; ret[0][2] = rhs[0][2] * lhs.scale.x; ret[0][3] = rhs[0][3] * lhs.scale.x;
	ret[1][0] = rhs[1][0] * lhs.scale.y; ret[1][1] = rhs[1][1] * lhs.scale.y; ret[1][2] = rhs[1][2] * lhs.scale.y; ret[1][3] = rhs[1][3] * lhs.scale.y;
	ret[2][0] = rhs[2][0] * lhs.scale.z; ret[2][1] = rhs[2][1] * lhs.scale.z; ret[2][2] = rhs[2][2] * lhs.scale.z; ret[2][3] = rhs[2][3] * lhs.scale.z;
	ret[3][0] = rhs[3][0];         ret[3][1] = rhs[3][1];         ret[3][2] = rhs[3][2];         ret[3][3] = rhs[3][3];
#endif
	mathassert(ret.Equals(lhs.ToFloat4x4() * rhs));
	return ret;
}

float4x4 operator *(const float4x4 &lhs, const ScaleOp &rhs)
{
	float4x4 ret;
	ret[0][0] = lhs[0][0] * rhs.scale.x; ret[0][1] = lhs[0][1] * rhs.scale.y; ret[0][2] = lhs[0][2] * rhs.scale.z; ret[0][3] = lhs[0][3];
	ret[1][0] = lhs[1][0] * rhs.scale.x; ret[1][1] = lhs[1][1] * rhs.scale.y; ret[1][2] = lhs[1][2] * rhs.scale.z; ret[1][3] = lhs[1][3];
	ret[2][0] = lhs[2][0] * rhs.scale.x; ret[2][1] = lhs[2][1] * rhs.scale.y; ret[2][2] = lhs[2][2] * rhs.scale.z; ret[2][3] = lhs[2][3];
	ret[3][0] = lhs[3][0] * rhs.scale.x; ret[3][1] = lhs[3][1] * rhs.scale.y; ret[3][2] = lhs[3][2] * rhs.scale.z; ret[3][3] = lhs[3][3];

	mathassert4(ret.Equals(lhs * rhs.ToFloat4x4()), lhs, rhs.ToFloat4x4(), ret, lhs * rhs.ToFloat4x4());
	return ret;
}

float3x4 operator *(const ScaleOp &lhs, const TranslateOp &rhs)
{
	float3x4 ret;
	ret[0][0] = lhs.scale.x; ret[0][1] =	       0; ret[0][2] =       	0; ret[0][3] = lhs.scale.x * rhs.offset.x;
	ret[1][0] =	          0; ret[1][1] = lhs.scale.y; ret[1][2] =       	0; ret[1][3] = lhs.scale.y * rhs.offset.y;
	ret[2][0] =	          0; ret[2][1] =	       0; ret[2][2] = lhs.scale.z; ret[2][3] = lhs.scale.z * rhs.offset.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

float3x4 operator *(const TranslateOp &lhs, const ScaleOp &rhs)
{
	float3x4 ret;
	ret[0][0] = rhs.scale.x; ret[0][1] =	 0; ret[0][2] =	 0; ret[0][3] = lhs.offset.x;
	ret[1][0] =	 0; ret[1][1] = rhs.scale.y; ret[1][2] =	 0; ret[1][3] = lhs.offset.y;
	ret[2][0] =	 0; ret[2][1] =	 0; ret[2][2] = rhs.scale.z; ret[2][3] = lhs.offset.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

vec ScaleOp::Offset() const
{
	return scale;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT ScaleOp::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f)", scale.x, scale.y, scale.z);
	return str;
}
#endif

MATH_END_NAMESPACE
