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
#include "float4.h"
#include "float3x4.h"
#include "float4x4.h"

MATH_BEGIN_NAMESPACE

TranslateOp::TranslateOp(float tx, float ty, float tz)
:x(tx), y(ty), z(tz)
{
}

TranslateOp::TranslateOp(const float3 &trans)
:x(trans.x), y(trans.y), z(trans.z)
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
	m.SetRow(0, 1, 0, 0, x);
	m.SetRow(1, 0, 1, 0, y);
	m.SetRow(2, 0, 0, 1, z);
	return m;
}

float4x4 TranslateOp::ToFloat4x4() const
{
	float4x4 m;
	m.SetRow(0, 1, 0, 0, x);
	m.SetRow(1, 0, 1, 0, y);
	m.SetRow(2, 0, 0, 1, z);
	m.SetRow(3, 0, 0, 0, 1.f);
	return m;
}

float3 TranslateOp::Offset() const
{
	return float3(x, y, z);
}

float3x4 operator *(const TranslateOp &lhs, const float3x4 &rhs)
{
	float3x4 r = rhs;
	r.SetTranslatePart(r.TranslatePart() + lhs.Offset());

	// Our optimized form of multiplication must be the same as this.
	mathassert(r.Equals((float3x4)lhs * rhs));
	return r;
}

float3x4 operator *(const float3x4 &lhs, const TranslateOp &rhs)
{
	float3x4 r = lhs;
	r.SetTranslatePart(lhs.TransformPos(rhs.Offset()));

	// Our optimized form of multiplication must be the same as this.
	mathassert(r.Equals(lhs * (float3x4)rhs));
	return r;
}

float4x4 operator *(const TranslateOp &lhs, const float4x4 &rhs)
{
	float4x4 r = rhs;
	r.SetTranslatePart(r.TranslatePart() + lhs.Offset());

	// Our optimized form of multiplication must be the same as this.
	mathassert(r.Equals(lhs.ToFloat4x4() * rhs));
	return r;
}

float4x4 operator *(const float4x4 &lhs, const TranslateOp &rhs)
{
	float4x4 r = lhs;
	r.SetTranslatePart(lhs.TransformPos(rhs.Offset()));

	// Our optimized form of multiplication must be the same as this.
	mathassert(r.Equals(lhs * rhs.ToFloat4x4()));
	return r;
}

ScaleOp::ScaleOp(float sx, float sy, float sz)
:x(sx), y(sy), z(sz)
{
}

ScaleOp::ScaleOp(const float3 &scale)
:x(scale.x), y(scale.y), z(scale.z)
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
	m.SetRow(0, x, 0, 0);
	m.SetRow(1, 0, y, 0);
	m.SetRow(2, 0, 0, z);
	return m;
}

float3x4 ScaleOp::ToFloat3x4() const
{
	float3x4 m;
	m.SetRow(0, x, 0, 0, 0);
	m.SetRow(1, 0, y, 0, 0);
	m.SetRow(2, 0, 0, z, 0);
	return m;
}

float4x4 ScaleOp::ToFloat4x4() const
{
	float4x4 m;
	m.SetRow(0, x, 0, 0, 0);
	m.SetRow(1, 0, y, 0, 0);
	m.SetRow(2, 0, 0, z, 0);
	m.SetRow(3, 0, 0, 0, 1.f);
	return m;
}

float3x3 operator *(const ScaleOp &lhs, const float3x3 &rhs)
{
	float3x3 ret = rhs;
	ret.ScaleRow(0, lhs.x);
	ret.ScaleRow(1, lhs.y);
	ret.ScaleRow(2, lhs.z);

	// Our optimized form of multiplication must be the same as this.
	mathassert(ret.Equals((float3x3)lhs * rhs));
	return ret;
}

float3x3 operator *(const float3x3 &lhs, const ScaleOp &rhs)
{
	float3x3 ret = lhs;
	ret.ScaleCol(0, rhs.x);
	ret.ScaleCol(1, rhs.y);
	ret.ScaleCol(2, rhs.z);

	// Our optimized form of multiplication must be the same as this.
	mathassert(ret.Equals(lhs * (float3x3)rhs));
	return ret;
}

float3x4 operator *(const ScaleOp &lhs, const float3x4 &rhs)
{
	float3x4 ret;
	ret[0][0] = rhs[0][0] * lhs.x; ret[0][1] = rhs[0][1] * lhs.x; ret[0][2] = rhs[0][2] * lhs.x; ret[0][3] = rhs[0][3] * lhs.x;
	ret[1][0] = rhs[1][0] * lhs.y; ret[1][1] = rhs[1][1] * lhs.y; ret[1][2] = rhs[1][2] * lhs.y; ret[1][3] = rhs[1][3] * lhs.y;
	ret[2][0] = rhs[2][0] * lhs.z; ret[2][1] = rhs[2][1] * lhs.z; ret[2][2] = rhs[2][2] * lhs.z; ret[2][3] = rhs[2][3] * lhs.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

float3x4 operator *(const float3x4 &lhs, const ScaleOp &rhs)
{
	float3x4 ret;
	ret[0][0] = lhs[0][0] * rhs.x; ret[0][1] = lhs[0][1] * rhs.y; ret[0][2] = lhs[0][2] * rhs.z; ret[0][3] = lhs[0][3];
	ret[1][0] = lhs[1][0] * rhs.x; ret[1][1] = lhs[1][1] * rhs.y; ret[1][2] = lhs[1][2] * rhs.z; ret[1][3] = lhs[1][3];
	ret[2][0] = lhs[2][0] * rhs.x; ret[2][1] = lhs[2][1] * rhs.y; ret[2][2] = lhs[2][2] * rhs.z; ret[2][3] = lhs[2][3];

	mathassert(ret.Equals(lhs * rhs.ToFloat3x4()));
	return ret;
}

float4x4 operator *(const ScaleOp &lhs, const float4x4 &rhs)
{
	float4x4 ret;
	ret[0][0] = rhs[0][0] * lhs.x; ret[0][1] = rhs[0][1] * lhs.x; ret[0][2] = rhs[0][2] * lhs.x; ret[0][3] = rhs[0][3] * lhs.x;
	ret[1][0] = rhs[1][0] * lhs.y; ret[1][1] = rhs[1][1] * lhs.y; ret[1][2] = rhs[1][2] * lhs.y; ret[1][3] = rhs[1][3] * lhs.y;
	ret[2][0] = rhs[2][0] * lhs.z; ret[2][1] = rhs[2][1] * lhs.z; ret[2][2] = rhs[2][2] * lhs.z; ret[2][3] = rhs[2][3] * lhs.z;
	ret[3][0] = rhs[3][0];		 ret[3][1] = rhs[3][1];		 ret[3][2] = rhs[3][2];		 ret[3][3] = rhs[3][3];

	mathassert(ret.Equals(lhs.ToFloat4x4() * rhs));
	return ret;
}

float4x4 operator *(const float4x4 &lhs, const ScaleOp &rhs)
{
	float4x4 ret;
	ret[0][0] = lhs[0][0] * rhs.x; ret[0][1] = lhs[0][1] * rhs.y; ret[0][2] = lhs[0][2] * rhs.z; ret[0][3] = lhs[0][3];
	ret[1][0] = lhs[1][0] * rhs.x; ret[1][1] = lhs[1][1] * rhs.y; ret[1][2] = lhs[1][2] * rhs.z; ret[1][3] = lhs[1][3];
	ret[2][0] = lhs[2][0] * rhs.x; ret[2][1] = lhs[2][1] * rhs.y; ret[2][2] = lhs[2][2] * rhs.z; ret[2][3] = lhs[2][3];
	ret[3][0] = lhs[3][0] * rhs.x; ret[3][1] = lhs[3][1] * rhs.y; ret[3][2] = lhs[3][2] * rhs.z; ret[3][3] = lhs[3][3];

	mathassert(ret.Equals(lhs * rhs.ToFloat4x4()));
	return ret;
}

float3x4 operator *(const ScaleOp &lhs, const TranslateOp &rhs)
{
	float3x4 ret;
	ret[0][0] = lhs.x; ret[0][1] =	 0; ret[0][2] =	 0; ret[0][3] = lhs.x * rhs.x;
	ret[1][0] =	 0; ret[1][1] = lhs.y; ret[1][2] =	 0; ret[1][3] = lhs.y * rhs.y;
	ret[2][0] =	 0; ret[2][1] =	 0; ret[2][2] = lhs.z; ret[2][3] = lhs.z * rhs.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

float3x4 operator *(const TranslateOp &lhs, const ScaleOp &rhs)
{
	float3x4 ret;
	ret[0][0] = rhs.x; ret[0][1] =	 0; ret[0][2] =	 0; ret[0][3] = lhs.x;
	ret[1][0] =	 0; ret[1][1] = rhs.y; ret[1][2] =	 0; ret[1][3] = lhs.y;
	ret[2][0] =	 0; ret[2][1] =	 0; ret[2][2] = rhs.z; ret[2][3] = lhs.z;

	mathassert(ret.Equals(lhs.ToFloat3x4() * rhs));
	return ret;
}

float3 ScaleOp::Offset() const
{
	return float3(x, y, z);
}

MATH_END_NAMESPACE
