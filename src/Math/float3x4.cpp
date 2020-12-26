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

/** @file float3x4.cpp
	@author Jukka Jylänki
	@brief */
#include "float3x4.h"
#include <string.h>

#include "MathFunc.h"
#include "Swap.h"
#include "float2.h"
#include "float3.h"
#include "float4.h"
#include "float3x3.h"
#include "float4x4.h"
#include "Matrix.inl"
#include "Quat.h"
#include "../Algorithm/Random/LCG.h"
#include "../Geometry/Plane.h"
#include "TransformOps.h"
#include "SSEMath.h"
#include "float4x4_sse.h"
#include "simd.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

float3x4::float3x4(float _00, float _01, float _02, float _03,
		 float _10, float _11, float _12, float _13,
		 float _20, float _21, float _22, float _23)
{
	Set(_00, _01, _02, _03,
		_10, _11, _12, _13,
		_20, _21, _22, _23);
}

float3x4::float3x4(const float3x3 &other)
{
	SetRotatePart(other);
	SetTranslatePart(0, 0, 0);
}

float3x4::float3x4(const float3x3 &other, const float3 &translate)
{
	SetRotatePart(other);
	SetTranslatePart(translate);
}

float3x4::float3x4(const float3 &col0, const float3 &col1, const float3 &col2, const float3 &col3)
{
	SetCol(0, col0);
	SetCol(1, col1);
	SetCol(2, col2);
	SetCol(3, col3);
}

float3x4::float3x4(const Quat &orientation)
{
	SetRotatePart(orientation);
	SetTranslatePart(0, 0, 0);
}

float3x4::float3x4(const Quat &orientation, const float3 &translation)
{
	SetRotatePart(orientation);
	SetTranslatePart(translation);
}

TranslateOp float3x4::Translate(float tx, float ty, float tz)
{
	return TranslateOp(tx, ty, tz);
}

TranslateOp float3x4::Translate(const float3 &offset)
{
	return TranslateOp(offset);
}

TranslateOp float3x4::Translate(const float4 &offset)
{
	return TranslateOp(offset.xyz());
}

float3x4 float3x4::RotateX(float angle)
{
	float3x4 r;
	r.SetRotatePartX(angle);
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::RotateX(float angleRadians, const float3 &pointOnAxis)
{
	return float3x4::Translate(pointOnAxis) * RotateX(angleRadians) * float3x4::Translate(-pointOnAxis);
}

float3x4 float3x4::RotateY(float angle)
{
	float3x4 r;
	r.SetRotatePartY(angle);
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::RotateY(float angleRadians, const float3 &pointOnAxis)
{
	return float3x4::Translate(pointOnAxis) * RotateY(angleRadians) * float3x4::Translate(-pointOnAxis);
}

float3x4 float3x4::RotateZ(float angle)
{
	float3x4 r;
	r.SetRotatePartZ(angle);
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::RotateZ(float angleRadians, const float3 &pointOnAxis)
{
	return float3x4::Translate(pointOnAxis) * RotateZ(angleRadians) * float3x4::Translate(-pointOnAxis);
}

float3x4 float3x4::RotateAxisAngle(const float3 &axisDirection, float angleRadians)
{
	float3x4 r;
	r.SetRotatePart(Quat::RotateAxisAngle(axisDirection, angleRadians));
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::RotateAxisAngle(const float3 &axisDirection, float angleRadians, const float3 &pointOnAxis)
{
	return float3x4::Translate(pointOnAxis) * RotateAxisAngle(axisDirection, angleRadians) * float3x4::Translate(-pointOnAxis);
}

float3x4 float3x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection)
{
	assume(sourceDirection.IsNormalized());
	assume(targetDirection.IsNormalized());
	float3x4 r;
	r.SetRotatePart(Quat::RotateFromTo(sourceDirection, targetDirection));
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection, const float3 &centerPoint)
{
	return float3x4::Translate(centerPoint) * RotateFromTo(sourceDirection, targetDirection) * float3x4::Translate(-centerPoint);
}

float3x4 float3x4::RandomGeneral(LCG &lcg, float minElem, float maxElem)
{
	float3x4 m;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			m[y][x] = lcg.Float(minElem, maxElem);
	return m;
}

float3x4 float3x4::RandomRotation(LCG &lcg)
{
	return float3x4(float3x3::RandomRotation(lcg));
}

float3x4 float3x4::FromQuat(const Quat &orientation)
{
	float3x4 r;
	r.SetRotatePart(orientation);
	r.SetTranslatePart(0, 0, 0);
	return r;
}

float3x4 float3x4::FromQuat(const Quat &orientation, const float3 &pointOnAxis)
{
	return float3x4::Translate(pointOnAxis) * float3x4::FromQuat(orientation) * float3x4::Translate(-pointOnAxis);
}

float3x4 float3x4::FromTRS(const float3 &translate, const Quat &rotate, const float3 &scale)
{
	return float3x4::Translate(translate) * float3x4(rotate) * float3x4::Scale(scale);
}

float3x4 float3x4::FromTRS(const float3 &translate, const float3x3 &rotate, const float3 &scale)
{
	return float3x4::Translate(translate) * float3x4(rotate) * float3x4::Scale(scale);
}

float3x4 float3x4::FromTRS(const float3 &translate, const float3x4 &rotate, const float3 &scale)
{
	return float3x4::Translate(translate) * float3x4(rotate) * float3x4::Scale(scale);
}

void float3x4::DecomposeToTRS(float3 &translate, float3x3 &rotate, float3 &scale) const
{
	translate = float3(At(0, 3), At(1, 3), At(2, 3));
	scale = float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
	rotate.SetCol(0, Col(0) / scale.x);
	rotate.SetCol(1, Col(1) / scale.y);
	rotate.SetCol(2, Col(2) / scale.z);
}

float3x4 float3x4::FromEulerXYX(float x2, float y, float x)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerXYX(r, x2, y, x);
	assume(r.Equals(float3x4::RotateX(x2) * float3x4::RotateY(y) * float3x4::RotateX(x)));
	return r;
}

float3x4 float3x4::FromEulerXZX(float x2, float z, float x)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerXZX(r, x2, z, x);
	assume(r.Equals(float3x4::RotateX(x2) * float3x4::RotateZ(z) * float3x4::RotateX(x)));
	return r;
}

float3x4 float3x4::FromEulerYXY(float y2, float x, float y)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerYXY(r, y2, x, y);
	assume(r.Equals(float3x4::RotateY(y2) * float3x4::RotateX(x) * float3x4::RotateY(y)));
	return r;
}

float3x4 float3x4::FromEulerYZY(float y2, float z, float y)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerYZY(r, y2, z, y);
	assume(r.Equals(float3x4::RotateY(y2) * float3x4::RotateZ(z) * float3x4::RotateY(y)));
	return r;
}

float3x4 float3x4::FromEulerZXZ(float z2, float x, float z)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerZXZ(r, z2, x, z);
	assume(r.Equals(float3x4::RotateZ(z2) * float3x4::RotateX(x) * float3x4::RotateZ(z)));
	return r;
}

float3x4 float3x4::FromEulerZYZ(float z2, float y, float z)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerZYZ(r, z2, y, z);
	assume(r.Equals(float3x4::RotateZ(z2) * float3x4::RotateY(y) * float3x4::RotateZ(z)));
	return r;
}

float3x4 float3x4::FromEulerXYZ(float x, float y, float z)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerXYZ(r, x, y, z);
	assume(r.Equals(float3x4::RotateX(x) * float3x4::RotateY(y) * float3x4::RotateZ(z)));
	return r;
}

float3x4 float3x4::FromEulerXZY(float x, float z, float y)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerXZY(r, x, z, y);
	assume(r.Equals(float3x4::RotateX(x) * float3x4::RotateZ(z) * float3x4::RotateY(y)));
	return r;
}

float3x4 float3x4::FromEulerYXZ(float y, float x, float z)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerYXZ(r, y, x, z);
	assume(r.Equals(float3x4::RotateY(y) * float3x4::RotateX(x) * float3x4::RotateZ(z)));
	return r;
}

float3x4 float3x4::FromEulerYZX(float y, float z, float x)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerYZX(r, y, z, x);
	assume(r.Equals(float3x4::RotateY(y) * float3x4::RotateZ(z) * float3x4::RotateX(x)));
	return r;
}

float3x4 float3x4::FromEulerZXY(float z, float x, float y)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerZXY(r, z, x, y);
	assume(r.Equals(float3x4::RotateZ(z) * float3x4::RotateX(x) * float3x4::RotateY(y)));
	return r;
}

float3x4 float3x4::FromEulerZYX(float z, float y, float x)
{
	float3x4 r;
	r.SetTranslatePart(0,0,0);
	Set3x3PartRotateEulerZYX(r, z, y, x);
	assume(r.Equals(float3x4::RotateZ(z) * float3x4::RotateY(y) * float3x4::RotateX(x)));
	return r;
}

ScaleOp float3x4::Scale(float sx, float sy, float sz)
{
	return ScaleOp(sx, sy, sz);
}

ScaleOp float3x4::Scale(const float3 &scale)
{
	return ScaleOp(scale);
}

ScaleOp float3x4::Scale(const float4 &scale)
{
	return ScaleOp(scale);
}

float3x4 float3x4::Scale(const float3 &scale, const float3 &scaleCenter)
{
	return float3x4::Translate(scaleCenter) * float3x4::Scale(scale) * float3x4::Translate(-scaleCenter);
}

float3x4 float3x4::Scale(const float4 &scale, const float4 &scaleCenter)
{
	return float3x4::Translate(scaleCenter) * float3x4::Scale(scale) * float3x4::Translate(-scaleCenter);
}

float3x4 float3x4::ScaleAlongAxis(const float3 &axis, float scalingFactor)
{
	return Scale(axis * scalingFactor);
}

float3x4 float3x4::ScaleAlongAxis(const float3 &axis, float scalingFactor, const float3 &scaleCenter)
{
	return float3x4::Translate(scaleCenter) * float3x4::Scale(axis * scalingFactor) * float3x4::Translate(-scaleCenter);
}

ScaleOp float3x4::UniformScale(float uniformScale)
{
	return ScaleOp(uniformScale, uniformScale, uniformScale);
}

float3x4 float3x4::UniformScale(float uniformScale, const float3 &scaleCenter)
{
	return float3x4::Translate(scaleCenter) * float3x4::UniformScale(uniformScale) * float3x4::Translate(-scaleCenter);
}

float3 float3x4::GetScale() const
{
	return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

float3x4 float3x4::ShearX(float yFactor, float zFactor)
{
	return float3x4(1.f, yFactor, zFactor, 0.f,
					0.f, 1.f, 0.f, 0.f,
					0.f, 0.f, 1.f, 0.f);
}

float3x4 float3x4::ShearY(float xFactor, float zFactor)
{
	return float3x4(1.f, 0.f, 0.f,  0.f,
					xFactor, 1.f, zFactor, 0.f,
					0.f, 0.f, 1.f, 0.f);
}

float3x4 float3x4::ShearZ(float xFactor, float yFactor)
{
	return float3x4(1.f, 0.f, 0.f, 0.f,
					0.f, 1.f, 0.f, 0.f,
					xFactor, yFactor, 1.f, 0.f);
}

float3x4 float3x4::Mirror(const Plane &p)
{
	float3x4 v;
	SetMatrix3x4AffinePlaneMirror(v, p.normal.x, p.normal.y, p.normal.z, p.d);
	return v;
}

float3x4 float3x4::OrthographicProjection(const Plane &p)
{
	float3x4 v;
	SetMatrix3x4AffinePlaneProject(v, p.normal.x, p.normal.y, p.normal.z, p.d);
	return v;
}

float3x4 float3x4::OrthographicProjectionYZ()
{
	float3x4 v = identity;
	v[0][0] = 0.f;
	return v;
}

float3x4 float3x4::OrthographicProjectionXZ()
{
	float3x4 v = identity;
	v[1][1] = 0.f;
	return v;
}

float3x4 float3x4::OrthographicProjectionXY()
{
	float3x4 v = identity;
	v[2][2] = 0.f;
	return v;
}

float &float3x4::At(int rowIndex, int colIndex)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(colIndex >= 0);
	assume(colIndex < Cols);
#ifdef MATH_COLMAJOR_MATRICES
	return v[colIndex][rowIndex];
#else
	return v[rowIndex][colIndex];
#endif
}

const float &float3x4::At(int rowIndex, int colIndex) const
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(colIndex >= 0);
	assume(colIndex < Cols);
#ifdef MATH_COLMAJOR_MATRICES
	return v[colIndex][rowIndex];
#else
	return v[rowIndex][colIndex];
#endif
}

#ifdef MATH_COLMAJOR_MATRICES
float3 &float3x4::Col(int col)
{
	assume(col >= 0);
	assume(col < Cols);
	
	return reinterpret_cast<float3 &>(v[col]);
}

const float3 &float3x4::Col(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	
	return reinterpret_cast<const float3 &>(v[col]);
}

float3 &float3x4::Col3(int col)
{
	assume(col >= 0);
	assume(col < Cols);
	
	return reinterpret_cast<float3 &>(v[col]);
}

const float3 &float3x4::Col3(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	
	return reinterpret_cast<const float3 &>(v[col]);
}

CONST_WIN32 float4 float3x4::Row(int row) const
{
	assume(row >= 0);
	assume(row < Rows);
	return float4(At(row, 0), At(row, 1), At(row, 2), At(row, 3));
}
#else
float4 &float3x4::Row(int rowIndex)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);

	return reinterpret_cast<float4 &>(v[rowIndex]);
}

const float4 &float3x4::Row(int rowIndex) const
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);

	return reinterpret_cast<const float4 &>(v[rowIndex]);
}

float3 &float3x4::Row3(int rowIndex)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);

	return reinterpret_cast<float3 &>(v[rowIndex]);
}

const float3 &float3x4::Row3(int rowIndex) const
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);

	return reinterpret_cast<const float3 &>(v[rowIndex]);
}

CONST_WIN32 float3 float3x4::Col(int colIndex) const
{
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	return float3(At(0, colIndex), At(1, colIndex), At(2, colIndex));
}
#endif

CONST_WIN32 float3 float3x4::Diagonal() const
{
	return float3(v[0][0], v[1][1], v[2][2]);
}

void float3x4::ScaleRow3(int r, float scalar)
{
	At(r, 0) *= scalar;
	At(r, 1) *= scalar;
	At(r, 2) *= scalar;
}

void float3x4::ScaleRow(int r, float scalar)
{
#ifdef MATH_SIMD
	row[r] = muls_ps(row[r], scalar);
#else
	At(r, 0) *= scalar;
	At(r, 1) *= scalar;
	At(r, 2) *= scalar;
	At(r, 3) *= scalar;
#endif
}

void float3x4::ScaleCol(int colIndex, float scalar)
{
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	assume(MATH_NS::IsFinite(scalar));

	At(0, colIndex) *= scalar;
	At(1, colIndex) *= scalar;
	At(2, colIndex) *= scalar;
}

CONST_WIN32 float3x3 float3x4::Float3x3Part() const
{
	return float3x3(At(0, 0), At(0, 1), At(0, 2),
					At(1, 0), At(1, 1), At(1, 2),
					At(2, 0), At(2, 1), At(2, 2));
}

CONST_WIN32 float3 float3x4::TranslatePart() const
{
	return Col(3);
}

CONST_WIN32 float3x3 float3x4::RotatePart() const
{
	return Float3x3Part();
}

float3 float3x4::WorldX() const
{
	return Col(0);
}

float3 float3x4::WorldY() const
{
	return Col(1);
}

float3 float3x4::WorldZ() const
{
	return Col(2);
}

void float3x4::SetRow(int row, float m_r0, float m_r1, float m_r2, float m_r3)
{
	assume(row >= 0);
	assume(row < Rows);
	assume(MATH_NS::IsFinite(m_r0));
	assume(MATH_NS::IsFinite(m_r1));
	assume(MATH_NS::IsFinite(m_r2));
	assume(MATH_NS::IsFinite(m_r3));

#ifdef MATH_SIMD
	this->row[row] = set_ps(m_r3, m_r2, m_r1, m_r0);
#else
	At(row, 0) = m_r0;
	At(row, 1) = m_r1;
	At(row, 2) = m_r2;
	At(row, 3) = m_r3;
#endif
}

void float3x4::SetRow(int rowIndex, const float3 &rowVector, float w)
{
	SetRow(rowIndex, rowVector.x, rowVector.y, rowVector.z, w);
}

void float3x4::SetRow(int rowIndex, const float4 &rowVector)
{
#ifdef MATH_SIMD

	this->row[rowIndex] = rowVector.v;
#else
	SetRow(rowIndex, rowVector.x, rowVector.y, rowVector.z, rowVector.w);
#endif
}

void float3x4::SetRow(int rowIndex, const float *data)
{
	assume(data);

#ifdef MATH_SIMD

	this->row[rowIndex] = loadu_ps(data); // Assume unaligned load, since we don't know if data is 16-byte-aligned.
#else
	SetRow(rowIndex, data[0], data[1], data[2], data[3]);
#endif
}

void float3x4::SetCol(int column, float m_0c, float m_1c, float m_2c)
{
	assume(column >= 0);
	assume(column < Cols);
	assume(MATH_NS::IsFinite(m_0c));
	assume(MATH_NS::IsFinite(m_1c));
	assume(MATH_NS::IsFinite(m_2c));
	At(0, column) = m_0c;
	At(1, column) = m_1c;
	At(2, column) = m_2c;
}

void float3x4::SetCol(int column, const float3 &columnVector)
{
	SetCol(column, columnVector.x, columnVector.y, columnVector.z);
}

void float3x4::SetCol(int column, const float *data)
{
	assume(data);
	SetCol(column, data[0], data[1], data[2]);
}

void float3x4::Set(float _00, float _01, float _02, float _03,
				   float _10, float _11, float _12, float _13,
				   float _20, float _21, float _22, float _23)
{
#ifdef MATH_SIMD
	row[0] = set_ps(_03, _02, _01, _00);
	row[1] = set_ps(_13, _12, _11, _10);
	row[2] = set_ps(_23, _22, _21, _20);
#else
	At(0, 0) = _00; At(0, 1) = _01; At(0, 2) = _02; At(0, 3) = _03;
	At(1, 0) = _10; At(1, 1) = _11; At(1, 2) = _12; At(1, 3) = _13;
	At(2, 0) = _20; At(2, 1) = _21; At(2, 2) = _22; At(2, 3) = _23;
#endif
}

void float3x4::Set(const float3x4 &rhs)
{
#ifdef MATH_SIMD
	row[0] = rhs.row[0];
	row[1] = rhs.row[1];
	row[2] = rhs.row[2];
#else
	Set(rhs.ptr());
#endif
}

void float3x4::Set(const float *values)
{
	assume(values);
	float *ptr = (float*)this;
	memcpy(ptr, values, 12*sizeof(float));
}

void float3x4::Set(int rowIndex, int colIndex, float value)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(colIndex >= 0);
	assume(colIndex < Cols);
#ifdef MATH_COLMAJOR_MATRICES
	v[colIndex][rowIndex] = value;
#else
	v[rowIndex][colIndex] = value;
#endif
}

void float3x4::SetIdentity()
{
	Set(1,0,0,0,
		0,1,0,0,
		0,0,1,0);
}

void float3x4::Set3x3Part(const float3x3 &r)
{
	assume(r.IsFinite());
	At(0, 0) = r[0][0]; At(0, 1) = r[0][1]; At(0, 2) = r[0][2];
	At(1, 0) = r[1][0]; At(1, 1) = r[1][1]; At(1, 2) = r[1][2];
	At(2, 0) = r[2][0]; At(2, 1) = r[2][1]; At(2, 2) = r[2][2];
}

void float3x4::SwapColumns(int col1, int col2)
{
	assume(col1 >= 0);
	assume(col1 < Cols);
	assume(col2 >= 0);
	assume(col2 < Cols);
	Swap(At(0, col1), At(0, col2));
	Swap(At(1, col1), At(1, col2));
	Swap(At(2, col1), At(2, col2));
}

void float3x4::SwapRows(int row1, int row2)
{
	assume(row1 >= 0);
	assume(row1 < Rows);
	assume(row2 >= 0);
	assume(row2 < Rows);

#ifdef MATH_SIMD
	Swap(row[row1], row[row2]);
#else
	Swap(At(row1, 0), At(row2, 0));
	Swap(At(row1, 1), At(row2, 1));
	Swap(At(row1, 2), At(row2, 2));
	Swap(At(row1, 3), At(row2, 3));
#endif
}

void float3x4::SetRotatePartX(float angle)
{
	Set3x3PartRotateX(*this, angle);
}

void float3x4::SetRotatePartY(float angle)
{
	Set3x3PartRotateY(*this, angle);
}

void float3x4::SetRotatePartZ(float angle)
{
	Set3x3PartRotateZ(*this, angle);
}

void float3x4::SetRotatePart(const float3 &axisDirection, float angle)
{
	SetRotationAxis3x3(*this, axisDirection, angle);
}

void float3x4::SetRotatePart(const Quat &q)
{
	SetMatrixRotatePart(*this, q);
}

float3x4 float3x4::LookAt(const float3 &localForwardDir, const float3 &targetForwardDir, const float3 &localUp, const float3 &worldUp)
{
	float3x4 m;
	m.SetRotatePart(float3x3::LookAt(localForwardDir, targetForwardDir, localUp, worldUp));
	m.SetTranslatePart(0,0,0);
	return m;
}

float3x4 float3x4::LookAt(const float3 &eyePos, const float3 &targetPos, const float3 &localForward,
                          const float3 &localUp, const float3 &worldUp)
{
	float3x4 m;
	m.SetRotatePart(float3x3::LookAt(localForward, (targetPos-eyePos).Normalized(), localUp, worldUp));
	m.SetTranslatePart(eyePos);
	return m;
}

float3x4 &float3x4::operator =(const float3x3 &rhs)
{
	SetRotatePart(rhs);
	SetTranslatePart(0,0,0);
	return *this;
}

/*
float3x4 &float3x4::operator =(const float3x4 &rhs)
{
	// We deliberately don't want to assume rhs is finite, it is ok
	// to copy around uninitialized matrices.
	// But note that when assigning through a conversion above (float3x3 -> float3x4),
	// we do assume the input matrix is finite.
//	assume(rhs.IsFinite());
#ifdef MATH_SIMD
	row[0] = rhs.row[0];
	row[1] = rhs.row[1];
	row[2] = rhs.row[2];
#else
	At(0, 0) = rhs.At(0, 0);
	At(0, 1) = rhs.At(0, 1);
	At(0, 2) = rhs.At(0, 2);
	At(0, 3) = rhs.At(0, 3);

	At(1, 0) = rhs.At(1, 0);
	At(1, 1) = rhs.At(1, 1);
	At(1, 2) = rhs.At(1, 2);
	At(1, 3) = rhs.At(1, 3);

	At(2, 0) = rhs.At(2, 0);
	At(2, 1) = rhs.At(2, 1);
	At(2, 2) = rhs.At(2, 2);
	At(2, 3) = rhs.At(2, 3);
#endif

	return *this;
}
*/

float3x4 &float3x4::operator =(const Quat &rhs)
{
	*this = rhs.ToFloat3x4();
	return *this;
}

float float3x4::Determinant() const
{
	assume(Float3x3Part().IsFinite());
//#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// 0,-2.873479127883911e-1,9.578263163566589e-1,1.1457166820764542e-1,1,0,0,1.101529598236084e-1,0,9.578263163566589e-1,2.873479127883911e-1,0.037929482758045197
	// TODO: Temporarily disabled for numerical issues. Check this closer and re-enable.
//return mat3x4_determinant(row);
//#else
	const float a = v[0][0];
	const float b = v[0][1];
	const float c = v[0][2];
	const float d = v[1][0];
	const float e = v[1][1];
	const float f = v[1][2];
	const float g = v[2][0];
	const float h = v[2][1];
	const float i = v[2][2];

	return a*e*i + b*f*g + c*d*h - a*f*h - b*d*i - c*e*g;
//#endif
}

bool float3x4::Inverse(float epsilon)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	MARK_UNUSED(epsilon);
	float det = mat3x4_inverse(row, row);
	return MATH_NS::Abs(det) > 1e-5f;
#else
	float4x4 temp(*this); ///@todo It is possible optimize to avoid copying here by writing the inverse function specifically for float3x4.
	bool success = temp.Inverse(epsilon);
	*this = temp.Float3x4Part();
	return success;
#endif
}

float3x4 float3x4::Inverted() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	float3x4 copy;
	mat4x4_inverse(row, copy.row);
#else
	float3x4 copy = *this;
	copy.Inverse();
#endif
	return copy;
}

bool float3x4::InverseColOrthogonal()
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	mat3x4_inverse_colorthogonal(row, row);
#else
	assume(IsColOrthogonal());
	float s1 = float3(At(0, 0), At(1, 0), At(2, 0)).LengthSq();
	float s2 = float3(At(0, 1), At(1, 1), At(2, 1)).LengthSq();
	float s3 = float3(At(0, 2), At(1, 2), At(2, 2)).LengthSq();
	if (s1 < 1e-8f || s2 < 1e-8f || s3 < 1e-8f)
		return false;
	s1 = 1.f / s1;
	s2 = 1.f / s2;
	s3 = 1.f / s3;
	Swap(At(0, 1), At(1, 0));
	Swap(At(0, 2), At(2, 0));
	Swap(At(1, 2), At(2, 1));

	At(0, 0) *= s1; At(0, 1) *= s1; At(0, 2) *= s1;
	At(1, 0) *= s2; At(1, 1) *= s2; At(1, 2) *= s2;
	At(2, 0) *= s3; At(2, 1) *= s3; At(2, 2) *= s3;

	SetTranslatePart(TransformDir(-At(0, 3), -At(1, 3), -At(2, 3)));
	mathassert(IsRowOrthogonal());
#endif
	return true;
}

bool float3x4::InverseOrthogonalUniformScale()
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	mat3x4_inverse_orthogonal_uniformscale(row, row);
	return true; ///\todo The return value is difficult here with SSE, figure out how to treat that.
#else
	assume(IsColOrthogonal(1e-3f));
	assume(HasUniformScale());
	Swap(At(0, 1), At(1, 0));
	Swap(At(0, 2), At(2, 0));
	Swap(At(1, 2), At(2, 1));
	float scale = float3(At(0, 0), At(1, 0), At(2, 0)).LengthSq();
	if (scale == 0.f)
		return false;
	scale = 1.f / scale;

	At(0, 0) *= scale; At(0, 1) *= scale; At(0, 2) *= scale;
	At(1, 0) *= scale; At(1, 1) *= scale; At(1, 2) *= scale;
	At(2, 0) *= scale; At(2, 1) *= scale; At(2, 2) *= scale;

	SetTranslatePart(TransformDir(-At(0, 3), -At(1, 3), -At(2, 3)));

	return true;
#endif
}

void float3x4::InverseOrthonormal()
{
	assume(IsOrthonormal());

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	mat3x4_inverse_orthonormal(row, row);
#else
	/* In this function, we seek to optimize the matrix inverse in the case this
	   matrix is orthonormal, i.e. it can be written in the following form:

	              [ R | T ]
	          M = [---+---]
	              [ 0 | 1 ]

	   where R is a 3x3 orthonormal (orthogonal vectors, normalized columns) rotation
	   matrix, and T is a 3x1 vector representing the translation performed by
	   this matrix.

	   In this form, the inverse of this matrix is simple to compute and will not
	   require the calculation of determinants or expensive Gaussian elimination. The
	   inverse is of form

	                 [ R^t | R^t(-T) ]
	          M^-1 = [-----+---------]
	                 [  0  |    1    ]

	   which can be seen by multiplying out M * M^(-1) in block form. Especially the top-
	   right cell turns out to (remember that R^(-1) == R^t since R is orthonormal)

	        R * R^t(-T) + T * 1 == (R * R^t)(-T) + T == -T + T == 0, as expected.

	   Therefore the inversion requires only two steps: */

	// a) Transpose the top-left 3x3 part in-place to produce R^t.
	Swap(v[0][1], v[1][0]);
	Swap(v[0][2], v[2][0]);
	Swap(v[1][2], v[2][1]);

	// b) Replace the top-right 3x1 part by computing R^t(-T).
	SetTranslatePart(TransformDir(-At(0, 3), -At(1, 3), -At(2, 3)));
#endif
}

void float3x4::Transpose3()
{
	///\todo SSE.
	Swap(v[0][1], v[1][0]);
	Swap(v[0][2], v[2][0]);
	Swap(v[1][2], v[2][1]);
}

float3x4 float3x4::Transposed3() const
{
	float3x4 copy = *this;
	copy.Transpose3();
	return copy;
}

bool float3x4::InverseTranspose()
{
	bool success = Inverse();
	Transpose3();
	// float3x4 cannot represent the translation element as the fourth row after transposing.
	// Since inverse transposes are used mainly to transform direction vectors, we can discard the translation component.
	SetTranslatePart(0,0,0);
	return success;
}

float3x4 float3x4::InverseTransposed() const
{
	float3x4 copy = *this;
	copy.Transpose3();
	copy.Inverse();
	// float3x4 cannot represent the translation element as the fourth row after transposing.
	// Since inverse transposes are used mainly to transform direction vectors, we can discard the translation component.
	copy.SetTranslatePart(0,0,0);
	return copy;
}

float float3x4::Trace() const
{
	assume(IsFinite());
	return v[0][0] + v[1][1] + v[2][2];
}

void float3x4::Orthonormalize(int c0, int c1, int c2)
{
	///\todo SSE.
	assume(c0 != c1 && c0 != c2 && c1 != c2);
	assume(c0 >= 0 && c1 >= 0 && c2 >= 0 && c0 < Cols && c1 < Cols && c2 < Cols);

	///@todo Optimize away copies.
	float3 v0 = Col(c0);
	float3 v1 = Col(c1);
	float3 v2 = Col(c2);
	float3::Orthonormalize(v0, v1, v2);
	SetCol(c0, v0);
	SetCol(c1, v1);
	SetCol(c2, v2);
}

void float3x4::RemoveScale()
{
	///\todo SSE.
#ifdef MATH_COLMAJOR_MATRICES
	float3 row0 = Row3(0);
	float3 row1 = Row3(1);
	float3 row2 = Row3(2);
	float tx = row0.Normalize();
	float ty = row1.Normalize();
	float tz = row2.Normalize();
#else
	float tx = Row3(0).Normalize();
	float ty = Row3(1).Normalize();
	float tz = Row3(2).Normalize();
#endif

	assume(tx != 0 && ty != 0 && tz != 0 && "float3x4::RemoveScale failed!");
	MARK_UNUSED(tx);
	MARK_UNUSED(ty);
	MARK_UNUSED(tz);
}

float2 float3x4::TransformPos(const float2 &pointVector) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(1.f, 0.f, pointVector.y, pointVector.x)).xy();
#else
	return TransformPos(pointVector.x, pointVector.y);
#endif
}

float2 float3x4::TransformPos(float tx, float ty) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(1.f, 0.f, ty, tx)).xy();
#else
	assume(Equal(At(2, 3), 0.f));
	return float2(At(0, 0) * tx + At(0, 1) * ty + At(0, 3),
	              At(1, 0) * tx + At(1, 1) * ty + At(1, 3));
#endif
}

float3 float3x4::TransformPos(const float3 &pointVector) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(1.f, pointVector.z, pointVector.y, pointVector.x));
#else
	return TransformPos(pointVector.x, pointVector.y, pointVector.z);
#endif
}

float3 float3x4::TransformPos(float tx, float ty, float tz) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(1.f, tz, ty, tx));
#else
	return float3(At(0, 0) * tx + At(0, 1) * ty + At(0, 2) * tz + At(0, 3),
				  At(1, 0) * tx + At(1, 1) * ty + At(1, 2) * tz + At(1, 3),
				  At(2, 0) * tx + At(2, 1) * ty + At(2, 2) * tz + At(2, 3));
#endif
}

float2 float3x4::TransformDir(const float2 &directionVector) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(0.f, 0.f, directionVector.y, directionVector.x)).xy();
#else
	return TransformDir(directionVector.x, directionVector.y);
#endif
}

float2 float3x4::TransformDir(float tx, float ty) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(0, 0, ty, tx)).xy();
#else
	return float2(At(0, 0) * tx + At(0, 1) * ty,
				  At(1, 0) * tx + At(1, 1) * ty);
#endif
}

float3 float3x4::TransformDir(const float3 &directionVector) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(0.f, directionVector.z, directionVector.y, directionVector.x));
#else
	return TransformDir(directionVector.x, directionVector.y, directionVector.z);
#endif
}

float3 float3x4::TransformDir(float tx, float ty, float tz) const
{
#ifdef MATH_SSE
	return mat3x4_mul_vec(row, set_ps(0, tz, ty, tx));
#else
	return float3(At(0, 0) * tx + At(0, 1) * ty + At(0, 2) * tz,
				  At(1, 0) * tx + At(1, 1) * ty + At(1, 2) * tz,
				  At(2, 0) * tx + At(2, 1) * ty + At(2, 2) * tz);
#endif
}

float4 float3x4::TransformDir(const float4 &directionVector) const
{
	assume(EqualAbs(directionVector.w, 0.f));
	return Transform(directionVector);
}

float4 float3x4::Transform(const float4 &vector) const
{
#ifdef MATH_SSE
	return float4(mat3x4_mul_sse(row, vector.v));
#else
	return float4(At(0, 0) * vector.x + At(0, 1) * vector.y + At(0, 2) * vector.z + At(0, 3) * vector.w,
				  At(1, 0) * vector.x + At(1, 1) * vector.y + At(1, 2) * vector.z + At(1, 3) * vector.w,
				  At(2, 0) * vector.x + At(2, 1) * vector.y + At(2, 2) * vector.z + At(2, 3) * vector.w,
				  vector.w);
#endif
}

void float3x4::BatchTransformPos(float3 *pointArray, int numPoints) const
{
	assume(pointArray);
	for(int i = 0; i < numPoints; ++i)
		pointArray[i] = MulPos(pointArray[i]);
}

void float3x4::BatchTransformPos(float3 *pointArray, int numPoints, int stride) const
{
	assume(pointArray);
	assume(stride >= (int)sizeof(float3));
	u8 *data = reinterpret_cast<u8*>(pointArray);
	for(int i = 0; i < numPoints; ++i)
	{
		float3 *vtx = reinterpret_cast<float3*>(data + stride*i);
		*vtx = MulPos(*vtx);
	}
}

void float3x4::BatchTransformDir(float3 *dirArray, int numVectors) const
{
	assume(dirArray);
	for(int i = 0; i < numVectors; ++i)
		dirArray[i] = MulPos(dirArray[i]);
}

void float3x4::BatchTransformDir(float3 *dirArray, int numVectors, int stride) const
{
	assume(dirArray);
	assume(stride >= (int)sizeof(float3));
	u8 *data = reinterpret_cast<u8*>(dirArray);
	for(int i = 0; i < numVectors; ++i)
	{
		float3 *vtx = reinterpret_cast<float3*>(data + stride*i);
		*vtx = MulDir(*vtx);
	}
}

void float3x4::BatchTransform(float4 *vectorArray, int numVectors) const
{
	assume(vectorArray);
	for(int i = 0; i < numVectors; ++i)
		vectorArray[i] = *this * vectorArray[i];
}

void float3x4::BatchTransform(float4 *vectorArray, int numVectors, int stride) const
{
	assume(vectorArray);
	assume(stride >= (int)sizeof(float4));
	u8 *data = reinterpret_cast<u8*>(vectorArray);
	for(int i = 0; i < numVectors; ++i)
	{
		float4 *vtx = reinterpret_cast<float4*>(data + stride*i);
		*vtx = *this * *vtx;
	}
}

float3x4 float3x4::operator *(const float3x3 &rhs) const
{
	///\todo SSE.
	float3x4 r;

	r[0][0] = At(0, 0) * rhs.At(0, 0) + At(0, 1) * rhs.At(1, 0) + At(0, 2) * rhs.At(2, 0);
	r[0][1] = At(0, 0) * rhs.At(0, 1) + At(0, 1) * rhs.At(1, 1) + At(0, 2) * rhs.At(2, 1);
	r[0][2] = At(0, 0) * rhs.At(0, 2) + At(0, 1) * rhs.At(1, 2) + At(0, 2) * rhs.At(2, 2);
	r[0][3] = At(0, 3);

	r[1][0] = At(1, 0) * rhs.At(0, 0) + At(1, 1) * rhs.At(1, 0) + At(1, 2) * rhs.At(2, 0);
	r[1][1] = At(1, 0) * rhs.At(0, 1) + At(1, 1) * rhs.At(1, 1) + At(1, 2) * rhs.At(2, 1);
	r[1][2] = At(1, 0) * rhs.At(0, 2) + At(1, 1) * rhs.At(1, 2) + At(1, 2) * rhs.At(2, 2);
	r[1][3] = At(1, 3);

	r[2][0] = At(2, 0) * rhs.At(0, 0) + At(2, 1) * rhs.At(1, 0) + At(2, 2) * rhs.At(2, 0);
	r[2][1] = At(2, 0) * rhs.At(0, 1) + At(2, 1) * rhs.At(1, 1) + At(2, 2) * rhs.At(2, 1);
	r[2][2] = At(2, 0) * rhs.At(0, 2) + At(2, 1) * rhs.At(1, 2) + At(2, 2) * rhs.At(2, 2);
	r[2][3] = At(2, 3);

	return r;
}

float3x4 float3x4::operator *(const float3x4 &rhs) const
{
	float3x4 r;
#ifdef MATH_SSE
	mat3x4_mul_sse(r.row, row, rhs.row);
#else
	r[0][0] = At(0, 0) * rhs.At(0, 0) + At(0, 1) * rhs.At(1, 0) + At(0, 2) * rhs.At(2, 0);
	r[0][1] = At(0, 0) * rhs.At(0, 1) + At(0, 1) * rhs.At(1, 1) + At(0, 2) * rhs.At(2, 1);
	r[0][2] = At(0, 0) * rhs.At(0, 2) + At(0, 1) * rhs.At(1, 2) + At(0, 2) * rhs.At(2, 2);
	r[0][3] = At(0, 0) * rhs.At(0, 3) + At(0, 1) * rhs.At(1, 3) + At(0, 2) * rhs.At(2, 3) + At(0, 3);
	
	r[1][0] = At(1, 0) * rhs.At(0, 0) + At(1, 1) * rhs.At(1, 0) + At(1, 2) * rhs.At(2, 0);
	r[1][1] = At(1, 0) * rhs.At(0, 1) + At(1, 1) * rhs.At(1, 1) + At(1, 2) * rhs.At(2, 1);
	r[1][2] = At(1, 0) * rhs.At(0, 2) + At(1, 1) * rhs.At(1, 2) + At(1, 2) * rhs.At(2, 2);
	r[1][3] = At(1, 0) * rhs.At(0, 3) + At(1, 1) * rhs.At(1, 3) + At(1, 2) * rhs.At(2, 3) + At(1, 3);

	r[2][0] = At(2, 0) * rhs.At(0, 0) + At(2, 1) * rhs.At(1, 0) + At(2, 2) * rhs.At(2, 0);
	r[2][1] = At(2, 0) * rhs.At(0, 1) + At(2, 1) * rhs.At(1, 1) + At(2, 2) * rhs.At(2, 1);
	r[2][2] = At(2, 0) * rhs.At(0, 2) + At(2, 1) * rhs.At(1, 2) + At(2, 2) * rhs.At(2, 2);
	r[2][3] = At(2, 0) * rhs.At(0, 3) + At(2, 1) * rhs.At(1, 3) + At(2, 2) * rhs.At(2, 3) + At(2, 3);
#endif

	return r;
}

float3x4 float3x4::operator *(const Quat &rhs) const
{
	float3x3 rot(rhs);
	return *this * rot;
}

float4 float3x4::operator *(const float4 &rhs) const
{
	return Transform(rhs);
}

float3x4 float3x4::operator *(float scalar) const
{
#ifdef MATH_SIMD
	float3x4 r;
	simd4f s = set1_ps(scalar);
	r.row[0] = mul_ps(row[0], s);
	r.row[1] = mul_ps(row[1], s);
	r.row[2] = mul_ps(row[2], s);
#else
	float3x4 r = *this;
	r *= scalar;
#endif

	return r;
}

float3x4 float3x4::operator /(float scalar) const
{
	assume(!EqualAbs(scalar, 0));

#ifdef MATH_SIMD
	float3x4 r;
	simd4f s = set1_ps(scalar);
	simd4f one = set1_ps(1.f);
	s = div_ps(one, s);
	r.row[0] = mul_ps(row[0], s);
	r.row[1] = mul_ps(row[1], s);
	r.row[2] = mul_ps(row[2], s);
#else
	float3x4 r = *this;
	r /= scalar;
#endif

	return r;
}

float3x4 float3x4::operator +(const float3x4 &rhs) const
{
#ifdef MATH_SIMD
	float3x4 r;
	r.row[0] = add_ps(row[0], rhs.row[0]);
	r.row[1] = add_ps(row[1], rhs.row[1]);
	r.row[2] = add_ps(row[2], rhs.row[2]);
#else
	float3x4 r = *this;
	r += rhs;
#endif

	return r;
}

float3x4 float3x4::operator -(const float3x4 &rhs) const
{
#ifdef MATH_SIMD
	float3x4 r;
	r.row[0] = sub_ps(row[0], rhs.row[0]);
	r.row[1] = sub_ps(row[1], rhs.row[1]);
	r.row[2] = sub_ps(row[2], rhs.row[2]);
#else
	float3x4 r = *this;
	r -= rhs;
#endif

	return r;
}

float3x4 float3x4::operator -() const
{
	float3x4 r;

#ifdef MATH_SIMD
	const simd4f zeroVec = zero_ps();
	r.row[0] = sub_ps(zeroVec, row[0]);
	r.row[1] = sub_ps(zeroVec, row[1]);
	r.row[2] = sub_ps(zeroVec, row[2]);
#else
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			r[iy][ix] = -At(iy, ix);
#endif

	return r;
}

float3x4 &float3x4::operator *=(float scalar)
{
#ifdef MATH_SIMD
	simd4f s = set1_ps(scalar);
	row[0] = mul_ps(row[0], s);
	row[1] = mul_ps(row[1], s);
	row[2] = mul_ps(row[2], s);
#else
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			At(iy, ix) *= scalar;
#endif

	return *this;
}

float3x4 &float3x4::operator /=(float scalar)
{
	assume(!EqualAbs(scalar, 0));

#ifdef MATH_SIMD
	simd4f s = set1_ps(scalar);
	simd4f one = set1_ps(1.f);
	s = div_ps(one, s);
	row[0] = mul_ps(row[0], s);
	row[1] = mul_ps(row[1], s);
	row[2] = mul_ps(row[2], s);
#else
	float invScalar = 1.f / scalar;
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			At(iy, ix) *= invScalar;
#endif

	return *this;
}

float3x4 &float3x4::operator +=(const float3x4 &rhs)
{
#ifdef MATH_SIMD
	row[0] = add_ps(row[0], rhs.row[0]);
	row[1] = add_ps(row[1], rhs.row[1]);
	row[2] = add_ps(row[2], rhs.row[2]);
#else
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			At(iy, ix) += rhs[iy][ix];
#endif

	return *this;
}

float3x4 &float3x4::operator -=(const float3x4 &rhs)
{
#ifdef MATH_SIMD
	row[0] = sub_ps(row[0], rhs.row[0]);
	row[1] = sub_ps(row[1], rhs.row[1]);
	row[2] = sub_ps(row[2], rhs.row[2]);
#else
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			At(iy, ix) -= rhs[iy][ix];
#endif

	return *this;
}

bool float3x4::IsFinite() const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!MATH_NS::IsFinite(At(iy, ix)))
				return false;
	return true;
}

bool float3x4::IsIdentity(float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!EqualAbs(At(iy, ix), (ix == iy) ? 1.f : 0.f, epsilon))
				return false;

	return true;
}

bool float3x4::IsLowerTriangular(float epsilon) const
{
	return EqualAbs(At(0, 1), 0.f, epsilon)
		&& EqualAbs(At(0, 2), 0.f, epsilon)
		&& EqualAbs(At(0, 3), 0.f, epsilon)
		&& EqualAbs(At(1, 2), 0.f, epsilon)
		&& EqualAbs(At(1, 3), 0.f, epsilon)
		&& EqualAbs(At(2, 3), 0.f, epsilon);
}

bool float3x4::IsUpperTriangular(float epsilon) const
{
	return EqualAbs(At(1, 0), 0.f, epsilon)
		&& EqualAbs(At(2, 0), 0.f, epsilon)
		&& EqualAbs(At(2, 1), 0.f, epsilon);
}

bool float3x4::IsInvertible(float epsilon) const
{
	float d = Determinant();
	bool isSingular = EqualAbs(d, 0.f, epsilon);
#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 temp = Float3x3Part();
	mathassert(temp.Inverse(epsilon) != isSingular); // IsInvertible() and Inverse() must match!
#endif
	return !isSingular;
}

bool float3x4::IsSymmetric(float epsilon) const
{
	return EqualAbs(At(0, 1), At(1, 0), epsilon) &&
		EqualAbs(At(0, 2), At(2, 0), epsilon) &&
		EqualAbs(At(1, 2), At(2, 1), epsilon);
}

bool float3x4::IsSkewSymmetric(float epsilon) const
{
	return EqualAbs(At(0, 0), 0.f, epsilon) &&
		EqualAbs(At(1, 1), 0.f, epsilon) &&
		EqualAbs(At(2, 2), 0.f, epsilon) &&
		EqualAbs(At(0, 1), -At(1, 0), epsilon) &&
		EqualAbs(At(0, 2), -At(2, 0), epsilon) &&
		EqualAbs(At(1, 2), -At(2, 1), epsilon);
}

bool float3x4::HasUnitaryScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return scale.Equals(1.f, 1.f, 1.f, epsilon);
}

bool float3x4::HasNegativeScale() const
{
	return Determinant() < 0.f;
}

bool float3x4::HasUniformScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return EqualAbs(scale.x, scale.y, epsilon) && EqualAbs(scale.x, scale.z, epsilon);
}

bool float3x4::IsRowOrthogonal(float epsilon) const
{
	return Row(0).IsPerpendicular3(Row(1), epsilon)
		&& Row(0).IsPerpendicular3(Row(2), epsilon)
		&& Row(1).IsPerpendicular3(Row(2), epsilon);
}

bool float3x4::IsColOrthogonal(float epsilon) const
{
	return Col(0).IsPerpendicular(Col(1), epsilon)
		&& Col(0).IsPerpendicular(Col(2), epsilon)
		&& Col(1).IsPerpendicular(Col(2), epsilon);
}

bool float3x4::IsOrthonormal(float epsilon) const
{
	///@todo Epsilon magnitudes don't match.
	return IsColOrthogonal(epsilon) && Row3(0).IsNormalized(epsilon) && Row3(1).IsNormalized(epsilon) && Row3(2).IsNormalized(epsilon);
}

bool float3x4::Equals(const float3x4 &other, float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!EqualAbs(At(iy, ix), other[iy][ix], epsilon))
				return false;
	return true;
}


#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float3x4::ToString() const
{
	char str[256];
	sprintf(str, "(%.2f, %.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f)",
		At(0, 0), At(0, 1), At(0, 2), At(0, 3),
		At(1, 0), At(1, 1), At(1, 2), At(1, 3),
		At(2, 0), At(2, 1), At(2, 2), At(2, 3));

	return str;
}

StringT float3x4::SerializeToString() const
{
	char str[512];
	char *s = SerializeFloat(At(0, 0), str); *s = ','; ++s;
	s = SerializeFloat(At(0, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(0, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(0, 3), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 0), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 3), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 0), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 3), s);
	assert(s+1 - str < 512);
	MARK_UNUSED(s);
	return str;
}

StringT float3x4::ToString2() const
{
	char str[256];
	sprintf(str, "float3x4(X:(%.2f,%.2f,%.2f) Y:(%.2f,%.2f,%.2f) Z:(%.2f,%.2f,%.2f), Pos:(%.2f,%.2f,%.2f))",
		At(0, 0), At(1, 0), At(2, 0),
		At(0, 1), At(1, 1), At(2, 1),
		At(0, 2), At(1, 2), At(2, 2),
		At(0, 3), At(1, 3), At(2, 3));

	return str;
}

bool IsNeutralCLocale();

float3x4 float3x4::FromString(const char *str, const char **outEndStr)
{
	assert(IsNeutralCLocale());
	assume(str);
	if (!str)
		return float3x4::nan;
	MATH_SKIP_WORD(str, "float3x4");
	MATH_SKIP_WORD(str, "(");
	float3x4 m;
	for (int i = 0; i < 12; ++i)
	{
		m.ptr()[i] = DeserializeFloat(str, &str);
		MATH_SKIP_WORD(str, "(");
		MATH_SKIP_WORD(str, ")");
	}
	if (*str == ')')
		++str;
	if (*str == ',')
		++str;
	if (outEndStr)
		*outEndStr = str;
	return m;
}

#endif

float3 float3x4::ToEulerXYX() const { float3 f; ExtractEulerXYX(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerXZX() const { float3 f; ExtractEulerXZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerYXY() const { float3 f; ExtractEulerYXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerYZY() const { float3 f; ExtractEulerYZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerZXZ() const { float3 f; ExtractEulerZXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerZYZ() const { float3 f; ExtractEulerZYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerXYZ() const { float3 f; ExtractEulerXYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerXZY() const { float3 f; ExtractEulerXZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerYXZ() const { float3 f; ExtractEulerYXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerYZX() const { float3 f; ExtractEulerYZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerZXY() const { float3 f; ExtractEulerZXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x4::ToEulerZYX() const { float3 f; ExtractEulerZYX(*this, f[0], f[1], f[2]); return f; }

float3 float3x4::ExtractScale() const
{
	return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

void float3x4::Decompose(float3 &translate, Quat &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal());

	float3x3 r;
	Decompose(translate, r, scale);
	rotate = Quat(r);

	// Test that composing back yields the original float3x4.
	assume(float3x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

void float3x4::Decompose(float3 &translate, float3x3 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal());

	translate = Col(3);
	rotate = RotatePart();
	scale.x = rotate.Col(0).Length();
	scale.y = rotate.Col(1).Length();
	scale.z = rotate.Col(2).Length();
	assume(!EqualAbs(scale.x, 0));
	assume(!EqualAbs(scale.y, 0));
	assume(!EqualAbs(scale.z, 0));
	rotate.ScaleCol(0, 1.f / scale.x);
	rotate.ScaleCol(1, 1.f / scale.y);
	rotate.ScaleCol(2, 1.f / scale.z);

	// Test that composing back yields the original float3x4.
	assume(float3x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

void float3x4::Decompose(float3 &translate, float3x4 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal());

	float3x3 r;
	Decompose(translate, r, scale);
	rotate.SetRotatePart(r);
	rotate.SetTranslatePart(0,0,0);

	// Test that composing back yields the original float3x4.
	assume(float3x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3x4 &rhs)
{
	out << rhs.ToString();
	return out;
}
#endif

float3x4 operator *(const Quat &lhs, const float3x4 &rhs)
{
	return float3x4(lhs) * rhs;
}

float3x4 operator *(const float3x3 &lhs, const float3x4 &rhs)
{
	///\todo SSE.
	return float3x4(lhs) * rhs;
}

float4 operator *(const float4 &lhs, const float3x4 &rhs)
{
	///\todo SSE.
	return float4(lhs.x * rhs.At(0, 0) + lhs.y * rhs.At(1, 0) + lhs.z * rhs.At(2, 0),
				  lhs.x * rhs.At(0, 1) + lhs.y * rhs.At(1, 1) + lhs.z * rhs.At(2, 1),
				  lhs.x * rhs.At(0, 2) + lhs.y * rhs.At(1, 2) + lhs.z * rhs.At(2, 2),
				  lhs.x * rhs.At(0, 3) + lhs.y * rhs.At(1, 3) + lhs.z * rhs.At(2, 3) + lhs.w);
}

float3x4 float3x4::Mul(const float3x3 &rhs) const { return *this * rhs; }
float3x4 float3x4::Mul(const float3x4 &rhs) const { return *this * rhs; }
float4x4 float3x4::Mul(const float4x4 &rhs) const { return *this * rhs; }
float3x4 float3x4::Mul(const Quat &rhs) const { return *this * rhs; }
float2 float3x4::MulPos(const float2 &pointVector) const { return this->TransformPos(pointVector); }
float3 float3x4::MulPos(const float3 &pointVector) const { return this->TransformPos(pointVector); }
float4 float3x4::MulPos(const float4 &pointVector) const
{
	assume(!EqualAbs(pointVector.w, 0.f));
	return this->Transform(pointVector);
}
float2 float3x4::MulDir(const float2 &directionVector) const { return this->TransformDir(directionVector); }
float3 float3x4::MulDir(const float3 &directionVector) const { return this->TransformDir(directionVector); }
float4 float3x4::MulDir(const float4 &directionVector) const
{
	assume(EqualAbs(directionVector.w, 0.f));
	return this->TransformDir(directionVector);
}
float4 float3x4::Mul(const float4 &vector) const { return *this * vector; }

const float3x4 float3x4::zero	 = float3x4(0,0,0,0, 0,0,0,0, 0,0,0,0);
const float3x4 float3x4::identity = float3x4(1,0,0,0, 0,1,0,0, 0,0,1,0);
const float3x4 float3x4::nan = float3x4(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);

MATH_END_NAMESPACE
