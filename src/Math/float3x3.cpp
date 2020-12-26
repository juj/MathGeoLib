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

/** @file float3x3.cpp
	@author Jukka Jylänki
	@brief */
#include "float3x3.h"
#include <string.h>
#include "assume.h"
#include "MathFunc.h"
#include "Swap.h"
#include "float3.h"
#include "float4.h"
#include "float3x4.h"
#include "float4x4.h"
#include "Matrix.inl"
#include "Quat.h"
#include "../Algorithm/Random/LCG.h"
#include "../Geometry/Plane.h"
#include "TransformOps.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

float3x3::float3x3(float _00, float _01, float _02,
                   float _10, float _11, float _12,
                   float _20, float _21, float _22)
{
	Set(_00, _01, _02,
	    _10, _11, _12,
	    _20, _21, _22);
}

float3x3::float3x3(const float3 &col0, const float3 &col1, const float3 &col2)
{
	SetCol(0, col0);
	SetCol(1, col1);
	SetCol(2, col2);
}

float3x3::float3x3(const Quat &orientation)
{
	SetRotatePart(orientation);
}

float3x3 float3x3::RotateX(float angle)
{
	float3x3 r;
	r.SetRotatePartX(angle);
	return r;
}

float3x3 float3x3::RotateY(float angle)
{
	float3x3 r;
	r.SetRotatePartY(angle);
	return r;
}

float3x3 float3x3::RotateZ(float angle)
{
	float3x3 r;
	r.SetRotatePartZ(angle);
	return r;
}

float3x3 float3x3::RotateAxisAngle(const float3 &axisDirection, float angleRadians)
{
	float3x3 r;
	r.SetRotatePart(axisDirection, angleRadians);
	return r;
}

float3x3 float3x3::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection)
{
	assume2(sourceDirection.IsNormalized(), sourceDirection, sourceDirection.Length());
	assume2(targetDirection.IsNormalized(), targetDirection, targetDirection.Length());

	// http://cs.brown.edu/research/pubs/pdfs/1999/Moller-1999-EBA.pdf
	float3x3 r;
	float dot = sourceDirection.Dot(targetDirection);
	if (Abs(dot) > 0.999f)
	{
		float3 s = sourceDirection.Abs();
		float3 unit = s.x < s.y && s.x < s.z ? float3::unitX : (s.y < s.z ? float3::unitY : float3::unitZ);
		float3 u = unit - sourceDirection;
		float3 v = unit - targetDirection;
		float uv = u.Dot(v);
		float uu = u.Dot(u);
		float vv = v.Dot(v);
		float a = -2.f / uu;
		float b = -2.f / vv;
		float c = a * b * uv;

		r[0][0] = 1.f + a*u.x*u.x + b*v.x*v.x + c*v.x*u.x;
		r[0][1] =       a*u.x*u.y + b*v.x*v.y + c*v.x*u.y;
		r[0][2] =       a*u.x*u.z + b*v.x*v.z + c*v.x*u.z;

		r[1][0] =       a*u.y*u.x + b*v.y*v.x + c*v.y*u.x;
		r[1][1] = 1.f + a*u.y*u.y + b*v.y*v.y + c*v.y*u.y;
		r[1][2] =       a*u.y*u.z + b*v.y*v.z + c*v.y*u.z;

		r[2][0] =       a*u.z*u.x + b*v.z*v.x + c*v.z*u.x;
		r[2][1] =       a*u.z*u.y + b*v.z*v.y + c*v.z*u.y;
		r[2][2] = 1.f + a*u.z*u.z + b*v.z*v.z + c*v.z*u.z;
	}
	else
	{
		float3 v = sourceDirection.Cross(targetDirection);
		float h = (1.f - dot) / v.Dot(v);
		float hvx = h * v.x;
		float hvy = h * v.y;
		float hvz = h * v.z;
		r[0][0] = hvx * v.x + dot; r[0][1] = hvy * v.x - v.z; r[0][2] = hvz * v.x + v.y;
		r[1][0] = hvx * v.y + v.z; r[1][1] = hvy * v.y + dot; r[1][2] = hvz * v.y - v.x;
		r[2][0] = hvx * v.z - v.y; r[2][1] = hvy * v.z + v.x; r[2][2] = hvz * v.z + dot;
	}
	return r;
}

float3x3 float3x3::RandomRotation(LCG &lcg)
{
	// The easiest way to generate a random orientation is through quaternions, so convert a
	// random quaternion to a rotation matrix.
	return FromQuat(Quat::RandomRotation(lcg));
}

float3x3 float3x3::RandomGeneral(LCG &lcg, float minElem, float maxElem)
{
	float3x3 m;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 3; ++x)
			m[y][x] = lcg.Float(minElem, maxElem);
	return m;
}

float3x3 float3x3::FromQuat(const Quat &orientation)
{
	float3x3 r;
	r.SetRotatePart(orientation);
	return r;
}

Quat float3x3::ToQuat() const
{
	return Quat(*this);
}

bool float3x3::TryConvertToQuat(Quat &q) const
{
	if (IsColOrthogonal() && HasUnitaryScale() && !HasNegativeScale())
	{
		q = ToQuat();
		return true;
	}
	return false;
}

float3x3 float3x3::FromRS(const Quat &rotate, const float3 &scale)
{
	return float3x3(rotate) * float3x3::Scale(scale);
}

float3x3 float3x3::FromRS(const float3x3 &rotate, const float3 &scale)
{
	return rotate * float3x3::Scale(scale);
}

float3x3 float3x3::FromEulerXYX(float x2, float y, float x)
{
	float3x3 r;
	Set3x3PartRotateEulerXYX(r, x2, y, x);
	assume(r.Equals(float3x3::RotateX(x2) * float3x3::RotateY(y) * float3x3::RotateX(x)));
	return r;
}

float3x3 float3x3::FromEulerXZX(float x2, float z, float x)
{
	float3x3 r;
	Set3x3PartRotateEulerXZX(r, x2, z, x);
	assume(r.Equals(float3x3::RotateX(x2) * float3x3::RotateZ(z) * float3x3::RotateX(x)));
	return r;
}

float3x3 float3x3::FromEulerYXY(float y2, float x, float y)
{
	float3x3 r;
	Set3x3PartRotateEulerYXY(r, y2, x, y);
	assume(r.Equals(float3x3::RotateY(y2) * float3x3::RotateX(x) * float3x3::RotateY(y)));
	return r;
}

float3x3 float3x3::FromEulerYZY(float y2, float z, float y)
{
	float3x3 r;
	Set3x3PartRotateEulerYZY(r, y2, z, y);
	assume(r.Equals(float3x3::RotateY(y2) * float3x3::RotateZ(z) * float3x3::RotateY(y)));
	return r;
}

float3x3 float3x3::FromEulerZXZ(float z2, float x, float z)
{
	float3x3 r;
	Set3x3PartRotateEulerZXZ(r, z2, x, z);
	assume(r.Equals(float3x3::RotateZ(z2) * float3x3::RotateX(x) * float3x3::RotateZ(z)));
	return r;
}

float3x3 float3x3::FromEulerZYZ(float z2, float y, float z)
{
	float3x3 r;
	Set3x3PartRotateEulerZYZ(r, z2, y, z);
	assume(r.Equals(float3x3::RotateZ(z2) * float3x3::RotateY(y) * float3x3::RotateZ(z)));
	return r;
}

float3x3 float3x3::FromEulerXYZ(float x, float y, float z)
{
	float3x3 r;
	Set3x3PartRotateEulerXYZ(r, x, y, z);
	assume(r.Equals(float3x3::RotateX(x) * float3x3::RotateY(y) * float3x3::RotateZ(z)));
	return r;
}

float3x3 float3x3::FromEulerXZY(float x, float z, float y)
{
	float3x3 r;
	Set3x3PartRotateEulerXZY(r, x, z, y);
	assume(r.Equals(float3x3::RotateX(x) * float3x3::RotateZ(z) * float3x3::RotateY(y)));
	return r;
}

float3x3 float3x3::FromEulerYXZ(float y, float x, float z)
{
	float3x3 r;
	Set3x3PartRotateEulerYXZ(r, y, x, z);
	assume(r.Equals(float3x3::RotateY(y) * float3x3::RotateX(x) * float3x3::RotateZ(z)));
	return r;
}

float3x3 float3x3::FromEulerYZX(float y, float z, float x)
{
	float3x3 r;
	Set3x3PartRotateEulerYZX(r, y, z, x);
	assume(r.Equals(float3x3::RotateY(y) * float3x3::RotateZ(z) * float3x3::RotateX(x)));
	return r;
}

float3x3 float3x3::FromEulerZXY(float z, float x, float y)
{
	float3x3 r;
	Set3x3PartRotateEulerZXY(r, z, x, y);
	assume(r.Equals(float3x3::RotateZ(z) * float3x3::RotateX(x) * float3x3::RotateY(y)));
	return r;
}

float3x3 float3x3::FromEulerZYX(float z, float y, float x)
{
	float3x3 r;
	Set3x3PartRotateEulerZYX(r, z, y, x);
	assume(r.Equals(float3x3::RotateZ(z) * float3x3::RotateY(y) * float3x3::RotateX(x)));
	return r;
}

ScaleOp float3x3::Scale(float sx, float sy, float sz)
{
	return ScaleOp(sx, sy, sz);
}

ScaleOp float3x3::Scale(const float3 &scale)
{
	return ScaleOp(scale);
}

float3x3 float3x3::ScaleAlongAxis(const float3 &axis, float scalingFactor)
{
	return Scale(axis * scalingFactor);
}

ScaleOp float3x3::UniformScale(float uniformScale)
{
	return ScaleOp(uniformScale, uniformScale, uniformScale);
}

float3 float3x3::GetScale() const
{
	return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

float3x3 float3x3::ShearX(float yFactor, float zFactor)
{
	assume(MATH_NS::IsFinite(yFactor));
	assume(MATH_NS::IsFinite(zFactor));
	return float3x3(1.f, yFactor, zFactor,
	                0.f,     1.f,     0.f,
	                0.f,     0.f,     1.f);
}

float3x3 float3x3::ShearY(float xFactor, float zFactor)
{
	assume(MATH_NS::IsFinite(xFactor));
	assume(MATH_NS::IsFinite(zFactor));
	return float3x3(1.f,     0.f,     0.f,
	                xFactor, 1.f, zFactor,
	                0.f,     0.f,     1.f);
}

float3x3 float3x3::ShearZ(float xFactor, float yFactor)
{
	assume(MATH_NS::IsFinite(xFactor));
	assume(MATH_NS::IsFinite(yFactor));
	return float3x3(1.f,         0.f, 0.f,
	                0.f,         1.f, 0.f,
	                xFactor, yFactor, 1.f);
}

float3x3 float3x3::Mirror(const Plane &p)
{
	assume(p.PassesThroughOrigin() && "A 3x3 matrix cannot represent mirroring about planes which do not pass through the origin! Use float3x4::Mirror instead!");
	float3x3 v;
	SetMatrix3x3LinearPlaneMirror(v, p.normal.x, p.normal.y, p.normal.z);
	return v;
}

float3x3 float3x3::OrthographicProjection(const Plane &p)
{
	assume(p.normal.IsNormalized());
	assume(p.PassesThroughOrigin() && "A 3x3 matrix cannot represent projection onto planes which do not pass through the origin! Use float3x4::OrthographicProjection instead!");
	float3x3 v;
	SetMatrix3x3LinearPlaneProject(v, p.normal.x, p.normal.y, p.normal.z);
	return v;
}

float3x3 float3x3::OrthographicProjectionYZ()
{
	float3x3 v = identity;
	v[0][0] = 0.f;
	return v;
}

float3x3 float3x3::OrthographicProjectionXZ()
{
	float3x3 v = identity;
	v[1][1] = 0.f;
	return v;
}

float3x3 float3x3::OrthographicProjectionXY()
{
	float3x3 v = identity;
	v[2][2] = 0.f;
	return v;
}

float &float3x3::At(int row, int col)
{
	assume(row >= 0);
	assume(row < Rows);
	assume(col >= 0);
	assume(col < Cols);

#ifdef MATH_COLMAJOR_MATRICES
	return v[col][row];
#else
	return v[row][col];
#endif
}

CONST_WIN32 float float3x3::At(int row, int col) const
{
	assume(row >= 0);
	assume(row < Rows);
	assume(col >= 0);
	assume(col < Cols);

#ifdef MATH_COLMAJOR_MATRICES
	return v[col][row];
#else
	return v[row][col];
#endif
}

#ifdef MATH_COLMAJOR_MATRICES
float3 &float3x3::Col(int col)
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<float3 &>(v[col]);
}

const float3 &float3x3::Col(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<const float3 &>(v[col]);
}

CONST_WIN32 float3 float3x3::Row(int row) const
{
	assume(row >= 0);
	assume(row < Rows);
	return float3(At(row, 0), At(row, 1), At(row, 2));
}

#else
float3 &float3x3::Row(int row)
{
	assume(row >= 0);
	assume(row < Rows);
	return reinterpret_cast<float3 &>(v[row]);
}

const float3 &float3x3::Row(int row) const
{
	assume(row >= 0);
	assume(row < Rows);
	return reinterpret_cast<const float3 &>(v[row]);
}

CONST_WIN32 float3 float3x3::Col(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	return float3(At(0, col), At(1, col), At(2, col));
}
#endif

CONST_WIN32 float3 float3x3::Diagonal() const
{
	return float3(v[0][0], v[1][1], v[2][2]);
}

void float3x3::ScaleRow(int row, float scalar)
{
	assume(row >= 0);
	assume(row < Rows);
	assume(MATH_NS::IsFinite(scalar));
	At(row, 0) *= scalar;
	At(row, 1) *= scalar;
	At(row, 2) *= scalar;
}

void float3x3::ScaleCol(int col, float scalar)
{
	assume(col >= 0);
	assume(col < Cols);
	At(0, col) *= scalar;
	At(1, col) *= scalar;
	At(2, col) *= scalar;
}

float3 float3x3::WorldX() const
{
	return Col(0);
}

float3 float3x3::WorldY() const
{
	return Col(1);
}

float3 float3x3::WorldZ() const
{
	return Col(2);
}

void float3x3::SetRow(int row, float x, float y, float z)
{
	assume(row >= 0);
	assume(row < Rows);
	assume(MATH_NS::IsFinite(x));
	assume(MATH_NS::IsFinite(y));
	assume(MATH_NS::IsFinite(z));
	At(row, 0) = x;
	At(row, 1) = y;
	At(row, 2) = z;
}

void float3x3::SetRow(int row, const float3 &rowVector)
{
	SetRow(row, rowVector.x, rowVector.y, rowVector.z);
}

void float3x3::SetRow(int row, const float *data)
{
	assume(data);
	SetRow(row, data[0], data[1], data[2]);
}

void float3x3::SetCol(int column, float x, float y, float z)
{
	assume(column >= 0);
	assume(column < Cols);
	assume(MATH_NS::IsFinite(x));
	assume(MATH_NS::IsFinite(y));
	assume(MATH_NS::IsFinite(z));
	At(0, column) = x;
	At(1, column) = y;
	At(2, column) = z;
}

void float3x3::SetCol(int column, const float3 &columnVector)
{
	SetCol(column, columnVector.x, columnVector.y, columnVector.z);
}

void float3x3::SetCol(int column, const float *data)
{
	assume(data);
	SetCol(column, data[0], data[1], data[2]);
}

void float3x3::Set(float _00, float _01, float _02,
                   float _10, float _11, float _12,
                   float _20, float _21, float _22)
{
	At(0, 0) = _00; At(0, 1) = _01; At(0, 2) = _02;
	At(1, 0) = _10; At(1, 1) = _11; At(1, 2) = _12;
	At(2, 0) = _20; At(2, 1) = _21; At(2, 2) = _22;
}

void float3x3::Set(const float3x3 &rhs)
{
	Set(rhs.ptr());
}

void float3x3::Set(const float *values)
{
	assume(values);
	memcpy(this, values, sizeof(float)*9);
}

void float3x3::Set(int row, int col, float value)
{
	assume(0 <= row && row <= 2);
	assume(0 <= col && col <= 2);
	At(row, col) = value;
}

void float3x3::SetIdentity()
{
	Set(1,0,0,
	    0,1,0,
	    0,0,1);
}

void float3x3::SwapColumns(int col1, int col2)
{
	assume(col1 >= 0);
	assume(col1 < Cols);
	assume(col2 >= 0);
	assume(col2 < Cols);
	Swap(At(0, col1), At(0, col2));
	Swap(At(1, col1), At(1, col2));
	Swap(At(2, col1), At(2, col2));
}

void float3x3::SwapRows(int row1, int row2)
{
	assume(row1 >= 0);
	assume(row1 < Rows);
	assume(row2 >= 0);
	assume(row2 < Rows);
	Swap(At(row1, 0), At(row2, 0));
	Swap(At(row1, 1), At(row2, 1));
	Swap(At(row1, 2), At(row2, 2));
}

void float3x3::SetRotatePartX(float angle)
{
	Set3x3PartRotateX(*this, angle);
}

void float3x3::SetRotatePartY(float angle)
{
	Set3x3PartRotateY(*this, angle);
}

void float3x3::SetRotatePartZ(float angle)
{
	Set3x3PartRotateZ(*this, angle);
}

void float3x3::SetRotatePart(const float3 &axisDirection, float angle)
{
	SetRotationAxis3x3(*this, axisDirection, angle);
}

void float3x3::SetRotatePart(const Quat &q)
{
	SetMatrixRotatePart(*this, q);
}

float3x3 float3x3::LookAt(const float3 &localForward, const float3 &targetDirection, const float3 &localUp, const float3 &worldUp)
{
	// The user must have inputted proper normalized input direction vectors.
	assume(localForward.IsNormalized());
	assume(targetDirection.IsNormalized());
	assume(localUp.IsNormalized());
	assume(worldUp.IsNormalized());

	// In the local space, the forward and up directions must be perpendicular to be well-formed.
	assume(localForward.IsPerpendicular(localUp));

	// In the world space, the targetDirection and worldUp cannot be degenerate (collinear)
	assume(!targetDirection.Cross(worldUp).IsZero() && "Passed a degenerate coordinate frame to look towards in float3x3::LookAt!");

	// Generate the third basis vector in the local space.
	float3 localRight = localUp.Cross(localForward).Normalized();

	// A. Now we have an orthonormal linear basis { localRight, localUp, localForward } for the object local space.

	// Generate the third basis vector for the world space.
	float3 worldRight = worldUp.Cross(targetDirection).Normalized();
	// Since the input worldUp vector is not necessarily perpendicular to the targetDirection vector,
	// we need to compute the real world space up vector that the "head" of the object will point
	// towards when the model is looking towards the desired target direction.
	float3 perpWorldUp = targetDirection.Cross(worldRight).Normalized();
	
	// B. Now we have an orthonormal linear basis { worldRight, perpWorldUp, targetDirection } for the desired target orientation.

	// We want to build a matrix M that performs the following mapping:
	// 1. localRight must be mapped to worldRight.        (M * localRight = worldRight)
	// 2. localUp must be mapped to perpWorldUp.          (M * localUp = perpWorldUp)
	// 3. localForward must be mapped to targetDirection. (M * localForward = targetDirection)
	// i.e. we want to map the basis A to basis B.

	// This matrix M exists, and it is an orthonormal rotation matrix with a determinant of +1, because
	// the bases A and B are orthonormal with the same handedness.

	// Below, use the notation that (a,b,c) is a 3x3 matrix with a as its first column, b second, and c third.
	
	// By algebraic manipulation, we can rewrite conditions 1, 2 and 3 in a matrix form:
	//        M * (localRight, localUp, localForward) = (worldRight, perpWorldUp, targetDirection)
	// or     M = (worldRight, perpWorldUp, targetDirection) * (localRight, localUp, localForward)^{-1}.
	// or     M = m1 * m2, where

	// m1 equals (worldRight, perpWorldUp, targetDirection):
	float3x3 m1(worldRight, perpWorldUp, targetDirection);

	// and m2 equals (localRight, localUp, localForward)^{-1}:
	float3x3 m2;
	m2.SetRow(0, localRight);
	m2.SetRow(1, localUp);
	m2.SetRow(2, localForward);
	// Above we used the shortcut that for an orthonormal matrix M, M^{-1} = M^T. So set the rows
	// and not the columns to directly produce the transpose, i.e. the inverse of (localRight, localUp, localForward).

	// Compute final M.
	m2 = m1 * m2;

	// And fix any numeric stability issues by re-orthonormalizing the result.
	m2.Orthonormalize(0, 1, 2);
	return m2;
}

float3x3 &float3x3::operator =(const Quat &rhs)
{
	SetRotatePart(rhs);
	return *this;
}

float3x3 &float3x3::operator =(const float3x3 &rhs)
{
	v[0][0] = rhs.v[0][0];
	v[0][1] = rhs.v[0][1];
	v[0][2] = rhs.v[0][2];

	v[1][0] = rhs.v[1][0];
	v[1][1] = rhs.v[1][1];
	v[1][2] = rhs.v[1][2];

	v[2][0] = rhs.v[2][0];
	v[2][1] = rhs.v[2][1];
	v[2][2] = rhs.v[2][2];

	return *this;
}

float float3x3::Determinant() const
{
	assume(IsFinite());
	const float a = v[0][0];
	const float b = v[0][1];
	const float c = v[0][2];
	const float d = v[1][0];
	const float e = v[1][1];
	const float f = v[1][2];
	const float g = v[2][0];
	const float h = v[2][1];
	const float i = v[2][2];

	/* a b c
	   d e f
	   g h i */

	// Perform a direct cofactor expansion:
	return a*(e*i - f*h) + b*(f*g - d*i) + c*(d*h - e*g);
}

float float3x3::DeterminantSymmetric() const
{
	assume(IsSymmetric());
	const float a = v[0][0];
	const float b = v[0][1];
	const float c = v[0][2];
	const float d = v[1][1];
	const float e = v[1][2];
	const float f = v[2][2];

	/* If the matrix is symmetric, it is of form
	   a b c
	   b d e
	   c e f
	*/

	// A direct cofactor expansion gives
	// det = a * (df - ee) -b * (bf - ce) + c * (be-dc)
	//     = adf - aee - bbf + bce + bce - ccd
	//     = adf - aee - bbf - ccd + 2*bce
	//     = a(df-ee) + b(2*ce - bf) - ccd

	return a * (d*f - e*e) + b * (2.f * c * e - b * f) - c*c*d;
}

/* ///@todo Enable when float2x2 is implemented.
#define SKIPNUM(val, skip) (val >= skip ? (val+1) : val)
float2x2 float3x3::SubMatrix(int i, int j) const
{
	int r0 = SKIPNUM(0, i);
	int r1 = SKIPNUM(1, i);
	int c0 = SKIPNUM(0, j);
	int c1 = SKIPNUM(1, j);

	return float2x2(v[r0][c0], v[r0][c1],
	                v[r1][c0], v[r1][c1]);
}

float float3x3::Minor(int i, int j) const
{
	int r0 = SKIPNUM(0, i);
	int r1 = SKIPNUM(1, i);
	int c0 = SKIPNUM(0, j);
	int c1 = SKIPNUM(1, j);

	return v[r0][c0] * v[r1][c1] - v[r0][c1] * v[r1][c0];
}

float3x3 float3x3::Adjugate() const
{
	float3x3 a;
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			a[y][x] = (((x+y) & 1) != 0) ? -Minor(y, x) : Minor(y, x);

	return a;
}
*/

bool float3x3::Inverse(float epsilon)
{
	// There exists a generic matrix inverse calculator that uses Gaussian elimination.
	// It would be invoked by calling
	// return InverseMatrix(*this, epsilon);

	float3x3 i = *this;
	bool success = InverseMatrix(i, epsilon);
	if (!success)
		return false;

	*this = i;
	return true;
}

bool float3x3::InverseFast(float epsilon)
{
#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 orig = *this;
#endif

	// Compute the inverse directly using Cramer's rule.
	// Warning: This method is numerically very unstable!
	float d = Determinant();
	if (EqualAbs(d, 0.f, epsilon))
		return false;

	d = 1.f / d;
	float3x3 i;
	i[0][0] = d * (At(1, 1) * At(2, 2) - At(1, 2) * At(2, 1));
	i[0][1] = d * (At(0, 2) * At(2, 1) - At(0, 1) * At(2, 2));
	i[0][2] = d * (At(0, 1) * At(1, 2) - At(0, 2) * At(1, 1));

	i[1][0] = d * (At(1, 2) * At(2, 0) - At(1, 0) * At(2, 2));
	i[1][1] = d * (At(0, 0) * At(2, 2) - At(0, 2) * At(2, 0));
	i[1][2] = d * (At(0, 2) * At(1, 0) - At(0, 0) * At(1, 2));

	i[2][0] = d * (At(1, 0) * At(2, 1) - At(1, 1) * At(2, 0));
	i[2][1] = d * (At(2, 0) * At(0, 1) - At(0, 0) * At(2, 1));
	i[2][2] = d * (At(0, 0) * At(1, 1) - At(0, 1) * At(1, 0));

#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 id = orig * i;
	float3x3 id2 = i * orig;
	mathassert(id.IsIdentity(0.5f));
	mathassert(id2.IsIdentity(0.5f));
#endif

	*this = i;
	return true;
}

bool float3x3::SolveAxb(float3 b, float3 &x) const
{
	// Solve by pivotization.
	float v00 = At(0, 0);
	float v10 = At(1, 0);
	float v20 = At(2, 0);

	float v01 = At(0, 1);
	float v11 = At(1, 1);
	float v21 = At(2, 1);

	float v02 = At(0, 2);
	float v12 = At(1, 2);
	float v22 = At(2, 2);

	float av00 = Abs(v00);
	float av10 = Abs(v10);
	float av20 = Abs(v20);

	// Find which item in first column has largest absolute value.
	if (av10 >= av00 && av10 >= av20)
	{
		Swap(v00, v10);
		Swap(v01, v11);
		Swap(v02, v12);
		Swap(b[0], b[1]);
	}
	else if (av20 >= av00)
	{
		Swap(v00, v20);
		Swap(v01, v21);
		Swap(v02, v22);
		Swap(b[0], b[2]);
	}

	/* a b c | x
	   d e f | y
	   g h i | z , where |a| >= |d| && |a| >= |g| */

	if (EqualAbs(v00, 0.f))
		return false;

	// Scale row so that leading element is one.
	float denom = 1.f / v00;
//	v00 = 1.f;
	v01 *= denom;
	v02 *= denom;
	b[0] *= denom;

	/* 1 b c | x
	   d e f | y
	   g h i | z */

	// Zero first column of second and third rows.
	v11 -= v10 * v01;
	v12 -= v10 * v02;
	b[1] -= v10 * b[0];

	v21 -= v20 * v01;
	v22 -= v20 * v02;
	b[2] -= v20 * b[0];

	/* 1 b c | x
	   0 e f | y
	   0 h i | z */

	// Pivotize again.
	if (Abs(v21) > Abs(v11))
	{
		Swap(v11, v21);
		Swap(v12, v22);
		Swap(b[1], b[2]);
	}

	if (EqualAbs(v11, 0.f))
		return false;

	/* 1 b c | x
	   0 e f | y
	   0 h i | z, where |e| >= |h| */

	denom = 1.f / v11;
//	v11 = 1.f;
	v12 *= denom;
	b[1] *= denom;

	/* 1 b c | x
	   0 1 f | y
	   0 h i | z */

	v22 -= v21 * v12;
	b[2] -= v21 * b[1];

	/* 1 b c | x
	   0 1 f | y
	   0 0 i | z */

	if (EqualAbs(v22, 0.f))
		return false;

	x[2] = b[2] / v22;
	x[1] = b[1] - x[2] * v12;
	x[0] = b[0] - x[2] * v02 - x[1] * v01;

	return true;
}

float3x3 float3x3::Inverted() const
{
	float3x3 copy = *this;
	copy.Inverse();
	return copy;
}

bool float3x3::InverseColOrthogonal()
{
#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 orig = *this;
#endif
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

	mathassert(!orig.IsInvertible()|| (orig * *this).IsIdentity());
	mathassert(IsRowOrthogonal());

	return true;
}

bool float3x3::InverseOrthogonalUniformScale()
{
#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 orig = *this;
#endif
	assume(IsColOrthogonal());
	assume(HasUniformScale());
	float scale = float3(At(0, 0), At(1, 0), At(2, 0)).LengthSq();
	if (scale < 1e-8f)
		return false;
	scale = 1.f / scale;
	Swap(At(0, 1), At(1, 0));
	Swap(At(0, 2), At(2, 0));
	Swap(At(1, 2), At(2, 1));

	At(0, 0) *= scale; At(0, 1) *= scale; At(0, 2) *= scale;
	At(1, 0) *= scale; At(1, 1) *= scale; At(1, 2) *= scale;
	At(2, 0) *= scale; At(2, 1) *= scale; At(2, 2) *= scale;

	assume(IsFinite());
	assume(IsColOrthogonal());
	assume(HasUniformScale());

	mathassert(!orig.IsInvertible()|| (orig * *this).IsIdentity());

	return true;
}

void float3x3::InverseOrthonormal()
{
#ifdef MATH_ASSERT_CORRECTNESS
	float3x3 orig = *this;
#endif
	assume(IsOrthonormal());
	Transpose();
	mathassert(!orig.IsInvertible()|| (orig * *this).IsIdentity());
}

bool float3x3::InverseSymmetric()
{
	assume(IsSymmetric());
	const float a = v[0][0];
	const float b = v[0][1];
	const float c = v[0][2];
	const float d = v[1][1];
	const float e = v[1][2];
	const float f = v[2][2];

	/* If the matrix is symmetric, it is of form
	   a b c
	   b d e
	   c e f
	*/

	// A direct cofactor expansion gives
	// det = a * (df - ee) + b * (ce - bf) + c * (be-dc)

	const float df_ee = d*f - e*e;
	const float ce_bf = c*e - b*f;
	const float be_dc = b*e - d*c;

	float det = a * df_ee + b * ce_bf + c * be_dc; // = DeterminantSymmetric();
	if (EqualAbs(det, 0.f))
		return false;
	det = 1.f / det;
	
	// The inverse of a symmetric matrix will also be symmetric, so can avoid some computations altogether.

	v[0][0] = det * df_ee;
	v[1][0] = v[0][1] = det * ce_bf;
	v[0][2] = v[2][0] = det * be_dc;
	v[1][1] = det * (a*f - c*c);
	v[1][2] = v[2][1] = det * (c*b - a*e);
	v[2][2] = det * (a*d - b*b);

	return true;
}

void float3x3::Transpose()
{
	Swap(v[0][1], v[1][0]);
	Swap(v[0][2], v[2][0]);
	Swap(v[1][2], v[2][1]);
}

float3x3 float3x3::Transposed() const
{
	float3x3 copy = *this;
	copy.Transpose();
	return copy;
}

bool float3x3::InverseTranspose()
{
	bool success = Inverse();
	Transpose();
	return success;
}

float3x3 float3x3::InverseTransposed() const
{
	float3x3 copy = *this;
	copy.Transpose();
	copy.Inverse();
	return copy;
}

float float3x3::Trace() const
{
	return v[0][0] + v[1][1] + v[2][2];
}

void float3x3::Orthonormalize(int c0, int c1, int c2)
{
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

void float3x3::RemoveScale()
{
	assume(IsFinite());
#ifdef MATH_COLMAJOR_MATRICES
	float3 row0 = Row(0);
	float3 row1 = Row(1);
	float3 row2 = Row(2);
	float x = row0.Normalize();
	float y = row1.Normalize();
	float z = row2.Normalize();
#else
	float x = Row(0).Normalize();
	float y = Row(1).Normalize();
	float z = Row(2).Normalize();
#endif
	assume(x != 0 && y != 0 && z != 0 && "float3x3::RemoveScale failed!");
	MARK_UNUSED(x);
	MARK_UNUSED(y);
	MARK_UNUSED(z);
}

float2 float3x3::Transform(const float2 &vector) const
{
	return Transform(vector.x, vector.y);
}

float3 float3x3::Transform(const float3 &vector) const
{
	return Transform(vector.x, vector.y, vector.z);
}

float3 float3x3::TransformLeft(const float3 &vector) const
{
	return float3(vector[0] * At(0, 0) + vector[1] * At(1, 0) + vector[2] * At(2, 0),
				  vector[0] * At(0, 1) + vector[1] * At(1, 1) + vector[2] * At(2, 1),
				  vector[0] * At(0, 2) + vector[1] * At(1, 2) + vector[2] * At(2, 2));
}

float2 float3x3::Transform(float x, float y) const
{
	return float2(At(0, 0) * x + At(0, 1) * y,
	              At(1, 0) * x + At(1, 1) * y);
}

float3 float3x3::Transform(float x, float y, float z) const
{
	return float3(At(0, 0) * x + At(0, 1) * y + At(0, 2) * z,
	              At(1, 0) * x + At(1, 1) * y + At(1, 2) * z,
				  At(2, 0) * x + At(2, 1) * y + At(2, 2) * z);
}

float4 float3x3::Transform(const float4 &vector) const
{
	return float4(At(0, 0) * vector.x + At(0, 1) * vector.y + At(0, 2) * vector.z,
				  At(1, 0) * vector.x + At(1, 1) * vector.y + At(1, 2) * vector.z,
				  At(2, 0) * vector.x + At(2, 1) * vector.y + At(2, 2) * vector.z,
				  vector.w);
}

void float3x3::BatchTransform(float3 *pointArray, int numPoints) const
{
	assume(pointArray || numPoints == 0);
	for(int i = 0; i < numPoints; ++i)
		pointArray[i] = *this * pointArray[i];
}

void float3x3::BatchTransform(float3 *pointArray, int numPoints, int stride) const
{
	assume(pointArray || numPoints == 0);
	assume(stride >= (int)sizeof(float3));
	u8 *data = reinterpret_cast<u8*>(pointArray);
	for(int i = 0; i < numPoints; ++i)
	{
		float3 *vtx = reinterpret_cast<float3*>(data + stride*i);
		*vtx = *this * *vtx;
	}
}

void float3x3::BatchTransform(float4 *vectorArray, int numVectors) const
{
	assume(vectorArray || numVectors == 0);
	for(int i = 0; i < numVectors; ++i)
		vectorArray[i] = *this * vectorArray[i];
}

void float3x3::BatchTransform(float4 *vectorArray, int numVectors, int stride) const
{
	assume(vectorArray || numVectors == 0);
	assume(stride >= (int)sizeof(float4));
	u8 *data = reinterpret_cast<u8*>(vectorArray);
	for(int i = 0; i < numVectors; ++i)
	{
		float4 *vtx = reinterpret_cast<float4*>(data + stride*i);
		*vtx = *this * *vtx;
	}
}

float3x3 float3x3::operator *(const float3x3 &rhs) const
{
	assume(IsFinite());
	assume(rhs.IsFinite());
	float3x3 r;
	r[0][0] = At(0, 0) * rhs.At(0, 0) + At(0, 1) * rhs.At(1, 0) + At(0, 2) * rhs.At(2, 0);
	r[0][1] = At(0, 0) * rhs.At(0, 1) + At(0, 1) * rhs.At(1, 1) + At(0, 2) * rhs.At(2, 1);
	r[0][2] = At(0, 0) * rhs.At(0, 2) + At(0, 1) * rhs.At(1, 2) + At(0, 2) * rhs.At(2, 2);

	r[1][0] = At(1, 0) * rhs.At(0, 0) + At(1, 1) * rhs.At(1, 0) + At(1, 2) * rhs.At(2, 0);
	r[1][1] = At(1, 0) * rhs.At(0, 1) + At(1, 1) * rhs.At(1, 1) + At(1, 2) * rhs.At(2, 1);
	r[1][2] = At(1, 0) * rhs.At(0, 2) + At(1, 1) * rhs.At(1, 2) + At(1, 2) * rhs.At(2, 2);

	r[2][0] = At(2, 0) * rhs.At(0, 0) + At(2, 1) * rhs.At(1, 0) + At(2, 2) * rhs.At(2, 0);
	r[2][1] = At(2, 0) * rhs.At(0, 1) + At(2, 1) * rhs.At(1, 1) + At(2, 2) * rhs.At(2, 1);
	r[2][2] = At(2, 0) * rhs.At(0, 2) + At(2, 1) * rhs.At(1, 2) + At(2, 2) * rhs.At(2, 2);

	return r;
}

float3x3 float3x3::operator *(const Quat &rhs) const
{
	return *this * float3x3(rhs);
}

float3 float3x3::operator *(const float3 &rhs) const
{
	return float3(At(0, 0) * rhs.x + At(0, 1) * rhs.y + At(0, 2) * rhs.z,
				  At(1, 0) * rhs.x + At(1, 1) * rhs.y + At(1, 2) * rhs.z,
				  At(2, 0) * rhs.x + At(2, 1) * rhs.y + At(2, 2) * rhs.z);
}

float4 float3x3::operator *(const float4 &rhs) const
{
	return float4(At(0, 0) * rhs.x + At(0, 1) * rhs.y + At(0, 2) * rhs.z,
				  At(1, 0) * rhs.x + At(1, 1) * rhs.y + At(1, 2) * rhs.z,
				  At(2, 0) * rhs.x + At(2, 1) * rhs.y + At(2, 2) * rhs.z,
				  rhs.w);
}

float3x3 float3x3::operator *(float scalar) const
{
	float3x3 r = *this;
	r *= scalar;
	return r;
}

float3x3 float3x3::operator /(float scalar) const
{
	float3x3 r = *this;
	r /= scalar;
	return r;
}

float3x3 float3x3::operator +(const float3x3 &rhs) const
{
	float3x3 r = *this;
	r += rhs;
	return r;
}

float3x3 float3x3::operator -(const float3x3 &rhs) const
{
	float3x3 r = *this;
	r -= rhs;
	return r;
}

float3x3 float3x3::operator -() const
{
	float3x3 r;
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			r[y][x] = -At(y, x);
	return r;
}

float3x3 &float3x3::operator *=(float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			v[y][x] *= scalar;

	return *this;
}

float3x3 &float3x3::operator /=(float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	assume(!EqualAbs(scalar, 0));
	float invScalar = 1.f / scalar;
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			v[y][x] *= invScalar;

	return *this;
}

float3x3 &float3x3::operator +=(const float3x3 &rhs)
{
	assume(rhs.IsFinite());
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			At(y, x) += rhs[y][x];

	return *this;
}

float3x3 &float3x3::operator -=(const float3x3 &rhs)
{
	assume(rhs.IsFinite());
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			At(y, x) -= rhs[y][x];

	return *this;
}

bool float3x3::IsFinite() const
{
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			if (!MATH_NS::IsFinite(v[y][x]))
				return false;
	return true;
}

bool float3x3::IsIdentity(float epsilon) const
{
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			if (!EqualAbs(v[y][x], (x == y) ? 1.f : 0.f, epsilon))
				return false;

	return true;
}

bool float3x3::IsLowerTriangular(float epsilon) const
{
	return EqualAbs(v[0][1], 0.f, epsilon)
	    && EqualAbs(v[0][2], 0.f, epsilon)
	    && EqualAbs(v[1][2], 0.f, epsilon);
}

bool float3x3::IsUpperTriangular(float epsilon) const
{
	return EqualAbs(v[1][0], 0.f, epsilon)
	    && EqualAbs(v[2][0], 0.f, epsilon)
	    && EqualAbs(v[2][1], 0.f, epsilon);
}

bool float3x3::IsInvertible(float epsilon) const
{
	float d = Determinant();
	bool isSingular = EqualAbs(d, 0.f, epsilon);
	mathassert(float3x3(*this).Inverse(epsilon) != isSingular); // IsInvertible() and Inverse() must match!
	return !isSingular;
}

bool float3x3::IsSymmetric(float epsilon) const
{
	return EqualAbs(v[0][1], v[1][0], epsilon) &&
		EqualAbs(v[0][2], v[2][0], epsilon) &&
		EqualAbs(v[1][2], v[2][1], epsilon);
}

bool float3x3::IsSkewSymmetric(float epsilon) const
{
	return EqualAbs(v[0][0], 0.f, epsilon) &&
		EqualAbs(v[1][1], 0.f, epsilon) &&
		EqualAbs(v[2][2], 0.f, epsilon) &&
		EqualAbs(v[0][1], -v[1][0], epsilon) &&
		EqualAbs(v[0][2], -v[2][0], epsilon) &&
		EqualAbs(v[1][2], -v[2][1], epsilon);
}

bool float3x3::HasUnitaryScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return scale.Equals(1.f, 1.f, 1.f, epsilon);
}

bool float3x3::HasNegativeScale() const
{
	return Determinant() < 0.f;
}

bool float3x3::HasUniformScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return EqualAbs(scale.x, scale.y, epsilon) && EqualAbs(scale.x, scale.z, epsilon);
}

bool float3x3::IsRowOrthogonal(float epsilon) const
{
	return Row(0).IsPerpendicular(Row(1), epsilon)
	    && Row(0).IsPerpendicular(Row(2), epsilon)
	    && Row(1).IsPerpendicular(Row(2), epsilon);
}

bool float3x3::IsColOrthogonal(float epsilon) const
{
	return Col(0).IsPerpendicular(Col(1), epsilon)
	    && Col(0).IsPerpendicular(Col(2), epsilon)
	    && Col(1).IsPerpendicular(Col(2), epsilon);
}

bool float3x3::IsOrthonormal(float epsilon) const
{
	///@todo Epsilon magnitudes don't match.
	return IsColOrthogonal(epsilon) && Row(0).IsNormalized(epsilon) && Row(1).IsNormalized(epsilon) && Row(2).IsNormalized(epsilon);
}

bool float3x3::Equals(const float3x3 &other, float epsilon) const
{
	for(int y = 0; y < Rows; ++y)
		for(int x = 0; x < Cols; ++x)
			if (!EqualAbs(At(y, x), other[y][x], epsilon))
				return false;
	return true;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float3x3::ToString() const
{
	char str[256];
	sprintf(str, "(%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)",
		At(0, 0), At(0, 1), At(0, 2),
		At(1, 0), At(1, 1), At(1, 2),
		At(2, 0), At(2, 1), At(2, 2));

	return str;
}

StringT float3x3::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(At(0, 0), str); *s = ','; ++s;
	s = SerializeFloat(At(0, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(0, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 0), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(1, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 0), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(2, 2), s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT float3x3::ToString2() const
{
	char str[256];
	sprintf(str, "float3x3(X:(%.2f,%.2f,%.2f) Y:(%.2f,%.2f,%.2f) Z:(%.2f,%.2f,%.2f)",
		At(0, 0), At(1, 0), At(2, 0),
		At(0, 1), At(1, 1), At(2, 1),
		At(0, 2), At(1, 2), At(2, 2));

	return str;
}
#endif

float3 float3x3::ToEulerXYX() const { float3 f; ExtractEulerXYX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXZX() const { float3 f; ExtractEulerXZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYXY() const { float3 f; ExtractEulerYXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYZY() const { float3 f; ExtractEulerYZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZXZ() const { float3 f; ExtractEulerZXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZYZ() const { float3 f; ExtractEulerZYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXYZ() const { float3 f; ExtractEulerXYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXZY() const { float3 f; ExtractEulerXZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYXZ() const { float3 f; ExtractEulerYXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYZX() const { float3 f; ExtractEulerYZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZXY() const { float3 f; ExtractEulerZXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZYX() const { float3 f; ExtractEulerZYX(*this, f[0], f[1], f[2]); return f; }

float3 float3x3::ExtractScale() const
{
	return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

void float3x3::Decompose(Quat &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal());

	float3x3 r;
	Decompose(r, scale);
	rotate = Quat(r);

	// Test that composing back yields the original float3x3.
	assume(float3x3::FromRS(rotate, scale).Equals(*this, 0.1f));
}

void float3x3::Decompose(float3x3 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal());

	rotate = *this;
	scale.x = rotate.Col(0).Length();
	scale.y = rotate.Col(1).Length();
	scale.z = rotate.Col(2).Length();
	assume(!EqualAbs(scale.x, 0));
	assume(!EqualAbs(scale.y, 0));
	assume(!EqualAbs(scale.z, 0));
	rotate.ScaleCol(0, 1.f / scale.x);
	rotate.ScaleCol(1, 1.f / scale.y);
	rotate.ScaleCol(2, 1.f / scale.z);

	// Test that composing back yields the original float3x3.
	assume(float3x3::FromRS(rotate, scale).Equals(*this, 0.1f));
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3x3 &rhs)
{
	out << rhs.ToString();
	return out;
}
#endif

float3x3 float3x3::Mul(const float3x3 &rhs) const { return *this * rhs; }
float3x4 float3x3::Mul(const float3x4 &rhs) const { return *this * rhs; }
float4x4 float3x3::Mul(const float4x4 &rhs) const { return *this * rhs; }
float3x3 float3x3::Mul(const Quat &rhs) const { return *this * rhs; }
float2 float3x3::Mul(const float2 &rhs) const { return Transform(rhs.x, rhs.y); }
float3 float3x3::Mul(const float3 &rhs) const { return *this * rhs; }

float3x3 operator *(const Quat &lhs, const float3x3 &rhs)
{
	float3x3 lhsRot(lhs);
	return lhsRot * rhs;
}

float3 operator *(const float3 &lhs, const float3x3 &rhs)
{
	return rhs.TransformLeft(lhs);
}

float4 operator *(const float4 &lhs, const float3x3 &rhs)
{
	assume(lhs.IsWZeroOrOne());
	return float4(rhs.TransformLeft(lhs.xyz()), lhs.w);
}

const float3x3 float3x3::zero	 = float3x3(0,0,0, 0,0,0, 0,0,0);
const float3x3 float3x3::identity = float3x3(1,0,0, 0,1,0, 0,0,1);
const float3x3 float3x3::nan = float3x3(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);

MATH_END_NAMESPACE
