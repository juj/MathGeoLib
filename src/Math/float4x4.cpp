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

/** @file float4x4.cpp
	@author Jukka Jylänki
	@brief */
#include "float4x4.h"
#include <string.h>

#include "MathFunc.h"
#include "Swap.h"
#include "float3.h"
#include "float4.h"
#include "float3x3.h"
#include "float3x4.h"
#include "Matrix.inl"
#include "Quat.h"
#include "TransformOps.h"
#include "../Geometry/Plane.h"
#include "../Algorithm/Random/LCG.h"
#include "SSEMath.h"
#include "float4x4_sse.h"
#include "float4x4_neon.h"
#include "quat_simd.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

float4x4::float4x4(float _00, float _01, float _02, float _03,
                   float _10, float _11, float _12, float _13,
                   float _20, float _21, float _22, float _23,
                   float _30, float _31, float _32, float _33)
{
	Set(_00, _01, _02, _03,
	    _10, _11, _12, _13,
	    _20, _21, _22, _23,
	    _30, _31, _32, _33);
}

float4x4::float4x4(const float3x3 &m)
{
#ifdef MATH_AUTOMATIC_SSE
	row[0] = load_vec3(m.ptr(), 0.f);
	row[1] = load_vec3(m.ptr() + 3, 0.f);
	row[2] = load_vec3(m.ptr() + 6, 0.f);
	row[3] = set_ps(1.f, 0.f, 0.f, 0.f);
#else
	Set(m.At(0,0), m.At(0,1), m.At(0,2), 0.f,
	    m.At(1,0), m.At(1,1), m.At(1,2), 0.f,
	    m.At(2,0), m.At(2,1), m.At(2,2), 0.f,
	          0.f,       0.f,       0.f, 1.f);
#endif
}

float4x4::float4x4(const float3x4 &m)
{
#ifdef MATH_AUTOMATIC_SSE
	row[0] = m.row[0];
	row[1] = m.row[1];
	row[2] = m.row[2];
	row[3] = set_ps(1.f, 0.f, 0.f, 0.f);
#else
	Set(m.At(0,0), m.At(0,1), m.At(0,2), m.At(0,3),
	    m.At(1,0), m.At(1,1), m.At(1,2), m.At(1,3),
	    m.At(2,0), m.At(2,1), m.At(2,2), m.At(2,3),
	          0.f,       0.f,       0.f,     1.f);
#endif
}

float4x4::float4x4(const float4 &col0, const float4 &col1, const float4 &col2, const float4 &col3)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	__m128 tmp0 = _mm_unpacklo_ps(col0.v, col1.v);
	__m128 tmp2 = _mm_unpacklo_ps(col2.v, col3.v);
	__m128 tmp1 = _mm_unpackhi_ps(col0.v, col1.v);
	__m128 tmp3 = _mm_unpackhi_ps(col2.v, col3.v);
	row[0] = _mm_movelh_ps(tmp0, tmp2);
	row[1] = _mm_movehl_ps(tmp2, tmp0);
	row[2] = _mm_movelh_ps(tmp1, tmp3);
	row[3] = _mm_movehl_ps(tmp3, tmp1);
#else
	SetCol(0, col0);
	SetCol(1, col1);
	SetCol(2, col2);
	SetCol(3, col3);
#endif
}

float4x4::float4x4(const Quat &orientation)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	quat_to_mat4x4(orientation.q, set_ps(1, 0, 0, 0), row);
#else
	SetRotatePart(orientation);
	SetRow(3, 0, 0, 0, 1);
	SetCol3(3, 0, 0, 0);
#endif
}

float4x4::float4x4(const Quat &orientation, const float3 &translation)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	quat_to_mat4x4(orientation.q, float4(translation, 1.f), row);
#else
	SetRotatePart(orientation);
	SetTranslatePart(translation);
	SetRow(3, 0, 0, 0, 1);
#endif
}

TranslateOp float4x4::Translate(float tx, float ty, float tz)
{
	return TranslateOp(tx, ty, tz);
}

TranslateOp float4x4::Translate(const float3 &offset)
{
	return TranslateOp(offset);
}

float4x4 float4x4::RotateX(float angleRadians)
{
	float4x4 r;
	r.SetRotatePartX(angleRadians);
	r.SetRow(3, 0, 0, 0, 1);
	r.SetCol3(3, 0, 0, 0);
	return r;
}

float4x4 float4x4::RotateX(float angleRadians, const float3 &pointOnAxis)
{
	return float4x4::Translate(pointOnAxis) * RotateX(angleRadians) * float4x4::Translate(-pointOnAxis);
}

float4x4 float4x4::RotateY(float angleRadians)
{
	float4x4 r;
	r.SetRotatePartY(angleRadians);
	r.SetRow(3, 0, 0, 0, 1);
	r.SetCol3(3, 0, 0, 0);
	return r;
}

float4x4 float4x4::RotateY(float angleRadians, const float3 &pointOnAxis)
{
	return float4x4::Translate(pointOnAxis) * RotateY(angleRadians) * float4x4::Translate(-pointOnAxis);
}

float4x4 float4x4::RotateZ(float angleRadians)
{
	float4x4 r;
	r.SetRotatePartZ(angleRadians);
	r.SetRow(3, 0, 0, 0, 1);
	r.SetCol3(3, 0, 0, 0);
	return r;
}

float4x4 float4x4::RotateZ(float angleRadians, const float3 &pointOnAxis)
{
	return float4x4::Translate(pointOnAxis) * RotateZ(angleRadians) * float4x4::Translate(-pointOnAxis);
}

float4x4 float4x4::RotateAxisAngle(const float3 &axisDirection, float angleRadians)
{
	float4x4 r;
	r.SetRotatePart(axisDirection, angleRadians);
	r.SetRow(3, 0, 0, 0, 1);
	r.SetCol3(3, 0, 0, 0);
	return r;
}

float4x4 float4x4::RotateAxisAngle(const float3 &axisDirection, float angleRadians, const float3 &pointOnAxis)
{
	return float4x4::Translate(pointOnAxis) * RotateAxisAngle(axisDirection, angleRadians) * float4x4::Translate(-pointOnAxis);
}

float4x4 float4x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection)
{
	return Quat::RotateFromTo(sourceDirection, targetDirection).ToFloat4x4();
}

float4x4 float4x4::RotateFromTo(const float4 &sourceDirection, const float4 &targetDirection)
{
	return float4x4(float3x3::RotateFromTo(sourceDirection.Float3Part(), targetDirection.Float3Part()));
	//return Quat::RotateFromTo(sourceDirection, targetDirection).ToFloat4x4();
}

float4x4 float4x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection, const float3 &centerPoint)
{
	return float4x4::Translate(centerPoint) * RotateFromTo(sourceDirection, targetDirection) * float4x4::Translate(-centerPoint);
}

float4x4 float4x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection,
	const float3 &sourceDirection2, const float3 &targetDirection2)
{
	return LookAt(sourceDirection, targetDirection, sourceDirection2, targetDirection2);
}

float4x4 float4x4::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection,
	const float3 &sourceDirection2, const float3 &targetDirection2, const float3 &centerPoint)
{
	return float4x4::Translate(centerPoint) * RotateFromTo(sourceDirection, targetDirection, sourceDirection2, targetDirection2) * float4x4::Translate(-centerPoint);
}

float4x4 float4x4::RandomGeneral(LCG &lcg, float minElem, float maxElem)
{
	assume(MATH_NS::IsFinite(minElem));
	assume(MATH_NS::IsFinite(maxElem));
	float4x4 m;
	for(int y = 0; y < 4; ++y)
		for(int x = 0; x < 4; ++x)
			m[y][x] = lcg.Float(minElem, maxElem);
	return m;
}

float4x4 float4x4::FromQuat(const Quat &orientation)
{
	return float4x4(orientation);
}

float4x4 float4x4::FromQuat(const Quat &orientation, const float3 &pointOnAxis)
{
	return float4x4::Translate(pointOnAxis) * float4x4(orientation) * float4x4::Translate(-pointOnAxis);
}

float4x4 float4x4::FromTRS(const float3 &translate, const Quat &rotate, const float3 &scale)
{
	return float4x4::Translate(translate) * float4x4(rotate) * float4x4::Scale(scale);
}

float4x4 float4x4::FromTRS(const float3 &translate, const float3x3 &rotate, const float3 &scale)
{
	return float4x4::Translate(translate) * float4x4(rotate) * float4x4::Scale(scale);
}

float4x4 float4x4::FromTRS(const float3 &translate, const float3x4 &rotate, const float3 &scale)
{
	return float4x4::Translate(translate) * float4x4(rotate) * float4x4::Scale(scale);
}

float4x4 float4x4::FromTRS(const float3 &translate, const float4x4 &rotate, const float3 &scale)
{
	return float4x4::Translate(translate) * rotate * float4x4::Scale(scale);
}

float4x4 float4x4::FromEulerXYX(float x2, float y, float x)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerXYX(r, x2, y, x);
	assume(r.Equals(float4x4::RotateX(x2) * float4x4::RotateY(y) * float4x4::RotateX(x)));
	return r;
}

float4x4 float4x4::FromEulerXZX(float x2, float z, float x)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerXZX(r, x2, z, x);
	assume(r.Equals(float4x4::RotateX(x2) * float4x4::RotateZ(z) * float4x4::RotateX(x)));
	return r;
}

float4x4 float4x4::FromEulerYXY(float y2, float x, float y)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerYXY(r, y2, x, y);
	assume(r.Equals(float4x4::RotateY(y2) * float4x4::RotateX(x) * float4x4::RotateY(y)));
	return r;
}

float4x4 float4x4::FromEulerYZY(float y2, float z, float y)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerYZY(r, y2, z, y);
	assume(r.Equals(float4x4::RotateY(y2) * float4x4::RotateZ(z) * float4x4::RotateY(y)));
	return r;
}

float4x4 float4x4::FromEulerZXZ(float z2, float x, float z)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerZXZ(r, z2, x, z);
	assume(r.Equals(float4x4::RotateZ(z2) * float4x4::RotateX(x) * float4x4::RotateZ(z)));
	return r;
}

float4x4 float4x4::FromEulerZYZ(float z2, float y, float z)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerZYZ(r, z2, y, z);
	assume(r.Equals(float4x4::RotateZ(z2) * float4x4::RotateY(y) * float4x4::RotateZ(z)));
	return r;
}

float4x4 float4x4::FromEulerXYZ(float x, float y, float z)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerXYZ(r, x, y, z);
	assume(r.Equals(float4x4::RotateX(x) * float4x4::RotateY(y) * float4x4::RotateZ(z)));
	return r;
}

float4x4 float4x4::FromEulerXZY(float x, float z, float y)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerXZY(r, x, z, y);
	assume(r.Equals(float4x4::RotateX(x) * float4x4::RotateZ(z) * float4x4::RotateY(y)));
	return r;
}

float4x4 float4x4::FromEulerYXZ(float y, float x, float z)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerYXZ(r, y, x, z);
	assume(r.Equals(float4x4::RotateY(y) * float4x4::RotateX(x) * float4x4::RotateZ(z)));
	return r;
}

float4x4 float4x4::FromEulerYZX(float y, float z, float x)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerYZX(r, y, z, x);
	assume(r.Equals(float4x4::RotateY(y) * float4x4::RotateZ(z) * float4x4::RotateX(x)));
	return r;
}

float4x4 float4x4::FromEulerZXY(float z, float x, float y)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerZXY(r, z, x, y);
	assume(r.Equals(float4x4::RotateZ(z) * float4x4::RotateX(x) * float4x4::RotateY(y)));
	return r;
}

float4x4 float4x4::FromEulerZYX(float z, float y, float x)
{
	float4x4 r;
	r.SetTranslatePart(0,0,0);
	r.SetRow(3, 0,0,0,1);
	Set3x3PartRotateEulerZYX(r, z, y, x);
	assume(r.Equals(float4x4::RotateZ(z) * float4x4::RotateY(y) * float4x4::RotateX(x)));
	return r;
}

ScaleOp float4x4::Scale(float sx, float sy, float sz)
{
	return ScaleOp(sx, sy, sz);
}

ScaleOp float4x4::Scale(const float3 &scale)
{
	return ScaleOp(scale);
}

float4x4 float4x4::Scale(const float3 &scale, const float3 &scaleCenter)
{
	return Translate(scaleCenter) * Scale(scale) * Translate(-scaleCenter);
}

float4x4 float4x4::ScaleAlongAxis(const float3 &axis, float scalingFactor)
{
	return Scale(axis * scalingFactor);
}

float4x4 float4x4::ScaleAlongAxis(const float3 &axis, float scalingFactor, const float3 &scaleCenter)
{
	return Translate(scaleCenter) * Scale(axis * scalingFactor) * Translate(-scaleCenter);
}

ScaleOp float4x4::UniformScale(float uniformScale)
{
	return ScaleOp(uniformScale, uniformScale, uniformScale);
}

float4x4 float4x4::UniformScale(float uniformScale, const float3 &scaleCenter)
{
	return float4x4::Translate(scaleCenter) * float4x4::UniformScale(uniformScale) * float4x4::Translate(-scaleCenter);
}

float3 float4x4::GetScale() const
{
	return float3(Col3(0).Length(), Col3(1).Length(), Col3(2).Length());
}

float4x4 float4x4::ShearX(float yFactor, float zFactor)
{
	return float4x4(1.f, yFactor, zFactor, 0.f,
	                0.f,     1.f,     0.f, 0.f,
	                0.f,     0.f,     1.f, 0.f,
	                0.f,     0.f,     0.f, 1.f);
}

float4x4 float4x4::ShearY(float xFactor, float zFactor)
{
	return float4x4(    1.f, 0.f,     0.f, 0.f,
	                xFactor, 1.f, zFactor, 0.f,
	                    0.f, 0.f,     1.f, 0.f,
	                    0.f, 0.f,     0.f, 1.f);
}

float4x4 float4x4::ShearZ(float xFactor, float yFactor)
{
	return float4x4(    1.f,     0.f, 0.f, 0.f,
	                    0.f,     1.f, 0.f, 0.f,
	                xFactor, yFactor, 1.f, 0.f,
	                    0.f,     0.f, 0.f, 1.f);
}

float4x4 float4x4::Mirror(const Plane &p)
{
	float4x4 m;
	SetMatrix3x4AffinePlaneMirror(m, p.normal.x, p.normal.y, p.normal.z, p.d);
	m[3][0] = 0.f; m[3][1] = 0.f; m[3][2] = 0.f; m[3][3] = 1.f;
	return m;
}

float4x4 float4x4::D3DOrthoProjLH(float n, float f, float h, float v)
{
	float4x4 p;
	p[0][0] = 2.f / h; p[0][1] = 0;       p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;       p[1][1] = 2.f / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;       p[2][1] = 0;       p[2][2] = 1.f / (f-n); p[2][3] = n / (n-f);
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0.f;         p[3][3] = 1.f;

	return p;
}

/** This function generates an orthographic projection matrix that maps from
	the Direct3D view space to the Direct3D normalized viewport space as follows:

	In Direct3D view space, we assume that the camera is positioned at the origin (0,0,0).
	The camera looks directly towards the positive Z axis (0,0,1).
	The -X axis spans to the left of the screen, +X goes to the right.
	-Y goes to the bottom of the screen, +Y goes to the top.

	After the transformation, we're in the Direct3D normalized viewport space as follows:

	(-1,-1,0) is the bottom-left corner of the viewport at the near plane.
	(1,1,0) is the top-right corner of the viewport at the near plane.
	(0,0,0) is the center point at the near plane.
	Coordinates with z=1 are at the far plane.

	Examples:
		(0,0,n) maps to (0,0,0).
		(0,0,f) maps to (0,0,1).
		(-h/2, -v/2, n) maps to (-1, -1, 0).
		(h/2, v/2, f) maps to (1, 1, 1).
	*/
float4x4 float4x4::D3DOrthoProjRH(float n, float f, float h, float v)
{
	// D3DOrthoProjLH and D3DOrthoProjRH differ from each other in that the third column is negated.
	// This corresponds to LH = RH * In, where In is a diagonal matrix with elements [1 1 -1 1].

	float4x4 p;
	p[0][0] = 2.f / h; p[0][1] = 0;       p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;       p[1][1] = 2.f / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;       p[2][1] = 0;       p[2][2] = 1.f / (n-f); p[2][3] = n / (n-f);
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0.f;         p[3][3] = 1.f;

	return p;
}

float4x4 float4x4::D3DPerspProjLH(float n, float f, float h, float v)
{
	float4x4 p;
	p[0][0] = 2.f * n / h; p[0][1] = 0;           p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;           p[1][1] = 2.f * n / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;           p[2][1] = 0;           p[2][2] = f / (f-n);   p[2][3] = n * f / (n-f);
	p[3][0] = 0;           p[3][1] = 0;           p[3][2] = 1.f;         p[3][3] = 0.f;

	return p;
}

float4x4 float4x4::D3DPerspProjRH(float n, float f, float h, float v)
{
	// D3DPerspProjLH and D3DPerspProjRH differ from each other in that the third column is negated.
	// This corresponds to LH = RH * In, where In is a diagonal matrix with elements [1 1 -1 1].

	// In Direct3D, the post-perspective unit cube ranges in [-1, 1] in X and Y directions,
	// and in [0, 1] for the Z direction. See http://msdn.microsoft.com/en-us/library/windows/desktop/bb147302(v=vs.85).aspx
	float4x4 p;
	p[0][0] = 2.f * n / h; p[0][1] = 0;           p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;           p[1][1] = 2.f * n / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;           p[2][1] = 0;           p[2][2] = f / (n-f);   p[2][3] = n * f / (n-f);
	p[3][0] = 0;           p[3][1] = 0;           p[3][2] = -1.f;        p[3][3] = 0.f;

	return p;
}

float4x4 float4x4::OpenGLOrthoProjLH(float n, float f, float h, float v)
{
	/// Same as OpenGLOrthoProjRH, except that the camera looks towards +Z in view space, instead of -Z.
	float4x4 p;
	p[0][0] = 2.f / h; p[0][1] = 0;       p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;       p[1][1] = 2.f / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;       p[2][1] = 0;       p[2][2] = 2.f / (f-n); p[2][3] = (f+n) / (n-f);
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0;           p[3][3] = 1.f;

	return p;
}

float4x4 float4x4::OpenGLOrthoProjRH(float n, float f, float h, float v)
{
	float4x4 p;
	p[0][0] = 2.f / h; p[0][1] = 0;       p[0][2] = 0;           p[0][3] = 0.f;
	p[1][0] = 0;       p[1][1] = 2.f / v; p[1][2] = 0;           p[1][3] = 0.f;
	p[2][0] = 0;       p[2][1] = 0;       p[2][2] = 2.f / (n-f); p[2][3] = (f+n) / (n-f);
	p[3][0] = 0;       p[3][1] = 0;       p[3][2] = 0;           p[3][3] = 1.f;

	return p;
}

float4x4 float4x4::OpenGLPerspProjLH(float n, float f, float h, float v)
{
	// Same as OpenGLPerspProjRH, except that the camera looks towards +Z in view space, instead of -Z.
	float4x4 p;
	p[0][0] = 2.f *n / h;  p[0][1] = 0;           p[0][2] = 0;              p[0][3] = 0.f;
	p[1][0] = 0;           p[1][1] = 2.f * n / v; p[1][2] = 0;              p[1][3] = 0.f;
	p[2][0] = 0;           p[2][1] = 0;           p[2][2] = (n+f) / (f-n);  p[2][3] = 2.f*n*f / (n-f);
	p[3][0] = 0;           p[3][1] = 0;           p[3][2] = 1.f;            p[3][3] = 0.f;

	return p;
}

float4x4 float4x4::OpenGLPerspProjRH(float n, float f, float h, float v)
{
	// In OpenGL, the post-perspective unit cube ranges in [-1, 1] in all X, Y and Z directions.
	// See http://www.songho.ca/opengl/gl_projectionmatrix.html , unlike in Direct3D, where the
	// Z coordinate ranges in [0, 1]. This is the only difference between D3DPerspProjRH and OpenGLPerspProjRH.
	float4x4 p;
	p[0][0] = 2.f *n / h;  p[0][1] = 0;           p[0][2] = 0;              p[0][3] = 0.f;
	p[1][0] = 0;           p[1][1] = 2.f * n / v; p[1][2] = 0;              p[1][3] = 0.f;
	p[2][0] = 0;           p[2][1] = 0;           p[2][2] = (n+f) / (n-f);  p[2][3] = 2.f*n*f / (n-f);
	p[3][0] = 0;           p[3][1] = 0;           p[3][2] = -1.f;           p[3][3] = 0.f;

	return p;
}

float4x4 float4x4::OrthographicProjection(const Plane &p)
{
	float4x4 v;
	SetMatrix3x4AffinePlaneProject(v, p.normal.x, p.normal.y, p.normal.z, p.d);
	return v;
}

float4x4 float4x4::OrthographicProjectionYZ()
{
	float4x4 v = identity;
	v[0][0] = 0.f;
	return v;
}

float4x4 float4x4::OrthographicProjectionXZ()
{
	float4x4 v = identity;
	v[1][1] = 0.f;
	return v;
}

float4x4 float4x4::OrthographicProjectionXY()
{
	float4x4 v = identity;
	v[2][2] = 0.f;
	return v;
}

float4x4 float4x4::ComplementaryProjection() const
{
	assume(IsIdempotent());

	return float4x4::identity - *this;
}

float &float4x4::At(int rowIndex, int colIndex)
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

CONST_WIN32 float float4x4::At(int rowIndex, int colIndex) const
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
float4 &float4x4::Col(int col)
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<float4 &>(v[col]);
}

const float4 &float4x4::Col(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<const float4 &>(v[col]);
}

float3 &float4x4::Col3(int col)
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<float3 &>(v[col]);
}

const float3 &float4x4::Col3(int col) const
{
	assume(col >= 0);
	assume(col < Cols);
	return reinterpret_cast<const float3 &>(v[col]);
}

CONST_WIN32 float4 float4x4::Row(int row) const
{
	assume(row >= 0);
	assume(row < Rows);
	return float4(At(row, 0), At(row, 1), At(row, 2), At(row, 3));
}

CONST_WIN32 float3 float4x4::Row3(int row) const
{
	assume(row >= 0);
	assume(row < Rows);
	return float3(At(row, 0), At(row, 1), At(row, 2));
}
#else
float4 &float4x4::Row(int rowIndex)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	return reinterpret_cast<float4 &>(v[rowIndex]);
}

const float4 &float4x4::Row(int rowIndex) const
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	return reinterpret_cast<const float4 &>(v[rowIndex]);
}

float3 &float4x4::Row3(int rowIndex)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	return reinterpret_cast<float3 &>(v[rowIndex]);
}

const float3 &float4x4::Row3(int rowIndex) const
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	return reinterpret_cast<const float3 &>(v[rowIndex]);
}

CONST_WIN32 float4 float4x4::Col(int colIndex) const
{
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	return float4(v[0][colIndex], v[1][colIndex], v[2][colIndex], v[3][colIndex]);
}

CONST_WIN32 float3 float4x4::Col3(int colIndex) const
{
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	return float3(v[0][colIndex], v[1][colIndex], v[2][colIndex]);
}
#endif

CONST_WIN32 float4 float4x4::Diagonal() const
{
	return float4(v[0][0], v[1][1], v[2][2], v[3][3]);
}

CONST_WIN32 float3 float4x4::Diagonal3() const
{
	return float3(v[0][0], v[1][1], v[2][2]);
}

void float4x4::ScaleRow3(int r, float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	At(r, 0) *= scalar;
	At(r, 1) *= scalar;
	At(r, 2) *= scalar;
}

void float4x4::ScaleRow(int r, float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	At(r, 0) *= scalar;
	At(r, 1) *= scalar;
	At(r, 2) *= scalar;
	At(r, 3) *= scalar;
}

void float4x4::ScaleCol3(int colIndex, float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	At(0, colIndex) *= scalar;
	At(1, colIndex) *= scalar;
	At(2, colIndex) *= scalar;
}

void float4x4::ScaleCol(int colIndex, float scalar)
{
	assume(MATH_NS::IsFinite(scalar));
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	At(0, colIndex) *= scalar;
	At(1, colIndex) *= scalar;
	At(2, colIndex) *= scalar;
	At(3, colIndex) *= scalar;
}

CONST_WIN32 float3x3 float4x4::Float3x3Part() const
{
	return float3x3(At(0, 0), At(0, 1), At(0, 2),
	                At(1, 0), At(1, 1), At(1, 2),
	                At(2, 0), At(2, 1), At(2, 2));
}

#ifdef MATH_COLMAJOR_MATRICES
CONST_WIN32 float3x4 float4x4::Float3x4Part() const
{
	return float3x4(At(0, 0), At(0, 1), At(0, 2), At(0, 3),
					At(1, 0), At(1, 1), At(1, 2), At(1, 3),
					At(2, 0), At(2, 1), At(2, 2), At(2, 3));
}
#else
float3x4 &float4x4::Float3x4Part()
{
	return reinterpret_cast<float3x4 &>(*this);
}

const float3x4 &float4x4::Float3x4Part() const
{
	return reinterpret_cast<const float3x4 &>(*this);
}
#endif

void float4x4::SetFloat3x4Part(const float3x4 &float3x4Part)
{
	At(0, 0) = float3x4Part[0][0]; At(0, 1) = float3x4Part[0][1]; At(0, 2) = float3x4Part[0][2]; At(0, 3) = float3x4Part[0][3];
	At(1, 0) = float3x4Part[1][0]; At(1, 1) = float3x4Part[1][1]; At(1, 2) = float3x4Part[1][2]; At(1, 3) = float3x4Part[1][3];
	At(2, 0) = float3x4Part[2][0]; At(2, 1) = float3x4Part[2][1]; At(2, 2) = float3x4Part[2][2]; At(2, 3) = float3x4Part[2][3];
}

CONST_WIN32 float3 float4x4::TranslatePart() const
{
	return Col3(3);
}

CONST_WIN32 float3x3 float4x4::RotatePart() const
{
	return Float3x3Part();
}

float3 float4x4::WorldX() const
{
	return Col3(0);
}

float3 float4x4::WorldY() const
{
	return Col3(1);
}

float3 float4x4::WorldZ() const
{
	return Col3(2);
}

void float4x4::SetRow3(int rowIndex, const float3 &rowVector)
{
	SetRow3(rowIndex, rowVector.x, rowVector.y, rowVector.z);
}

void float4x4::SetRow3(int rowIndex, const float *data)
{
	assume(data);
	SetRow3(rowIndex, data[0], data[1], data[2]);
}

void float4x4::SetRow3(int rowIndex, float m_r0, float m_r1, float m_r2)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(MATH_NS::IsFinite(m_r0));
	assume(MATH_NS::IsFinite(m_r1));
	assume(MATH_NS::IsFinite(m_r2));
	At(rowIndex, 0) = m_r0;
	At(rowIndex, 1) = m_r1;
	At(rowIndex, 2) = m_r2;
}

void float4x4::SetRow(int rowIndex, const float3 &rowVector, float m_r3)
{
	SetRow(rowIndex, rowVector.x, rowVector.y, rowVector.z, m_r3);
}

void float4x4::SetRow(int rowIndex, const float4 &rowVector)
{
	SetRow(rowIndex, rowVector.x, rowVector.y, rowVector.z, rowVector.w);
}

void float4x4::SetRow(int rowIndex, const float *data)
{
	assume(data);
	SetRow(rowIndex, data[0], data[1], data[2], data[3]);
}

void float4x4::SetRow(int rowIndex, float m_r0, float m_r1, float m_r2, float m_r3)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(MATH_NS::IsFinite(m_r0));
	assume(MATH_NS::IsFinite(m_r1));
	assume(MATH_NS::IsFinite(m_r2));
	assume(MATH_NS::IsFinite(m_r3));

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	this->row[rowIndex] = set_ps(m_r3, m_r2, m_r1, m_r0);
#else
	At(rowIndex, 0) = m_r0;
	At(rowIndex, 1) = m_r1;
	At(rowIndex, 2) = m_r2;
	At(rowIndex, 3) = m_r3;
#endif
}

void float4x4::SetCol3(int column, const float3 &columnVector)
{
	SetCol3(column, columnVector.x, columnVector.y, columnVector.z);
}

void float4x4::SetCol3(int column, const float *data)
{
	assume(data);
	SetCol3(column, data[0], data[1], data[2]);
}

void float4x4::SetCol3(int column, float m_0c, float m_1c, float m_2c)
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

void float4x4::SetCol(int column, const float3 &columnVector, float m_3c)
{
	SetCol(column, columnVector.x, columnVector.y, columnVector.z, m_3c);
}

void float4x4::SetCol(int column, const float4 &columnVector)
{
	SetCol(column, columnVector.x, columnVector.y, columnVector.z, columnVector.w);
}

void float4x4::SetCol(int column, const float *data)
{
	assume(data);
	SetCol(column, data[0], data[1], data[2], data[3]);
}

void float4x4::SetCol(int column, float m_0c, float m_1c, float m_2c, float m_3c)
{
	assume(column >= 0);
	assume(column < Cols);
	assume(MATH_NS::IsFinite(m_0c));
	assume(MATH_NS::IsFinite(m_1c));
	assume(MATH_NS::IsFinite(m_2c));
	assume(MATH_NS::IsFinite(m_3c));
	At(0, column) = m_0c;
	At(1, column) = m_1c;
	At(2, column) = m_2c;
	At(3, column) = m_3c;
}

void float4x4::Set(float _00, float _01, float _02, float _03,
                   float _10, float _11, float _12, float _13,
                   float _20, float _21, float _22, float _23,
                   float _30, float _31, float _32, float _33)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	mat4x4_set(row, _00, _01, _02, _03,
	                _10, _11, _12, _13,
	                _20, _21, _22, _23,
	                _30, _31, _32, _33);
#else
	At(0, 0) = _00; At(0, 1) = _01; At(0, 2) = _02; At(0, 3) = _03;
	At(1, 0) = _10; At(1, 1) = _11; At(1, 2) = _12; At(1, 3) = _13;
	At(2, 0) = _20; At(2, 1) = _21; At(2, 2) = _22; At(2, 3) = _23;
	At(3, 0) = _30; At(3, 1) = _31; At(3, 2) = _32; At(3, 3) = _33;
#endif
}

void float4x4::Set(const float4x4 &rhs)
{
#ifdef MATH_AVX
	row2[0] = rhs.row2[0];
	row2[1] = rhs.row2[1];
#elif defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	row[0] = rhs.row[0];
	row[1] = rhs.row[1];
	row[2] = rhs.row[2];
	row[3] = rhs.row[3];
#else
	Set(rhs.ptr());
#endif
}

void float4x4::Set(const float *p)
{
	assume(p);
	Set( p[0],  p[1],  p[2],  p[3],
	     p[4],  p[5],  p[6],  p[7],
	     p[8],  p[9], p[10], p[11],
	    p[12], p[13], p[14], p[15]);
}

void float4x4::Set(int rowIndex, int colIndex, float value)
{
	assume(rowIndex >= 0);
	assume(rowIndex < Rows);
	assume(colIndex >= 0);
	assume(colIndex < Cols);
	At(rowIndex, colIndex) = value;
}

void float4x4::SetIdentity()
{
	Set(1,0,0,0,
	    0,1,0,0,
	    0,0,1,0,
	    0,0,0,1);
}

void float4x4::Set3x3Part(const float3x3 &r)
{
	assume(r.IsFinite());
	At(0, 0) = r[0][0]; At(0, 1) = r[0][1]; At(0, 2) = r[0][2];
	At(1, 0) = r[1][0]; At(1, 1) = r[1][1]; At(1, 2) = r[1][2];
	At(2, 0) = r[2][0]; At(2, 1) = r[2][1]; At(2, 2) = r[2][2];
}

void float4x4::Set3x4Part(const float3x4 &r)
{
	assume(r.IsFinite());

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	row[0] = r.row[0];
	row[1] = r.row[1];
	row[2] = r.row[2];
#else
	At(0, 0) = r[0][0]; At(0, 1) = r[0][1]; At(0, 2) = r[0][2]; At(0, 3) = r[0][3];
	At(1, 0) = r[1][0]; At(1, 1) = r[1][1]; At(1, 2) = r[1][2]; At(1, 3) = r[1][3];
	At(2, 0) = r[2][0]; At(2, 1) = r[2][1]; At(2, 2) = r[2][2]; At(2, 3) = r[2][3];
#endif
}

void float4x4::SwapColumns(int col1, int col2)
{
	assume(col1 >= 0);
	assume(col1 < Cols);
	assume(col2 >= 0);
	assume(col2 < Cols);
	Swap(At(0, col1), At(0, col2));
	Swap(At(1, col1), At(1, col2));
	Swap(At(2, col1), At(2, col2));
	Swap(At(3, col1), At(3, col2));
}

void float4x4::SwapColumns3(int col1, int col2)
{
	assume(col1 >= 0);
	assume(col1 < Cols);
	assume(col2 >= 0);
	assume(col2 < Cols);
	Swap(At(0, col1), At(0, col2));
	Swap(At(1, col1), At(1, col2));
	Swap(At(2, col1), At(2, col2));
}

void float4x4::SwapRows(int r1, int r2)
{
	assume(r1 >= 0);
	assume(r1 < Rows);
	assume(r2 >= 0);
	assume(r2 < Rows);

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	Swap(row[r1], row[r2]);
#else
	Swap(At(r1, 0), At(r2, 0));
	Swap(At(r1, 1), At(r2, 1));
	Swap(At(r1, 2), At(r2, 2));
	Swap(At(r1, 3), At(r2, 3));
#endif
}

void float4x4::SwapRows3(int r1, int r2)
{
	assume(r1 >= 0);
	assume(r1 < Rows);
	assume(r2 >= 0);
	assume(r2 < Rows);
	Swap(At(r1, 0), At(r2, 0));
	Swap(At(r1, 1), At(r2, 1));
	Swap(At(r1, 2), At(r2, 2));
}

void float4x4::SetTranslatePart(float translateX, float translateY, float translateZ)
{
	At(0, 3) = translateX;
	At(1, 3) = translateY;
	At(2, 3) = translateZ;
}

void float4x4::SetTranslatePart(const float3 &offset)
{
	At(0, 3) = offset.x;
	At(1, 3) = offset.y;
	At(2, 3) = offset.z;
}

void float4x4::SetTranslatePart(const float4 &offset)
{
	At(0, 3) = offset.x;
	At(1, 3) = offset.y;
	At(2, 3) = offset.z;
	assume(EqualAbs(offset.w, 1.f) || EqualAbs(offset.w, 0.f));
}

void float4x4::SetRotatePartX(float angle)
{
	Set3x3PartRotateX(*this, angle);
}

void float4x4::SetRotatePartY(float angle)
{
	Set3x3PartRotateY(*this, angle);
}

void float4x4::SetRotatePartZ(float angle)
{
	Set3x3PartRotateZ(*this, angle);
}

void float4x4::SetRotatePart(const float3 &a, float angle)
{
	SetRotationAxis3x3(*this, a, angle);
}

void float4x4::SetRotatePart(const Quat &q)
{
	SetMatrixRotatePart(*this, q);
}

float4x4 float4x4::LookAt(const float3 &localForward, const float3 &targetDirection, const float3 &localUp, const float3 &worldUp)
{
	float4x4 m;
	m.SetRotatePart(float3x3::LookAt(localForward, targetDirection, localUp, worldUp));
	m.SetTranslatePart(0,0,0);
	m.SetRow(3, 0,0,0,1);
	return m;
}

float4x4 float4x4::LookAt(const float3 &eyePos, const float3 &targetPos, const float3 &localForward,
                          const float3 &localUp, const float3 &worldUp)
{
	float4x4 m;
	m.SetRotatePart(float3x3::LookAt(localForward, (targetPos-eyePos).Normalized(), localUp, worldUp));
	m.SetTranslatePart(eyePos);
	m.SetRow(3, 0,0,0,1);
	return m;
}

float4x4 &float4x4::operator =(const float3x3 &rhs)
{
	SetRotatePart(rhs);
	SetTranslatePart(0,0,0);
	SetRow(3, 0,0,0,1);
	return *this;
}

float4x4 &float4x4::operator =(const float3x4 &rhs)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	row[0] = rhs.row[0];
	row[1] = rhs.row[1];
	row[2] = rhs.row[2];
	row[3] = set_ps(1.f, 0.f, 0.f, 0.f);
#else
	At(0, 0) = rhs[0][0];
	At(0, 1) = rhs[0][1];
	At(0, 2) = rhs[0][2];
	At(0, 3) = rhs[0][3];

	At(1, 0) = rhs[1][0];
	At(1, 1) = rhs[1][1];
	At(1, 2) = rhs[1][2];
	At(1, 3) = rhs[1][3];

	At(2, 0) = rhs[2][0];
	At(2, 1) = rhs[2][1];
	At(2, 2) = rhs[2][2];
	At(2, 3) = rhs[2][3];

	At(3, 0) = 0.f;
	At(3, 1) = 0.f;
	At(3, 2) = 0.f;
	At(3, 3) = 1.f;
#endif
	return *this;
}

#if 0
float4x4 &float4x4::operator =(const float4x4 &rhs)
{
	// We deliberately don't want to assume rhs is finite, it is ok
	// to copy around uninitialized matrices.
	// But note that when assigning through a conversion above (float3x3 -> float4x4 or float3x4 -> float4x4),
	// we do assume the input matrix is finite.
//	assume(rhs.IsFinite());

/* // AVX path determined to be one clock cycle slower than SSE path: (6 clock cycles on AVX, 5 on SSE)
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_AVX)
	assert(IS32ALIGNED(this));
	assert(IS32ALIGNED(&rhs));
	row2[0] = rhs.row2[0];
	row2[1] = rhs.row2[1];
#elif defined(MATH_SSE) */

#if defined(MATH_AUTOMATIC_SSE)

#if !defined(ANDROID) // Android NEON doesn't currently use aligned loads.
	assert(IS16ALIGNED(this));
	assert(IS16ALIGNED(&rhs));
#endif
	row[0] = rhs.row[0];
	row[1] = rhs.row[1];
	row[2] = rhs.row[2];
	row[3] = rhs.row[3];
#else
	v[0][0] = rhs.v[0][0];
	v[0][1] = rhs.v[0][1];
	v[0][2] = rhs.v[0][2];
	v[0][3] = rhs.v[0][3];

	v[1][0] = rhs.v[1][0];
	v[1][1] = rhs.v[1][1];
	v[1][2] = rhs.v[1][2];
	v[1][3] = rhs.v[1][3];

	v[2][0] = rhs.v[2][0];
	v[2][1] = rhs.v[2][1];
	v[2][2] = rhs.v[2][2];
	v[2][3] = rhs.v[2][3];

	v[3][0] = rhs.v[3][0];
	v[3][1] = rhs.v[3][1];
	v[3][2] = rhs.v[3][2];
	v[3][3] = rhs.v[3][3];
#endif
	return *this;
}
#endif

float4x4 &float4x4::operator =(const Quat &rhs)
{
	*this = rhs.ToFloat4x4();
	return *this;
}

float4x4 &float4x4::operator =(const TranslateOp &rhs)
{
	Set(1.f,   0,   0, rhs.offset.x,
	      0, 1.f,   0, rhs.offset.y,
	      0,   0, 1.f, rhs.offset.z,
	      0,   0,   0,          1.f);

	return *this;
}

float float4x4::Determinant3() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return mat3x4_determinant(row);
#else
	assume(Float3x3Part().IsFinite());
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
#endif
}

float float4x4::Determinant4() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	return mat4x4_determinant(row);
#else
	assume(IsFinite());
	return At(0, 0) * Minor(0,0) - At(0, 1) * Minor(0,1) + At(0, 2) * Minor(0,2) - At(0, 3) * Minor(0,3);
#endif
}

#define SKIPNUM(val, skip) (val >= skip ? (val+1) : val)

float3x3 float4x4::SubMatrix(int i, int j) const
{
	int r0 = SKIPNUM(0, i);
	int r1 = SKIPNUM(1, i);
	int r2 = SKIPNUM(2, i);
	int c0 = SKIPNUM(0, j);
	int c1 = SKIPNUM(1, j);
	int c2 = SKIPNUM(2, j);

	return float3x3(At(r0, c0), At(r0, c1), At(r0, c2),
	                At(r1, c0), At(r1, c1), At(r1, c2),
	                At(r2, c0), At(r2, c1), At(r2, c2));
}

float float4x4::Minor(int i, int j) const
{
	int r0 = SKIPNUM(0, i);
	int r1 = SKIPNUM(1, i);
	int r2 = SKIPNUM(2, i);
	int c0 = SKIPNUM(0, j);
	int c1 = SKIPNUM(1, j);
	int c2 = SKIPNUM(2, j);

	float a = At(r0, c0);
	float b = At(r0, c1);
	float c = At(r0, c2);
	float d = At(r1, c0);
	float e = At(r1, c1);
	float f = At(r1, c2);
	float g = At(r2, c0);
	float h = At(r2, c1);
	float k = At(r2, c2);

	return a*e*k + b*f*g + c*d*h - a*f*h - b*d*k - c*e*g;
}

float4x4 float4x4::Adjugate() const
{
	float4x4 a;
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			a[iy][ix] = (((ix+iy) & 1) != 0) ? -Minor(iy, ix) : Minor(iy, ix);

	return a;
}

bool float4x4::CholeskyDecompose(float4x4 &outL) const
{
	return CholeskyDecomposeMatrix(*this, outL);
}

bool float4x4::LUDecompose(float4x4 &outLower, float4x4 &outUpper) const
{
	return LUDecomposeMatrix(*this, outLower, outUpper);
}

bool float4x4::Inverse(float epsilon)
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	MARK_UNUSED(epsilon);
	float det = mat4x4_inverse(row, row);
	return MATH_NS::Abs(det) > 1e-5f;
#else
	return InverseMatrix(*this, epsilon);
#endif
}

float4x4 float4x4::Inverted() const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	float4x4 copy;
	mat4x4_inverse(row, copy.row);
#else
	float4x4 copy = *this;
	copy.Inverse();
#endif
	return copy;
}

bool float4x4::InverseColOrthogonal()
{
	///\todo SSE
	assume(!ContainsProjection());
#ifdef MATH_COLMAJOR_MATRICES
	///\todo Optimize away redundant copies
	float3x4 mat3x4 = Float3x4Part();
	bool ret = mat3x4.InverseColOrthogonal();
	SetFloat3x4Part(mat3x4);
	return ret;
#else
	return Float3x4Part().InverseColOrthogonal();
#endif
}

bool float4x4::InverseOrthogonalUniformScale()
{
	///\todo SSE
	assume(!ContainsProjection());
	assume(IsColOrthogonal3(1e-3f));
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
}

void float4x4::InverseOrthonormal()
{
	assume(!ContainsProjection());
#ifdef MATH_SSE
	mat3x4_inverse_orthonormal(row, row);
#else
	// a) Transpose the top-left 3x3 part in-place to produce R^t.
	Swap(v[0][1], v[1][0]);
	Swap(v[0][2], v[2][0]);
	Swap(v[1][2], v[2][1]);
	
	// b) Replace the top-right 3x1 part by computing R^t(-T).
	SetTranslatePart(TransformDir(-At(0, 3), -At(1, 3), -At(2, 3)));
#endif
}

void float4x4::Transpose()
{
#if defined(MATH_AUTOMATIC_SSE) && !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
	mat4x4_transpose(row, row);
#else
	Swap(v[0][1], v[1][0]);
	Swap(v[0][2], v[2][0]);
	Swap(v[0][3], v[3][0]);
	Swap(v[1][2], v[2][1]);
	Swap(v[1][3], v[3][1]);
	Swap(v[2][3], v[3][2]);
#endif
}

float4x4 float4x4::Transposed() const
{
#if defined(MATH_AUTOMATIC_SSE) && !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
	float4x4 copy;
	mat4x4_transpose(copy.row, row);
	return copy;
#else
	float4x4 copy;
	copy.v[0][0] = v[0][0]; copy.v[0][1] = v[1][0]; copy.v[0][2] = v[2][0]; copy.v[0][3] = v[3][0];
	copy.v[1][0] = v[0][1]; copy.v[1][1] = v[1][1]; copy.v[1][2] = v[2][1]; copy.v[1][3] = v[3][1];
	copy.v[2][0] = v[0][2]; copy.v[2][1] = v[1][2]; copy.v[2][2] = v[2][2]; copy.v[2][3] = v[3][2];
	copy.v[3][0] = v[0][3]; copy.v[3][1] = v[1][3]; copy.v[3][2] = v[2][3]; copy.v[3][3] = v[3][3];
	return copy;
#endif
}

bool float4x4::InverseTranspose()
{
	bool success = Inverse();
	Transpose();
	return success;
}

float4x4 float4x4::InverseTransposed() const
{
	float4x4 copy = *this;
	copy.Transpose();
	copy.Inverse();
	return copy;
}

float float4x4::Trace() const
{
	assume(IsFinite());
	return v[0][0] + v[1][1] + v[2][2] + v[3][3];
}

void float4x4::Orthogonalize3(int c0, int c1, int c2)
{
	///\todo SSE
	assume(c0 != c1 && c0 != c2 && c1 != c2);
	assume(c0 >= 0 && c1 >= 0 && c2 >= 0 && c0 < Cols && c1 < Cols && c2 < Cols);
	///@todo Optimize away copies.
	float3 v0 = Col3(c0);
	float3 v1 = Col3(c1);
	float3 v2 = Col3(c2);
	float3::Orthogonalize(v0, v1, v2);
	SetCol3(c0, v0);
	SetCol3(c1, v1);
	SetCol3(c2, v2);
}

void float4x4::Orthonormalize3(int c0, int c1, int c2)
{
	///\todo SSE
	assume(c0 != c1 && c0 != c2 && c1 != c2);
	assume(c0 >= 0 && c1 >= 0 && c2 >= 0 && c0 < Cols && c1 < Cols && c2 < Cols);
	///@todo Optimize away copies.
	float3 v0 = Col3(c0);
	float3 v1 = Col3(c1);
	float3 v2 = Col3(c2);
	float3::Orthonormalize(v0, v1, v2);
	SetCol3(c0, v0);
	SetCol3(c1, v1);
	SetCol3(c2, v2);
}

void float4x4::RemoveScale()
{
	///\todo SSE
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
	assume(tx != 0 && ty != 0 && tz != 0 && "float4x4::RemoveScale failed!");
	MARK_UNUSED(tx);
	MARK_UNUSED(ty);
	MARK_UNUSED(tz);
}

/// Algorithm from Eric Lengyel's Mathematics for 3D Game Programming & Computer Graphics, 2nd Ed.
void float4x4::Pivot()
{
	int rowIndex = 0;

	for(int col = 0; col < Cols; ++col)
	{
		int greatest = rowIndex;

		// find the rowIndex k with k >= 1 for which Mkj has the largest absolute value.
		for(int i = rowIndex; i < Rows; ++i)
			if (MATH_NS::Abs(At(i, col)) > MATH_NS::Abs(At(greatest, col)))
				greatest = i;

		if (!EqualAbs(At(greatest, col), 0))
		{
			if (rowIndex != greatest)
				SwapRows(rowIndex, greatest); // the greatest now in rowIndex

			ScaleRow(rowIndex, 1.f/At(rowIndex, col));

			for(int r = 0; r < Rows; ++r)
				if (r != rowIndex)
					SetRow(r, Row(r) - Row(rowIndex) * At(r, col));

			++rowIndex;
		}
	}
}

float3 float4x4::TransformPos(const float3 &pointVector) const
{
	assume(!this->ContainsProjection()); // This function does not divide by w or output it, so cannot have projection.
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return mat3x4_mul_vec(row, set_ps(1.f, pointVector.z, pointVector.y, pointVector.x));
#else
	return TransformPos(pointVector.x, pointVector.y, pointVector.z);
#endif
}

float3 float4x4::TransformPos(float tx, float ty, float tz) const
{
	assume(!this->ContainsProjection()); // This function does not divide by w or output it, so cannot have projection.
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return mat3x4_mul_vec(row, set_ps(1.f, tz, ty, tx));
#else
	return float3(At(0, 0) * tx + At(0, 1) * ty + At(0, 2) * tz + At(0, 3),
				  At(1, 0) * tx + At(1, 1) * ty + At(1, 2) * tz + At(1, 3),
				  At(2, 0) * tx + At(2, 1) * ty + At(2, 2) * tz + At(2, 3));
#endif
}

float3 float4x4::TransformDir(const float3 &directionVector) const
{
	assume(!this->ContainsProjection()); // This function does not divide by w or output it, so cannot have projection.
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return mat3x4_mul_vec(row, set_ps(0.f, directionVector.z, directionVector.y, directionVector.x));
#else
	return TransformDir(directionVector.x, directionVector.y, directionVector.z);
#endif
}

float3 float4x4::TransformDir(float tx, float ty, float tz) const
{
	assume(!this->ContainsProjection()); // This function does not divide by w or output it, so cannot have projection.
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	return mat3x4_mul_vec(row, set_ps(0.f, tz, ty, tx));
#else
	return float3(At(0, 0) * tx + At(0, 1) * ty + At(0, 2) * tz,
				  At(1, 0) * tx + At(1, 1) * ty + At(1, 2) * tz,
				  At(2, 0) * tx + At(2, 1) * ty + At(2, 2) * tz);
#endif
}

float4 float4x4::Transform(const float4 &vector) const
{
#if defined(MATH_AUTOMATIC_SSE) && !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
	return mat4x4_mul_vec4(row, vector.v);
#else
	return float4(At(0, 0) * vector.x + At(0, 1) * vector.y + At(0, 2) * vector.z + At(0, 3) * vector.w,
				  At(1, 0) * vector.x + At(1, 1) * vector.y + At(1, 2) * vector.z + At(1, 3) * vector.w,
				  At(2, 0) * vector.x + At(2, 1) * vector.y + At(2, 2) * vector.z + At(2, 3) * vector.w,
				  At(3, 0) * vector.x + At(3, 1) * vector.y + At(3, 2) * vector.z + At(3, 3) * vector.w);
#endif
}

void float4x4::TransformPos(float3 *pointArray, int numPoints) const
{
	///\todo SSE.
	assume(pointArray);
	for(int i = 0; i < numPoints; ++i)
		pointArray[i] = this->TransformPos(pointArray[i]);
}

void float4x4::TransformPos(float3 *pointArray, int numPoints, int strideBytes) const
{
	///\todo SSE.
	assume(pointArray);
	u8 *data = reinterpret_cast<u8*>(pointArray);
	for(int i = 0; i < numPoints; ++i)
	{
		float3 *vtx = reinterpret_cast<float3*>(data);
		*vtx = this->TransformPos(*vtx);
		data += strideBytes;
	}
}

void float4x4::TransformDir(float3 *dirArray, int numVectors) const
{
	///\todo SSE.
	assume(dirArray);
	for(int i = 0; i < numVectors; ++i)
		dirArray[i] = this->TransformDir(dirArray[i]);
}

void float4x4::TransformDir(float3 *dirArray, int numVectors, int strideBytes) const
{
	///\todo SSE.
	assume(dirArray);
	u8 *data = reinterpret_cast<u8*>(dirArray);
	for(int i = 0; i < numVectors; ++i)
	{
		float3 *vtx = reinterpret_cast<float3*>(data);
		*vtx = this->TransformDir(*vtx);
		data += strideBytes;
	}
}

void float4x4::Transform(float4 *vectorArray, int numVectors) const
{
	///\todo SSE.
	assume(vectorArray);
	for(int i = 0; i < numVectors; ++i)
		vectorArray[i] = *this * vectorArray[i];
}

void float4x4::Transform(float4 *vectorArray, int numVectors, int strideBytes) const
{
	///\todo SSE.
	assume(vectorArray);
	u8 *data = reinterpret_cast<u8*>(vectorArray);
	for(int i = 0; i < numVectors; ++i)
	{
		float4 *vtx = reinterpret_cast<float4*>(data);
		*vtx = *this * *vtx;
		data += strideBytes;
	}
}

float4x4 float4x4::operator *(const float3x3 &rhs) const
{
	float4x4 r;
#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
	mat4x4_mul_mat3x3_sse(r.row, this->row, rhs.ptr());
#else
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

	r[3][0] = At(3, 0) * rhs.At(0, 0) + At(3, 1) * rhs.At(1, 0) + At(3, 2) * rhs.At(2, 0);
	r[3][1] = At(3, 0) * rhs.At(0, 1) + At(3, 1) * rhs.At(1, 1) + At(3, 2) * rhs.At(2, 1);
	r[3][2] = At(3, 0) * rhs.At(0, 2) + At(3, 1) * rhs.At(1, 2) + At(3, 2) * rhs.At(2, 2);
	r[3][3] = At(3, 3);
#endif
	return r;
}

float4x4 float4x4::operator *(const float3x4 &rhs) const
{
	float4x4 r;
#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
	mat4x4_mul_mat3x4_sse(r.row, this->row, rhs.row);
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
	
	r[3][0] = At(3, 0) * rhs.At(0, 0) + At(3, 1) * rhs.At(1, 0) + At(3, 2) * rhs.At(2, 0);
	r[3][1] = At(3, 0) * rhs.At(0, 1) + At(3, 1) * rhs.At(1, 1) + At(3, 2) * rhs.At(2, 1);
	r[3][2] = At(3, 0) * rhs.At(0, 2) + At(3, 1) * rhs.At(1, 2) + At(3, 2) * rhs.At(2, 2);
	r[3][3] = At(3, 0) * rhs.At(0, 3) + At(3, 1) * rhs.At(1, 3) + At(3, 2) * rhs.At(2, 3) + At(3, 3);
#endif
	return r;
}

float4x4 float4x4::operator *(const float4x4 &rhs) const
{
	float4x4 r;
#ifdef MATH_AUTOMATIC_SSE
	mat4x4_mul_mat4x4(r.row, this->row, rhs.row);
#else
	r[0][0] = At(0, 0) * rhs.At(0, 0) + At(0, 1) * rhs.At(1, 0) + At(0, 2) * rhs.At(2, 0) + At(0, 3) * rhs.At(3, 0);
	r[0][1] = At(0, 0) * rhs.At(0, 1) + At(0, 1) * rhs.At(1, 1) + At(0, 2) * rhs.At(2, 1) + At(0, 3) * rhs.At(3, 1);
	r[0][2] = At(0, 0) * rhs.At(0, 2) + At(0, 1) * rhs.At(1, 2) + At(0, 2) * rhs.At(2, 2) + At(0, 3) * rhs.At(3, 2);
	r[0][3] = At(0, 0) * rhs.At(0, 3) + At(0, 1) * rhs.At(1, 3) + At(0, 2) * rhs.At(2, 3) + At(0, 3) * rhs.At(3, 3);
	
	r[1][0] = At(1, 0) * rhs.At(0, 0) + At(1, 1) * rhs.At(1, 0) + At(1, 2) * rhs.At(2, 0) + At(1, 3) * rhs.At(3, 0);
	r[1][1] = At(1, 0) * rhs.At(0, 1) + At(1, 1) * rhs.At(1, 1) + At(1, 2) * rhs.At(2, 1) + At(1, 3) * rhs.At(3, 1);
	r[1][2] = At(1, 0) * rhs.At(0, 2) + At(1, 1) * rhs.At(1, 2) + At(1, 2) * rhs.At(2, 2) + At(1, 3) * rhs.At(3, 2);
	r[1][3] = At(1, 0) * rhs.At(0, 3) + At(1, 1) * rhs.At(1, 3) + At(1, 2) * rhs.At(2, 3) + At(1, 3) * rhs.At(3, 3);
	
	r[2][0] = At(2, 0) * rhs.At(0, 0) + At(2, 1) * rhs.At(1, 0) + At(2, 2) * rhs.At(2, 0) + At(2, 3) * rhs.At(3, 0);
	r[2][1] = At(2, 0) * rhs.At(0, 1) + At(2, 1) * rhs.At(1, 1) + At(2, 2) * rhs.At(2, 1) + At(2, 3) * rhs.At(3, 1);
	r[2][2] = At(2, 0) * rhs.At(0, 2) + At(2, 1) * rhs.At(1, 2) + At(2, 2) * rhs.At(2, 2) + At(2, 3) * rhs.At(3, 2);
	r[2][3] = At(2, 0) * rhs.At(0, 3) + At(2, 1) * rhs.At(1, 3) + At(2, 2) * rhs.At(2, 3) + At(2, 3) * rhs.At(3, 3);
	
	r[3][0] = At(3, 0) * rhs.At(0, 0) + At(3, 1) * rhs.At(1, 0) + At(3, 2) * rhs.At(2, 0) + At(3, 3) * rhs.At(3, 0);
	r[3][1] = At(3, 0) * rhs.At(0, 1) + At(3, 1) * rhs.At(1, 1) + At(3, 2) * rhs.At(2, 1) + At(3, 3) * rhs.At(3, 1);
	r[3][2] = At(3, 0) * rhs.At(0, 2) + At(3, 1) * rhs.At(1, 2) + At(3, 2) * rhs.At(2, 2) + At(3, 3) * rhs.At(3, 2);
	r[3][3] = At(3, 0) * rhs.At(0, 3) + At(3, 1) * rhs.At(1, 3) + At(3, 2) * rhs.At(2, 3) + At(3, 3) * rhs.At(3, 3);
#endif

	return r;
}

float4x4 float4x4::operator *(const Quat &rhs) const
{
#ifdef MATH_SIMD
	float4x4 rot(rhs);
	return *this * rot;
#else
	float3x3 rot(rhs);
	return *this * rot;
#endif
}

float4 float4x4::operator *(const float4 &rhs) const
{
	return this->Transform(rhs);
}

float4x4 float4x4::operator *(float scalar) const
{
#ifdef MATH_AUTOMATIC_SSE
	float4x4 r;
	mat4x4_mul_float(r.row, row, scalar);
#else
	float4x4 r = *this;
	r *= scalar;
#endif
	return r;
}

float4x4 float4x4::operator /(float scalar) const
{
	assume(!EqualAbs(scalar, 0));

#ifdef MATH_AUTOMATIC_SSE
	float4x4 r;
	mat4x4_div_float(r.row, row, scalar);
#else
	float4x4 r = *this;
	r /= scalar;
#endif
	return r;
}

float4x4 float4x4::operator +(const float4x4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	float4x4 r;
	mat4x4_add_mat4x4(r.row, row, rhs.row);
#else
	float4x4 r = *this;
	r += rhs;
#endif
	return r;
}

float4x4 float4x4::operator -(const float4x4 &rhs) const
{
#ifdef MATH_AUTOMATIC_SSE
	float4x4 r;
	mat4x4_sub_mat4x4(r.row, row, rhs.row);
#else
	float4x4 r = *this;
	r -= rhs;
#endif
	return r;
}

float4x4 float4x4::operator -() const
{
#ifdef MATH_AUTOMATIC_SSE
	float4x4 r;
	mat4x4_negate(r.row, row);
	return r;
#else
	return float4x4(-At(0, 0), -At(0, 1), -At(0, 2), -At(0, 3),
	                -At(1, 0), -At(1, 1), -At(1, 2), -At(1, 3),
	                -At(2, 0), -At(2, 1), -At(2, 2), -At(2, 3),
	                -At(3, 0), -At(3, 1), -At(3, 2), -At(3, 3));
#endif
}

float4x4 &float4x4::operator *=(float s)
{
#ifdef MATH_AUTOMATIC_SSE
	mat4x4_mul_float(row, row, s);
#else
	v[0][0] *= s; v[0][1] *= s; v[0][2] *= s; v[0][3] *= s;
	v[1][0] *= s; v[1][1] *= s; v[1][2] *= s; v[1][3] *= s;
	v[2][0] *= s; v[2][1] *= s; v[2][2] *= s; v[2][3] *= s;
	v[3][0] *= s; v[3][1] *= s; v[3][2] *= s; v[3][3] *= s;
#endif
	return *this;
}

float4x4 &float4x4::operator /=(float scalar)
{
	assume(!EqualAbs(scalar, 0));
	return *this *= (1.f / scalar);
}

float4x4 &float4x4::operator +=(const float4x4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	mat4x4_add_mat4x4(row, row, rhs.row);
#else
	v[0][0] += rhs.v[0][0]; v[0][1] += rhs.v[0][1]; v[0][2] += rhs.v[0][2]; v[0][3] += rhs.v[0][3];
	v[1][0] += rhs.v[1][0]; v[1][1] += rhs.v[1][1]; v[1][2] += rhs.v[1][2]; v[1][3] += rhs.v[1][3];
	v[2][0] += rhs.v[2][0]; v[2][1] += rhs.v[2][1]; v[2][2] += rhs.v[2][2]; v[2][3] += rhs.v[2][3];
	v[3][0] += rhs.v[3][0]; v[3][1] += rhs.v[3][1]; v[3][2] += rhs.v[3][2]; v[3][3] += rhs.v[3][3];
#endif
	return *this;
}

float4x4 &float4x4::operator -=(const float4x4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	mat4x4_sub_mat4x4(row, row, rhs.row);
#else
	v[0][0] -= rhs.v[0][0]; v[0][1] -= rhs.v[0][1]; v[0][2] -= rhs.v[0][2]; v[0][3] -= rhs.v[0][3];
	v[1][0] -= rhs.v[1][0]; v[1][1] -= rhs.v[1][1]; v[1][2] -= rhs.v[1][2]; v[1][3] -= rhs.v[1][3];
	v[2][0] -= rhs.v[2][0]; v[2][1] -= rhs.v[2][1]; v[2][2] -= rhs.v[2][2]; v[2][3] -= rhs.v[2][3];
	v[3][0] -= rhs.v[3][0]; v[3][1] -= rhs.v[3][1]; v[3][2] -= rhs.v[3][2]; v[3][3] -= rhs.v[3][3];
#endif
	return *this;
}

bool float4x4::IsFinite() const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!MATH_NS::IsFinite(v[iy][ix]))
				return false;
	return true;
}

bool float4x4::IsIdentity(float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!EqualAbs(v[iy][ix], (ix == iy) ? 1.f : 0.f, epsilon))
				return false;

	return true;
}

bool float4x4::IsLowerTriangular(float epsilon) const
{
	return EqualAbs(At(0, 1), 0.f, epsilon)
	    && EqualAbs(At(0, 2), 0.f, epsilon)
	    && EqualAbs(At(0, 3), 0.f, epsilon)
	    && EqualAbs(At(1, 2), 0.f, epsilon)
	    && EqualAbs(At(1, 3), 0.f, epsilon)
	    && EqualAbs(At(2, 3), 0.f, epsilon);
}

bool float4x4::IsUpperTriangular(float epsilon) const
{
	return EqualAbs(At(1, 0), 0.f, epsilon)
	    && EqualAbs(At(2, 0), 0.f, epsilon)
	    && EqualAbs(At(3, 0), 0.f, epsilon)
	    && EqualAbs(At(2, 1), 0.f, epsilon)
	    && EqualAbs(At(3, 1), 0.f, epsilon)
	    && EqualAbs(At(3, 2), 0.f, epsilon);
}

bool float4x4::IsInvertible(float epsilon) const
{
	///@todo Optimize.
	float4x4 copy = *this;
	return copy.Inverse(epsilon);
}

bool float4x4::IsSymmetric(float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = iy+1; ix < Cols; ++ix)
			if (!EqualAbs(v[iy][ix], v[ix][iy], epsilon))
				return false;
	return true;
}

bool float4x4::IsSkewSymmetric(float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = iy; ix < Cols; ++ix)
			if (!EqualAbs(v[iy][ix], -v[ix][iy], epsilon))
				return false;
	return true;
}

bool float4x4::IsIdempotent(float epsilon) const
{
	float4x4 m2 = *this * *this;
	return this->Equals(m2, epsilon);
}

bool float4x4::HasUnitaryScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return scale.Equals(1.f, 1.f, 1.f, epsilon);
}

bool float4x4::HasNegativeScale() const
{
	return Determinant3() < 0.f;
}

bool float4x4::HasUniformScale(float epsilon) const
{
	float3 scale = ExtractScale();
	return EqualAbs(scale.x, scale.y, epsilon) && EqualAbs(scale.x, scale.z, epsilon);
}

bool float4x4::IsRowOrthogonal3(float epsilon) const
{
	return Row3(0).IsPerpendicular(Row3(1), epsilon)
	    && Row3(0).IsPerpendicular(Row3(2), epsilon)
	    && Row3(1).IsPerpendicular(Row3(2), epsilon);
}

bool float4x4::IsColOrthogonal3(float epsilon) const
{
	return Col3(0).IsPerpendicular(Col3(1), epsilon)
	    && Col3(0).IsPerpendicular(Col3(2), epsilon)
	    && Col3(1).IsPerpendicular(Col3(2), epsilon);
}

bool float4x4::IsOrthonormal3(float epsilon) const
{
	///@todo Epsilon magnitudes don't match.
	return IsColOrthogonal3(epsilon) && Row3(0).IsNormalized(epsilon) && Row3(1).IsNormalized(epsilon) && Row3(2).IsNormalized(epsilon);
}

bool float4x4::Equals(const float4x4 &other, float epsilon) const
{
	for(int iy = 0; iy < Rows; ++iy)
		for(int ix = 0; ix < Cols; ++ix)
			if (!EqualAbs(At(iy, ix), other[iy][ix], epsilon))
				return false;
	return true;
}

bool float4x4::ContainsProjection(float epsilon) const
{
	return Row(3).Equals(0.f, 0.f, 0.f, 1.f, epsilon) == false;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float4x4::ToString() const
{
	char str[256];
	sprintf(str, "(%.2f, %.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f)",
		At(0, 0), At(0, 1), At(0, 2), At(0, 3),
		At(1, 0), At(1, 1), At(1, 2), At(1, 3),
		At(2, 0), At(2, 1), At(2, 2), At(2, 3),
		At(3, 0), At(3, 1), At(3, 2), At(3, 3));

	return str;
}

StringT float4x4::SerializeToString() const
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
	s = SerializeFloat(At(2, 3), s); *s = ','; ++s;
	s = SerializeFloat(At(3, 0), s); *s = ','; ++s;
	s = SerializeFloat(At(3, 1), s); *s = ','; ++s;
	s = SerializeFloat(At(3, 2), s); *s = ','; ++s;
	s = SerializeFloat(At(3, 3), s);
	assert(s+1 - str < 512);
	MARK_UNUSED(s);
	return str;
}

StringT float4x4::ToString2() const
{
	char str[256];
	sprintf(str, "float4x4(X:(%.2f,%.2f,%.2f,%.2f) Y:(%.2f,%.2f,%.2f,%.2f) Z:(%.2f,%.2f,%.2f,%.2f), Pos:(%.2f,%.2f,%.2f,%.2f))",
		At(0, 0), At(1, 0), At(2, 0), At(3, 0),
		At(0, 1), At(1, 1), At(2, 1), At(3, 1),
		At(0, 2), At(1, 2), At(2, 2), At(3, 2),
		At(0, 3), At(1, 3), At(2, 3), At(3, 3));

	return str;
}
#endif

float3 float4x4::ToEulerXYX() const { float3 f; ExtractEulerXYX(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerXZX() const { float3 f; ExtractEulerXZX(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerYXY() const { float3 f; ExtractEulerYXY(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerYZY() const { float3 f; ExtractEulerYZY(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerZXZ() const { float3 f; ExtractEulerZXZ(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerZYZ() const { float3 f; ExtractEulerZYZ(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerXYZ() const { float3 f; ExtractEulerXYZ(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerXZY() const { float3 f; ExtractEulerXZY(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerYXZ() const { float3 f; ExtractEulerYXZ(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerYZX() const { float3 f; ExtractEulerYZX(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerZXY() const { float3 f; ExtractEulerZXY(*this, f[0], f[1], f[2]); return f; }
float3 float4x4::ToEulerZYX() const { float3 f; ExtractEulerZYX(*this, f[0], f[1], f[2]); return f; }

float3 float4x4::ExtractScale() const
{
	return float3(Col3(0).Length(), Col3(1).Length(), Col3(2).Length());
}

void float4x4::Decompose(float3 &translate, Quat &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal3());

	float3x3 r;
	Decompose(translate, r, scale);
	rotate = Quat(r);

	// Test that composing back yields the original float4x4.
	assume(float4x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

void float4x4::Decompose(float3 &translate, float3x3 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal3());

	assume(Row(3).Equals(0,0,0,1));

	assume(this->IsColOrthogonal3());
	
	translate = Col3(3);
	rotate = RotatePart();
	scale.x = rotate.Col3(0).Length();
	scale.y = rotate.Col3(1).Length();
	scale.z = rotate.Col3(2).Length();
	assume(!EqualAbs(scale.x, 0));
	assume(!EqualAbs(scale.y, 0));
	assume(!EqualAbs(scale.z, 0));
	rotate.ScaleCol(0, 1.f / scale.x);
	rotate.ScaleCol(1, 1.f / scale.y);
	rotate.ScaleCol(2, 1.f / scale.z);

	// Test that composing back yields the original float4x4.
	assume(float4x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

void float4x4::Decompose(float3 &translate, float3x4 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal3());

	float3x3 r;
	Decompose(translate, r, scale);
	rotate.SetRotatePart(r);
	rotate.SetTranslatePart(0,0,0);

	// Test that composing back yields the original float4x4.
	assume(float4x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

void float4x4::Decompose(float3 &translate, float4x4 &rotate, float3 &scale) const
{
	assume(this->IsColOrthogonal3());

	float3x3 r;
	Decompose(translate, r, scale);
	rotate.SetRotatePart(r);
	rotate.SetTranslatePart(0,0,0);
	rotate.SetRow(3, 0, 0, 0, 1);

	// Test that composing back yields the original float4x4.
	assume(float4x4::FromTRS(translate, rotate, scale).Equals(*this, 0.1f));
}

float4x4 float4x4::Abs() const
{
	float4x4 ret;
#ifdef MATH_AUTOMATIC_SSE
	ret.row[0] = abs_ps(row[0]);
	ret.row[1] = abs_ps(row[1]);
	ret.row[2] = abs_ps(row[2]);
	ret.row[3] = abs_ps(row[3]);
#else
	for(int iy = 0; iy < 4; ++iy)
		for(int ix = 0; ix < 4; ++ix)
			ret.v[iy][ix] = MATH_NS::Abs(v[iy][ix]);
#endif
	return ret;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float4x4 &rhs)
{
	out << rhs.ToString();
	return out;
}
#endif

float4x4 operator *(const Quat &lhs, const float4x4 &rhs)
{
	float3x3 rot(lhs);
	return rot * rhs;
}

float4x4 operator *(const float3x3 &lhs, const float4x4 &rhs)
{
	float4x4 r;
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	mat3x3_mul_mat4x4_sse(r.row, lhs.ptr(), rhs.row);
#else
	r[0][0] = lhs.At(0, 0) * rhs.At(0, 0) + lhs.At(0, 1) * rhs.At(1, 0) + lhs.At(0, 2) * rhs.At(2, 0);
	r[0][1] = lhs.At(0, 0) * rhs.At(0, 1) + lhs.At(0, 1) * rhs.At(1, 1) + lhs.At(0, 2) * rhs.At(2, 1);
	r[0][2] = lhs.At(0, 0) * rhs.At(0, 2) + lhs.At(0, 1) * rhs.At(1, 2) + lhs.At(0, 2) * rhs.At(2, 2);
	r[0][3] = lhs.At(0, 0) * rhs.At(0, 3) + lhs.At(0, 1) * rhs.At(1, 3) + lhs.At(0, 2) * rhs.At(2, 3);
	
	r[1][0] = lhs.At(1, 0) * rhs.At(0, 0) + lhs.At(1, 1) * rhs.At(1, 0) + lhs.At(1, 2) * rhs.At(2, 0);
	r[1][1] = lhs.At(1, 0) * rhs.At(0, 1) + lhs.At(1, 1) * rhs.At(1, 1) + lhs.At(1, 2) * rhs.At(2, 1);
	r[1][2] = lhs.At(1, 0) * rhs.At(0, 2) + lhs.At(1, 1) * rhs.At(1, 2) + lhs.At(1, 2) * rhs.At(2, 2);
	r[1][3] = lhs.At(1, 0) * rhs.At(0, 3) + lhs.At(1, 1) * rhs.At(1, 3) + lhs.At(1, 2) * rhs.At(2, 3);
	
	r[2][0] = lhs.At(2, 0) * rhs.At(0, 0) + lhs.At(2, 1) * rhs.At(1, 0) + lhs.At(2, 2) * rhs.At(2, 0);
	r[2][1] = lhs.At(2, 0) * rhs.At(0, 1) + lhs.At(2, 1) * rhs.At(1, 1) + lhs.At(2, 2) * rhs.At(2, 1);
	r[2][2] = lhs.At(2, 0) * rhs.At(0, 2) + lhs.At(2, 1) * rhs.At(1, 2) + lhs.At(2, 2) * rhs.At(2, 2);
	r[2][3] = lhs.At(2, 0) * rhs.At(0, 3) + lhs.At(2, 1) * rhs.At(1, 3) + lhs.At(2, 2) * rhs.At(2, 3);
	
	r[3][0] = rhs.At(3, 0);
	r[3][1] = rhs.At(3, 1);
	r[3][2] = rhs.At(3, 2);
	r[3][3] = rhs.At(3, 3);
#endif
	return r;
}

float4x4 operator *(const float3x4 &lhs, const float4x4 &rhs)
{
	float4x4 r;
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	mat3x4_mul_mat4x4_sse(r.row, lhs.row, rhs.row);
#else
	r[0][0] = lhs.At(0, 0) * rhs.At(0, 0) + lhs.At(0, 1) * rhs.At(1, 0) + lhs.At(0, 2) * rhs.At(2, 0) + lhs.At(0, 3) * rhs.At(3, 0);
	r[0][1] = lhs.At(0, 0) * rhs.At(0, 1) + lhs.At(0, 1) * rhs.At(1, 1) + lhs.At(0, 2) * rhs.At(2, 1) + lhs.At(0, 3) * rhs.At(3, 1);
	r[0][2] = lhs.At(0, 0) * rhs.At(0, 2) + lhs.At(0, 1) * rhs.At(1, 2) + lhs.At(0, 2) * rhs.At(2, 2) + lhs.At(0, 3) * rhs.At(3, 2);
	r[0][3] = lhs.At(0, 0) * rhs.At(0, 3) + lhs.At(0, 1) * rhs.At(1, 3) + lhs.At(0, 2) * rhs.At(2, 3) + lhs.At(0, 3) * rhs.At(3, 3);
	
	r[1][0] = lhs.At(1, 0) * rhs.At(0, 0) + lhs.At(1, 1) * rhs.At(1, 0) + lhs.At(1, 2) * rhs.At(2, 0) + lhs.At(1, 3) * rhs.At(3, 0);
	r[1][1] = lhs.At(1, 0) * rhs.At(0, 1) + lhs.At(1, 1) * rhs.At(1, 1) + lhs.At(1, 2) * rhs.At(2, 1) + lhs.At(1, 3) * rhs.At(3, 1);
	r[1][2] = lhs.At(1, 0) * rhs.At(0, 2) + lhs.At(1, 1) * rhs.At(1, 2) + lhs.At(1, 2) * rhs.At(2, 2) + lhs.At(1, 3) * rhs.At(3, 2);
	r[1][3] = lhs.At(1, 0) * rhs.At(0, 3) + lhs.At(1, 1) * rhs.At(1, 3) + lhs.At(1, 2) * rhs.At(2, 3) + lhs.At(1, 3) * rhs.At(3, 3);
	
	r[2][0] = lhs.At(2, 0) * rhs.At(0, 0) + lhs.At(2, 1) * rhs.At(1, 0) + lhs.At(2, 2) * rhs.At(2, 0) + lhs.At(2, 3) * rhs.At(3, 0);
	r[2][1] = lhs.At(2, 0) * rhs.At(0, 1) + lhs.At(2, 1) * rhs.At(1, 1) + lhs.At(2, 2) * rhs.At(2, 1) + lhs.At(2, 3) * rhs.At(3, 1);
	r[2][2] = lhs.At(2, 0) * rhs.At(0, 2) + lhs.At(2, 1) * rhs.At(1, 2) + lhs.At(2, 2) * rhs.At(2, 2) + lhs.At(2, 3) * rhs.At(3, 2);
	r[2][3] = lhs.At(2, 0) * rhs.At(0, 3) + lhs.At(2, 1) * rhs.At(1, 3) + lhs.At(2, 2) * rhs.At(2, 3) + lhs.At(2, 3) * rhs.At(3, 3);
	
	r[3][0] = rhs.At(3, 0);
	r[3][1] = rhs.At(3, 1);
	r[3][2] = rhs.At(3, 2);
	r[3][3] = rhs.At(3, 3);
#endif
	return r;
}

float4 operator *(const float4 &lhs, const float4x4 &rhs)
{
#ifdef MATH_AUTOMATIC_SSE
	return vec4_mul_mat4x4(lhs.v, rhs.row);
#else
	return float4(lhs.x * rhs.At(0, 0) + lhs.y * rhs.At(1, 0) + lhs.z * rhs.At(2, 0) + lhs.w * rhs.At(3, 0),
				  lhs.x * rhs.At(0, 1) + lhs.y * rhs.At(1, 1) + lhs.z * rhs.At(2, 1) + lhs.w * rhs.At(3, 1),
				  lhs.x * rhs.At(0, 2) + lhs.y * rhs.At(1, 2) + lhs.z * rhs.At(2, 2) + lhs.w * rhs.At(3, 2),
				  lhs.x * rhs.At(0, 3) + lhs.y * rhs.At(1, 3) + lhs.z * rhs.At(2, 3) + lhs.w * rhs.At(3, 3));
#endif
}

float4x4 float4x4::Mul(const float3x3 &rhs) const { return *this * rhs; }
float4x4 float4x4::Mul(const float3x4 &rhs) const { return *this * rhs; }
float4x4 float4x4::Mul(const float4x4 &rhs) const { return *this * rhs; }
float4x4 float4x4::Mul(const Quat &rhs) const { return *this * rhs; }
float3 float4x4::MulPos(const float3 &pointVector) const { return this->TransformPos(pointVector); }
float3 float4x4::MulDir(const float3 &directionVector) const { return this->TransformDir(directionVector); }
float4 float4x4::Mul(const float4 &vector) const { return *this * vector; }

const float4x4 float4x4::zero	 = float4x4(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);
const float4x4 float4x4::identity = float4x4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
const float4x4 float4x4::nan = float4x4(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);

MATH_END_NAMESPACE
