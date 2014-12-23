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

/** @file OBB.cpp
	@author Jukka Jylänki
	@brief Implementation for the Oriented Bounding Box (OBB) geometry object. */
#include "OBB.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include <utility>
#endif
#include "../Math/MathFunc.h"
#include "AABB.h"
#include "Frustum.h"
#include "../Algorithm/Random/LCG.h"
#include "LineSegment.h"
#include "Line.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Sphere.h"
#include "Capsule.h"
#include "../Math/float2.inl"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "PBVolume.h"
#include "Ray.h"
#include "Triangle.h"
#include <stdlib.h>
#include <unordered_map>

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Algorithm/Sort/Sort.h"
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
#include "../Math/float4_sse.h"
#include "../Math/float4x4_sse.h"
#endif

MATH_BEGIN_NAMESPACE

OBB::OBB(const vec &pos, const vec &r, const vec &axis0, const vec &axis1, const vec &axis2)
:pos(pos), r(r)
{
	axis[0] = axis0;
	axis[1] = axis1;
	axis[2] = axis2;
}

OBB::OBB(const AABB &aabb)
{
	SetFrom(aabb);
}

void OBB::SetNegativeInfinity()
{
	pos = POINT_VEC_SCALAR(0.f);
	r.SetFromScalar(-FLOAT_INF);
	axis[0] = DIR_VEC(1,0,0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

void OBB::SetFrom(const AABB &aabb)
{
	pos = aabb.CenterPoint();
	r = aabb.HalfSize();
	axis[0] = DIR_VEC(1, 0, 0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

template<typename Matrix>
void OBBSetFrom(OBB &obb, const AABB &aabb, const Matrix &m)
{
	assume(m.IsColOrthogonal()); // We cannot convert transform an AABB to OBB if it gets sheared in the process.
	assume(m.HasUniformScale()); // Nonuniform scale will produce shear as well.
	obb.pos = m.MulPos(aabb.CenterPoint());
	obb.r = aabb.HalfSize();
	obb.axis[0] = DIR_VEC(m.Col(0));
	obb.axis[1] = DIR_VEC(m.Col(1));
	obb.axis[2] = DIR_VEC(m.Col(2));
	// If the matrix m contains scaling, propagate the scaling from the axis vectors to the half-length vectors,
	// since we want to keep the axis vectors always normalized in our representation.
	float matrixScale = obb.axis[0].LengthSq();
	matrixScale = Sqrt(matrixScale);
	obb.r *= matrixScale;
	matrixScale = 1.f / matrixScale;
	obb.axis[0] *= matrixScale;
	obb.axis[1] *= matrixScale;
	obb.axis[2] *= matrixScale;

//	mathassert(vec::AreOrthogonal(obb.axis[0], obb.axis[1], obb.axis[2]));
//	mathassert(vec::AreOrthonormal(obb.axis[0], obb.axis[1], obb.axis[2]));
	///@todo Would like to simply do the above, but instead numerical stability requires to do the following:
	vec::Orthonormalize(obb.axis[0], obb.axis[1], obb.axis[2]);
}

void OBB::SetFrom(const AABB &aabb, const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float3x4 &transform)
{
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float4x4 &transform)
{
	assume(transform.Row(3).Equals(0,0,0,1));
	OBBSetFrom(*this, aabb, transform.Float3x4Part());
}

void OBB::SetFrom(const AABB &aabb, const Quat &transform)
{
	OBBSetFrom(*this, aabb, float3x3(transform));
}

void OBB::SetFrom(const Sphere &sphere)
{
	pos = sphere.pos;
	r.SetFromScalar(sphere.r);
	axis[0] = DIR_VEC(1,0,0);
	axis[1] = DIR_VEC(0,1,0);
	axis[2] = DIR_VEC(0,0,1);
}

#ifdef MATH_CONTAINERLIB_SUPPORT
bool OBB::SetFrom(const Polyhedron &polyhedron)
{
	if (!polyhedron.v.empty())
	{
		*this = OBB::OptimalEnclosingOBB((vec*)&polyhedron.v[0], (int)polyhedron.v.size());
		return true;
	}
	else
	{
		SetNegativeInfinity();
		return false;
	}
}
#endif

#if 0
void OBB::SetFromApproximate(const vec *pointArray, int numPoints)
{
	*this = PCAEnclosingOBB(pointArray, numPoints);
}
#endif

Polyhedron OBB::ToPolyhedron() const
{
	// Note to maintainer: This function is an exact copy of AABB:ToPolyhedron() and Frustum::ToPolyhedron().

	Polyhedron p;
	// Populate the corners of this OBB.
	// The will be in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
	for(int i = 0; i < 8; ++i)
		p.v.push_back(CornerPoint(i));

	// Generate the 6 faces of this OBB.
	const int faces[6][4] =
	{
		{ 0, 1, 3, 2 }, // X-
		{ 4, 6, 7, 5 }, // X+
		{ 0, 4, 5, 1 }, // Y-
		{ 7, 6, 2, 3 }, // Y+
		{ 0, 2, 6, 4 }, // Z-
		{ 1, 5, 7, 3 }, // Z+
	};

	for(int f = 0; f < 6; ++f)
	{
		Polyhedron::Face face;
		for(int v = 0; v < 4; ++v)
			face.v.push_back(faces[f][v]);
		p.f.push_back(face);
	}

	return p;
}

PBVolume<6> OBB::ToPBVolume() const
{
	PBVolume<6> pbVolume;
	for(int i = 0; i < 6; ++i)
		pbVolume.p[i] = FacePlane(i);

	return pbVolume;
}

AABB OBB::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetFrom(*this);
	return aabb;
}

#if 0

AABB OBB::MaximalContainedAABB() const
{
#ifdef _MSC_VER
#pragma warning(OBB::MaximalContainedAABB not implemented!)
#else
#warning OBB::MaximalContainedAABB not implemented!
#endif
	assume(false && "OBB::MaximalContainedAABB not implemented!"); /// @todo Implement.
	return AABB();
}
#endif

Sphere OBB::MinimalEnclosingSphere() const
{
	Sphere s;
	s.pos = pos;
	s.r = HalfDiagonal().Length();
	return s;
}

Sphere OBB::MaximalContainedSphere() const
{
	Sphere s;
	s.pos = pos;
	s.r = r.MinElement();
	return s;
}

bool OBB::IsFinite() const
{
	return pos.IsFinite() && r.IsFinite() && axis[0].IsFinite() && axis[1].IsFinite() && axis[2].IsFinite();
}

bool OBB::IsDegenerate() const
{
	return !(r.x > 0.f && r.y > 0.f && r.z > 0.f);
}

vec OBB::CenterPoint() const
{
	return pos;
}

vec OBB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	return pos + axis[0] * (2.f * r.x * x - r.x)
			   + axis[1] * (2.f * r.y * y - r.y)
			   + axis[2] * (2.f * r.z * z - r.z);
}

LineSegment OBB::Edge(int edgeIndex) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	switch(edgeIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return LineSegment(CornerPoint(0), CornerPoint(1));
		case 1: return LineSegment(CornerPoint(0), CornerPoint(2));
		case 2: return LineSegment(CornerPoint(0), CornerPoint(4));
		case 3: return LineSegment(CornerPoint(1), CornerPoint(3));
		case 4: return LineSegment(CornerPoint(1), CornerPoint(5));
		case 5: return LineSegment(CornerPoint(2), CornerPoint(3));
		case 6: return LineSegment(CornerPoint(2), CornerPoint(6));
		case 7: return LineSegment(CornerPoint(3), CornerPoint(7));
		case 8: return LineSegment(CornerPoint(4), CornerPoint(5));
		case 9: return LineSegment(CornerPoint(4), CornerPoint(6));
		case 10: return LineSegment(CornerPoint(5), CornerPoint(7));
		case 11: return LineSegment(CornerPoint(6), CornerPoint(7));
	}
}

vec OBB::CornerPoint(int cornerIndex) const
{	
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return pos - r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 1: return pos - r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 2: return pos - r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 3: return pos - r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
		case 4: return pos + r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 5: return pos + r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 6: return pos + r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 7: return pos + r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
	}
}

vec OBB::ExtremePoint(const vec &direction) const
{
	vec pt = pos;
	pt += axis[0] * (Dot(direction, axis[0]) >= 0.f ? r.x : -r.x);
	pt += axis[1] * (Dot(direction, axis[1]) >= 0.f ? r.y : -r.y);
	pt += axis[2] * (Dot(direction, axis[2]) >= 0.f ? r.z : -r.z);
	return pt;
}

vec OBB::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void OBB::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	float x = Abs(Dot(direction, axis[0]) * r.x);
	float y = Abs(Dot(direction, axis[1]) * r.y);
	float z = Abs(Dot(direction, axis[2]) * r.z);
	float pt = Dot(direction, pos);
	outMin = pt - x - y - z;
	outMax = pt + x + y + z;
}

int OBB::UniqueFaceNormals(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

int OBB::UniqueEdgeDirections(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

vec OBB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	edgeIndex = Clamp(edgeIndex, 0, 11);
	vec d = axis[edgeIndex/4] * (2.f * u - 1.f) * r[edgeIndex/4];
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.y * axis[1] - r.z * axis[2] + d;
	case 1: return pos - r.y * axis[1] + r.z * axis[2] + d;
	case 2: return pos + r.y * axis[1] - r.z * axis[2] + d;
	case 3: return pos + r.y * axis[1] + r.z * axis[2] + d;

	case 4: return pos - r.x * axis[0] - r.z * axis[2] + d;
	case 5: return pos - r.x * axis[0] + r.z * axis[2] + d;
	case 6: return pos + r.x * axis[0] - r.z * axis[2] + d;
	case 7: return pos + r.x * axis[0] + r.z * axis[2] + d;

	case 8: return pos - r.x * axis[0] - r.y * axis[1] + d;
	case 9: return pos - r.x * axis[0] + r.y * axis[1] + d;
	case 10: return pos + r.x * axis[0] - r.y * axis[1] + d;
	case 11: return pos + r.x * axis[0] + r.y * axis[1] + d;
	}
}

vec OBB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.x * axis[0];
	case 1: return pos + r.x * axis[0];
	case 2: return pos - r.y * axis[1];
	case 3: return pos + r.y * axis[1];
	case 4: return pos - r.z * axis[2];
	case 5: return pos + r.z * axis[2];
	}
}

vec OBB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	int uIdx = faceIndex/2;
	int vIdx = (faceIndex/2 + 1) % 3;
	vec U = axis[uIdx] * (2.f * u - 1.f) * r[uIdx];
	vec V = axis[vIdx] * (2.f * v - 1.f) * r[vIdx];
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.z * axis[2] + U + V;
	case 1: return pos + r.z * axis[2] + U + V;
	case 2: return pos - r.x * axis[0] + U + V;
	case 3: return pos + r.x * axis[0] + U + V;
	case 4: return pos - r.y * axis[1] + U + V;
	case 5: return pos + r.y * axis[1] + U + V;
	}
}

Plane OBB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return Plane(FaceCenterPoint(0), -axis[0]);
	case 1: return Plane(FaceCenterPoint(1), axis[0]);
	case 2: return Plane(FaceCenterPoint(2), -axis[1]);
	case 3: return Plane(FaceCenterPoint(3), axis[1]);
	case 4: return Plane(FaceCenterPoint(4), -axis[2]);
	case 5: return Plane(FaceCenterPoint(5), axis[2]);
	}
}

void OBB::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

void OBB::GetFacePlanes(Plane *outPlaneArray) const
{
	assume(outPlaneArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPlaneArray)
		return;
#endif
	for(int i = 0; i < 6; ++i)
		outPlaneArray[i] = FacePlane(i);
}

/// See Christer Ericson's book Real-Time Collision Detection, page 83.
void OBB::ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest, float &smallestD, float &largestD)
{
	assume(pointArray || numPoints == 0);

	idxSmallest = idxLargest = 0;

#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pointArray)
		return;
#endif

	smallestD = FLOAT_INF;
	largestD = -FLOAT_INF;
	for(int i = 0; i < numPoints; ++i)
	{
		float d = Dot(pointArray[i], dir);
		if (d < smallestD)
		{
			smallestD = d;
			idxSmallest = i;
		}
		if (d > largestD)
		{
			largestD = d;
			idxLargest = i;
		}
	}
}

#if 0
OBB OBB::PCAEnclosingOBB(const vec * /*pointArray*/, int /*numPoints*/)
{
#ifdef _MSC_VER
#pragma warning(OBB::PCAEnclosingOBB not implemented!)
#else
#warning OBB::PCAEnclosingOBB not implemented!
#endif
	assume(false && "OBB::PCAEnclosingOBB not implemented!"); /// @todo Implement.
	return OBB();
}
#endif

#define LEX_ORDER(x, y) if ((x) < (y)) return -1; else if ((x) > (y)) return 1;
int LexFloat3Cmp(const vec &a, const vec &b)
{
	LEX_ORDER(a.x, b.x);
	return 0;
}
int LexFloat3CmpV(const void *a, const void *b) { return LexFloat3Cmp(*(const vec*)a, *(const vec*)b); }

float SmallestOBBVolumeJiggle(const vec &edge_, const Polyhedron &convexHull, std::vector<float2> &pts,
//	const std::vector<std::vector<int> > &adjacencyData,
//	std::vector<int> &floodFillVisited,
//	int &floodFillVisitColor,
	int jiggles, vec &outEdgeA, vec &outEdgeB)
{
	vec edge = edge_;
	bool takeC1 = true;
	float rectArea = -1.f, edgeLength = -1.f;
	float2 c10, c20;
	vec u, v;
	for(int i = 0; i <= jiggles; ++i)
	{
		int e1, e2;
		OBB::ExtremePointsAlongDirection(edge, (const vec*)&convexHull.v[0], convexHull.v.size(), e1, e2);
		edgeLength = Abs(Dot((vec)convexHull.v[e1] - convexHull.v[e2], edge));
//		float dMin, dMax;
//		int emin = convexHull.ExtremeVertexConvex(adjacencyData, -edge, floodFillVisited, floodFillVisitColor++, dMin, 0);
//		int emax = convexHull.ExtremeVertexConvex(adjacencyData, edge, floodFillVisited, floodFillVisitColor++, dMax, 0);
//		edgeLength = emin + emax;

		edge.PerpendicularBasis(u, v);
		for(size_t k = 0; k < convexHull.v.size(); ++k)
			pts[k] = float2(u.Dot(convexHull.v[k]), v.Dot(convexHull.v[k]));

		float2 rectCenter;
		float2 uDir;
		float2 vDir;
		float minU, maxU, minV, maxV;
		rectArea = float2::MinAreaRectInPlace(&pts[0], (int)pts.size(), rectCenter, uDir, vDir, minU, maxU, minV, maxV);

		c10 = (maxV - minV) * vDir;
		c20 = (maxU - minU) * uDir;

		if (i < jiggles)
		{
			if (takeC1)
				edge = (c10.x*u + c10.y*v).Normalized();
			else
				edge = (c20.x*u + c20.y*v).Normalized();
			takeC1 = !takeC1;
		}
	}
	outEdgeA = edge;
	outEdgeB = (c10.x*u + c10.y*v).Normalized();
	return rectArea * edgeLength;
}

// Moves the floating point sign bit from src to dst.
#ifdef MATH_SSE
#define MoveSign(dst, src) \
	dst = s4f_x(xor_ps(setx_ps(dst), and_ps(setx_ps(src), simd4fSignBit))); \
	src = s4f_x(abs_ps(setx_ps(src)));
#else
#define MoveSign(dst, src) if (src < 0.f) { dst = -dst; src = -src; }
#endif

int ComputeBasis(const vec &f1a, const vec &f1b,
	const vec &f2a, const vec &f2b,
	const vec &f3a, const vec &f3b,
	vec *n1,
	vec *n2,
	vec *n3)
{
	float a1_a2 = f1a.Dot(f2a);
	float a1_a3 = f1a.Dot(f3a);
//	float a1_b1 = f1a.Dot(f1b);
	float a1_b2 = f1a.Dot(f2b);
	float a1_b3 = f1a.Dot(f3b);

	float a2_a3 = f2a.Dot(f3a);
	float a2_b1 = f2a.Dot(f1b);
//	float a2_b2 = f2a.Dot(f2b);
	float a2_b3 = f2a.Dot(f3b);

	float a3_b1 = f3a.Dot(f1b);
	float a3_b2 = f3a.Dot(f2b);
//	float a3_b3 = f3a.Dot(f3b);

	float b1_b2 = f1b.Dot(f2b);
	float b1_b3 = f1b.Dot(f3b);

	float b2_b3 = f2b.Dot(f3b);

	float A = (a3_b1 - b1_b3) * ((a3_b2 - b2_b3) * a1_a2 + (a2_b3 - a2_a3) * a1_b2)
	        + (a1_a3 - a1_b3) * ((a2_a3 - a2_b3) * b1_b2 + (b2_b3 - a3_b2) * a2_b1);

/*
	float A2 = 
		(f3a-f3b).Dot(f1b) *
		(
		    (f3a-f3b).Dot(f2b) * f1a.Dot(f2a)
		  - (f3a-f3b).Dot(f2a) * f1a.Dot(f2b)
		)
		+
		(f3a-f3b).Dot(f1a) *
		(
		    (f3a-f3b).Dot(f2a) * f1b.Dot(f2b)
		  - (f3a-f3b).Dot(f2b) * f1b.Dot(f2a)
		);

	if (!EqualAbs(A, A2)) LOGE("Asdfasd");
*/
	float B = a1_b3 * (a2_b1 * (b2_b3 * 2.f - a3_b2) + b1_b2 * (a2_a3 - a2_b3 * 2.f))
	        + a1_a3 * (b1_b2 * a2_b3 - a2_b1 * b2_b3)
	        + a3_b1 * (a1_a2 * b2_b3 - a1_b2 * a2_b3)
	        + b1_b3 * (a1_a2 * (a3_b2 - b2_b3 * 2.f) + a1_b2 * (a2_b3 * 2.f - a2_a3));
/*
	float B2 =
		
		f1a.Dot(f3b) *
		(
		- f1b.Dot(f2a) * f2b.Dot(f3a)
		+ f1b.Dot(f2a) * f2b.Dot(f3b) * 2.f
		+ f1b.Dot(f2b) * f2a.Dot(f3a)
		- f1b.Dot(f2b) * f2a.Dot(f3b) * 2.f
		)

		+ f1a.Dot(f3a) *
		(
		+ f1b.Dot(f2b) * f2a.Dot(f3b)
		- f1b.Dot(f2a) * f2b.Dot(f3b)
		)

		+ f1b.Dot(f3a) *
		(
		+ f1a.Dot(f2a) * f2b.Dot(f3b)
		- f1a.Dot(f2b) * f2a.Dot(f3b)
		)

		+ f1b.Dot(f3b) *
		(
		+ f1a.Dot(f2a) * f2b.Dot(f3a)
		- f1a.Dot(f2a) * f2b.Dot(f3b) * 2.f
		- f1a.Dot(f2b) * f2a.Dot(f3a)
		+ f1a.Dot(f2b) * f2a.Dot(f3b) * 2.f
		);

	if (!EqualAbs(B, B2)) LOGE("Bsdfasd");
*/
	float C = a1_b3 * (b1_b2 * a2_b3 - a2_b1 * b2_b3)
	        + b1_b3 * (a1_a2 * b2_b3 - a1_b2 * a2_b3);
/*
	float C2 =
		
		f1a.Dot(f3b) *
		(
		- f1b.Dot(f2a) * f2b.Dot(f3b)
		+ f1b.Dot(f2b) * f2a.Dot(f3b)
		)
		+ f1b.Dot(f3b) *
		(
		+ f1a.Dot(f2a) * f2b.Dot(f3b)
		- f1a.Dot(f2b) * f2a.Dot(f3b)
		);

	if (!EqualAbs(C, C2)) LOGE("Csdfasd");
*/
	float e = b2_b3;
	float f = a2_b3 - b2_b3;
	float g = a3_b2 - b2_b3;
	float h = a2_a3 - a3_b2 - a2_b3 + b2_b3;

	float i = b1_b3;
	float j = a1_b3 - b1_b3;
	float k = a3_b1 - b1_b3;
	float l = a1_a3 - a3_b1 - a1_b3 + b1_b3;
/*
	float e = f2b.Dot(f3b);
	float f = (f2a-f2b).Dot(f3b);
	float g = (f3a-f3b).Dot(f2b);
	float h = (f2a-f2b).Dot(f3a-f3b);

	float i = f1b.Dot(f3b);
	float j = (f1a-f1b).Dot(f3b);
	float k = (f3a-f3b).Dot(f1b);
	float l = (f1a-f1b).Dot(f3a-f3b);
*/
	float D = B*B - 4.f * A * C;

	int nSolutions = 0;
	if (D > 0.f)
	{
		D = Sqrt(D);
//		float denomV = A;
//		float numV = D - B;
		float v = (-B + D) / (2.f * A);
		//MoveSign(numV, denomV);

		//if (numV < 0.f)
		//	return 0;
		//denomV = 0.5f / denomV;
		//float v = numV * denomV;
		float t = (-i-v*k)/(j + v*l);
		float u = (-e -v*g) / (f + v*h);

		if (t >= 0.f && u >= 0.f && v >= 0.f && t <= 1.f && u <= 1.f && v <= 1.f)
		{
			n1[nSolutions] = (f1a*t + f1b*(1-t)).Normalized();
			n2[nSolutions] = (f2a*u + f2b*(1-u)).Normalized();
			n3[nSolutions] = (f3a*v + f3b*(1-v)).Normalized();
			if (n1[nSolutions].IsPerpendicular(n2[nSolutions])
				&& n2[nSolutions].IsPerpendicular(n3[nSolutions])
				&& n1[nSolutions].IsPerpendicular(n3[nSolutions]))
				++nSolutions;
			else
			{
//				LOGE("Notperp! %f vs %f vs %f", n1[nSolutions].Dot(n2[nSolutions]),
//					n1[nSolutions].Dot(n3[nSolutions]), n2[nSolutions].Dot(n3[nSolutions]));
			}
		}

		v = (-B - D) / (2.f * A);
		//v = (-B - D) * denomV;
		t = (-i-v*k)/(j + v*l);
		u = (-e -v*g) / (f + v*h);

		if (t >= 0.f && u >= 0.f && v >= 0.f && t <= 1.f && u <= 1.f && v <= 1.f)
		{
			n1[nSolutions] = (f1a*t + f1b*(1-t)).Normalized();
			n2[nSolutions] = (f2a*u + f2b*(1-u)).Normalized();
			n3[nSolutions] = (f3a*v + f3b*(1-v)).Normalized();
			if (n1[nSolutions].IsPerpendicular(n2[nSolutions])
				&& n2[nSolutions].IsPerpendicular(n3[nSolutions])
				&& n1[nSolutions].IsPerpendicular(n3[nSolutions]))
				++nSolutions;
			else
			{
//				LOGE("Notperp! %f vs %f vs %f", n1[nSolutions].Dot(n2[nSolutions]),
//					n1[nSolutions].Dot(n3[nSolutions]), n2[nSolutions].Dot(n3[nSolutions]));
			}
		}
	}
	else if (D == 0.f)
	{
		float v = -B / (2.f * A);
		float t = (-i-v*k)/(j + v*l);
		float u = (-e -v*g) / (f + v*h);

		if (t >= 0.f && u >= 0.f && v >= 0.f && t <= 1.f && u <= 1.f && v <= 1.f)
		{
			n1[nSolutions] = (f1a*t + f1b*(1-t)).Normalized();
			n2[nSolutions] = (f2a*u + f2b*(1-u)).Normalized();
			n3[nSolutions] = (f3a*v + f3b*(1-v)).Normalized();
			if (n1[nSolutions].IsPerpendicular(n2[nSolutions])
				&& n2[nSolutions].IsPerpendicular(n3[nSolutions])
				&& n1[nSolutions].IsPerpendicular(n3[nSolutions]))
				++nSolutions;
			else
			{
//				LOGE("Notperp! %f vs %f vs %f", n1[nSolutions].Dot(n2[nSolutions]),
//					n1[nSolutions].Dot(n3[nSolutions]), n2[nSolutions].Dot(n3[nSolutions]));
			}
		}
	}
	return nSolutions;
}

static bool AreEdgesCompatibleForOBB(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	const float epsilon = 1e-3f;

/*	
	// TODO: It feels sensible that computing four dot products at the same time would be a win,
	//       however at least with AVX enabled, this was benchmarked to be slower than the four individual
	//       dpps versions below.
	simd4f v1 = f1a - f1b;
	simd4f v2 = f2a - f2b;
	simd4f A = mul_ps(f1b, f2b);
	simd4f B = mul_ps(v1, f2b);
	simd4f C = mul_ps(v2, f1b);
	simd4f D = mul_ps(v1, v2);
	_MM_TRANSPOSE4_PS(A, B, C, D);
	A = add_ps(add_ps(A, B), C); // D is all zeroes, not needed.
	float a = s4f_x(A);
	float b = s4f_y(A);
	float c = s4f_z(A);
	float d = s4f_w(A);
*/

	float a = f1b.Dot(f2b);
	float b = (f1a-f1b).Dot(f2b);
	float c = (f2a-f2b).Dot(f1b);
	float d = (f1a-f1b).Dot(f2a-f2b);

	if (d == 0.f)
	{
#if 0
		if (c != 0.f)
		{
			/*
			float u_t0 = -a/c;
			float u_t1 = -(a+b)/c;
			if (Max(u_t0, u_t1) < -epsilon || Min(u_t0, u_t1) > 1.f + epsilon)
				return false;
			*/
			// if (-Min(a/c, (a+b)/c) < -epsilon || -Max(a/c, (a+b)/c) > 1.f + epsilon)
			// if (Min(a/c, (a+b)/c) > epsilon || Max(a/c, (a+b)/c) < -1.f - epsilon)

			// c > 0:
			//   if (Min(a, a+b) > c*epsilon || Max(a, a+b) < c * (-1.f - epsilon))
			// c < 0:
			//   if (Max(a, a+b) < c*epsilon || Min(a, a+b) > c * (-1.f - epsilon))
		}
#endif
		if (c > 0.f)
		{
			if (Min(a, a+b) > c*epsilon || Max(a, a+b) < c * (-1.f - epsilon))
				return false;
		}
		else if (c < 0.f)
		{
			if (Max(a, a+b) < c*epsilon || Min(a, a+b) > c * (-1.f - epsilon))
				return false;
		}
		else
		{
#if 0
			if (b != 0.f)
			{
				float t = -a / b;
				if (t < -epsilon || t > 1.f + epsilon)
					return false;
			}
#endif
			if (b > 0.f)
			{
				if (a > b * epsilon || a < b * (-1.f - epsilon))
					return false;
			}
			else if (b < 0.f)
			{
				if (a < b * epsilon || a > b * (-1.f - epsilon))
					return false;
			}
			else if (a != 0.f)
				return false;
		}
	}
	else
	{
//		float a_b = f1a.Dot(f2b);
//		float c_d = f1a.Dot(f2a-f2b);
//		float denomZero = -c / d;

		if (c == 0.f || c == -d)
		// if (denomZero == 0.f || denomZero == 1.f)
		{
			float t1 = -a/c;
			float t2 = -(a+b)/(c+d);//-a_b/c_d;

			float deriv = a*d-b*c;
			if (deriv == 0.f)
				return t1 >= 0.f && t2 <= 1.f;
			if (c == 0.f)
//			if (denomZero == 0.f)
			{
				return (deriv > 0.f && t2 >= 0.f) || (deriv < 0.f && t2 <= 1.f);
			}
			else
			{
				return (deriv > 0.f && t1 >= 0.f) || (deriv < 0.f && t1 <= 1.f);
			}
		}

		float t1cd = -a*(c+d);
		float t2c = -(a+b)*c;
		float denom = c*(c+d);

		MoveSign(c, d);
//			LOGI("t1: %f, t2: %f", t1, t2);
		//if (denomZero > 0.f && denomZero < 1.f)
//		if ((d > 0.f && c < 0.f && c > -d) || (d < 0.f && c > 0.f && c < -d))
		if (c < 0.f && c > -d)
		{
			if (denom > 0.f)
			{
				if (Min(t1cd, t2c) < denom * -epsilon && Max(t1cd, t2c) > denom * (1.f + epsilon))
					return false;
			}
			else
			{
				if (Max(t1cd, t2c) > denom * -epsilon && Min(t1cd, t2c) < denom * (1.f + epsilon))
					return false;
			}
//			if (Min(t1, t2) < -epsilon && Max(t1, t2) > 1.f + epsilon)
//				return false;
		}
		else
		{
			if (denom > 0.f)
			{
				if (Max(t1cd, t2c) < denom * -epsilon || Min(t1cd, t2c) > denom * (1.f + epsilon))
					return false;
			}
			else
			{
				if (Min(t1cd, t2c) > denom * -epsilon || Max(t1cd, t2c) < denom * (1.f + epsilon))
					return false;
			}
//			if (Max(t1, t2) < -epsilon || Min(t1, t2) > 1.f + epsilon)
//				return false; // If no solution for the two edges already, skip to next edge configuration.
		}
	}
	return true;
}

#define TIMING(...) ((void)0)
#define TIMING_TICK(...) ((void)0)

//#define TIMING_TICK(...) __VA_ARGS__
//#define TIMING LOGI

namespace
{
	struct hash_edge
	{
		size_t operator()(const std::pair<int, int> &e) const
		{
			return (e.first << 16) ^ e.second;
		}
	};
}

bool AreCompatibleOpposingEdges(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b, vec &outN)
{
	/*
		n1 = f1a*t + f1b*(1-t)
		n2 = f2a*u + f2b*(1-u)
		n1 = -c*n2, where c > 0
		f1a*t + f1b*(1-t) = -c*f2a*u - c*f2b*(1-u)
		f1a*t - f1b*t + cu*f2a + c*f2b - cu*f2b = -f1b
		c*f2b + t*(f1a-f1b) + cu*(f2a-f2b) = -f1b

		M * v = -f1b, where

		M = [ f2b, (f1a-f1b), (f2a-f2b) ] column vectors
		v = [c, t, cu]
	*/

	float3x3 A;
	A.SetCol(0, f2b.xyz()); // c
	A.SetCol(1, (f1a - f1b).xyz()); // t
	A.SetCol(2, (f2a - f2b).xyz()); // r = c*u
	float3 x;
	bool success = A.SolveAxb(-f1b.xyz(), x);
	float c = x[0];
	float t = x[1];
	float cu = x[2];
	if (!success || c <= 0.f || t < 0.f || t > 1.f)
		return false;
//	float u = cu / c;
//	if (u < 0.f || u > 1.f)
	if (cu < 0.f || cu > c)
		return false;
	outN = f1b + (f1a-f1b)*t;
	return true;
}

OBB OBB::OptimalEnclosingOBB(const vec *pointArray, int numPoints)
{
	// Precomputation: Generate the convex hull of the input point set. This is because
	// we need vertex-edge-face connectivity information about the convex hull shape, and
	// this also allows discarding all points in the interior of the input hull, which
	// are irrelevant.
	Polyhedron convexHull = Polyhedron::ConvexHull(pointArray, numPoints);
	if (!pointArray || convexHull.v.size() == 0)
	{
		OBB minOBB;
		minOBB.SetNegativeInfinity();
		return minOBB;
	}
	return OptimalEnclosingOBB(convexHull);
}

bool ContainsAndRemove(std::vector<int> &arr, int val)
{
	for(size_t i = 0; i < arr.size(); ++i)
		if (arr[i] == val)
		{
			arr.erase(arr.begin() + i);
			return true;
		}
	return false;
}

bool SortedArrayContains(const std::vector<int> &arr, int i)
{
	size_t left = 0;
	size_t right = arr.size() - 1;
	if (arr[left] == i || arr[right] == i)
		return true;
	if (arr[left] > i || arr[right] < i)
		return false;

	while(left < right)
	{
		int middle = (left + right + 1) >> 1;
		if (arr[middle] < i)
			left = i;
		else if (arr[middle] > i)
			right = i;
		else
			return true;
	}
	return false;
}

OBB OBB::OptimalEnclosingOBB(const Polyhedron &convexHull)
{
	OBB minOBB;
	float minVolume = FLOAT_INF;

	TIMING_TICK(tick_t t1 = Clock::Tick());
	// Precomputation: For each vertex in the convex hull, compute their neighboring vertices.
	std::vector<std::vector<int> > adjacencyData = convexHull.GenerateVertexAdjacencyData(); // O(|V|)
	TIMING_TICK(tick_t t2 = Clock::Tick());
	TIMING("Adjacencygeneration: %f msecs", Clock::TimespanToMillisecondsF(t1, t2));

	// Precomputation: Compute normalized face direction vectors for each face of the hull.
	std::vector<vec_storage> faceNormals;
	faceNormals.reserve(convexHull.NumFaces());
	for(int i = 0; i < convexHull.NumFaces(); ++i) // O(|F|)
		faceNormals.push_back(convexHull.FaceNormal(i));

	TIMING_TICK(tick_t t23 = Clock::Tick());
	TIMING("Facenormalsgen: %f msecs", Clock::TimespanToMillisecondsF(t2, t23));

	// For each edge i, specifies the two vertices v0 and v1 through which that edge goes.
	// This array does not have duplicates, i.e. there only exists one edge index for (v0->v1), and no
	// edge for (v1->v0).
	std::vector<std::pair<int, int> > edges;
	edges.reserve(convexHull.v.size() * 2);
	// For each edge i, specifies the two face indices f0 and f1 that share that edge.
	std::vector<std::pair<int, int> > facesForEdge;
	facesForEdge.reserve(convexHull.v.size()*2);
	// For each vertex pair (v0, v1) through which there is an edge, specifies the index i of the edge that passes through them.
	// This map contains duplicates, so both (v0, v1) and (v1, v0) map to the same edge index.
	std::unordered_map<std::pair<int, int>, int, hash_edge> vertexPairsToEdges;

	// Precomputation: for each edge through vertices (i,j), we need to know
	// the face indices for the two adjoining faces that share the edge.
	// This is O(|V|).
	for (size_t i = 0; i < convexHull.f.size(); ++i)
	{
		const Polyhedron::Face &f = convexHull.f[i];
		int v0 = f.v.back();
		for(size_t j = 0; j < f.v.size(); ++j)
		{
			int v1 = f.v[j];
			std::pair<int, int> e = std::make_pair(v0, v1);
			std::unordered_map<std::pair<int, int>, int, hash_edge>::const_iterator iter = vertexPairsToEdges.find(e);
			if (iter == vertexPairsToEdges.end())
			{
				vertexPairsToEdges[e] = (int)edges.size();
				vertexPairsToEdges[std::make_pair(v1, v0)] = (int)edges.size(); // Mark that we know we have seen v0->v1 already.
				edges.push_back(e);
				facesForEdge.push_back(std::make_pair(i, -1)); // The -1 will be filled once we see the edge v1->v0.
			}
			else
				facesForEdge[iter->second].second = i;
			v0 = v1;
		}
	}
	TIMING_TICK(tick_t t3 = Clock::Tick());
	TIMING("Adjoiningfaces: %f msecs", Clock::TimespanToMillisecondsF(t23, t3));

	// Throughout the whole algorithm, this array stores an auxiliary structure for performing graph searches
	// on the vertices of the convex hull. Conceptually each index of the array stores a boolean whether we
	// have visited that vertex or not during the current search. However storing such booleans is slow, since
	// we would have to perform a linear-time scan through this array before next search to reset each boolean
	// to unvisited false state. Instead, store a number, called a "color" for each vertex to specify whether
	// that vertex has been visited, and manage a global color counter floodFillVisitColor that represents the
	// visited vertices. At any given time, the vertices that have already been visited have the value
	// floodFillVisited[i] == floodFillVisitColor in them. This gives a win that we can perform constant-time
	// clears of the floodFillVisited array, by simply incrementing the "color" counter to clear the array.
	std::vector<int> floodFillVisited(convexHull.v.size());
	int floodFillVisitColor = 1;

	// As a syntactic aid, use the helpers MARK_VISITED(v), HAVE_VISITED_VERTEX(v) and CLEAR_GRAPH_SEARCH to
	// remind of the conceptual meaning of these values.
#define MARK_VERTEX_VISITED(v) (floodFillVisited[(v)] = floodFillVisitColor)
#define HAVE_VISITED_VERTEX(v) (floodFillVisited[(v)] == floodFillVisitColor)
#define CLEAR_GRAPH_SEARCH() (++floodFillVisitColor)

	// Stores for each edge index i the complete list of antipodal vertices for that edge.
	std::vector<std::vector<int> > antipodalPointsForEdge(edges.size());

	// Stores a memory of yet unvisited vertices for current graph search.
	std::vector<int> traverseStack;

	// Since we do several extreme vertex searches, and the search directions have a lot of spatial locality,
	// always start the search for the next extreme vertex from the extreme vertex that was found during the
	// previous iteration for the previous edge. This has been profiled to improve overall performance by as
	// much as 15-25%.
	int startingVertex = 0;

	// Precomputation: for each edge, we need to compute the list of potential antipodal points (points on
	// the opposing face of an enclosing OBB of the face that is flush with the given edge of the polyhedron).
	// This is O(|E|*log(|V|) ?
	for (size_t i = 0; i < edges.size(); ++i) // O(|E|)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		float dummy;
		CLEAR_GRAPH_SEARCH(); // ExtremeVertexConvex performs a graph search, initialize the search data structure for it.
		startingVertex = convexHull.ExtremeVertexConvex(adjacencyData, -f1a, floodFillVisited, floodFillVisitColor, dummy, startingVertex); // O(log(|V|)?
		CLEAR_GRAPH_SEARCH(); // Search through the graph for all adjacent antipodal vertices.
		traverseStack.push_back(startingVertex);
		while(!traverseStack.empty()) // In amortized analysis, only a constant number of vertices are antipodal points for any edge?
		{
			int v = traverseStack.back();
			traverseStack.pop_back();
			MARK_VERTEX_VISITED(v);

			float tMin = 0.f;
			float tMax = 1.f;
			const std::vector<int> &n = adjacencyData[v];
			for(size_t j = 0; j < n.size(); ++j)
			{
				/* Is an edge and a vertex compatible to be antipodal?

					n1 = f1b + (f1a-f1b)*t
					e { v-vn }

					n1.e <= 0
					(f1b + (f1a-f1b)*t).e <= 0
					t*(f1a-f1b).e <= -f1b.e 
					if (f1a-f1b).e > 0:
						t <= -f1b.e / (f1a-f1b).e && t \in [0,1]
						-f1b.e / (f1a-f1b).e >= 0
						f1b.e <= 0
					if (f1a-f1b).e < 0:
						t >= -f1b.e / (f1a-f1b).e && t \in [0,1]
						-f1b.e / (f1a-f1b).e <= 1
						-f1b.e >= (f1a-f1b).e
						f1b.e + (f1a-f1b).e <= 0 
					if (f1a-f1b).e == 0:
						0 <= -f1b.e
				*/
				vec e = (vec)convexHull.v[v] - convexHull.v[n[j]];
				float s = (f1a-f1b).Dot(e);
				float n = -f1b.Dot(e);
				if (s > 0.f)
					tMax = Min(tMax, n / s);
				else if (s < 0.f)
					tMin = Max(tMin, n / s);
				else if (n < 0.f)
					tMax = tMin - 1.f;

				// The interval of possible solutions for t is now degenerate?
				if (tMax - tMin < -1e-4f)
					break;
			}

			if (tMin - 1e-4f <= tMax)
			{
				antipodalPointsForEdge[i].push_back(v);
				for(size_t j = 0; j < n.size(); ++j)
					if (!HAVE_VISITED_VERTEX(n[j]))
						traverseStack.push_back(n[j]);
			}
		}
		// Robustness: If the above search did not find any antipodal points, add the first found extreme
		// point at least, since it is always an antipodal point. This is known to occur very rarely due
		// to numerical imprecision in the above loop over adjacent edges.
		if (antipodalPointsForEdge[i].empty())
			antipodalPointsForEdge[i].push_back(startingVertex);
	}

	TIMING_TICK(
		tick_t t4 = Clock::Tick();
		size_t numTotalAntipodals = 0;
		for (size_t i = 0; i < antipodalPointsForEdge.size(); ++i)
			numTotalAntipodals += antipodalPointsForEdge[i].size();
	);
	TIMING("Antipodalpoints: %f msecs (avg edge has %.3f antipodal points)", Clock::TimespanToMillisecondsF(t3, t4), (float)numTotalAntipodals/edges.size());

	// Stores for each edge i the list of all sidepodal edge indices j that it can form an OBB with.
	std::vector<std::vector<int> > compatibleEdges(edges.size());
	// The array numSmallerCompatibleEdgeIndices remembers for each edge index i the number of edge indices
	// j that are stored in the compatibleEdges array such that j <= i. That is, it stores the precondition
	// that compatibleEdges[i][0]...compatibleEdges[i][numSmallerCompatibleEdgeIndices[i]-1] are all strictly
	// smaller than i. This is used to skip over symmetrical positions and avoid iterating twice over edge
	// pairs i,j and j,i.
	std::vector<int> numSmallerCompatibleEdgeIndices(edges.size());

#if 0
	// Precomputation: Compute all potential companion edges for each edge.
	// This is O(|E|^2)
	// Important! And edge can be its own companion edge! So have each edge test itself during iteration.
	for(size_t i = 0; i < edges.size(); ++i) // O(|E|)
		for(size_t j = i; j < edges.size(); ++j) // O(|E|)
			if (AreEdgesCompatibleForOBB(faceNormals[facesForEdge[i].first], faceNormals[facesForEdge[i].second],
				faceNormals[facesForEdge[j].first], faceNormals[facesForEdge[j].second]))
			{
				compatibleEdges[i].push_back(j);
				compatibleEdgesAll[i].push_back(j);
				if (i != j)
					compatibleEdgesAll[j].push_back(i);
			}
#endif

	// Compute all sidepodal edges for each edge by performing a graph search. The set of sidepodal edges is
	// connected in the graph, which lets us avoid having to iterate over each edge pair of the convex hull.
	for(size_t i = 0; i < edges.size(); ++i)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		float dummy;
		vec dir = f1a.Perpendicular();
		CLEAR_GRAPH_SEARCH();
		startingVertex = convexHull.ExtremeVertexConvex(adjacencyData, dir, floodFillVisited, floodFillVisitColor, dummy, startingVertex);
		CLEAR_GRAPH_SEARCH();
		traverseStack.push_back(startingVertex);
		while(!traverseStack.empty())
		{
			int v = traverseStack.back();
			traverseStack.pop_back();
			MARK_VERTEX_VISITED(v);

			const std::vector<int> &n = adjacencyData[v];
			for(size_t j = 0; j < n.size(); ++j)
			{
				int vAdj = n[j];
				if (HAVE_VISITED_VERTEX(vAdj))
					continue;

				int edge = vertexPairsToEdges[std::make_pair(v, vAdj)];
				if (AreEdgesCompatibleForOBB(f1a, f1b, faceNormals[facesForEdge[edge].first], faceNormals[facesForEdge[edge].second]))
				{
					if ((int)i <= edge)
					{
						compatibleEdges[i].push_back(edge);
						if ((int)i != edge)
						{
							compatibleEdges[edge].push_back(i);
							++numSmallerCompatibleEdgeIndices[edge];
						}
					}
					traverseStack.push_back(vAdj);
				}
			}
		}
	}

	// We will later perform set intersection operations on the compatibleEdges arrays, so these must be sorted.
	for(size_t i = 0; i < compatibleEdges.size(); ++i)
		std::sort(compatibleEdges[i].begin(), compatibleEdges[i].end());

#if 0
	for (size_t i = 0; i < edges.size(); ++i) // O(|E|)
	{
		String s;
		for (size_t j = 0; j < compatibleEdgesAll[i].size(); ++j) // O(|E|)
		{
			String s2;
			s2.SPrintf("%d ", (int)compatibleEdgesAll[i][j]);
			s += s2;
		}
		LOGI("Edge %d is compatible with: %s", (int)i, s.c_str());

		s = "";
		for (size_t j = 0; j < compatibleEdges[i].size(); ++j) // O(|E|)
		{
			String s2;
			s2.SPrintf("%d ", (int)compatibleEdges[i][j]);
			s += s2;
		}
		LOGI("EDGE %d is compatible with: %s", (int)i, s.c_str());
	}
#endif

	TIMING_TICK(
		tick_t t5 = Clock::Tick();
		size_t numTotalEdges = 0;
		for(size_t i = 0; i < compatibleEdges.size(); ++i)
			numTotalEdges += compatibleEdges[i].size();
		);
	TIMING("Companionedges: %f msecs (%d edges have on average %d companion edges each)", Clock::TimespanToMillisecondsF(t4, t5), (int)compatibleEdges.size(), (int)(numTotalEdges/compatibleEdges.size()));

	// Main algorithm body for testing three edges all on adjacent faces of the box.
	// This is O(|E|^2)?
	TIMING_TICK(int numConfigsExplored = 0;);
	for(size_t i = 0; i < edges.size(); ++i) // O(|E|)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];
		for(size_t j = numSmallerCompatibleEdgeIndices[i] /*to remove symmetry, don't start at 0*/; j < compatibleEdgesI.size(); ++j) // O(sqrt(|E|))?
		{
			int edgeJ = compatibleEdgesI[j];
			vec f2a = faceNormals[facesForEdge[edgeJ].first];
			vec f2b = faceNormals[facesForEdge[edgeJ].second];

			const std::vector<int> &compatibleEdgesJ = compatibleEdges[edgeJ];
			size_t s_i = j+1;
			size_t s_j = numSmallerCompatibleEdgeIndices[edgeJ]; // Instead of starting at zero, can skip over first elements so that i <= j to avoid checking same positions twice due to symmetry.
			while(s_i < compatibleEdgesI.size() && s_j < compatibleEdgesJ.size()) // O(sqrt(|E|))?
			{
				if (compatibleEdgesI[s_i] == compatibleEdgesJ[s_j])
				{
					int edgeK = compatibleEdgesJ[s_j];
					++s_i;
					++s_j;

					// Test edge triplet i, edgeJ, edgeK.
					vec f3a = faceNormals[facesForEdge[edgeK].first];
					vec f3b = faceNormals[facesForEdge[edgeK].second];

					vec n1[2], n2[2], n3[2];
					TIMING_TICK(++numConfigsExplored;);
					int nSolutions = ComputeBasis(f1a, f1b, f2a, f2b, f3a, f3b, n1, n2, n3);
					for(int s = 0; s < nSolutions; ++s) // O(constant), nSolutions == 0, 1 or 2.
					{
						// Compute the most extreme points in each direction.
						float maxN1 = n1[s].Dot(convexHull.v[edges[i].first]);
						float maxN2 = n2[s].Dot(convexHull.v[edges[edgeJ].first]);
						float maxN3 = n3[s].Dot(convexHull.v[edges[edgeK].first]);
						float minN1 = FLOAT_INF;
						float minN2 = FLOAT_INF;
						float minN3 = FLOAT_INF;
						for(size_t l = 0; l < antipodalPointsForEdge[i].size(); ++l) // O(constant)?
							minN1 = Min(minN1, n1[s].Dot(convexHull.v[antipodalPointsForEdge[i][l]]));
						for(size_t l = 0; l < antipodalPointsForEdge[edgeJ].size(); ++l) // O(constant)?
							minN2 = Min(minN2, n2[s].Dot(convexHull.v[antipodalPointsForEdge[edgeJ][l]]));
						for(size_t l = 0; l < antipodalPointsForEdge[edgeK].size(); ++l) // O(constant)?
							minN3 = Min(minN3, n3[s].Dot(convexHull.v[antipodalPointsForEdge[edgeK][l]]));
						float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
						if (volume < minVolume)
						{
							minOBB.axis[0] = n1[s];
							minOBB.axis[1] = n2[s];
							minOBB.axis[2] = n3[s];
							minOBB.r[0] = (maxN1 - minN1) * 0.5f;
							minOBB.r[1] = (maxN2 - minN2) * 0.5f;
							minOBB.r[2] = (maxN3 - minN3) * 0.5f;
							minOBB.pos = (minN1 + minOBB.r[0])*n1[s] + (minN2 + minOBB.r[1])*n2[s] + (minN3 + minOBB.r[2])*n3[s];
							minVolume = volume;
						}
					}
				}
				else if (compatibleEdgesI[s_i] < compatibleEdgesJ[s_j])
					++s_i;
				else
					++s_j;
			}
		}
	}
	TIMING_TICK(tick_t t6 = Clock::Tick());
	TIMING("Edgetripletconfigs: %f msecs (%d configs)", Clock::TimespanToMillisecondsF(t5, t6), numConfigsExplored);
	TIMING_TICK(
		t6 = Clock::Tick();
		int numTwoOpposingFacesConfigs = 0;
		);

	// Take advantage of spatial locality: start the search for the extreme vertex from the extreme vertex
	// that was found during the previous iteration for the previous edge. This speeds up the search since
	// edge directions have some amount of spatial locality and the next extreme vertex is often close
	// to the previous one. Track two hint variables since we are performing extreme vertex searches to
	// two opposing directions at the same time.
	int extremeVertexSearchHint1 = 0;
	int extremeVertexSearchHint2 = 0;

	// Main algorithm body for finding all search directions where the OBB is flush with the edges of the
	// convex hull from two opposing faces. This is O(|E|)?
	for (size_t i = 0; i < edges.size(); ++i) // O(|E|)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		const std::vector<int> &antipodals = antipodalPointsForEdge[i];
		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];
		for(size_t j = 0; j < antipodals.size(); ++j) // O(constant)?
		{
			int antipodalVertex = antipodals[j];
			const std::vector<int> &adjacents = adjacencyData[antipodalVertex];
			for(size_t k = 0; k < adjacents.size(); ++k) // O(constant)?
			{
				int vAdj = adjacents[k];
				if (vAdj < antipodalVertex)
					continue; // We search unordered edges, so no need to process edge (v1, v2) and (v2, v1) twice - take the canonical order to be antipodalVertex < vAdj

				int edge = vertexPairsToEdges[std::make_pair(antipodalVertex, vAdj)];
				if ((int)i > edge) // We search pairs of edges, so no need to process twice - take the canonical order to be i < edge.
					continue;

				vec f2a = faceNormals[facesForEdge[edge].first];
				vec f2b = faceNormals[facesForEdge[edge].second];

				vec n;
				bool success = AreCompatibleOpposingEdges(f1a, f1b, f2a, f2b, n);
				if (success)
				{
					const std::vector<int> &compatibleEdgesJ = compatibleEdges[edge];
					n = n.Normalized();

					float minN1 = n.Dot(convexHull.v[edges[edge].first]);
					float maxN1 = n.Dot(convexHull.v[edges[i].first]);

					// Test all mutual compatible edges.
					size_t s_i = 0;
					size_t s_j = 0;
					while(s_i < compatibleEdgesI.size() && s_j < compatibleEdgesJ.size()) // O(sqrt(|E|))?
					{
						if (compatibleEdgesI[s_i] == compatibleEdgesJ[s_j])
						{
							const int edge3 = compatibleEdgesI[s_i];
							vec f3a = faceNormals[facesForEdge[edge3].first];
							vec f3b = faceNormals[facesForEdge[edge3].second];
							// Is edge3 compatible with direction n?
							// n3 = f3b + (f3a-f3b)*v
							// n1.n3 = 0
							// n1.(f3b + (f3a-f3b)*v) = 0
							// n1.f3b + n1.((f3a-f3b)*v) = 0
							// n1.f3b = (n1.(f3b-f3a))*v
							// If n1.(f3b-f3a) != 0:
							//    v = n1.f3b / n1.(f3b-f3a)
							// If n1.(f3b-f3a) == 0:
							//    n1.f3b must be zero as well, then arbitrary v is ok.
							float num = n.Dot(f3b);
							float denom = n.Dot(f3b-f3a);
							MoveSign(num, denom);
//							float v;
							/*
							if (!EqualAbs(denom, 0.f))
								v = num / denom;
							else
							{
								v = EqualAbs(num, 0.f) ? 0.f : -1.f;
								denom = 1.f;
							}
							*/
							const float epsilon = 1e-4f;
							if (denom < epsilon)//EqualAbs(denom, 0.f))
							{
								num = EqualAbs(num, 0.f) ? 0.f : -1.f;
								denom = 1.f;
							}

//							if (v >= 0.f - epsilon && v <= 1.f + epsilon)
							if (num >= denom * -epsilon && num <= denom * (1.f + epsilon))
							{
								float v = num / denom;
								vec n3 = (f3b + (f3a - f3b) * v).Normalized();
								vec n2 = n3.Cross(n).Normalized();

								float minN2, maxN2;
								CLEAR_GRAPH_SEARCH();
								extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, extremeVertexSearchHint1); // O(log|V|)?
								CLEAR_GRAPH_SEARCH();
								extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, extremeVertexSearchHint2); // O(log|V|)?
								minN2 = -minN2;
								float maxN3 = n3.Dot(convexHull.v[edges[edge3].first]);
								const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edge3];
								float minN3 = FLOAT_INF;
								for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
									minN3 = Min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));

								float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
								TIMING_TICK(++numTwoOpposingFacesConfigs;);
								if (volume < minVolume)
								{
									minOBB.pos = ((minN1 + maxN1) * n + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
									minOBB.axis[0] = n;
									minOBB.axis[1] = n2;
									minOBB.axis[2] = n3;
									minOBB.r[0] = (maxN1 - minN1) * 0.5f;
									minOBB.r[1] = (maxN2 - minN2) * 0.5f;
									minOBB.r[2] = (maxN3 - minN3) * 0.5f;
									minVolume = volume;
								}
							}
							++s_i;
							++s_j;
						}
						else if (compatibleEdgesI[s_i] < compatibleEdgesJ[s_j])
							++s_i;
						else
							++s_j;
					}
				}
			}
		}
	}

	TIMING_TICK(tick_t t7 = Clock::Tick());
	TIMING("Edgepairsforfaces: %f msecs (%d configs)", Clock::TimespanToMillisecondsF(t6, t7), numTwoOpposingFacesConfigs);
	TIMING_TICK(
		tick_t t72 = Clock::Tick();
		int numTwoSameFacesConfigs = 0;
		);

	// Main algorithm body for computing all search directions where the OBB touches two edges on the same face.
	for (size_t i = 0; i < convexHull.f.size(); ++i) // O(|F|)
	{
		vec n1 = faceNormals[i];

		// Find two edges on the face. Since we have flexibility to choose from multiple edges of the same face,
		// choose two that are possibly most opposing to each other, in the hope that their sets of sidepodal
		// edges are most mutually exclusive as possible, speeding up the search below.
		int v0 = convexHull.f[i].v[0];
		int v1 = convexHull.f[i].v[1];
		int second = (convexHull.f[i].v.size()+1)>>1;
		int v2 = convexHull.f[i].v[second];
		int v3 = (second+1) < (int)convexHull.f[i].v.size() ? convexHull.f[i].v[second+1] : v0;
		int e1 = vertexPairsToEdges[std::make_pair(v0, v1)];
		int e2 = vertexPairsToEdges[std::make_pair(v2, v3)];

		const std::vector<int> &antipodals = antipodalPointsForEdge[e1];
		const std::vector<int> &compatibleEdgesI = compatibleEdges[e1];
		const std::vector<int> &compatibleEdgesJ = compatibleEdges[e2];

		float maxN1 = n1.Dot(convexHull.v[edges[e1].first]);
		float minN1 = FLOAT_INF;
		for(size_t j = 0; j < antipodals.size(); ++j)
			minN1 = Min(minN1, n1.Dot(convexHull.v[antipodals[j]]));

		// Test all mutual compatible edges.
		size_t s_i = 0;
		size_t s_j = 0;
		while(s_i < compatibleEdgesI.size() && s_j < compatibleEdgesJ.size()) // O(sqrt(|E|))?
		{
			if (compatibleEdgesI[s_i] == compatibleEdgesJ[s_j])
			{
				const int edge3 = compatibleEdgesI[s_i];
				vec f3a = faceNormals[facesForEdge[edge3].first];
				vec f3b = faceNormals[facesForEdge[edge3].second];
				// Is edge3 compatible with direction n?
				// n3 = f3b + (f3a-f3b)*v
				// n1.n3 = 0
				// n1.(f3b + (f3a-f3b)*v) = 0
				// n1.f3b + n1.((f3a-f3b)*v) = 0
				// n1.f3b = (n1.(f3b-f3a))*v
				// If n1.(f3b-f3a) != 0:
				//    v = n1.f3b / n1.(f3b-f3a)
				// If n1.(f3b-f3a) == 0:
				//    n1.f3b must be zero as well, then arbitrary v is ok.
				float num = n1.Dot(f3b);
				float denom = n1.Dot(f3b-f3a);
				float v;
				if (!EqualAbs(denom, 0.f))
					v = num / denom;
				else
					v = EqualAbs(num, 0.f) ? 0.f : -1.f;

				const float epsilon = 1e-4f;
				if (v >= 0.f - epsilon && v <= 1.f + epsilon)
				{
					vec n3 = (f3b + (f3a - f3b) * v).Normalized();
					vec n2 = n3.Cross(n1).Normalized();

					float minN2, maxN2;
					CLEAR_GRAPH_SEARCH();
					extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, extremeVertexSearchHint1); // O(log|V|)?
					CLEAR_GRAPH_SEARCH();
					extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, extremeVertexSearchHint2); // O(log|V|)?
					minN2 = -minN2;
					float maxN3 = n3.Dot(convexHull.v[edges[edge3].first]);
					const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edge3];
					float minN3 = FLOAT_INF;
					for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
						minN3 = Min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));

					float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
					TIMING_TICK(++numTwoSameFacesConfigs;);
					if (volume < minVolume)
					{
						minOBB.pos = ((minN1 + maxN1) * n1 + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
						minOBB.axis[0] = n1;
						minOBB.axis[1] = n2;
						minOBB.axis[2] = n3;
						minOBB.r[0] = (maxN1 - minN1) * 0.5f;
						minOBB.r[1] = (maxN2 - minN2) * 0.5f;
						minOBB.r[2] = (maxN3 - minN3) * 0.5f;
						minVolume = volume;
					}
				}
				++s_i;
				++s_j;
			}
			else if (compatibleEdgesI[s_i] < compatibleEdgesJ[s_j])
				++s_i;
			else
				++s_j;
		}
	}

	TIMING_TICK(tick_t t8 = Clock::Tick());
	TIMING("Facenormalsgen: %f msecs (%d configs)", Clock::TimespanToMillisecondsF(t72, t8), numTwoSameFacesConfigs);

	// The search for edge triplets does not follow cross-product orientation, so
	// fix that up at the very last step, if necessary.
	if (minOBB.axis[0].Cross(minOBB.axis[1]).Dot(minOBB.axis[2]) < 0.f)
		minOBB.axis[2] = -minOBB.axis[2];
#ifdef MATH_VEC_IS_FLOAT4
	minOBB.r.w = 0.f;
#endif
	return minOBB;
}

OBB OBB::FixedOrientationEnclosingOBB(const vec *pointArray, int numPoints, const vec &dir0, const vec &dir1)
{
	assume(dir0.IsNormalized());
	assume(dir1.IsNormalized());

	int d0, d1;
	float mind0, maxd0, mind1, maxd1, mind2, maxd2;
	OBB::ExtremePointsAlongDirection(dir0, pointArray, numPoints, d0, d1, mind0, maxd0);
	OBB::ExtremePointsAlongDirection(dir1, pointArray, numPoints, d0, d1, mind1, maxd1);
	vec edgeC = dir0.Cross(dir1).Normalized();
	OBB::ExtremePointsAlongDirection(edgeC, pointArray, numPoints, d0, d1, mind2, maxd2);
	float rd0 = (maxd0 - mind0) * 0.5f;
	float rd1 = (maxd1 - mind1) * 0.5f;
	float rd2 = (maxd2 - mind2) * 0.5f;
	OBB minOBB;
	minOBB.pos = (mind0 + rd0) * dir0 + (mind1 + rd1) * dir1 + (mind2 + rd2) * edgeC;
	minOBB.axis[0] = dir0;
	minOBB.axis[1] = dir1;
	minOBB.axis[2] = edgeC;
	minOBB.r = DIR_VEC(rd0, rd1, rd2);
	return minOBB;
}

OBB OBB::BruteEnclosingOBB(const vec *pointArray, int numPoints)
{
	OBB minOBB;
	Polyhedron convexHull = Polyhedron::ConvexHull(pointArray, numPoints);
	if (!pointArray || convexHull.v.size() == 0)
	{
		minOBB.SetNegativeInfinity();
		return minOBB;
	}

	pointArray = (const vec*)&convexHull.v[0];
	numPoints = convexHull.v.size();

//	std::vector<std::vector<int> > adjacencyData = convexHull.GenerateVertexAdjacencyData();
//	std::vector<int> floodFillVisited(convexHull.v.size());
//	int floodFillVisitColor = 1;

	std::vector<float2> pts;
	pts.resize(numPoints);
	float minVolume = FLOAT_INF;
	vec minVolumeEdgeA;
	vec minVolumeEdgeB;

	const int jiggleAxisTimes = 3;
	const int Y = 256;
	const int X = 256;
	for(int y = 0; y < Y; ++y)
		for(int x = 0; x < X; ++x)
		{
			float fx = (float)x / (X-1) * 2.0f - 1.0f;
			float fy = (float)y / (Y-1) * 2.0f - 1.0f;

			float lenSq = fx*fx + fy*fy;
			if (lenSq > 1.00f)
				continue;
			float fz = Sqrt(1.0f - lenSq);

			vec edge = DIR_VEC(fx, fy, fz);

			vec edgeA, edgeB;
			float volume = SmallestOBBVolumeJiggle(edge, convexHull, pts, /*adjacencyData, floodFillVisited, floodFillVisitColor,*/
				jiggleAxisTimes, edgeA, edgeB);

			if (volume < minVolume)
			{
				minVolumeEdgeA = edgeA;
				minVolumeEdgeB = edgeB;
				minVolume = volume;
			}
		}

	return FixedOrientationEnclosingOBB(pointArray, numPoints, minVolumeEdgeA, minVolumeEdgeB);
}

vec OBB::Size() const
{
	return r * 2.f;
}

vec OBB::HalfSize() const
{
	return r;
}

vec OBB::Diagonal() const
{
	return 2.f * HalfDiagonal();
}

vec OBB::HalfDiagonal() const
{
	return axis[0] * r[0] + axis[1] * r[1] + axis[2] * r[2];
}

float3x4 OBB::WorldToLocal() const
{
	float3x4 m = LocalToWorld();
	m.InverseOrthonormal();
	return m;
}

float3x4 OBB::LocalToWorld() const
{
	// To produce a normalized local->world matrix, do the following.
	/*
	float3x4 m;
	vec x = axis[0] * r.x;
	vec y = axis[1] * r.y;
	vec z = axis[2] * r.z;
	m.SetCol(0, 2.f * x);
	m.SetCol(1, 2.f * y);
	m.SetCol(2, 2.f * z);
	m.SetCol(3, pos - x - y - z);
	return m;
	*/

	assume2(axis[0].IsNormalized(), axis[0], axis[0].LengthSq());
	assume2(axis[1].IsNormalized(), axis[1], axis[1].LengthSq());
	assume2(axis[2].IsNormalized(), axis[2], axis[2].LengthSq());
	float3x4 m; ///\todo sse-matrix
	m.SetCol(0, axis[0].ptr());
	m.SetCol(1, axis[1].ptr());
	m.SetCol(2, axis[2].ptr());
	vec p = pos - axis[0] * r.x - axis[1] * r.y - axis[2] * r.z;
	m.SetCol(3, p.ptr());
	assume(m.IsOrthonormal());
	return m;
}

/// The implementation of this function is from Christer Ericson's Real-Time Collision Detection, p.133.
vec OBB::ClosestPoint(const vec &targetPoint) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Best: 8.833 nsecs / 24 ticks, Avg: 9.044 nsecs, Worst: 9.217 nsecs
	simd4f d = sub_ps(targetPoint.v, pos.v);
	simd4f x = xxxx_ps(r.v);
	simd4f closestPoint = pos.v;
	closestPoint = add_ps(closestPoint, mul_ps(max_ps(min_ps(dot4_ps(d, axis[0].v), x), negate_ps(x)), axis[0].v));
	simd4f y = yyyy_ps(r.v);
	closestPoint = add_ps(closestPoint, mul_ps(max_ps(min_ps(dot4_ps(d, axis[1].v), y), negate_ps(y)), axis[1].v));
	simd4f z = zzzz_ps(r.v);
	closestPoint = add_ps(closestPoint, mul_ps(max_ps(min_ps(dot4_ps(d, axis[2].v), z), negate_ps(z)), axis[2].v));
	return closestPoint;
#else
	// Best: 33.412 nsecs / 89.952 ticks, Avg: 33.804 nsecs, Worst: 34.180 nsecs
	vec d = targetPoint - pos;
	vec closestPoint = pos; // Start at the center point of the OBB.
	for(int i = 0; i < 3; ++i) // Project the target onto the OBB axes and walk towards that point.
		closestPoint += Clamp(Dot(d, axis[i]), -r[i], r[i]) * axis[i];

	return closestPoint;
#endif
}

float OBB::Volume() const
{
	vec size = Size();
	return size.x*size.y*size.z;
}

float OBB::SurfaceArea() const
{
	const vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

vec OBB::RandomPointInside(LCG &rng) const
{
	float f1 = rng.Float();
	float f2 = rng.Float();
	float f3 = rng.Float();
	return PointInside(f1, f2, f3);
}

vec OBB::RandomPointOnSurface(LCG &rng) const
{
	int i = rng.Int(0, 5);
	float f1 = rng.Float();
	float f2 = rng.Float();
	return FacePoint(i, f1, f2);
}

vec OBB::RandomPointOnEdge(LCG &rng) const
{
	int i = rng.Int(0, 11);
	float f = rng.Float();
	return PointOnEdge(i, f);
}

vec OBB::RandomCornerPoint(LCG &rng) const
{
	return CornerPoint(rng.Int(0, 7));
}

void OBB::Translate(const vec &offset)
{
	pos += offset;
}

void OBB::Scale(const vec &centerPoint, float scaleFactor)
{
	return Scale(centerPoint, DIR_VEC_SCALAR(scaleFactor));
}

void OBB::Scale(const vec &centerPoint, const vec &scaleFactor)
{
	///@bug This scales in global axes, not local axes.
	float3x4 transform = float3x4::Scale(scaleFactor, centerPoint);
	Transform(transform);
}

template<typename Matrix>
void OBBTransform(OBB &o, const Matrix &transform)
{
	o.pos = transform.MulPos(o.pos);
	o.axis[0] = transform.MulDir(o.r.x * o.axis[0]);
	o.axis[1] = transform.MulDir(o.r.y * o.axis[1]);
	o.axis[2] = transform.MulDir(o.r.z * o.axis[2]);
	o.r.x = o.axis[0].Normalize();
	o.r.y = o.axis[1].Normalize();
	o.r.z = o.axis[2].Normalize();
}

void OBB::Transform(const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float3x4 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float4x4 &transform)
{
	assume(transform.IsColOrthogonal3());
	OBBTransform(*this, transform);
}

void OBB::Transform(const Quat &transform)
{
	OBBTransform(*this, transform.ToFloat3x3());
}

float OBB::Distance(const vec &point) const
{
	///@todo This code can be optimized a bit. See Christer Ericson's Real-Time Collision Detection,
	/// p.134.
	vec closestPoint = ClosestPoint(point);
	return point.Distance(closestPoint);
}

float OBB::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

bool OBB::Contains(const vec &point) const
{
#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
// Best: 9.985 nsecs / 26.816 ticks, Avg: 10.112 nsecs, Worst: 11.137 nsecs
	simd4f pt = sub_ps(point.v, pos.v);
	simd4f s1 = mul_ps(pt, axis[0].v);
	simd4f s2 = mul_ps(pt, axis[1].v);
	simd4f s3 = mul_ps(pt, axis[2].v);
	s1 = abs_ps(sum_xyzw_ps(s1));
	s2 = abs_ps(sum_xyzw_ps(s2));
	s3 = abs_ps(sum_xyzw_ps(s3));

	s1 = _mm_sub_ss(s1, r.v);
	s2 = _mm_sub_ss(s2, yyyy_ps(r.v));
	simd4f s12 = _mm_max_ss(s1, s2);
	s3 = _mm_sub_ss(s3, zzzz_ps(r.v));
	s3 = _mm_max_ss(s12, s3);
	return _mm_cvtss_f32(s3) <= 0.f; // Note: This might be micro-optimized further out by switching to a signature "float OBB::SignedDistance(point)" instead.
#else
// Best: 14.978 nsecs / 39.944 ticks, Avg: 15.350 nsecs, Worst: 39.941 nsecs
	vec pt = point - pos;
	return Abs(Dot(pt, axis[0])) <= r[0] &&
	       Abs(Dot(pt, axis[1])) <= r[1] &&
	       Abs(Dot(pt, axis[2])) <= r[2];
#endif
}

bool OBB::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool OBB::Contains(const AABB &aabb) const
{
	// Since both AABB and OBB are convex objects, this OBB contains the AABB
	// if and only if it contains all its corner points.
	for(int i = 0; i < 8; ++i)
	if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool OBB::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool OBB::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

bool OBB::Intersects(const AABB &aabb) const
{
	return Intersects(OBB(aabb));
}

void OBB::Enclose(const vec &point)
{
	vec p = point - pos;
	for(int i = 0; i < 3; ++i)
	{
		assume2(EqualAbs(axis[i].Length(), 1.f), axis[i], axis[i].Length());
		float dist = p.Dot(axis[i]);
		float distanceFromOBB = Abs(dist) - r[i];
		if (distanceFromOBB > 0.f)
		{
			r[i] += distanceFromOBB * 0.5f;
			if (dist > 0.f) ///\todo Optimize out this comparison!
				pos += axis[i] * distanceFromOBB * 0.5f;
			else
				pos -= axis[i] * distanceFromOBB * 0.5f;

			p = point-pos; ///\todo Can we omit this? (redundant since axis[i] are orthonormal?)

			mathassert(EqualAbs(Abs(p.Dot(axis[i])), r[i], 1e-1f));
		}
	}
	// Should now contain the point.
	assume2(Distance(point) <= 1e-3f, point, Distance(point));
}

void OBB::Triangulate(int x, int y, int z, vec *outPos, vec *outNormal, float2 *outUV, bool ccwIsFrontFacing) const
{
	AABB aabb(POINT_VEC_SCALAR(0), r*2.f);
	aabb.Triangulate(x, y, z, outPos, outNormal, outUV, ccwIsFrontFacing);
	float3x4 localToWorld = LocalToWorld();
	assume(localToWorld.HasUnitaryScale()); // Transforming of normals will fail otherwise.
	localToWorld.BatchTransformPos(outPos, NumVerticesInTriangulation(x,y,z), sizeof(vec));
	localToWorld.BatchTransformDir(outNormal, NumVerticesInTriangulation(x,y,z), sizeof(vec));
}

void OBB::ToEdgeList(vec *outPos) const
{
	assume(outPos);
	if (!outPos)
		return;
	for(int i = 0; i < 12; ++i)
	{
		LineSegment edge = Edge(i);
		outPos[i*2] = edge.a;
		outPos[i*2+1] = edge.b;
	}
}

bool OBB::Intersects(const OBB &b, float epsilon) const
{
	assume(pos.IsFinite());
	assume(b.pos.IsFinite());
	assume(vec::AreOrthogonal(axis[0], axis[1], axis[2]));
	assume(vec::AreOrthogonal(b.axis[0], b.axis[1], b.axis[2]));

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// SSE4.1:
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 23.913 nsecs / 40.645 ticks, Avg: 26.490 nsecs, Worst: 43.729 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 42.585 nsecs / 72.413 ticks, Avg: 44.373 nsecs, Worst: 70.774 nsecs

	simd4f bLocalToWorld[3];
	mat3x4_transpose((const simd4f*)b.axis, bLocalToWorld);

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	simd4f R[3];
	mat3x4_mul_sse(R, (const simd4f*)axis, bLocalToWorld);

	// Express the translation vector in a's coordinate frame.
	simd4f t = mat3x4_mul_sse((const simd4f*)axis, sub_ps(b.pos, pos));

	// This trashes the w component, which should technically be zero, but this does not matter since
	// AbsR will only be used with direction vectors.
	const vec epsilonxyz = set1_ps(epsilon);
	simd4f AbsR[3];
	AbsR[0] = add_ps(abs_ps(R[0]), epsilonxyz);
	AbsR[1] = add_ps(abs_ps(R[1]), epsilonxyz);
	AbsR[2] = add_ps(abs_ps(R[2]), epsilonxyz);

	// Test the three major axes of this OBB.
	simd4f res = cmpgt_ps(abs_ps(t), add_ps(r, mat3x4_mul_sse(AbsR, b.r)));
	if (!allzero_ps(res)) return false;

	// Test the three major axes of the OBB b.
	simd4f transpR[3];
	mat3x4_transpose(R, transpR);
	vec l = abs_ps(mat3x4_mul_sse(transpR, r));
	simd4f transpAbsR[3];
	mat3x4_transpose(AbsR, transpAbsR);
	vec s = mat3x4_mul_sse(transpAbsR, r);
	res = cmpgt_ps(l, add_ps(s, b.r));
	if (!allzero_ps(res)) return false;

	// Test the 9 different cross-axes.

	// A.x <cross> B.x
	// A.x <cross> B.y
	// A.x <cross> B.z
	simd4f ra = mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,1,1,1)), AbsR[2]);
	ra = add_ps(ra, mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,2,2,2)), AbsR[1]));
	simd4f rb = mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,0,0,1)), shuffle1_ps(AbsR[0], _MM_SHUFFLE(3,1,2,2)));
	rb = add_ps(rb, mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,1,2,2)), shuffle1_ps(AbsR[0], _MM_SHUFFLE(3,0,0,1))));
	simd4f lhs = mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,2,2,2)), R[1]);
	lhs = sub_ps(lhs, mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,1,1,1)), R[2]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.y <cross> B.x
	// A.y <cross> B.y
	// A.y <cross> B.z
	ra = mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,0,0,0)), AbsR[2]);
	ra = add_ps(ra, mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,2,2,2)), AbsR[0]));
	rb = mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,0,0,1)), shuffle1_ps(AbsR[1], _MM_SHUFFLE(3,1,2,2)));
	rb = add_ps(rb, mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,1,2,2)), shuffle1_ps(AbsR[1], _MM_SHUFFLE(3,0,0,1))));
	lhs = mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,0,0,0)), R[2]);
	lhs = sub_ps(lhs, mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,2,2,2)), R[0]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.z <cross> B.x
	// A.z <cross> B.y
	// A.z <cross> B.z
	ra = mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,0,0,0)), AbsR[1]);
	ra = add_ps(ra, mul_ps(shuffle1_ps(r, _MM_SHUFFLE(3,1,1,1)), AbsR[0]));
	rb = mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,0,0,1)), shuffle1_ps(AbsR[2], _MM_SHUFFLE(3,1,2,2)));
	rb = add_ps(rb, mul_ps(shuffle1_ps(b.r, _MM_SHUFFLE(3,1,2,2)), shuffle1_ps(AbsR[2], _MM_SHUFFLE(3,0,0,1))));
	lhs = mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,1,1,1)), R[0]);
	lhs = sub_ps(lhs, mul_ps(shuffle1_ps(t, _MM_SHUFFLE(3,0,0,0)), R[1]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	return allzero_ps(res) != 0;

#else
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 100.830 nsecs / 171.37 ticks, Avg: 110.533 nsecs, Worst: 155.582 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 95.771 nsecs / 162.739 ticks, Avg: 107.935 nsecs, Worst: 173.110 nsecs

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	float3x3 R;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			R[i][j] = Dot(axis[i], b.axis[j]);

	vec t = b.pos - pos;
	// Express the translation vector in a's coordinate frame.
	t = DIR_VEC(Dot(t, axis[0]), Dot(t, axis[1]), Dot(t, axis[2]));

	float3x3 AbsR;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			AbsR[i][j] = Abs(R[i][j]) + epsilon;

	// Test the three major axes of this OBB.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[i];
		float rb = DOT3(b.r, AbsR[i]);
		if (Abs(t[i]) > ra + rb)
			return false;
	}

	// Test the three major axes of the OBB b.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[0] * AbsR[0][i] + r[1] * AbsR[1][i] + r[2] * AbsR[2][i];
		float rb = b.r[i];
		if (Abs(t.x * R[0][i] + t.y * R[1][i] + t.z * R[2][i]) > ra + rb)
			return false;
	}

	// Test the 9 different cross-axes.

	// A.x <cross> B.x
	float ra = r.y * AbsR[2][0] + r.z * AbsR[1][0];
	float rb = b.r.y * AbsR[0][2] + b.r.z * AbsR[0][1];
	if (Abs(t.z * R[1][0] - t.y * R[2][0]) > ra + rb)
		return false;

	// A.x < cross> B.y
	ra = r.y * AbsR[2][1] + r.z * AbsR[1][1];
	rb = b.r.x * AbsR[0][2] + b.r.z * AbsR[0][0];
	if (Abs(t.z * R[1][1] - t.y * R[2][1]) > ra + rb)
		return false;

	// A.x <cross> B.z
	ra = r.y * AbsR[2][2] + r.z * AbsR[1][2];
	rb = b.r.x * AbsR[0][1] + b.r.y * AbsR[0][0];
	if (Abs(t.z * R[1][2] - t.y * R[2][2]) > ra + rb)
		return false;

	// A.y <cross> B.x
	ra = r.x * AbsR[2][0] + r.z * AbsR[0][0];
	rb = b.r.y * AbsR[1][2] + b.r.z * AbsR[1][1];
	if (Abs(t.x * R[2][0] - t.z * R[0][0]) > ra + rb)
		return false;

	// A.y <cross> B.y
	ra = r.x * AbsR[2][1] + r.z * AbsR[0][1];
	rb = b.r.x * AbsR[1][2] + b.r.z * AbsR[1][0];
	if (Abs(t.x * R[2][1] - t.z * R[0][1]) > ra + rb)
		return false;

	// A.y <cross> B.z
	ra = r.x * AbsR[2][2] + r.z * AbsR[0][2];
	rb = b.r.x * AbsR[1][1] + b.r.y * AbsR[1][0];
	if (Abs(t.x * R[2][2] - t.z * R[0][2]) > ra + rb)
		return false;

	// A.z <cross> B.x
	ra = r.x * AbsR[1][0] + r.y * AbsR[0][0];
	rb = b.r.y * AbsR[2][2] + b.r.z * AbsR[2][1];
	if (Abs(t.y * R[0][0] - t.x * R[1][0]) > ra + rb)
		return false;

	// A.z <cross> B.y
	ra = r.x * AbsR[1][1] + r.y * AbsR[0][1];
	rb = b.r.x * AbsR[2][2] + b.r.z * AbsR[2][0];
	if (Abs(t.y * R[0][1] - t.x * R[1][1]) > ra + rb)
		return false;

	// A.z <cross> B.z
	ra = r.x * AbsR[1][2] + r.y * AbsR[0][2];
	rb = b.r.x * AbsR[2][1] + b.r.y * AbsR[2][0];
	if (Abs(t.y * R[0][2] - t.x * R[1][2]) > ra + rb)
		return false;

	// No separating axis exists, so the two OBB don't intersect.
	return true;
#endif
}

/// The implementation of OBB-Plane intersection test follows Christer Ericson's Real-Time Collision Detection, p. 163. [groupSyntax]
bool OBB::Intersects(const Plane &p) const
{
	// Compute the projection interval radius of this OBB onto L(t) = this->pos + x * p.normal;
	float t = r[0] * Abs(Dot(p.normal, axis[0])) +
			  r[1] * Abs(Dot(p.normal, axis[1])) +
			  r[2] * Abs(Dot(p.normal, axis[2]));
	// Compute the distance of this OBB center from the plane.
	float s = Dot(p.normal, pos) - p.d;
	return Abs(s) <= t;
}

bool OBB::Intersects(const Ray &ray) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray r = WorldToLocal() * ray;
	return aabb.Intersects(r);
}

bool OBB::Intersects(const Ray &ray, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray r = WorldToLocal() * ray;
	return aabb.Intersects(r, dNear, dFar);
}

bool OBB::Intersects(const Line &line) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const Line &line, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l, dNear, dFar);
}

bool OBB::Intersects(const LineSegment &lineSegment) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const LineSegment &lineSegment, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l, dNear, dFar);
}

/// The implementation of the OBB-Sphere intersection test follows Christer Ericson's Real-Time Collision Detection, p. 166. [groupSyntax]
bool OBB::Intersects(const Sphere &sphere, vec *closestPointOnOBB) const
{
	// Find the point on this AABB closest to the sphere center.
	vec pt = ClosestPoint(sphere.pos);

	// If that point is inside sphere, the AABB and sphere intersect.
	if (closestPointOnOBB)
		*closestPointOnOBB = pt;

	return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool OBB::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool OBB::Intersects(const Triangle &triangle) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Triangle t = WorldToLocal() * triangle;
	return t.Intersects(aabb);
}

bool OBB::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool OBB::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool OBB::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string OBB::ToString() const
{
	char str[256];
	sprintf(str, "OBB(Pos:(%.2f, %.2f, %.2f) Halfsize:(%.2f, %.2f, %.2f) X:(%.2f, %.2f, %.2f) Y:(%.2f, %.2f, %.2f) Z:(%.2f, %.2f, %.2f))",
		pos.x, pos.y, pos.z, r.x, r.y, r.z, axis[0].x, axis[0].y, axis[0].z, axis[1].x, axis[1].y, axis[1].z, axis[2].x, axis[2].y, axis[2].z);
	return str;
}

std::string OBB::SerializeToString() const
{
	std::string s = pos.xyz().SerializeToString() + " "
	              + r.xyz().SerializeToString() + " "
	              + axis[0].xyz().SerializeToString() + " "
	              + axis[1].xyz().SerializeToString() + " "
	              + axis[2].xyz().SerializeToString();
	return s;
}

std::string OBB::SerializeToCodeString() const
{
	return "OBB(" + pos.SerializeToCodeString() + ","
	              + r.SerializeToCodeString() + ","
	              + axis[0].SerializeToCodeString() + ","
	              + axis[1].SerializeToCodeString() + ","
	              + axis[2].SerializeToCodeString() + ")";
}

std::ostream &operator <<(std::ostream &o, const OBB &obb)
{
	o << obb.ToString();
	return o;
}

#endif

OBB OBB::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return OBB(vec::nan, vec::nan, vec::nan, vec::nan, vec::nan);
	OBB o;
	MATH_SKIP_WORD(str, "OBB(");
	MATH_SKIP_WORD(str, "Pos:(");
	o.pos = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Halfsize:(");
	o.r = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " X:(");
	o.axis[0] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Y:(");
	o.axis[1] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Z:(");
	o.axis[2] = DirVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return o;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void OBB::Triangulate(VertexBuffer &vb, int x, int y, int z, bool ccwIsFrontFacing) const
{
	Array<vec> pos;
	Array<vec> normal;
	Array<float2> uv;
	int numVertices = (x*y+y*z+x*z)*2*6;
	pos.Resize_pod(numVertices);
	normal.Resize_pod(numVertices);
	uv.Resize_pod(numVertices);
	Triangulate(x,y,z, &pos[0], &normal[0], &uv[0], ccwIsFrontFacing);
	int startIndex = vb.AppendVertices(numVertices);
	for(int i = 0; i < (int)pos.size(); ++i)
	{
		vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(pos[i]));
		if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			vb.Set(startIndex+i, VDNormal, DIR_TO_FLOAT4(normal[i]));
		if (vb.Declaration()->TypeOffset(VDUV) >= 0)
			vb.SetFloat2(startIndex+i, VDUV, 0, uv[i]);
	}
}

void OBB::ToLineList(VertexBuffer &vb) const
{
	Array<vec> pos;
	pos.Resize_pod(NumVerticesInEdgeList());
	ToEdgeList(&pos[0]);
	int startIndex = vb.AppendVertices((int)pos.size());
	for(int i = 0; i < (int)pos.size(); ++i)
		vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(pos[i]));
}

#endif

OBB operator *(const float3x3 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float3x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float4x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const Quat &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

MATH_END_NAMESPACE
