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
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Ray.h"
#include "Triangle.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Algorithm/Sort/Sort.h"
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
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
void OBB::ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest)
{
	assume(pointArray || numPoints == 0);

	idxSmallest = idxLargest = 0;

#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pointArray)
		return;
#endif

	float smallestD = FLOAT_INF;
	float largestD = -FLOAT_INF;
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

#ifdef MATH_CONTAINERLIB_SUPPORT
int LexFloat3Cmp(const vec &a, const vec &b)
{
	LEXCMP(a.x, b.x);
	LEXCMP(a.y, b.y);
	LEXCMP(a.z, b.z);
	return 0;
}

OBB OBB::OptimalEnclosingOBB(const vec *pointArray, int numPoints)
{
	OBB minOBB;
	float minVolume = FLOAT_INF;

	std::vector<float2> pts;
	pts.resize(numPoints);
	///\todo Convex hull'ize.

	VecArray dirs;
	dirs.reserve((numPoints * numPoints-1) / 2);
	for(int i = 0; i < numPoints; ++i)
		for(int j = i+1; j < numPoints; ++j)
		{
			vec edge = pointArray[i]-pointArray[j];
			float oldLength = edge.Normalize();
			if (edge.z < 0.f)
				edge = -edge;
			if (oldLength > 0.f)
				dirs.push_back(edge);
		}

	LOGI("Got %d directions.", (int)dirs.size());
	sort::QuickSort(&dirs[0], (int)dirs.size(), LexFloat3Cmp);
	for(int i = (int)dirs.size()-1; i >= 0; --i)
		for(int j = i-1; j >= 0; --j)
		{
			float distX = dirs[i].x - dirs[j].x;
			if (distX > 1e-1f)
				break;
			if (vec(dirs[i]).DistanceSq(dirs[j]) < 1e-3f)
			{
				dirs.erase(dirs.begin() + j);
				--i;
			}
		}
	LOGI("Pruned to %d directions.", (int)dirs.size());

//	int incr = 1;//Max(1, numPoints / 10);
//	for(int i = 0; i < numPoints; ++i)
	{
		//for(int j = i+1; j < numPoints; ++j)
		for(size_t i = 0; i < dirs.size(); ++i)
		{
			vec edge = dirs[i];//(pointArray[i]-pointArray[j]).Normalized();

			int e1, e2;
			ExtremePointsAlongDirection(edge, pointArray, numPoints, e1, e2);
			float edgeLength = Abs(Dot(pointArray[e1], edge) - Dot(pointArray[e2], edge));

			vec u = edge.Perpendicular();
			vec v = edge.AnotherPerpendicular();
			for(int k = 0; k < numPoints; ++k)
				pts[k] = float2(pointArray[k].Dot(u), pointArray[k].Dot(v));

			float2 rectCenter;
			float2 rectU;
			float2 rectV;
			float minU, maxU, minV, maxV;
			float rectArea = float2::MinAreaRect(&pts[0], (int)pts.size(), rectCenter, rectU, rectV, minU, maxU, minV, maxV);
			vec rectCenterPos = u * rectCenter.x + v * rectCenter.y;
			
			float volume = rectArea * edgeLength;
			if (volume < minVolume)
			{
				minOBB.axis[0] = edge;
				minOBB.axis[1] = rectU.x * u + rectU.y * v;
				minOBB.axis[2] = rectV.x * u + rectV.y * v;
				minOBB.pos = (Dot(pointArray[e1], edge) + Dot(pointArray[e2], edge)) * 0.5f * edge + rectCenterPos;
				minOBB.r[0] = edgeLength * 0.5f;
				minOBB.r[1] = (maxU - minU) * 0.5f;
				minOBB.r[2] = (maxV - minV) * 0.5f;
				minVolume = volume;
			}
			if (i % 100 == 0)
				LOGI("%d/%d.", (int)i, (int)dirs.size());
		}
	}

	return minOBB;
}
#endif

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
	vec d = targetPoint - pos;
	vec closestPoint = pos; // Start at the center point of the OBB.
	for(int i = 0; i < 3; ++i) // Project the target onto the OBB axes and walk towards that point.
		closestPoint += Clamp(Dot(d, axis[i]), -r[i], r[i]) * axis[i];

	return closestPoint;
}

float OBB::Volume() const
{
	return Size().ProductOfElements();
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
	vec pt = point - pos;
	return Abs(Dot(pt, axis[0])) <= r[0] &&
	       Abs(Dot(pt, axis[1])) <= r[1] &&
	       Abs(Dot(pt, axis[2])) <= r[2];
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
		assert(EqualAbs(axis[i].Length(), 1.f));
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
	assume(Distance(point) <= 1e-3f);
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
