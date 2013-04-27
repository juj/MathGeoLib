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

/** @file AABB.cpp
	@author Jukka Jylänki
	@brief Implementation for the Axis-Aligned Bounding Box (AABB) geometry object. */
#include "AABB.h"
#include "../Math/MathFunc.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include <utility>
#endif
#include "Frustum.h"
#include "LineSegment.h"
#include "Line.h"
#include "Ray.h"
#include "../Algorithm/Random/LCG.h"
#include "OBB.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Sphere.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Triangle.h"
#include "Capsule.h"

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

AABB::AABB(const float3 &minPoint_, const float3 &maxPoint_)
:minPoint(minPoint_), maxPoint(maxPoint_)
{
}

AABB::AABB(const OBB &obb)
{
	SetFrom(obb);
}

AABB::AABB(const Sphere &s)
{
	SetFrom(s);
}

void AABB::SetNegativeInfinity()
{
	minPoint.SetFromScalar(FLOAT_INF);
	maxPoint.SetFromScalar(-FLOAT_INF);
}

void AABB::SetFromCenterAndSize(const float3 &center, const float3 &size)
{
	float3 halfSize = 0.5f * size;
	minPoint = center - halfSize;
	maxPoint = center + halfSize;
}

void AABB::SetFrom(const OBB &obb)
{
	float3 halfSize = Abs(obb.axis[0]*obb.r[0]) + Abs(obb.axis[1]*obb.r[1]) + Abs(obb.axis[2]*obb.r[2]);
	SetFromCenterAndSize(obb.pos, 2.f*halfSize);
}

void AABB::SetFrom(const Sphere &s)
{
	minPoint = s.pos - float3(s.r, s.r, s.r);
	maxPoint = s.pos + float3(s.r, s.r, s.r);
}

void AABB::SetFrom(const float3 *pointArray, int numPoints)
{
	assume(pointArray || numPoints == 0);
	SetNegativeInfinity();
	if (!pointArray)
		return;
	for(int i = 0; i < numPoints; ++i)
		Enclose(pointArray[i]);
}

Polyhedron AABB::ToPolyhedron() const
{
	// Note to maintainer: This function is an exact copy of OBB:ToPolyhedron() and Frustum::ToPolyhedron().
	Polyhedron p;
	// Populate the corners of this AABB.
	// The will be in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
	for(int i = 0; i < 8; ++i)
		p.v.push_back(CornerPoint(i));

	// Generate the 6 faces of this AABB.
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

OBB AABB::ToOBB() const
{
	return OBB(*this);
}

Sphere AABB::MinimalEnclosingSphere() const
{
	return Sphere(CenterPoint(), Size().Length()/2.f);
}

Sphere AABB::MaximalContainedSphere() const
{
	float3 halfSize = HalfSize();
	return Sphere(CenterPoint(), Min(halfSize.x, halfSize.y, halfSize.z));
}

bool AABB::IsFinite() const
{
	return minPoint.IsFinite() && maxPoint.IsFinite();
}

bool AABB::IsDegenerate() const
{
	return minPoint.x > maxPoint.x || minPoint.y > maxPoint.y || minPoint.z > maxPoint.z;
}

float3 AABB::CenterPoint() const
{
	return (minPoint + maxPoint) / 2.f;
}

float3 AABB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	float3 d = maxPoint - minPoint;
	return minPoint + float3(d.x*x, d.y*y, d.z*z);
}

LineSegment AABB::Edge(int edgeIndex) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	switch(edgeIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		/* For documentation, here's the segments that are returned:
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
		*/
		// Force-optimize to avoid calling to CornerPoint for another switch-case statement.
		case 0: return LineSegment(float3(minPoint.x, minPoint.y, minPoint.z), float3(minPoint.x, minPoint.y, maxPoint.z));
		case 1: return LineSegment(float3(minPoint.x, minPoint.y, minPoint.z), float3(minPoint.x, maxPoint.y, minPoint.z));
		case 2: return LineSegment(float3(minPoint.x, minPoint.y, minPoint.z), float3(maxPoint.x, minPoint.y, minPoint.z));
		case 3: return LineSegment(float3(minPoint.x, minPoint.y, maxPoint.z), float3(minPoint.x, maxPoint.y, maxPoint.z));
		case 4: return LineSegment(float3(minPoint.x, minPoint.y, maxPoint.z), float3(maxPoint.x, minPoint.y, maxPoint.z));
		case 5: return LineSegment(float3(minPoint.x, maxPoint.y, minPoint.z), float3(minPoint.x, maxPoint.y, maxPoint.z));
		case 6: return LineSegment(float3(minPoint.x, maxPoint.y, minPoint.z), float3(maxPoint.x, maxPoint.y, minPoint.z));
		case 7: return LineSegment(float3(minPoint.x, maxPoint.y, maxPoint.z), float3(maxPoint.x, maxPoint.y, maxPoint.z));
		case 8: return LineSegment(float3(maxPoint.x, minPoint.y, minPoint.z), float3(maxPoint.x, minPoint.y, maxPoint.z));
		case 9: return LineSegment(float3(maxPoint.x, minPoint.y, minPoint.z), float3(maxPoint.x, maxPoint.y, minPoint.z));
		case 10: return LineSegment(float3(maxPoint.x, minPoint.y, maxPoint.z), float3(maxPoint.x, maxPoint.y, maxPoint.z));
		case 11: return LineSegment(float3(maxPoint.x, maxPoint.y, minPoint.z), float3(maxPoint.x, maxPoint.y, maxPoint.z));
	}
}

float3 AABB::CornerPoint(int cornerIndex) const
{
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return float3(minPoint.x, minPoint.y, minPoint.z);
		case 1: return float3(minPoint.x, minPoint.y, maxPoint.z);
		case 2: return float3(minPoint.x, maxPoint.y, minPoint.z);
		case 3: return float3(minPoint.x, maxPoint.y, maxPoint.z);
		case 4: return float3(maxPoint.x, minPoint.y, minPoint.z);
		case 5: return float3(maxPoint.x, minPoint.y, maxPoint.z);
		case 6: return float3(maxPoint.x, maxPoint.y, minPoint.z);
		case 7: return float3(maxPoint.x, maxPoint.y, maxPoint.z);
	}
}

float3 AABB::ExtremePoint(const float3 &direction) const
{
	float3 pt;
	pt.x = (direction.x >= 0.f ? maxPoint.x : minPoint.x);
	pt.y = (direction.y >= 0.f ? maxPoint.y : minPoint.y);
	pt.z = (direction.z >= 0.f ? maxPoint.z : minPoint.z);
	return pt;
}

float3 AABB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	float3 d = maxPoint - minPoint;
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return float3(minPoint.x, minPoint.y, minPoint.z + u * d.z);
	case 1: return float3(minPoint.x, maxPoint.y, minPoint.z + u * d.z);
	case 2: return float3(maxPoint.x, minPoint.y, minPoint.z + u * d.z);
	case 3: return float3(maxPoint.x, maxPoint.y, minPoint.z + u * d.z);

	case 4: return float3(minPoint.x, minPoint.y + u * d.y, minPoint.z);
	case 5: return float3(maxPoint.x, minPoint.y + u * d.y, minPoint.z);
	case 6: return float3(minPoint.x, minPoint.y + u * d.y, maxPoint.z);
	case 7: return float3(maxPoint.x, minPoint.y + u * d.y, maxPoint.z);

	case 8: return float3(minPoint.x + u * d.x, minPoint.y, minPoint.z);
	case 9: return float3(minPoint.x + u * d.x, minPoint.y, maxPoint.z);
	case 10: return float3(minPoint.x + u * d.x, maxPoint.y, minPoint.z);
	case 11: return float3(minPoint.x + u * d.x, maxPoint.y, maxPoint.z);
	}
}

float3 AABB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	float3 center = (minPoint + maxPoint) / 2.f;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return float3(minPoint.x, center.y, center.z);
	case 1: return float3(maxPoint.x, center.y, center.z);
	case 2: return float3(center.x, minPoint.y, center.z);
	case 3: return float3(center.x, maxPoint.y, center.z);
	case 4: return float3(center.x, center.y, minPoint.z);
	case 5: return float3(center.x, center.y, maxPoint.z);
	}
}

float3 AABB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	float3 d = maxPoint - minPoint;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return float3(minPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
	case 1: return float3(maxPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
	case 2: return float3(minPoint.x + u * d.x, minPoint.y, minPoint.z + v * d.z);
	case 3: return float3(minPoint.x + u * d.x, maxPoint.y, minPoint.z + v * d.z);
	case 4: return float3(minPoint.x + u * d.x, minPoint.y + v * d.y, minPoint.z);
	case 5: return float3(minPoint.x + u * d.x, minPoint.y + v * d.y, maxPoint.z);
	}
}

float3 AABB::FaceNormal(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return float3(-1,0,0);
	case 1: return float3( 1,0,0);
	case 2: return float3(0,-1,0);
	case 3: return float3(0, 1,0);
	case 4: return float3(0,0,-1);
	case 5: return float3(0,0, 1);
	}
}

Plane AABB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	return Plane(FaceCenterPoint(faceIndex), FaceNormal(faceIndex));
}

void AABB::GetCornerPoints(float3 *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

void AABB::GetFacePlanes(Plane *outPlaneArray) const
{
	assume(outPlaneArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPlaneArray)
		return;
#endif
	for(int i = 0; i < 6; ++i)
		outPlaneArray[i] = FacePlane(i);
}

AABB AABB::MinimalEnclosingAABB(const float3 *pointArray, int numPoints)
{
	AABB aabb;
	aabb.SetFrom(pointArray, numPoints);
	return aabb;
}

void AABB::ExtremePointsAlongAABB(const float3 *pts, int numPoints, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz)
{
	assume(pts || numPoints == 0);
	if (!pts)
		return;
	minx = maxx = miny = maxy = minz = maxz = 0;
	for(int i = 1; i < numPoints; ++i)
	{
		if (pts[i].x < pts[minx].x) minx = i;
		if (pts[i].x > pts[maxx].x) maxx = i;
		if (pts[i].y < pts[miny].y) miny = i;
		if (pts[i].y > pts[maxy].y) maxy = i;
		if (pts[i].z < pts[minz].z) minz = i;
		if (pts[i].z > pts[maxz].z) maxz = i;
	}
}

AABB AABB::FromCenterAndSize(const float3 &aabbCenterPos, const float3 &aabbSize)
{
	float3 halfSize = aabbSize * 0.5f;
	return AABB(aabbCenterPos - halfSize, aabbCenterPos + halfSize);
}

float3 AABB::Size() const
{
	return maxPoint - minPoint;
}

float3 AABB::HalfSize() const
{
	return Size() / 2.f;
}

float AABB::Volume() const
{
	return Size().ProductOfElements();
}

float AABB::SurfaceArea() const
{
	float3 size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

float3 AABB::RandomPointInside(LCG &rng) const
{
	return PointInside(rng.Float(), rng.Float(), rng.Float());
}

float3 AABB::RandomPointOnSurface(LCG &rng) const
{
	return FacePoint(rng.Int(0, 5), rng.Float(), rng.Float());
}

float3 AABB::RandomPointOnEdge(LCG &rng) const
{
	return PointOnEdge(rng.Int(0, 11), rng.Float());
}

float3 AABB::RandomCornerPoint(LCG &rng) const
{
	return CornerPoint(rng.Int(0, 7));
}

void AABB::Translate(const float3 &offset)
{
	minPoint += offset;
	maxPoint += offset;
}

void AABB::Scale(const float3 &centerPoint, float scaleFactor)
{
	return Scale(centerPoint, float3(scaleFactor, scaleFactor, scaleFactor));
}

void AABB::Scale(const float3 &centerPoint, const float3 &scaleFactor)
{
	float3x4 transform = float3x4::Scale(scaleFactor, centerPoint);
	minPoint = transform.MulPos(minPoint);
	maxPoint = transform.MulPos(maxPoint);
}

/// See Christer Ericson's Real-time Collision Detection, p. 87, or
/// James Arvo's "Transforming Axis-aligned Bounding Boxes" in Graphics Gems 1, pp. 548-550.
/// http://www.graphicsgems.org/
template<typename Matrix>
void AABBTransformAsAABB(AABB &aabb, Matrix &m)
{
	float3 newCenter = m.MulPos(aabb.CenterPoint());

	float3 newDir;
	float3 h = aabb.HalfSize();
	// The following is equal to taking the absolute value of the whole matrix m.
	newDir.x = ABSDOT3(m[0], h);
	newDir.y = ABSDOT3(m[1], h);
	newDir.z = ABSDOT3(m[2], h);
	aabb.minPoint = newCenter - newDir;
	aabb.maxPoint = newCenter + newDir;
}

void AABB::TransformAsAABB(const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	assume(transform.HasUniformScale());

	AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float3x4 &transform)
{
	assume(transform.IsColOrthogonal());
	assume(transform.HasUniformScale());

	AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float4x4 &transform)
{
	assume(transform.IsColOrthogonal3());
	assume(transform.HasUniformScale());
	assume(transform.Row(3).Equals(0,0,0,1));

	AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const Quat &transform)
{
	float3 newCenter = transform.Transform(CenterPoint());
	float3 newDir = Abs((transform.Transform(Size()) / 2.f));
	minPoint = newCenter - newDir;
	maxPoint = newCenter + newDir;
}

OBB AABB::Transform(const float3x3 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const float3x4 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const float4x4 &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

OBB AABB::Transform(const Quat &transform) const
{
	OBB obb;
	obb.SetFrom(*this, transform);
	return obb;
}

float3 AABB::ClosestPoint(const float3 &targetPoint) const
{
	return targetPoint.Clamp(minPoint, maxPoint);
}

float AABB::Distance(const float3 &point) const
{
	///@todo This function could be slightly optimized. See Christer Ericson's
	/// Real-Time Collision Detection, p.131.
	return ClosestPoint(point).Distance(point);
}

float AABB::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

bool AABB::Contains(const float3 &point) const
{
	return minPoint.x <= point.x && point.x <= maxPoint.x &&
		   minPoint.y <= point.y && point.y <= maxPoint.y &&
		   minPoint.z <= point.z && point.z <= maxPoint.z;
}

bool AABB::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool AABB::Contains(const AABB &aabb) const
{
	return Contains(aabb.minPoint) && Contains(aabb.maxPoint);
}

bool AABB::Contains(const OBB &obb) const
{
	return Contains(obb.MinimalEnclosingAABB());
}

bool AABB::Contains(const Sphere &sphere) const
{
	///@todo Optimize.
	return Contains(sphere.MinimalEnclosingAABB());
}

bool AABB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool AABB::Contains(const Polygon &polygon) const
{
	return Contains(polygon.MinimalEnclosingAABB());
}

bool AABB::Contains(const Frustum &frustum) const
{
	return Contains(frustum.MinimalEnclosingAABB());
}

bool AABB::Contains(const Polyhedron &polyhedron) const
{
	return Contains(polyhedron.MinimalEnclosingAABB());
}

bool AABB::IntersectLineAABB(const float3 &linePos, const float3 &lineDir, float &tNear, float &tFar) const
{
	// Never call the SSE version here. The SSE version does not output tNear and tFar, because
	// the memory stores have been profiled to make it slower than the CPP version. Therefore the SSE
	// version does not output tNear and tFar (profile shows it to be about 10x faster than the CPP version).
	return IntersectLineAABB_CPP(linePos, lineDir, tNear, tFar);
}

bool AABB::Intersects(const Line &line) const
{
	float tNear = -FLOAT_INF;
	float tFar = FLOAT_INF;

#ifdef MATH_SSE
	return IntersectLineAABB_SSE(float4(line.pos, 1.f), float4(line.dir, 0.f), tNear, tFar);
#else
	return IntersectLineAABB_CPP(line.pos, line.dir, tNear, tFar);
#endif
}

bool AABB::Intersects(const Ray &ray) const
{
	float tNear = 0;
	float tFar = FLOAT_INF;

#ifdef MATH_SSE
	return IntersectLineAABB_SSE(float4(ray.pos, 1.f), float4(ray.dir, 0.f), tNear, tFar);
#else
	return IntersectLineAABB_CPP(ray.pos, ray.dir, tNear, tFar);
#endif
}

bool AABB::Intersects(const LineSegment &lineSegment) const
{
	float3 dir = lineSegment.b - lineSegment.a;
	float len = dir.Length();
	if (len <= 1e-4f) // Degenerate line segment? Fall back to point-in-AABB test.
		return Contains(lineSegment.a);

	float invLen = 1.f / len;
	dir *= invLen;
	float tNear = 0.f, tFar = len;
#ifdef MATH_SSE
	return IntersectLineAABB_SSE(float4(lineSegment.a, 1.f), float4(dir, 0.f), tNear, tFar);
#else
	return IntersectLineAABB_CPP(lineSegment.a, dir, tNear, tFar);
#endif
}

bool AABB::IntersectLineAABB_CPP(const float3 &linePos, const float3 &lineDir, float &tNear, float &tFar) const
{
	assume(lineDir.IsNormalized());
	assume(tNear <= tFar && "AABB::IntersectLineAABB: User gave a degenerate line as input for the intersection test!");
	// The user should have inputted values for tNear and tFar to specify the desired subrange [tNear, tFar] of the line
	// for this intersection test.
	// For a Line-AABB test, pass in
	//    tNear = -FLOAT_INF;
	//    tFar = FLOAT_INF;
	// For a Ray-AABB test, pass in
	//    tNear = 0.f;
	//    tFar = FLOAT_INF;
	// For a LineSegment-AABB test, pass in
	//    tNear = 0.f;
	//    tFar = LineSegment.Length();

	// Test each cardinal plane (X, Y and Z) in turn.
	if (!EqualAbs(lineDir.x, 0.f))
	{
		float recipDir = RecipFast(lineDir.x);
		float t1 = (minPoint.x - linePos.x) * recipDir;
		float t2 = (maxPoint.x - linePos.x) * recipDir;

		// tNear tracks distance to intersect (enter) the AABB.
		// tFar tracks the distance to exit the AABB.
		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);

		if (tNear > tFar)
			return false; // Box is missed since we "exit" before entering it.
	}
	else if (linePos.x < minPoint.x || linePos.x > maxPoint.x)
		return false; // The ray can't possibly enter the box, abort.

	if (!EqualAbs(lineDir.y, 0.f))
	{
		float recipDir = RecipFast(lineDir.y);
		float t1 = (minPoint.y - linePos.y) * recipDir;
		float t2 = (maxPoint.y - linePos.y) * recipDir;

		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);

		if (tNear > tFar)
			return false; // Box is missed since we "exit" before entering it.
	}
	else if (linePos.y < minPoint.y || linePos.y > maxPoint.y)
		return false; // The ray can't possibly enter the box, abort.

	if (!EqualAbs(lineDir.z, 0.f)) // ray is parallel to plane in question
	{
		float recipDir = RecipFast(lineDir.z);
		float t1 = (minPoint.z - linePos.z) * recipDir;
		float t2 = (maxPoint.z - linePos.z) * recipDir;

		if (t1 < t2)
			tNear = Max(t1, tNear), tFar = Min(t2, tFar);
		else // Swap t1 and t2.
			tNear = Max(t2, tNear), tFar = Min(t1, tFar);
	}
	else if (linePos.z < minPoint.z || linePos.z > maxPoint.z)
		return false; // The ray can't possibly enter the box, abort.

	return tNear <= tFar;
}

#ifdef MATH_SSE
bool AABB::IntersectLineAABB_SSE(const float4 &rayPos, const float4 &rayDir, float tNear, float tFar) const
{
	assume(rayDir.IsNormalized4());
	assume(tNear <= tFar && "AABB::IntersectLineAABB: User gave a degenerate line as input for the intersection test!");
	/* For reference, this is the C++ form of the vectorized SSE code below.

	float4 recipDir = rayDir.RecipFast4();
	float4 t1 = (aabbMinPoint - rayPos).Mul(recipDir);
	float4 t2 = (aabbMaxPoint - rayPos).Mul(recipDir);
	float4 near = t1.Min(t2);
	float4 far = t1.Max(t2);
	float4 rayDirAbs = rayDir.Abs();

	if (rayDirAbs.x > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.x, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.x, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.x < aabbMinPoint.x || rayPos.x > aabbMaxPoint.x) // early-out if the ray can't possibly enter the box.
		return false;

	if (rayDirAbs.y > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.y, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.y, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.y < aabbMinPoint.y || rayPos.y > aabbMaxPoint.y) // early-out if the ray can't possibly enter the box.
		return false;

	if (rayDirAbs.z > 1e-4f) // ray is parallel to plane in question
	{
		tNear = Max(near.z, tNear); // tNear tracks distance to intersect (enter) the AABB.
		tFar = Min(far.z, tFar); // tFar tracks the distance to exit the AABB.
	}
	else if (rayPos.z < aabbMinPoint.z || rayPos.z > aabbMaxPoint.z) // early-out if the ray can't possibly enter the box.
		return false;

	return tNear < tFar;
	*/

	__m128 recipDir = _mm_rcp_ps(rayDir.v);
	// Note: The above performs an approximate reciprocal (11 bits of precision).
	// For a full precision reciprocal, perform a div:
//	__m128 recipDir = _mm_div_ps(_mm_set1_ps(1.f), rayDir.v);

	__m128 t1 = _mm_mul_ps(_mm_sub_ps(MinPoint_SSE(), rayPos.v), recipDir);
	__m128 t2 = _mm_mul_ps(_mm_sub_ps(MaxPoint_SSE(), rayPos.v), recipDir);

	__m128 nearD = _mm_min_ps(t1, t2); // [0 n3 n2 n1]
	__m128 farD = _mm_max_ps(t1, t2);  // [0 f3 f2 f1]

	// Check if the ray direction is parallel to any of the cardinal axes, and if so,
	// mask those [near, far] ranges away from the hit test computations.
	__m128 rayDirAbs = abs_ps(rayDir.v);

	const __m128 epsilon = _mm_set1_ps(1e-4f);
	// zeroDirections[i] will be nonzero for each axis i the ray is parallel to.
	__m128 zeroDirections = _mm_cmple_ps(rayDirAbs, epsilon);

	const __m128 floatInf = _mm_set1_ps(FLOAT_INF);
	const __m128 floatNegInf = _mm_set1_ps(-FLOAT_INF);

	// If the ray is parallel to one of the axes, replace the slab range for that axis
	// with [-inf, inf] range instead. (which is a no-op in the comparisons below)
	nearD = cmov_ps(nearD, floatNegInf, zeroDirections);
	farD = cmov_ps(farD , floatInf, zeroDirections);

	// Next, we need to compute horizontally max(nearD[0], nearD[1], nearD[2]) and min(farD[0], farD[1], farD[2])
	// to see if there is an overlap in the hit ranges.
	__m128 v1 = _mm_shuffle_ps(nearD, farD, _MM_SHUFFLE(0, 0, 0, 0)); // [f1 f1 n1 n1]
	__m128 v2 = _mm_shuffle_ps(nearD, farD, _MM_SHUFFLE(1, 1, 1, 1)); // [f2 f2 n2 n2]
	__m128 v3 = _mm_shuffle_ps(nearD, farD, _MM_SHUFFLE(2, 2, 2, 2)); // [f3 f3 n3 n3]
	nearD = _mm_max_ps(v1, _mm_max_ps(v2, v3));
	farD = _mm_min_ps(v1, _mm_min_ps(v2, v3));
	farD = _mm_shuffle_ps(farD, farD, _MM_SHUFFLE(3, 3, 3, 3)); // Unpack the result from high offset in the register.
	nearD = _mm_max_ps(nearD, _mm_set_ss(tNear));
	farD = _mm_min_ps(farD, _mm_set_ss(tFar));

	// Finally, test if the ranges overlap.
	__m128 rangeIntersects = _mm_cmple_ss(nearD, farD);

	// To store out out the interval of intersection, uncomment the following:
	// These are disabled, since without these, the whole function runs without a single memory store,
	// which has been profiled to be very fast! Uncommenting these causes an order-of-magnitude slowdown.
	// For now, using the SSE version only where the tNear and tFar ranges are not interesting.
//	_mm_store_ss(&tNear, nearD);
//	_mm_store_ss(&tFar, farD);

	// To avoid false positives, need to have an additional rejection test for each cardinal axis the ray direction
	// is parallel to.
	__m128 out2 = _mm_cmplt_ps(rayPos.v, MinPoint_SSE());
	__m128 out3 = _mm_cmpgt_ps(rayPos.v, MaxPoint_SSE());
	out2 = _mm_or_ps(out2, out3);
	zeroDirections = _mm_and_ps(zeroDirections, out2);

	__m128 yOut = _mm_shuffle_ps(zeroDirections, zeroDirections, _MM_SHUFFLE(1,1,1,1));
	__m128 zOut = _mm_shuffle_ps(zeroDirections, zeroDirections, _MM_SHUFFLE(2,2,2,2));

	zeroDirections = _mm_or_ps(_mm_or_ps(zeroDirections, yOut), zOut);
	// Intersection occurs if the slab ranges had positive overlap and if the test was not rejected by the ray being
	// parallel to some cardinal axis.
	__m128 intersects = _mm_andnot_ps(zeroDirections, rangeIntersects);
	__m128 epsilonMasked = _mm_and_ps(epsilon, intersects);
	return _mm_comieq_ss(epsilon, epsilonMasked) != 0;
}
#endif

bool AABB::Intersects(const Ray &ray, float &dNear, float &dFar) const
{
	dNear = 0.f;
	dFar = FLOAT_INF;
	return IntersectLineAABB(ray.pos, ray.dir, dNear, dFar);
}

bool AABB::Intersects(const Line &line, float &dNear, float &dFar) const
{
	dNear = -FLOAT_INF;
	dFar = FLOAT_INF;
	return IntersectLineAABB(line.pos, line.dir, dNear, dFar);
}

bool AABB::Intersects(const LineSegment &lineSegment, float &dNear, float &dFar) const
{
	float3 dir = lineSegment.b - lineSegment.a;
	float len = dir.Length();
	if (len <= 1e-4f) // Degenerate line segment? Fall back to point-in-AABB test.
	{
		dNear = 0.f;
		dFar = 1.f;
		return Contains(lineSegment.a);
	}
	float invLen = 1.f / len;
	dir *= invLen;
	dNear = 0.f;
	dFar = len;
	bool hit = IntersectLineAABB(lineSegment.a, dir, dNear, dFar);
	dNear *= invLen;
	dFar *= invLen;
	return hit;
}

bool AABB::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool AABB::Intersects(const AABB &aabb) const
{
	// If any of the cardinal X,Y,Z axes is a separating axis, then
	// there is no intersection.
	return minPoint.x < aabb.maxPoint.x &&
	       minPoint.y < aabb.maxPoint.y &&
	       minPoint.z < aabb.maxPoint.z &&
	       aabb.minPoint.x < maxPoint.x &&
	       aabb.minPoint.y < maxPoint.y &&
	       aabb.minPoint.z < maxPoint.z;
}

bool AABB::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool AABB::Intersects(const Sphere &sphere, float3 *closestPointOnAABB) const
{
	// Find the point on this AABB closest to the sphere center.
	float3 pt = ClosestPoint(sphere.pos);

	// If that point is inside sphere, the AABB and sphere intersect.
	if (closestPointOnAABB)
		*closestPointOnAABB = pt;

	return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool AABB::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool AABB::Intersects(const Triangle &triangle) const
{
	return triangle.Intersects(*this);
}

bool AABB::Intersects(const Polygon &polygon) const
{
	return ToPolyhedron().Intersects(polygon);
}

bool AABB::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool AABB::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

void AABB::ProjectToAxis(const float3 &axis, float &dMin, float &dMax) const
{
	float3 c = CenterPoint();
	float3 e = HalfDiagonal();

	// Compute the projection interval radius of the AABB onto L(t) = aabb.center + t * plane.normal;
	float r = e[0]*Abs(axis[0]) + e[1]*Abs(axis[1]) + e[2]*Abs(axis[2]);
	// Compute the distance of the box center from plane.
	float s = Dot(axis, c);
	dMin = s - r;
	dMax = s + r;
	if (dMin > dMax)
		Swap(dMin, dMax);
}

void AABB::Enclose(const float3 &point)
{
	minPoint = Min(minPoint, point);
	maxPoint = Max(maxPoint, point);
}

void AABB::Enclose(const LineSegment &lineSegment)
{
	Enclose(lineSegment.a);
	Enclose(lineSegment.b);
}

void AABB::Enclose(const AABB &aabb)
{
	minPoint = Min(minPoint, aabb.minPoint);
	maxPoint = Max(maxPoint, aabb.maxPoint);
}

void AABB::Enclose(const OBB &obb)
{
	for(int i = 0; i < 8; ++i)
		Enclose(obb.CornerPoint(i));
}

void AABB::Enclose(const Sphere &sphere)
{
	Enclose(sphere.pos - float3(sphere.r,sphere.r,sphere.r));
	Enclose(sphere.pos + float3(sphere.r,sphere.r,sphere.r));
}

void AABB::Enclose(const Triangle &triangle)
{
	Enclose(triangle.a);
	Enclose(triangle.b);
	Enclose(triangle.c);
}

void AABB::Enclose(const Capsule &capsule)
{
	Enclose(capsule.l.a - float3(capsule.r, capsule.r, capsule.r));
	Enclose(capsule.l.a + float3(capsule.r, capsule.r, capsule.r));
	Enclose(capsule.l.b - float3(capsule.r, capsule.r, capsule.r));
	Enclose(capsule.l.b + float3(capsule.r, capsule.r, capsule.r));
}

void AABB::Enclose(const Frustum &frustum)
{
	for(int i = 0; i < 8; ++i)
		Enclose(frustum.CornerPoint(i));
}

void AABB::Enclose(const Polygon &polygon)
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		Enclose(polygon.Vertex(i));
}

void AABB::Enclose(const Polyhedron &polyhedron)
{
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		Enclose(polyhedron.Vertex(i));
}

void AABB::Enclose(const float3 *pointArray, int numPoints)
{
	assume(pointArray || numPoints == 0);
	if (!pointArray)
		return;
	for(int i = 0; i < numPoints; ++i)
		Enclose(pointArray[i]);
}

void AABB::Triangulate(int numFacesX, int numFacesY, int numFacesZ,
                       float3 *outPos, float3 *outNormal, float2 *outUV,
                       bool ccwIsFrontFacing) const
{
	assume(numFacesX >= 1);
	assume(numFacesY >= 1);
	assume(numFacesZ >= 1);

	assume(outPos);
	if (!outPos)
		return;

	// Generate both X-Y planes.
	int i = 0;
	for(int face = 0; face < 6; ++face) // Faces run in the order -X, +X, -Y, +Y, -Z, +Z.
	{
		int numFacesU;
		int numFacesV;
		bool flip = (face == 1 || face == 2 || face == 5);
		if (ccwIsFrontFacing)
			flip = !flip;
		if (face == 0 || face == 1)
		{
			numFacesU = numFacesY;
			numFacesV = numFacesZ;
		}
		else if (face == 2 || face == 3)
		{
			numFacesU = numFacesX;
			numFacesV = numFacesZ;
		}
		else// if (face == 4 || face == 5)
		{
			numFacesU = numFacesX;
			numFacesV = numFacesY;
		}
		for(int x = 0; x < numFacesU; ++x)
			for(int y = 0; y < numFacesV; ++y)
			{
				float u = (float)x / (numFacesU);
				float v = (float)y / (numFacesV);
				float u2 = (float)(x+1) / (numFacesU);
				float v2 = (float)(y+1) / (numFacesV);
			
				outPos[i]   = FacePoint(face, u, v);
				outPos[i+1] = FacePoint(face, u, v2);
				outPos[i+2] = FacePoint(face, u2, v);
				if (flip)
					Swap(outPos[i+1], outPos[i+2]);
				outPos[i+3] = outPos[i+2];
				outPos[i+4] = outPos[i+1];
				outPos[i+5] = FacePoint(face, u2, v2);

				if (outUV)
				{
					outUV[i]   = float2(u,v);
					outUV[i+1] = float2(u,v2);
					outUV[i+2] = float2(u2,v);
					if (flip)
						Swap(outUV[i+1], outUV[i+2]);
					outUV[i+3] = outUV[i+2];
					outUV[i+4] = outUV[i+1];
					outUV[i+5] = float2(u2,v2);
				}

				if (outNormal)
					for(int j = 0; j < 6; ++j)
						outNormal[i+j] = FaceNormal(face);

				i += 6;
			}
	}
	assert(i == NumVerticesInTriangulation(numFacesX, numFacesY, numFacesZ));
}

void AABB::ToEdgeList(float3 *outPos) const
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

#ifdef MATH_ENABLE_STL_SUPPORT
std::string AABB::ToString() const
{
	char str[256];
	sprintf(str, "AABB(Min:(%.2f, %.2f, %.2f) Max:(%.2f, %.2f, %.2f))", minPoint.x, minPoint.y, minPoint.z, maxPoint.x, maxPoint.y, maxPoint.z);
	return str;
}

std::ostream &operator <<(std::ostream &o, const AABB &aabb)
{
	o << aabb.ToString();
	return o;
}

#endif

AABB AABB::Intersection(const AABB &aabb) const
{
	return AABB(Max(minPoint, aabb.minPoint), Min(maxPoint, aabb.maxPoint));
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void AABB::Triangulate(VertexBuffer &vb, int numFacesX, int numFacesY, int numFacesZ, bool ccwIsFrontFacing) const
{
	Array<float3> pos;
	Array<float3> normal;
	Array<float2> uv;
	int numVertices = (numFacesX*numFacesY + numFacesY*numFacesZ + numFacesX*numFacesZ)*2*6;
	pos.Resize_pod(numVertices);
	normal.Resize_pod(numVertices);
	uv.Resize_pod(numVertices);
	Triangulate(numFacesX, numFacesY, numFacesZ, &pos[0], &normal[0], &uv[0], ccwIsFrontFacing);
	int startIndex = vb.AppendVertices(numVertices);
	for(int i = 0; i < (int)pos.size(); ++i)
	{
		vb.Set(startIndex+i, VDPosition, float4(pos[i],1.f));
		if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			vb.Set(startIndex+i, VDNormal, float4(normal[i],0.f));
		if (vb.Declaration()->TypeOffset(VDUV) >= 0)
			vb.SetFloat2(startIndex+i, VDUV, 0, uv[i]);
	}
}

void AABB::ToLineList(VertexBuffer &vb) const
{
	Array<float3> pos;
	pos.Resize_pod(NumVerticesInEdgeList());
	ToEdgeList(&pos[0]);
	int startIndex = vb.AppendVertices((int)pos.size());
	for(int i = 0; i < (int)pos.size(); ++i)
		vb.Set(startIndex+i, VDPosition, float4(pos[i], 1.f));
}

#endif

OBB operator *(const float3x3 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const float3x4 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const float4x4 &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

OBB operator *(const Quat &transform, const AABB &aabb)
{
	return aabb.Transform(transform);
}

MATH_END_NAMESPACE
