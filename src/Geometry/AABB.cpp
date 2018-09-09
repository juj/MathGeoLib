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
#include "PBVolume.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Triangle.h"
#include "Capsule.h"

#include "../Math/float4x4_neon.h"

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

AABB::AABB(const vec &minPoint_, const vec &maxPoint_)
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

void AABB::SetFromCenterAndSize(const vec &center, const vec &size)
{
	vec halfSize = 0.5f * size;
	minPoint = center - halfSize;
	maxPoint = center + halfSize;
}

void AABB::SetFrom(const OBB &obb)
{
	vec halfSize = Abs(obb.axis[0]*obb.r[0]) + Abs(obb.axis[1]*obb.r[1]) + Abs(obb.axis[2]*obb.r[2]);
	SetFromCenterAndSize(obb.pos, 2.f*halfSize);
}

void AABB::SetFrom(const Sphere &s)
{
	vec d = DIR_VEC_SCALAR(s.r);
	minPoint = s.pos - d;
	maxPoint = s.pos + d;
}

void AABB::SetFrom(const vec *pointArray, int numPoints)
{
	assume(pointArray || numPoints == 0);
	SetNegativeInfinity();
	if (!pointArray)
		return;
	for(int i = 0; i < numPoints; ++i)
		Enclose(pointArray[i]);
}

PBVolume<6> AABB::ToPBVolume() const
{
	PBVolume<6> pbVolume;
	for(int i = 0; i < 6; ++i)
		pbVolume.p[i] = FacePlane(i);

	return pbVolume;
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
	
	assume(p.IsClosed());
	assume(p.IsConvex());
	assume(p.EulerFormulaHolds());
	assume(p.FaceIndicesValid());
	assume(p.FacesAreNondegeneratePlanar());
	assume(p.Contains(this->CenterPoint()));
	return p;
}

OBB AABB::ToOBB() const
{
	return OBB(*this);
}

Sphere AABB::MinimalEnclosingSphere() const
{
	return Sphere(CenterPoint(), Size().Length() * 0.5f);
}

Sphere AABB::MaximalContainedSphere() const
{
	vec halfSize = HalfSize();
	return Sphere(CenterPoint(), Min(halfSize.x, halfSize.y, halfSize.z));
}

bool AABB::IsFinite() const
{
	return minPoint.IsFinite() && maxPoint.IsFinite();
}

bool AABB::IsDegenerate() const
{
#ifdef _MSC_VER
	// MSVC generates code that assumes nans can't be present - add an explicit check for that case.
	return IsNan(minPoint.x) || IsNan(minPoint.y) || IsNan(minPoint.z) ||
		IsNan(maxPoint.x) || IsNan(maxPoint.y) || IsNan(maxPoint.z) ||
		!(minPoint.x < maxPoint.x && minPoint.y < maxPoint.y && minPoint.z < maxPoint.z);
#else
	return !(minPoint.x < maxPoint.x && minPoint.y < maxPoint.y && minPoint.z < maxPoint.z);
#endif
}

vec AABB::CenterPoint() const
{
	return (minPoint + maxPoint) * 0.5f;
}

vec AABB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	vec d = maxPoint - minPoint;
	return minPoint + d.Mul(POINT_VEC(x, y, z));
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
		case 0: return LineSegment(minPoint, POINT_VEC(minPoint.x, minPoint.y, maxPoint.z));
		case 1: return LineSegment(minPoint, POINT_VEC(minPoint.x, maxPoint.y, minPoint.z));
		case 2: return LineSegment(minPoint, POINT_VEC(maxPoint.x, minPoint.y, minPoint.z));
		case 3: return LineSegment(POINT_VEC(minPoint.x, minPoint.y, maxPoint.z), POINT_VEC(minPoint.x, maxPoint.y, maxPoint.z));
		case 4: return LineSegment(POINT_VEC(minPoint.x, minPoint.y, maxPoint.z), POINT_VEC(maxPoint.x, minPoint.y, maxPoint.z));
		case 5: return LineSegment(POINT_VEC(minPoint.x, maxPoint.y, minPoint.z), POINT_VEC(minPoint.x, maxPoint.y, maxPoint.z));
		case 6: return LineSegment(POINT_VEC(minPoint.x, maxPoint.y, minPoint.z), POINT_VEC(maxPoint.x, maxPoint.y, minPoint.z));
		case 7: return LineSegment(POINT_VEC(minPoint.x, maxPoint.y, maxPoint.z), maxPoint);
		case 8: return LineSegment(POINT_VEC(maxPoint.x, minPoint.y, minPoint.z), POINT_VEC(maxPoint.x, minPoint.y, maxPoint.z));
		case 9: return LineSegment(POINT_VEC(maxPoint.x, minPoint.y, minPoint.z), POINT_VEC(maxPoint.x, maxPoint.y, minPoint.z));
		case 10: return LineSegment(POINT_VEC(maxPoint.x, minPoint.y, maxPoint.z), maxPoint);
		case 11: return LineSegment(POINT_VEC(maxPoint.x, maxPoint.y, minPoint.z), maxPoint);
	}
}

vec AABB::CornerPoint(int cornerIndex) const
{
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return minPoint;
		case 1: return POINT_VEC(minPoint.x, minPoint.y, maxPoint.z);
		case 2: return POINT_VEC(minPoint.x, maxPoint.y, minPoint.z);
		case 3: return POINT_VEC(minPoint.x, maxPoint.y, maxPoint.z);
		case 4: return POINT_VEC(maxPoint.x, minPoint.y, minPoint.z);
		case 5: return POINT_VEC(maxPoint.x, minPoint.y, maxPoint.z);
		case 6: return POINT_VEC(maxPoint.x, maxPoint.y, minPoint.z);
		case 7: return maxPoint;
	}
}

vec AABB::ExtremePoint(const vec &direction) const
{
	return POINT_VEC((direction.x >= 0.f ? maxPoint.x : minPoint.x),
	                 (direction.y >= 0.f ? maxPoint.y : minPoint.y),
	                 (direction.z >= 0.f ? maxPoint.z : minPoint.z));
}

vec AABB::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

vec AABB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	vec d = maxPoint - minPoint;
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minPoint.x, minPoint.y, minPoint.z + u * d.z);
	case 1: return POINT_VEC(minPoint.x, maxPoint.y, minPoint.z + u * d.z);
	case 2: return POINT_VEC(maxPoint.x, minPoint.y, minPoint.z + u * d.z);
	case 3: return POINT_VEC(maxPoint.x, maxPoint.y, minPoint.z + u * d.z);

	case 4: return POINT_VEC(minPoint.x, minPoint.y + u * d.y, minPoint.z);
	case 5: return POINT_VEC(maxPoint.x, minPoint.y + u * d.y, minPoint.z);
	case 6: return POINT_VEC(minPoint.x, minPoint.y + u * d.y, maxPoint.z);
	case 7: return POINT_VEC(maxPoint.x, minPoint.y + u * d.y, maxPoint.z);

	case 8: return POINT_VEC(minPoint.x + u * d.x, minPoint.y, minPoint.z);
	case 9: return POINT_VEC(minPoint.x + u * d.x, minPoint.y, maxPoint.z);
	case 10: return POINT_VEC(minPoint.x + u * d.x, maxPoint.y, minPoint.z);
	case 11: return POINT_VEC(minPoint.x + u * d.x, maxPoint.y, maxPoint.z);
	}
}

vec AABB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	vec center = (minPoint + maxPoint) * 0.5f;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minPoint.x, center.y, center.z);
	case 1: return POINT_VEC(maxPoint.x, center.y, center.z);
	case 2: return POINT_VEC(center.x, minPoint.y, center.z);
	case 3: return POINT_VEC(center.x, maxPoint.y, center.z);
	case 4: return POINT_VEC(center.x, center.y, minPoint.z);
	case 5: return POINT_VEC(center.x, center.y, maxPoint.z);
	}
}

vec AABB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	vec d = maxPoint - minPoint;
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return POINT_VEC(minPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
	case 1: return POINT_VEC(maxPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
	case 2: return POINT_VEC(minPoint.x + u * d.x, minPoint.y, minPoint.z + v * d.z);
	case 3: return POINT_VEC(minPoint.x + u * d.x, maxPoint.y, minPoint.z + v * d.z);
	case 4: return POINT_VEC(minPoint.x + u * d.x, minPoint.y + v * d.y, minPoint.z);
	case 5: return POINT_VEC(minPoint.x + u * d.x, minPoint.y + v * d.y, maxPoint.z);
	}
}

vec AABB::FaceNormal(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return DIR_VEC(-1,  0,  0);
	case 1: return DIR_VEC( 1,  0,  0);
	case 2: return DIR_VEC( 0, -1,  0);
	case 3: return DIR_VEC( 0,  1,  0);
	case 4: return DIR_VEC( 0,  0, -1);
	case 5: return DIR_VEC( 0,  0,  1);
	}
}

Plane AABB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	return Plane(FaceCenterPoint(faceIndex), FaceNormal(faceIndex));
}

void AABB::GetCornerPoints(vec *outPointArray) const
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

AABB AABB::MinimalEnclosingAABB(const vec *pointArray, int numPoints)
{
	AABB aabb;
	aabb.SetFrom(pointArray, numPoints);
	return aabb;
}

void AABB::ExtremePointsAlongAABB(const vec *pts, int numPoints, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz)
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

AABB AABB::FromCenterAndSize(const vec &aabbCenterPos, const vec &aabbSize)
{
	vec halfSize = aabbSize * 0.5f;
	return AABB(aabbCenterPos - halfSize, aabbCenterPos + halfSize);
}

vec AABB::Size() const
{
	return maxPoint - minPoint;
}

vec AABB::HalfSize() const
{
	return Size() * 0.5f;
}

float AABB::Volume() const
{
	vec sz = Size();
	return sz.x * sz.y * sz.z;
}

float AABB::SurfaceArea() const
{
	vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

vec AABB::RandomPointInside(LCG &rng) const
{
	float f1 = rng.Float();
	float f2 = rng.Float();
	float f3 = rng.Float();
	return PointInside(f1, f2, f3);
}

vec AABB::RandomPointOnSurface(LCG &rng) const
{
	int i = rng.Int(0, 5);
	float f1 = rng.Float();
	float f2 = rng.Float();
	return FacePoint(i, f1, f2);
}

vec AABB::RandomPointOnEdge(LCG &rng) const
{
	int i = rng.Int(0, 11);
	float f = rng.Float();
	return PointOnEdge(i, f);
}

vec AABB::RandomCornerPoint(LCG &rng) const
{
	return CornerPoint(rng.Int(0, 7));
}

void AABB::Translate(const vec &offset)
{
	minPoint += offset;
	maxPoint += offset;
}

void AABB::Scale(const vec &centerPoint, float scaleFactor)
{
	minPoint = (minPoint - centerPoint) * scaleFactor + centerPoint;
	maxPoint = (maxPoint - centerPoint) * scaleFactor + centerPoint;
}

void AABB::Scale(const vec &centerPoint, const vec &scaleFactor)
{
	float3x4 transform = float3x4::Scale(DIR_TO_FLOAT3(scaleFactor), POINT_TO_FLOAT3(centerPoint)); ///\todo mat
	minPoint = POINT_VEC(transform.MulPos(POINT_TO_FLOAT3(minPoint))); ///\todo mat
	maxPoint = POINT_VEC(transform.MulPos(POINT_TO_FLOAT3(maxPoint))); ///\todo mat
}

/// See Christer Ericson's Real-time Collision Detection, p. 87, or
/// James Arvo's "Transforming Axis-aligned Bounding Boxes" in Graphics Gems 1, pp. 548-550.
/// http://www.graphicsgems.org/
template<typename Matrix>
void AABBTransformAsAABB(AABB &aabb, Matrix &m)
{
	const vec centerPoint = (aabb.minPoint + aabb.maxPoint) * 0.5f;
	const vec halfSize = centerPoint - aabb.minPoint;
	vec newCenter = m.MulPos(centerPoint);

	// The following is equal to taking the absolute value of the whole matrix m.
	vec newDir = DIR_VEC(ABSDOT3(m[0], halfSize), ABSDOT3(m[1], halfSize), ABSDOT3(m[2], halfSize));
	aabb.minPoint = newCenter - newDir;
	aabb.maxPoint = newCenter + newDir;
}

#ifdef MATH_SIMD
void AABBTransformAsAABB_SIMD(AABB &aabb, const float4x4 &m)
{
	simd4f minPt = aabb.minPoint;
	simd4f maxPt = aabb.maxPoint;
	simd4f centerPoint = muls_ps(add_ps(minPt, maxPt), 0.5f);
	simd4f newCenter = mat4x4_mul_vec4(m.row, centerPoint);

	simd4f halfSize = sub_ps(centerPoint, minPt);
	simd4f x = abs_ps(mul_ps(m.row[0], halfSize));
	simd4f y = abs_ps(mul_ps(m.row[1], halfSize));
	simd4f z = abs_ps(mul_ps(m.row[2], halfSize));
	simd4f w = zero_ps();
	simd4f newDir = hadd4_ps(x, y, z, w);
	aabb.minPoint = sub_ps(newCenter, newDir);
	aabb.maxPoint = add_ps(newCenter, newDir);
}
#endif

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

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	AABBTransformAsAABB_SIMD(*this, transform);
#else
	AABBTransformAsAABB(*this, transform);
#endif
}

void AABB::TransformAsAABB(const Quat &transform)
{
	vec newCenter = transform.Transform(CenterPoint());
	vec newDir = Abs((transform.Transform(Size()) * 0.5f));
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

vec AABB::ClosestPoint(const vec &targetPoint) const
{
#ifdef MATH_VEC_IS_FLOAT4
	assume(EqualAbs(minPoint.w, 1.f));
	assume(EqualAbs(maxPoint.w, 1.f));
	assume(EqualAbs(targetPoint.w, 1.f));
#endif
	return targetPoint.Clamp(minPoint, maxPoint);
}

float AABB::Distance(const vec &point) const
{
	///@todo This function could be slightly optimized. See Christer Ericson's
	/// Real-Time Collision Detection, p.131.
	return ClosestPoint(point).Distance(point);
}

float AABB::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

bool AABB::Contains(const vec &point) const
{
// Benchmarking this code is very difficult, since branch prediction makes the scalar version
// look very good. In isolation the scalar version might be better, however when joined with
// other SSE computation, the SIMD variants are probably more efficient because the data is
// already "hot" in the registers. Therefore favoring the SSE version over the scalar version
// when possible.

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Benchmark 'AABBContains_positive': AABB::Contains(point) positive
	//    Best: 2.048 nsecs / 3.5128 ticks, Avg: 2.241 nsecs, Worst: 4.277 nsecs
	// Benchmark 'AABBContains_negative': AABB::Contains(point) negative
	//    Best: 2.048 nsecs / 3.467 ticks, Avg: 2.115 nsecs, Worst: 4.156 nsecs
	// Benchmark 'AABBContains_unpredictable': AABB::Contains(point) unpredictable
	//    Best: 2.590 nsecs / 4.4106 ticks, Avg: 2.978 nsecs, Worst: 6.084 nsecs
	simd4f a = cmplt_ps(point, minPoint);
	simd4f b = cmpgt_ps(point, maxPoint);
	a = or_ps(a, b);
	return allzero_ps(a) != 0;
#else
	// Benchmark 'AABBContains_positive': AABB::Contains(point) positive
	//    Best: 2.108 nsecs / 3.6022 ticks, Avg: 2.232 nsecs, Worst: 4.638 nsecs
	// Benchmark 'AABBContains_negative': AABB::Contains(point) negative
	//    Best: 1.988 nsecs / 3.361 ticks, Avg: 2.148 nsecs, Worst: 4.457 nsecs
	// Benchmark 'AABBContains_unpredictable': AABB::Contains(point) unpredictable
	//    Best: 3.554 nsecs / 6.0764 ticks, Avg: 3.803 nsecs, Worst: 6.264 nsecs
	return minPoint.x <= point.x && point.x <= maxPoint.x &&
	       minPoint.y <= point.y && point.y <= maxPoint.y &&
	       minPoint.z <= point.z && point.z <= maxPoint.z;
#endif
}

bool AABB::Contains(const LineSegment &lineSegment) const
{
	return Contains(Min(lineSegment.a, lineSegment.b), Max(lineSegment.a, lineSegment.b));
}

bool AABB::Contains(const vec &aabbMinPoint, const vec &aabbMaxPoint) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	simd4f a = cmplt_ps(aabbMinPoint, minPoint);
	simd4f b = cmpgt_ps(aabbMaxPoint, maxPoint);
	a = or_ps(a, b);
	return allzero_ps(a) != 0;
#else
	return minPoint.x <= aabbMinPoint.x && maxPoint.x >= aabbMaxPoint.x &&
	       minPoint.y <= aabbMinPoint.y && maxPoint.y >= aabbMaxPoint.y &&
	       minPoint.z <= aabbMinPoint.z && maxPoint.z >= aabbMaxPoint.z;
#endif
}

bool AABB::Contains(const OBB &obb) const
{
	return Contains(obb.MinimalEnclosingAABB());
}

bool AABB::Contains(const Sphere &sphere) const
{
	return Contains(sphere.pos - DIR_VEC_SCALAR(sphere.r), sphere.pos + DIR_VEC_SCALAR(sphere.r));
}

bool AABB::Contains(const Capsule &capsule) const
{
	return Contains(capsule.MinimalEnclosingAABB());
}

bool AABB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.BoundingAABB());
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

bool AABB::IntersectLineAABB(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const
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

#ifdef MATH_SIMD
	return IntersectLineAABB_SSE(line.pos, line.dir, tNear, tFar);
#else
	return IntersectLineAABB_CPP(line.pos, line.dir, tNear, tFar);
#endif
}

bool AABB::Intersects(const Ray &ray) const
{
	float tNear = 0;
	float tFar = FLOAT_INF;

#ifdef MATH_SIMD
	return IntersectLineAABB_SSE(ray.pos, ray.dir, tNear, tFar);
#else
	return IntersectLineAABB_CPP(ray.pos, ray.dir, tNear, tFar);
#endif
}

bool AABB::Intersects(const LineSegment &lineSegment) const
{
	vec dir = lineSegment.b - lineSegment.a;
	float len = dir.Length();
	if (len <= 1e-4f) // Degenerate line segment? Fall back to point-in-AABB test.
		return Contains(lineSegment.a);

	float invLen = 1.f / len;
	dir *= invLen;
	float tNear = 0.f, tFar = len;
#ifdef MATH_SIMD
	return IntersectLineAABB_SSE(lineSegment.a, dir, tNear, tFar);
#else
	return IntersectLineAABB_CPP(lineSegment.a, dir, tNear, tFar);
#endif
}

bool AABB::IntersectLineAABB_CPP(const vec &linePos, const vec &lineDir, float &tNear, float &tFar) const
{
	assume2(lineDir.IsNormalized(), lineDir, lineDir.LengthSq());
	assume2(tNear <= tFar && "AABB::IntersectLineAABB: User gave a degenerate line as input for the intersection test!", tNear, tFar);
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

#ifdef MATH_SIMD
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

	simd4f recipDir = rcp_ps(rayDir.v);
	// Note: The above performs an approximate reciprocal (11 bits of precision).
	// For a full precision reciprocal, perform a div:
//	simd4f recipDir = div_ps(set1_ps(1.f), rayDir.v);

	simd4f t1 = mul_ps(sub_ps(minPoint, rayPos.v), recipDir);
	simd4f t2 = mul_ps(sub_ps(maxPoint, rayPos.v), recipDir);

	simd4f nearD = min_ps(t1, t2); // [0 n3 n2 n1]
	simd4f farD = max_ps(t1, t2);  // [0 f3 f2 f1]

	// Check if the ray direction is parallel to any of the cardinal axes, and if so,
	// mask those [near, far] ranges away from the hit test computations.
	simd4f rayDirAbs = abs_ps(rayDir.v);

	const simd4f epsilon = set1_ps(1e-4f);
	// zeroDirections[i] will be nonzero for each axis i the ray is parallel to.
	simd4f zeroDirections = cmple_ps(rayDirAbs, epsilon);

	const simd4f floatInf = set1_ps(FLOAT_INF);
	const simd4f floatNegInf = set1_ps(-FLOAT_INF);

	// If the ray is parallel to one of the axes, replace the slab range for that axis
	// with [-inf, inf] range instead. (which is a no-op in the comparisons below)
	nearD = cmov_ps(nearD, floatNegInf, zeroDirections);
	farD = cmov_ps(farD, floatInf, zeroDirections);

	// Next, we need to compute horizontally max(nearD[0], nearD[1], nearD[2]) and min(farD[0], farD[1], farD[2])
	// to see if there is an overlap in the hit ranges.
	simd4f v1 = axx_bxx_ps(nearD, farD); // [f1 f1 n1 n1]
	simd4f v2 = ayy_byy_ps(nearD, farD); // [f2 f2 n2 n2]
	simd4f v3 = azz_bzz_ps(nearD, farD); // [f3 f3 n3 n3]
	nearD = max_ps(v1, max_ps(v2, v3));
	farD = min_ps(v1, min_ps(v2, v3));
	farD = wwww_ps(farD); // Unpack the result from high offset in the register.
	nearD = max_ps(nearD, setx_ps(tNear));
	farD = min_ps(farD, setx_ps(tFar));

	// Finally, test if the ranges overlap.
	simd4f rangeIntersects = cmple_ps(nearD, farD); // Only x channel used, higher ones ignored.

	// To store out out the interval of intersection, uncomment the following:
	// These are disabled, since without these, the whole function runs without a single memory store,
	// which has been profiled to be very fast! Uncommenting these causes an order-of-magnitude slowdown.
	// For now, using the SSE version only where the tNear and tFar ranges are not interesting.
//	_mm_store_ss(&tNear, nearD);
//	_mm_store_ss(&tFar, farD);

	// To avoid false positives, need to have an additional rejection test for each cardinal axis the ray direction
	// is parallel to.
	simd4f out2 = cmplt_ps(rayPos.v, minPoint);
	simd4f out3 = cmpgt_ps(rayPos.v, maxPoint);
	out2 = or_ps(out2, out3);
	zeroDirections = and_ps(zeroDirections, out2);

	simd4f yOut = yyyy_ps(zeroDirections);
	simd4f zOut = zzzz_ps(zeroDirections);

	zeroDirections = or_ps(or_ps(zeroDirections, yOut), zOut);
	// Intersection occurs if the slab ranges had positive overlap and if the test was not rejected by the ray being
	// parallel to some cardinal axis.
	simd4f intersects = andnot_ps(zeroDirections, rangeIntersects);
	simd4f epsilonMasked = and_ps(epsilon, intersects);
	return comieq_ss(epsilon, epsilonMasked) != 0;
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
	vec dir = lineSegment.b - lineSegment.a;
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
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Benchmark 'AABBIntersectsAABB_positive': AABB::Intersects(AABB) positive
	//    Best: 2.229 nsecs / 3.848 ticks, Avg: 2.409 nsecs, Worst: 4.457 nsecs
	// Benchmark 'AABBIntersectsAABB_random': AABB::Intersects(AABB) random
	//    Best: 3.072 nsecs / 5.2904 ticks, Avg: 3.262 nsecs, Worst: 5.301 nsecs

	simd4f a = cmpge_ps(minPoint.v, aabb.maxPoint.v);
	simd4f b = cmpge_ps(aabb.minPoint.v, maxPoint.v);
	a = or_ps(a, b);
	return a_and_b_allzero_ps(a, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)) != 0; // Mask off results from the W channel.
#else
	// Benchmark 'AABBIntersectsAABB_positive': AABB::Intersects(AABB) positive
	//    Best: 2.108 nsecs / 3.588 ticks, Avg: 2.310 nsecs, Worst: 5.481 nsecs
	// Benchmark 'AABBIntersectsAABB_random': AABB::Intersects(AABB) random
	//    Best: 7.529 nsecs / 12.8282 ticks, Avg: 8.892 nsecs, Worst: 16.323 nsecs

	// If any of the cardinal X,Y,Z axes is a separating axis, then
	// there is no intersection.
	return minPoint.x < aabb.maxPoint.x &&
	       minPoint.y < aabb.maxPoint.y &&
	       minPoint.z < aabb.maxPoint.z &&
	       aabb.minPoint.x < maxPoint.x &&
	       aabb.minPoint.y < maxPoint.y &&
	       aabb.minPoint.z < maxPoint.z;
#endif
}

bool AABB::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool AABB::Intersects(const Sphere &sphere, vec *closestPointOnAABB) const
{
	// Find the point on this AABB closest to the sphere center.
	vec pt = ClosestPoint(sphere.pos);

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
	return polygon.Intersects(*this);
}

bool AABB::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool AABB::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

void AABB::ProjectToAxis(const vec &axis, float &dMin, float &dMax) const
{
	vec c = (minPoint + maxPoint) * 0.5f;
	vec e = maxPoint - c;

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	vec absAxis = axis.Abs();
	float r = Abs(e.Dot(absAxis));
#else
	// Compute the projection interval radius of the AABB onto L(t) = aabb.center + t * plane.normal;
	float r = Abs(e[0]*Abs(axis[0]) + e[1]*Abs(axis[1]) + e[2]*Abs(axis[2]));
#endif
	// Compute the distance of the box center from plane.
	float s = axis.Dot(c);
	dMin = s - r;
	dMax = s + r;
}

int AABB::UniqueFaceNormals(vec *out) const
{
	out[0] = DIR_VEC(1,0,0);
	out[1] = DIR_VEC(0,1,0);
	out[2] = DIR_VEC(0,0,1);
	return 3;
}

int AABB::UniqueEdgeDirections(vec *out) const
{
	out[0] = DIR_VEC(1,0,0);
	out[1] = DIR_VEC(0,1,0);
	out[2] = DIR_VEC(0,0,1);
	return 3;
}

void AABB::Enclose(const vec &point)
{
	minPoint = Min(minPoint, point);
	maxPoint = Max(maxPoint, point);
}

void AABB::Enclose(const LineSegment &lineSegment)
{
	Enclose(Min(lineSegment.a, lineSegment.b), Max(lineSegment.a, lineSegment.b));
}

void AABB::Enclose(const vec &aabbMinPoint, const vec &aabbMaxPoint)
{
	minPoint = Min(minPoint, aabbMinPoint);
	maxPoint = Max(maxPoint, aabbMaxPoint);
}

void AABB::Enclose(const OBB &obb)
{
	vec absAxis0 = obb.axis[0].Abs();
	vec absAxis1 = obb.axis[1].Abs();
	vec absAxis2 = obb.axis[2].Abs();
	vec d = obb.r.x * absAxis0 + obb.r.y * absAxis1 + obb.r.z * absAxis2;
	Enclose(obb.pos - d, obb.pos + d);
}

void AABB::Enclose(const Sphere &sphere)
{
	vec d = DIR_VEC_SCALAR(sphere.r);
	Enclose(sphere.pos - d, sphere.pos + d);
}

void AABB::Enclose(const Triangle &triangle)
{
	Enclose(Min(triangle.a, triangle.b, triangle.c), Max(triangle.a, triangle.b, triangle.c));
}

void AABB::Enclose(const Capsule &capsule)
{
	vec d = DIR_VEC_SCALAR(capsule.r);
	minPoint = Min(minPoint, Min(capsule.l.a, capsule.l.b) - d);
	maxPoint = Max(maxPoint, Max(capsule.l.a, capsule.l.b) + d);
}

void AABB::Enclose(const Frustum &frustum)
{
	Enclose(frustum.MinimalEnclosingAABB());
}

void AABB::Enclose(const Polygon &polygon)
{
	Enclose(polygon.MinimalEnclosingAABB());
}

void AABB::Enclose(const Polyhedron &polyhedron)
{
	Enclose(polyhedron.MinimalEnclosingAABB());
}

void AABB::Enclose(const vec *pointArray, int numPoints)
{
	assume(pointArray || numPoints == 0);
	if (!pointArray)
		return;
	for(int i = 0; i < numPoints; ++i)
		Enclose(pointArray[i]);
}

void AABB::Triangulate(int numFacesX, int numFacesY, int numFacesZ,
                       vec *outPos, vec *outNormal, float2 *outUV,
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

void AABB::ToEdgeList(vec *outPos) const
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

std::string AABB::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(minPoint.x, str); *s = ','; ++s;
	s = SerializeFloat(minPoint.y, s); *s = ','; ++s;
	s = SerializeFloat(minPoint.z, s); *s = ','; ++s;
	s = SerializeFloat(maxPoint.x, s); *s = ','; ++s;
	s = SerializeFloat(maxPoint.y, s); *s = ','; ++s;
	s = SerializeFloat(maxPoint.z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string AABB::SerializeToCodeString() const
{
	return "AABB(" + minPoint.SerializeToCodeString() + "," + maxPoint.SerializeToCodeString() + ")";
}

AABB AABB::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return AABB(vec::nan, vec::nan);
	AABB a;
	MATH_SKIP_WORD(str, "AABB(");
	MATH_SKIP_WORD(str, "Min:(");
	a.minPoint = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Max:(");
	a.maxPoint = PointVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return a;
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
	Array<vec> pos;
	Array<vec> normal;
	Array<float2> uv;
	int numVertices = (numFacesX*numFacesY + numFacesY*numFacesZ + numFacesX*numFacesZ)*2*6;
	pos.Resize_unspecified(numVertices);
	normal.Resize_unspecified(numVertices);
	uv.Resize_unspecified(numVertices);
	Triangulate(numFacesX, numFacesY, numFacesZ, &pos[0], &normal[0], &uv[0], ccwIsFrontFacing);
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

void AABB::ToLineList(VertexBuffer &vb) const
{
	Array<vec> pos;
	pos.Resize_unspecified(NumVerticesInEdgeList());
	ToEdgeList(&pos[0]);
	int startIndex = vb.AppendVertices((int)pos.size());
	for(int i = 0; i < (int)pos.size(); ++i)
		vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(pos[i]));
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
