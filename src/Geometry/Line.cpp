/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Line.cpp
	@author Jukka Jylänki
	@brief Implementation for the Line geometry object. */
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Geometry/OBB.h"
#include "Math/Quat.h"
#include "Geometry/Frustum.h"
#include "Geometry/Triangle.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Sphere.h"
#include "Geometry/AABB.h"
#include "Geometry/Capsule.h"
#include "Geometry/Circle.h"
#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

/// A helper function to compute the line-line closest point.
/** This code is adapted from http://paulbourke.net/geometry/lineline3d/ .
	dmnop = (xm - xn)(xo - xp) + (ym - yn)(yo - yp) + (zm - zn)(zo - zp).
	@param v An array of four floats: [0]: line 0 start. [1]: line 0 end. [2]: line 1 start. [3]: line 1 end. */
float Dmnop(const float3 *v, int m, int n, int o, int p)
{
	return (v[m].x - v[n].x) * (v[o].x - v[p].x) + (v[m].y - v[n].y) * (v[o].y - v[p].y) + (v[m].z - v[n].z) * (v[o].z - v[p].z);
}

/// Computes the closest point pair on two lines.
/** The first line is specified by two points start0 and end0. The second line is specified by
	two points start1 and end1.
	The implementation of this function follows http://paulbourke.net/geometry/lineline3d/ .
	@param d [out] If specified, receives the normalized distance of the closest point along the first line.
		This pointer may be left null.
	@param d2 [out] If specified, receives the normalized distance of the closest point along the second line.
		This pointer may be left null.
	@return Returns the closest point on line start0<->end0 to the second line.
	@note This is a low-level utility function. You probably want to use ClosestPoint() or Distance() instead.
	@see ClosestPoint(), Distance(). */
float3 Line::ClosestPointLineLine(float3 start0, float3 end0, float3 start1, float3 end1, float *d, float *d2)
{
	const float3 v[4] = { start0, end0, start1, end1 };

	float d0232 = Dmnop(v,0,2,3,2);
	float d3210 = Dmnop(v,3,2,1,0);
	float d3232 = Dmnop(v,3,2,3,2);
	float mu = (d0232 * d3210 - Dmnop(v,0,2,1,0)*d3232) / (Dmnop(v,1,0,1,0)*Dmnop(v,3,2,3,2) - Dmnop(v,3,2,1,0)*Dmnop(v,3,2,1,0));
	if (d)
		*d = mu;

	if (d2)
		*d2 = (d0232 + mu * d3210) / d3232;

	return start0 + mu * (end0 - start0);
}

Line::Line(const float3 &pos_, const float3 &dir_)
:pos(pos_), dir(dir_)
{
	assume(dir.IsNormalized());
}

Line::Line(const Ray &ray)
:pos(ray.pos), dir(ray.dir)
{
	assume(dir.IsNormalized());
}

Line::Line(const LineSegment &lineSegment)
:pos(lineSegment.a), dir(lineSegment.Dir())
{
}

float3 Line::GetPoint(float d) const
{
	assert(dir.IsNormalized());
	return pos + d * dir;
}

void Line::Transform(const float3x3 &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

void Line::Transform(const float3x4 &transform)
{
	pos = transform.TransformPos(pos);
	dir = transform.TransformDir(dir);
}

void Line::Transform(const float4x4 &transform)
{
	pos = transform.TransformPos(pos);
	dir = transform.TransformDir(dir);
}

void Line::Transform(const Quat &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

bool Line::Contains(const float3 &point, float distanceThreshold) const
{
	return ClosestPoint(point).DistanceSq(point) <= distanceThreshold;
}

bool Line::Contains(const Ray &ray, float epsilon) const
{
	return Contains(ray.pos, epsilon) && dir.Equals(ray.dir, epsilon);
}

bool Line::Contains(const LineSegment &lineSegment, float epsilon) const
{
	return Contains(lineSegment.a, epsilon) && Contains(lineSegment.b, epsilon);
}

bool Line::Equals(const Line &line, float epsilon) const
{
	assume(dir.IsNormalized());
	assume(line.dir.IsNormalized());
	// If the point of the other line is on this line, and the two lines point to the same, or exactly reverse directions,
	// they must be equal.
	return Contains(line.pos, epsilon) && EqualAbs(Abs(dir.Dot(line.dir)), 1.f, epsilon);
}

float Line::Distance(const float3 &point, float *d) const
{
	return ClosestPoint(point, d).Distance(point);
}

float Line::Distance(const Ray &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Line::Distance(const Ray &other) const
{
	return Distance(other, 0, 0);
}

float Line::Distance(const Line &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Line::Distance(const Line &other) const
{
	return Distance(other, 0, 0);
}

float Line::Distance(const LineSegment &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Line::Distance(const LineSegment &other) const
{
	return Distance(other, 0, 0);
}

float Line::Distance(const Sphere &other) const
{
	return Max(0.f, Distance(other.pos) - other.r);
}

float Line::Distance(const Capsule &other) const
{
	return Max(0.f, Distance(other.l) - other.r);
}

bool Line::Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const
{
	return triangle.Intersects(*this, d, intersectionPoint);
}

bool Line::Intersects(const Plane &plane, float *d) const
{
	return plane.Intersects(*this, d);
}

bool Line::Intersects(const Sphere &s, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
	return s.Intersects(*this, intersectionPoint, intersectionNormal, d);
}

bool Line::Intersects(const AABB &aabb, float *dNear, float *dFar) const
{
	return aabb.Intersects(*this, dNear, dFar);
}

bool Line::Intersects(const OBB &obb, float *dNear, float *dFar) const
{
	return obb.Intersects(*this, dNear, dFar);
}

bool Line::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool Line::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Line::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Line::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

bool Line::IntersectsDisc(const Circle &disc) const
{
	return disc.IntersectsDisc(*this);
}

float3 Line::ClosestPoint(const float3 &targetPoint, float *d) const
{
	float u = Dot(targetPoint - pos, dir);
	if (d)
		*d = u;
	return GetPoint(u);
}

float3 Line::ClosestPoint(const Ray &other, float *d, float *d2) const
{
	///\bug Properly cap d2.
	return ClosestPointLineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Line::ClosestPoint(const Line &other, float *d, float *d2) const
{
	return ClosestPointLineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Line::ClosestPoint(const LineSegment &other, float *d, float *d2) const
{
	///\bug Properly cap d2.
	return ClosestPointLineLine(pos, pos + dir, other.a, other.b, d, d2);
}

float3 Line::ClosestPoint(const Triangle &triangle, float *outU, float *outV, float *outD) const
{
	float d;
	if (!outD)
		outD = &d;
	triangle.ClosestPoint(*this, outU, outV, outD);
	return GetPoint(*outD);
}

bool Line::AreCollinear(const float3 &p1, const float3 &p2, const float3 &p3, float epsilon)
{
	///@todo Improve this check to be distance length -invariant.
	return Abs((p2-p1).Dot(p3-p1)) <= epsilon;
}

Ray Line::ToRay() const
{
	return Ray(pos, dir);
}

LineSegment Line::ToLineSegment(float d) const
{
	return LineSegment(pos, GetPoint(d));
}

Line operator *(const float3x3 &transform, const Line &l)
{
	return Line(transform * l.pos, transform * l.dir);
}

Line operator *(const float3x4 &transform, const Line &l)
{
	return Line(transform.MulPos(l.pos), transform.MulDir(l.dir));
}

Line operator *(const float4x4 &transform, const Line &l)
{
	return Line(transform.MulPos(l.pos), transform.MulDir(l.dir));
}

Line operator *(const Quat &transform, const Line &l)
{
	return Line(transform * l.pos, transform * l.dir);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Line::ToString() const
{
	char str[256];
	sprintf(str, "Line(pos:(%.2f, %.2f, %.2f) dir:(%.2f, %.2f, %.2f))", 
		pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
	return str;
}
#endif

MATH_END_NAMESPACE
