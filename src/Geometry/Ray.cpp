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

/** @file Ray.cpp
	@author Jukka Jylänki
	@brief Implementation for the Ray geometry object. */
#include "Geometry/AABB.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Geometry/OBB.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Frustum.h"
#include "Math/Quat.h"
#include "Geometry/Sphere.h"
#include "Geometry/Capsule.h"
#include "Geometry/Triangle.h"
#include "Geometry/Circle.h"
#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

Ray::Ray(const float3 &pos_, const float3 &dir_)
:pos(pos_), dir(dir_)
{
	assume(dir.IsNormalized());
}

Ray::Ray(const Line &line)
:pos(line.pos), dir(line.dir)
{
	assume(dir.IsNormalized());
}

Ray::Ray(const LineSegment &lineSegment)
:pos(lineSegment.a), dir(lineSegment.Dir())
{
}

float3 Ray::GetPoint(float d) const
{
	assert(dir.IsNormalized());
	return pos + d * dir;
}

void Ray::Transform(const float3x3 &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

void Ray::Transform(const float3x4 &transform)
{
	pos = transform.TransformPos(pos);
	dir = transform.TransformDir(dir);
}

void Ray::Transform(const float4x4 &transform)
{
	pos = transform.TransformPos(pos);
	dir = transform.TransformDir(dir);
}

void Ray::Transform(const Quat &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

bool Ray::Contains(const float3 &point, float distanceThreshold) const
{
	return ClosestPoint(point).DistanceSq(point) <= distanceThreshold;
}

bool Ray::Contains(const LineSegment &lineSegment, float distanceThreshold) const
{
	return Contains(lineSegment.a, distanceThreshold) && Contains(lineSegment.b, distanceThreshold);
}

bool Ray::Equals(const Ray &rhs, float epsilon) const
{
	return pos.Equals(rhs.pos, epsilon) && dir.Equals(rhs.dir, epsilon);
}

float Ray::Distance(const float3 &point, float *d) const
{
	return ClosestPoint(point, d).Distance(point);
}

float Ray::Distance(const float3 &point) const
{
	return Distance(point, 0);
}

float Ray::Distance(const Ray &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const Ray &ray) const
{
	return Distance(ray, 0, 0);
}

float Ray::Distance(const Line &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const Line &line) const
{
	return Distance(line, 0, 0);
}

float Ray::Distance(const LineSegment &other, float *d, float *d2) const
{
	float u2;
	float3 c = ClosestPoint(other, d, &u2);
	if (d2) *d2 = u2;
	return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const LineSegment &lineSegment) const
{
	return Distance(lineSegment, 0, 0);
}

float Ray::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Ray::Distance(const Capsule &capsule) const
{
	return Max(0.f, Distance(capsule.l) - capsule.r);
}

float3 Ray::ClosestPoint(const float3 &targetPoint, float *d) const
{
	float u = Max(0.f, Dot(targetPoint - pos, dir));
	if (d)
		*d = u;
	return GetPoint(u);
}

float3 Ray::ClosestPoint(const Ray &other, float *d, float *d2) const
{
	///\bug Properly cap d2.
	return Line::ClosestPointLineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Ray::ClosestPoint(const Line &other, float *d, float *d2) const
{
	return Line::ClosestPointLineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Ray::ClosestPoint(const LineSegment &other, float *d, float *d2) const
{
	///\bug Properly cap d2.
	return Line::ClosestPointLineLine(pos, pos + dir, other.a, other.b, d, d2);
}

bool Ray::Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const
{
	return triangle.Intersects(*this, d, intersectionPoint);
}

bool Ray::Intersects(const Triangle &triangle) const
{
	return triangle.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const Plane &plane, float *d) const
{
	return plane.Intersects(*this, d);
}

bool Ray::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this, 0);
}

bool Ray::Intersects(const Sphere &sphere, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
	return sphere.Intersects(*this, intersectionPoint, intersectionNormal, d);
}

bool Ray::Intersects(const Sphere &sphere) const
{
	return sphere.Intersects(*this, 0, 0, 0);
}

bool Ray::Intersects(const AABB &aabb, float *dNear, float *dFar) const
{
	return aabb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const AABB &aabb) const
{
	return aabb.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const OBB &obb, float *dNear, float *dFar) const
{
	return obb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool Ray::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Ray::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Ray::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

bool Ray::IntersectsDisc(const Circle &disc) const
{
	return disc.IntersectsDisc(*this);
}

Line Ray::ToLine() const
{
	return Line(pos, dir);
}

LineSegment Ray::ToLineSegment(float d) const
{
	return LineSegment(pos, GetPoint(d));
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Ray::ToString() const
{
	char str[256];
	sprintf(str, "Ray(Pos:(%.2f, %.2f, %.2f) Dir:(%.2f, %.2f, %.2f))", pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
	return str;
}
#endif

Ray operator *(const float3x3 &transform, const Ray &ray)
{
	return Ray(transform * ray.pos, transform * ray.dir);
}

Ray operator *(const float3x4 &transform, const Ray &ray)
{
	return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir));
}

Ray operator *(const float4x4 &transform, const Ray &ray)
{
	return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir));
}

Ray operator *(const Quat &transform, const Ray &ray)
{
	return Ray(transform * ray.pos, transform * ray.dir);
}

MATH_END_NAMESPACE
