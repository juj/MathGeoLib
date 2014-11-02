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

/** @file Ray.cpp
	@author Jukka Jylänki
	@brief Implementation for the Ray geometry object. */
#include "Ray.h"
#include "AABB.h"
#include "Line.h"
#include "LineSegment.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "OBB.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Frustum.h"
#include "../Math/Quat.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Triangle.h"
#include "Circle.h"
#include "../Math/MathFunc.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Ray::Ray(const vec &pos_, const vec &dir_)
:pos(pos_), dir(dir_)
{
	assume2(dir.IsNormalized(), dir, dir.LengthSq());
}

Ray::Ray(const Line &line)
:pos(line.pos), dir(line.dir)
{
	assume2(dir.IsNormalized(), dir, dir.LengthSq());
}

Ray::Ray(const LineSegment &lineSegment)
:pos(lineSegment.a), dir(lineSegment.Dir())
{
}

bool Ray::IsFinite() const
{
	return pos.IsFinite() && dir.IsFinite();
}

vec Ray::GetPoint(float d) const
{
	assume2(dir.IsNormalized(), dir, dir.LengthSq());
	return pos + d * dir;
}

void Ray::Translate(const vec &offset)
{
	pos += offset;
}

void Ray::Transform(const float3x3 &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

void Ray::Transform(const float3x4 &transform)
{
	pos = transform.MulPos(pos);
	dir = transform.MulDir(dir);
}

void Ray::Transform(const float4x4 &transform)
{
	pos = transform.MulPos(pos);
	dir = transform.MulDir(dir);
}

void Ray::Transform(const Quat &transform)
{
	pos = transform.Transform(pos);
	dir = transform.Transform(dir);
}

bool Ray::Contains(const vec &point, float distanceThreshold) const
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

float Ray::Distance(const vec &point, float &d) const
{
	return ClosestPoint(point, d).Distance(point);
}

float Ray::Distance(const Ray &other, float &d, float &d2) const
{
	vec c = ClosestPoint(other, d, d2);
	return c.Distance(other.GetPoint(d2));
}

float Ray::Distance(const Line &other, float &d, float &d2) const
{
	vec c = ClosestPoint(other, d, d2);
	return c.Distance(other.GetPoint(d2));
}

float Ray::Distance(const LineSegment &other, float &d, float &d2) const
{
	vec c = ClosestPoint(other, d, d2);
	return c.Distance(other.GetPoint(d2));
}

float Ray::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Ray::Distance(const Capsule &capsule) const
{
	return Max(0.f, Distance(capsule.l) - capsule.r);
}

vec Ray::ClosestPoint(const vec &targetPoint, float &d) const
{
	d = Max(0.f, Dot(targetPoint - pos, dir));
	return GetPoint(d);
}

vec Ray::ClosestPoint(const Ray &other, float &d, float &d2) const
{
	Line::ClosestPointLineLine(pos, dir, other.pos, other.dir, d, d2);
	if (d < 0.f && d2 < 0.f)
	{
		vec closestPoint = ClosestPoint(other.pos, d);
		vec closestPoint2 = other.ClosestPoint(pos, d2);
		if (closestPoint.DistanceSq(other.pos) <= closestPoint2.DistanceSq(pos))
		{
			d2 = 0.f;
			return closestPoint;
		}
		else
		{
			d = 0.f;
			return pos;
		}
	}
	else if (d < 0.f)
	{
		d = 0.f;
		other.ClosestPoint(pos, d2);
		d2 = Max(0.f, d2);
		return pos;
	}
	else if (d2 < 0.f)
	{
		vec pt = ClosestPoint(other.pos, d);
		d = Max(0.f, d);
		d2 = 0.f;
		return pt;
	}
	else
	{
		return GetPoint(d);
	}
}

vec Ray::ClosestPoint(const Line &other, float &d, float &d2) const
{
	Line::ClosestPointLineLine(pos, dir, other.pos, other.dir, d, d2);
	if (d < 0.f)
	{
		d = 0.f;
		other.ClosestPoint(pos, d2);
		return pos;
	}
	else
		return GetPoint(d);
}

vec Ray::ClosestPoint(const LineSegment &other, float &d, float &d2) const
{
	Line::ClosestPointLineLine(pos, dir, other.a, other.b - other.a, d, d2);
	if (d < 0.f)
	{
		d = 0.f;
		if (d2 >= 0.f && d2 <= 1.f)
		{
			other.ClosestPoint(pos, d2);
			return pos;
		}

		vec p;
		float t2;

		if (d2 < 0.f)
		{
			p = other.a;
			t2 = 0.f;
		}
		else // u2 > 1.f
		{
			p = other.b;
			t2 = 1.f;
		}

		vec closestPoint = ClosestPoint(p, d);
		vec closestPoint2 = other.ClosestPoint(pos, d2);
		if (closestPoint.DistanceSq(p) <= closestPoint2.DistanceSq(pos))
		{
			d2 = t2;
			return closestPoint;
		}
		else
		{
			d = 0.f;
			return pos;
		}
	}
	else if (d2 < 0.f)
	{
		d2 = 0.f;
		return ClosestPoint(other.a, d);
	}
	else if (d2 > 1.f)
	{
		d2 = 1.f;
		return ClosestPoint(other.b, d);
	}
	else
		return GetPoint(d);
}

bool Ray::Intersects(const Triangle &triangle, float *d, vec *intersectionPoint) const
{
	return triangle.Intersects(*this, d, intersectionPoint);
}

bool Ray::Intersects(const Triangle &triangle) const
{
	float u, v;
	float t = Triangle::IntersectLineTri(pos, dir, triangle.a, triangle.b, triangle.c, u, v);
	if (t < 0.f || t == FLOAT_INF)
		return false;
	return true;
}

bool Ray::Intersects(const Plane &plane, float *d) const
{
	return plane.Intersects(*this, d);
}

bool Ray::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this, 0);
}

bool Ray::Intersects(const Sphere &sphere, vec *intersectionPoint, vec *intersectionNormal, float *d) const
{
	return sphere.Intersects(*this, intersectionPoint, intersectionNormal, d) > 0;
}

bool Ray::Intersects(const Sphere &sphere) const
{
	return sphere.Intersects(*this, 0, 0, 0) > 0;
}

bool Ray::Intersects(const AABB &aabb) const
{
	return aabb.Intersects(*this);
}

bool Ray::Intersects(const AABB &aabb, float &dNear, float &dFar) const
{
	return aabb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const OBB &obb, float &dNear, float &dFar) const
{
	return obb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
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

LineSegment Ray::ToLineSegment(float dStart, float dEnd) const
{
	return LineSegment(GetPoint(dStart), GetPoint(dEnd));
}

void Ray::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	outMin = outMax = Dot(direction, pos);
	float d = Dot(direction, dir);

	// Most of the time, the projection interval will be a half-infinite range, extending to either -inf or +inf.
	if (d > 1e-4f)
		outMax = FLOAT_INF;
	else if (d < -1e4f)
		outMin = -FLOAT_INF;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Ray::ToString() const
{
	char str[256];
	sprintf(str, "Ray(Pos:(%.2f, %.2f, %.2f) Dir:(%.3f, %.3f, %.3f))", pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
	return str;
}

std::string Ray::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(pos.x, str); *s = ','; ++s;
	s = SerializeFloat(pos.y, s); *s = ','; ++s;
	s = SerializeFloat(pos.z, s); *s = ','; ++s;
	s = SerializeFloat(dir.x, s); *s = ','; ++s;
	s = SerializeFloat(dir.y, s); *s = ','; ++s;
	s = SerializeFloat(dir.z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string Ray::SerializeToCodeString() const
{
	return "Ray(" + pos.SerializeToCodeString() + "," + dir.SerializeToCodeString() + ")";
}

std::ostream &operator <<(std::ostream &o, const Ray &ray)
{
	o << ray.ToString();
	return o;
}

#endif

Ray Ray::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return Ray(vec::nan, vec::nan);
	Ray r;
	MATH_SKIP_WORD(str, "Ray(");
	MATH_SKIP_WORD(str, "Pos:(");
	r.pos = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Dir:(");
	r.dir = DirVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return r;
}

Ray operator *(const float3x3 &transform, const Ray &ray)
{
	assume(transform.IsInvertible());
	return Ray(transform * ray.pos, (transform * ray.dir).Normalized());
}

Ray operator *(const float3x4 &transform, const Ray &ray)
{
	assume(transform.IsInvertible());
	return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir).Normalized());
}

Ray operator *(const float4x4 &transform, const Ray &ray)
{
	assume(transform.IsInvertible());
	return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir).Normalized());
}

Ray operator *(const Quat &transform, const Ray &ray)
{
	return Ray(transform * ray.pos, transform * ray.dir);
}

MATH_END_NAMESPACE
