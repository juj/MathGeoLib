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

/** @file Circle.cpp
	@author Jukka Jylänki
	@brief Implementation for the Circle geometry object. */
#include "Circle.h"
#include "Plane.h"
#include "../Math/MathFunc.h"
#include "../Math/float3x3.h"
#include "../Math/Quat.h"
#include "Ray.h"
#include "AABB.h"
#include "OBB.h"
#include "LineSegment.h"
#include "Line.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Circle::Circle(const float3 &center, const float3 &n, float radius)
:pos(center),
normal(n),
r(radius)
{
}

float3 Circle::BasisU() const
{
	return normal.Perpendicular();
}

float3 Circle::BasisV() const
{
	return normal.AnotherPerpendicular();
}

float3 Circle::GetPoint(float angleRadians) const
{
	return pos + r * (Cos(angleRadians) * BasisU() + Sin(angleRadians) * BasisV());
}

float3 Circle::GetPoint(float angleRadians, float d) const
{
	return pos + r * d * (Cos(angleRadians) * BasisU() + Sin(angleRadians) * BasisV());
}

float3 Circle::ExtremePoint(const float3 &direction) const
{
	float3 d = direction - direction.ProjectToNorm(normal);
	if (d.IsZero())
		return pos;
	else
		return pos + d.ScaledToLength(r);
}

Plane Circle::ContainingPlane() const
{
	return Plane(pos, normal);
}

void Circle::Translate(const float3 &offset)
{
	pos += offset;
}

void Circle::Transform(const float3x3 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal());
	pos = transform.Mul(pos);
	normal = transform.Mul(normal).Normalized();
	r *= transform.Col(0).Length(); // Scale the radius of the circle.
}

void Circle::Transform(const float3x4 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal());
	pos = transform.MulPos(pos);
	normal = transform.MulDir(normal).Normalized();
	r *= transform.Col(0).Length(); // Scale the radius of the circle.
}

void Circle::Transform(const float4x4 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal3());
	pos = transform.MulPos(pos);
	normal = transform.MulDir(normal).Normalized();
	r *= transform.Col3(0).Length(); // Scale the radius of the circle.
}

void Circle::Transform(const Quat &transform)
{
	pos = transform.Mul(pos);
	normal = transform.Mul(normal);
}

bool Circle::EdgeContains(const float3 &point, float maxDistance) const
{
	return DistanceToEdge(point) <= maxDistance;
}
/*
bool Circle::DiscContains(const float3 &point, float maxDistance) const
{
	return DistanceToDisc(point) <= maxDistance;
}

*/
float Circle::DistanceToEdge(const float3 &point) const
{
	return ClosestPointToEdge(point).Distance(point);
}
/*
float Circle::DistanceToEdge(const Ray &ray, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(ray, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(ray.GetPoint(t));
}

float Circle::DistanceToEdge(const LineSegment &lineSegment, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(lineSegment, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(lineSegment.GetPoint(t));
}

float Circle::DistanceToEdge(const Line &line, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(line, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(line.GetPoint(t));
}
*/
float Circle::DistanceToDisc(const float3 &point) const
{
	return ClosestPointToDisc(point).Distance(point);
}

float3 Circle::ClosestPointToEdge(const float3 &point) const
{
	float3 pointOnPlane = ContainingPlane().Project(point);
	float3 diff = pointOnPlane - pos;
	if (diff.IsZero())
		return GetPoint(0); // The point is in the center of the circle, all points are equally close.
	return pos + diff.ScaledToLength(r);
}

float3 Circle::ClosestPointToDisc(const float3 &point) const
{
	float3 pointOnPlane = ContainingPlane().Project(point);
	float3 diff = pointOnPlane - pos;
	float dist = diff.LengthSq();
	if (dist > r*r)
		diff = diff * (r / Sqrt(dist));

	return pos + diff;
}

int Circle::Intersects(const Plane &plane, float3 *pt1, float3 *pt2) const
{
	return plane.Intersects(*this, pt1, pt2);
}

int Circle::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool Circle::IntersectsDisc(const Line &line) const
{
	float d;
	bool intersectsPlane = line.Intersects(ContainingPlane(), &d);
	if (intersectsPlane)
		return false;
	return line.GetPoint(d).DistanceSq(pos) <= r*r;
}

bool Circle::IntersectsDisc(const LineSegment &lineSegment) const
{
	float d;
	bool intersectsPlane = lineSegment.Intersects(ContainingPlane(), &d);
	if (intersectsPlane)
		return false;
	return lineSegment.GetPoint(d).DistanceSq(pos) <= r*r;
}

bool Circle::IntersectsDisc(const Ray &ray) const
{
	float d;
	bool intersectsPlane = ray.Intersects(ContainingPlane(), &d);
	if (intersectsPlane)
		return false;
	return ray.GetPoint(d).DistanceSq(pos) <= r*r;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::vector<float3> Circle::IntersectsFaces(const AABB &aabb) const
{
    return IntersectsFaces(aabb.ToOBB());
}

std::vector<float3> Circle::IntersectsFaces(const OBB &obb) const
{
	std::vector<float3> intersectionPoints;
	for(int i = 0; i < 6; ++i)
	{		
		Plane p = obb.FacePlane(i);
		float3 pt1, pt2;
		int numIntersections = Intersects(p, &pt1, &pt2);
		if (numIntersections >= 1 && obb.Contains(pt1))
			intersectionPoints.push_back(pt1);
		if (numIntersections >= 2 && obb.Contains(pt2))
			intersectionPoints.push_back(pt2);
	}
	return intersectionPoints;
}

std::string Circle::ToString() const
{
	char str[256];
	sprintf(str, "Circle(pos:(%.2f, %.2f, %.2f) normal:(%.2f, %.2f, %.2f), r:%.2f)",
		pos.x, pos.y, pos.z, normal.x, normal.y, normal.z, r);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Circle &circle)
{
	o << circle.ToString();
	return o;
}

#endif

Circle operator *(const float3x3 &transform, const Circle &circle)
{
	Circle c(circle);
	c.Transform(transform);
	return c;
}

Circle operator *(const float3x4 &transform, const Circle &circle)
{
	Circle c(circle);
	c.Transform(transform);
	return c;
}

Circle operator *(const float4x4 &transform, const Circle &circle)
{
	Circle c(circle);
	c.Transform(transform);
	return c;
}

Circle operator *(const Quat &transform, const Circle &circle)
{
	Circle c(circle);
	c.Transform(transform);
	return c;
}

MATH_END_NAMESPACE
