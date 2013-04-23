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

/** @file Capsule.cpp
	@author Jukka Jylänki
	@brief Implementation for the Capsule geometry object. */
#include "Capsule.h"
#include "../Math/MathConstants.h"
#include "../Math/MathFunc.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "AABB.h"
#include "OBB.h"
#include "Frustum.h"
#include "Plane.h"
#include "Ray.h"
#include "Line.h"
#include "LineSegment.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Sphere.h"
#include "Circle.h"
#include "Triangle.h"
#include "../Algorithm/Random/LCG.h"
#include "../Math/assume.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Capsule::Capsule(const LineSegment &endPoints, float radius)
:l(endPoints), r(radius)
{
}

Capsule::Capsule(const float3 &bottomPoint, const float3 &topPoint, float radius)
:l(bottomPoint, topPoint), r(radius)
{
}

void Capsule::SetFrom(const Sphere &s)
{
	l = LineSegment(s.pos, s.pos);
	r = s.r;
}

float Capsule::LineLength() const
{
	return l.Length();
}

float Capsule::Diameter() const
{
	return 2.f * r;
}

float3 Capsule::Bottom() const
{
	return l.a - UpDirection() * r;
}

float3 Capsule::Center() const
{
	return l.CenterPoint();
}

float3 Capsule::ExtremePoint(const float3 &direction) const
{
	return (Dot(direction, l.b - l.a) >= 0.f ? l.b : l.a) + direction.ScaledToLength(r);
}

void Capsule::ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const
{
	outMin = Dot(direction, l.a);
	outMax = Dot(direction, l.b);
	if (outMax < outMin)
		Swap(outMin, outMax);

	// The following requires that direction is normalized, otherwise we would have to sub/add 'r * direction.Length()', but
	// don't want to do that for performance reasons.
	assume(direction.IsNormalized());
	outMin -= r;
	outMax += r;
}

float3 Capsule::Top() const
{
	return l.b + UpDirection() * r;
}

float3 Capsule::UpDirection() const
{
	float3 d = l.b - l.a;
	d.Normalize(); // Will always result in a normalized vector, even if l.a == l.b.
	return d;
}

float Capsule::Height() const
{
	return LineLength() + Diameter();
}

float Capsule::Volume() const
{
	return pi * r * r * LineLength() + 4.f * pi * r * r * r / 3.f;
}

float Capsule::SurfaceArea() const
{
	return 2.f * pi * r * LineLength() + 4.f * pi * r * r;
}

Circle Capsule::CrossSection(float yPos) const
{
	assume(yPos >= 0.f);
	assume(yPos <= 1.f);
	yPos *= Height();
	float3 up = UpDirection();
	float3 centerPos = Bottom() + up * yPos;
	if (yPos < r) // First section, between Bottom() and lower point.
		return Circle(centerPos, up, Sqrt(r*r - (r-yPos)*(r-yPos)));
	if (yPos < l.Length() + r) // Second section, between lower and upper points.
		return Circle(centerPos, up, r);
	float d = yPos - r - l.Length(); // Third section, above upper point.
	return Circle(centerPos, up, Sqrt(r*r - d*d));
}

LineSegment Capsule::HeightLineSegment() const
{
	return LineSegment(Bottom(), Top());
}

bool Capsule::IsFinite() const
{
	return l.IsFinite() && MATH_NS::IsFinite(r);
}

float3 Capsule::PointInside(float l, float a, float d) const
{
	Circle c = CrossSection(l);
	return c.GetPoint(a*2.f*pi, d);
}

float3 Capsule::UniformPointPerhapsInside(float l, float x, float y) const
{
	return MinimalEnclosingOBB().PointInside(l, x, y);
}

AABB Capsule::MinimalEnclosingAABB() const
{
	AABB aabb(Min(l.a, l.b) - float3(r, r, r), Max(l.a, l.b) + float3(r, r, r));
	return aabb;
}

OBB Capsule::MinimalEnclosingOBB() const
{
	OBB obb;
	obb.axis[0] = UpDirection();
	obb.axis[1] = obb.axis[0].Perpendicular();
	obb.axis[2] = obb.axis[0].AnotherPerpendicular();
	obb.pos = Center();
	obb.r[0] = Height() * 0.5f;
	obb.r[1] = r;
	obb.r[2] = r;
	return obb;
}

float3 Capsule::RandomPointInside(LCG &rng) const
{
	assume(IsFinite());

	OBB obb = MinimalEnclosingOBB();
	for(int i = 0; i < 1000; ++i)
	{
		float3 pt = obb.RandomPointInside(rng);
		if (Contains(pt))
			return pt;
	}
	assume(false && "Warning: Capsule::RandomPointInside ran out of iterations to perform!");
	return Center(); // Just return some point that is known to be inside.
}

float3 Capsule::RandomPointOnSurface(LCG &rng) const
{
	return PointInside(rng.Float(), rng.Float(), 1.f);
}

void Capsule::Translate(const float3 &offset)
{
	l.a += offset;
	l.b += offset;
}

void Capsule::Scale(const float3 &centerPoint, float scaleFactor)
{
	float3x4 tm = float3x4::Scale(float3::FromScalar(scaleFactor), centerPoint);
	l.Transform(tm);
	r *= scaleFactor;
}

void Capsule::Transform(const float3x3 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal());
	l.Transform(transform);
	r *= transform.Col(0).Length(); // Scale the radius.
}

void Capsule::Transform(const float3x4 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal());
	l.Transform(transform);
	r *= transform.Col(0).Length(); // Scale the radius.
}

void Capsule::Transform(const float4x4 &transform)
{
	assume(transform.HasUniformScale());
	assume(transform.IsColOrthogonal3());
	l.Transform(transform);
	r *= transform.Col3(0).Length(); // Scale the radius.
}

void Capsule::Transform(const Quat &transform)
{
	l.Transform(transform);
}

float3 Capsule::ClosestPoint(const float3 &targetPoint) const
{
	float3 ptOnLine = l.ClosestPoint(targetPoint);
	if (ptOnLine.DistanceSq(targetPoint) <= r*r)
		return targetPoint;
	else
		return ptOnLine + (targetPoint - ptOnLine).ScaledToLength(r);
}

float Capsule::Distance(const float3 &point) const
{
	return Max(0.f, l.Distance(point) - r);
}

float Capsule::Distance(const Capsule &capsule) const
{
	return Max(0.f, l.Distance(capsule.l) - r - capsule.r);
}

float Capsule::Distance(const Plane &plane) const
{
	return plane.Distance(*this);
}

float Capsule::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Capsule::Distance(const Ray &ray) const
{
	return ray.Distance(*this);
}

float Capsule::Distance(const Line &line) const
{
	return line.Distance(*this);
}

float Capsule::Distance(const LineSegment &lineSegment) const
{
	return lineSegment.Distance(*this);
}

bool Capsule::Contains(const float3 &point) const
{
	return l.Distance(point) <= r;
}

bool Capsule::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool Capsule::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool Capsule::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool Capsule::Contains(const AABB &aabb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool Capsule::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool Capsule::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool Capsule::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

bool Capsule::Intersects(const Ray &ray) const
{
	return l.Distance(ray) <= r;
}

bool Capsule::Intersects(const Line &line) const
{
	return l.Distance(line) <= r;
}

bool Capsule::Intersects(const LineSegment &lineSegment) const
{
	return l.Distance(lineSegment) <= r;
}

bool Capsule::Intersects(const Plane &plane) const
{
	return l.Distance(plane) <= r;
}

bool Capsule::Intersects(const AABB &aabb) const
{
	return Intersects(aabb.ToPolyhedron());
}

bool Capsule::Intersects(const OBB &obb) const
{
	return Intersects(obb.ToPolyhedron());
}

/// [groupSyntax]
bool Capsule::Intersects(const Sphere &sphere) const
{
	///@todo Optimize to avoid square roots.
	return l.Distance(sphere.pos) <= r + sphere.r;
}

/// [groupSyntax]
bool Capsule::Intersects(const Capsule &capsule) const
{
	///@todo Optimize to avoid square roots.
	return l.Distance(capsule.l) <= r + capsule.r;
}

bool Capsule::Intersects(const Triangle &triangle) const
{
	float3 thisPoint;
	float3 trianglePoint = triangle.ClosestPoint(l, &thisPoint);
	return thisPoint.DistanceSq(trianglePoint) <= r*r;
}

bool Capsule::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Capsule::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Capsule::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Capsule::ToString() const
{
	char str[256];
	sprintf(str, "Capsule(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f), r:%.2f)", l.a.x, l.a.y, l.a.z, l.b.x, l.b.y, l.b.z, r);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Capsule &capsule)
{
	o << capsule.ToString();
	return o;
}

#endif

Capsule operator *(const float3x3 &transform, const Capsule &capsule)
{
	Capsule c(capsule);
	c.Transform(transform);
	return c;
}

Capsule operator *(const float3x4 &transform, const Capsule &capsule)
{
	Capsule c(capsule);
	c.Transform(transform);
	return c;
}

Capsule operator *(const float4x4 &transform, const Capsule &capsule)
{
	Capsule c(capsule);
	c.Transform(transform);
	return c;
}

Capsule operator *(const Quat &transform, const Capsule &capsule)
{
	Capsule c(capsule);
	c.Transform(transform);
	return c;
}

MATH_END_NAMESPACE
