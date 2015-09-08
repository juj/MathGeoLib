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
#include "../Algorithm/GJK.h"
#include "../Math/assume.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

MATH_BEGIN_NAMESPACE

Capsule::Capsule(const LineSegment &endPoints, float radius)
:l(endPoints), r(radius)
{
}

Capsule::Capsule(const vec &bottomPoint, const vec &topPoint, float radius)
:l(bottomPoint, topPoint), r(radius)
{
}

void Capsule::SetFrom(const Sphere &s)
{
	l = LineSegment(s.pos, s.pos);
	r = s.r;
}

void Capsule::SetDegenerate()
{
	r = -1.f;
}

bool Capsule::IsDegenerate() const
{
	return r <= 0.f;
}

float Capsule::LineLength() const
{
	return l.Length();
}

float Capsule::Diameter() const
{
	return 2.f * r;
}

vec Capsule::Bottom() const
{
	return l.a - UpDirection() * r;
}

vec Capsule::Center() const
{
	return l.CenterPoint();
}

vec Capsule::ExtremePoint(const vec &direction) const
{
	float len = direction.Length();
	assume(len > 0.f);
	return (Dot(direction, l.b - l.a) >= 0.f ? l.b : l.a) + direction * (r / len);
}

vec Capsule::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void Capsule::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
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

vec Capsule::Top() const
{
	return l.b + UpDirection() * r;
}

vec Capsule::UpDirection() const
{
	vec d = l.b - l.a;
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
	vec up = UpDirection();
	vec centerPos = Bottom() + up * yPos;
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

vec Capsule::PointInside(float height, float angle, float dist) const
{
	Circle c = CrossSection(height);
	return c.GetPoint(angle*2.f*pi, dist);
}

vec Capsule::UniformPointPerhapsInside(float height, float x, float y) const
{
	return MinimalEnclosingOBB().PointInside(height, x, y);
}

Sphere Capsule::SphereA() const
{
	return Sphere(l.a, r);
}

Sphere Capsule::SphereB() const
{
	return Sphere(l.b, r);
}

AABB Capsule::MinimalEnclosingAABB() const
{
	vec d = DIR_VEC_SCALAR(r);
	AABB aabb(Min(l.a, l.b) - d, Max(l.a, l.b) + d);
	return aabb;
}

OBB Capsule::MinimalEnclosingOBB() const
{
	OBB obb;
	obb.axis[0] = UpDirection();
	obb.axis[0].PerpendicularBasis(obb.axis[1], obb.axis[2]);
	obb.pos = Center();
	obb.r[0] = Height() * 0.5f;
	obb.r[1] = r;
	obb.r[2] = r;
	return obb;
}

vec Capsule::RandomPointInside(LCG &rng) const
{
	assume(IsFinite());

	OBB obb = MinimalEnclosingOBB();
	for(int i = 0; i < 1000; ++i)
	{
		vec pt = obb.RandomPointInside(rng);
		if (Contains(pt))
			return pt;
	}
	assume(false && "Warning: Capsule::RandomPointInside ran out of iterations to perform!");
	return Center(); // Just return some point that is known to be inside.
}

vec Capsule::RandomPointOnSurface(LCG &rng) const
{
	float f1 = rng.Float();
	float f2 = rng.Float();
	return PointInside(f1, f2, 1.f);
}

void Capsule::Translate(const vec &offset)
{
	l.a += offset;
	l.b += offset;
}

void Capsule::Scale(const vec &centerPoint, float scaleFactor)
{
	float3x4 tm = float3x4::Scale(DIR_VEC_SCALAR(scaleFactor), centerPoint);
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

vec Capsule::ClosestPoint(const vec &targetPoint) const
{
	vec ptOnLine = l.ClosestPoint(targetPoint);
	if (ptOnLine.DistanceSq(targetPoint) <= r*r)
		return targetPoint;
	else
		return ptOnLine + (targetPoint - ptOnLine).ScaledToLength(r);
}

float Capsule::Distance(const vec &point) const
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

bool Capsule::Contains(const vec &point) const
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
	return GJKIntersect(*this, aabb);
}

bool Capsule::Intersects(const OBB &obb) const
{
	return GJKIntersect(*this, obb);
}

/// [groupSyntax]
bool Capsule::Intersects(const Sphere &sphere) const
{
	float R = r + sphere.r;
	return l.DistanceSq(sphere.pos) <= R*R;
}

/// [groupSyntax]
bool Capsule::Intersects(const Capsule &capsule) const
{
	float R = r + capsule.r;
	return l.DistanceSq(capsule.l) <= R*R;
}

bool Capsule::Intersects(const Triangle &triangle) const
{
	vec thisPoint;
	vec trianglePoint = triangle.ClosestPoint(l, &thisPoint);
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

std::string Capsule::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(l.a.x, str); *s = ','; ++s;
	s = SerializeFloat(l.a.y, s); *s = ','; ++s;
	s = SerializeFloat(l.a.z, s); *s = ','; ++s;
	s = SerializeFloat(l.b.x, s); *s = ','; ++s;
	s = SerializeFloat(l.b.y, s); *s = ','; ++s;
	s = SerializeFloat(l.b.z, s); *s = ','; ++s;
	s = SerializeFloat(r, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

std::string Capsule::SerializeToCodeString() const
{
	char str[256];
	sprintf(str, "%.9g", r);
	return "Capsule(" + l.SerializeToCodeString() + "," + str + ")";
}

std::ostream &operator <<(std::ostream &o, const Capsule &capsule)
{
	o << capsule.ToString();
	return o;
}

#endif

Capsule Capsule::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return Capsule(vec::nan, vec::nan, FLOAT_NAN);
	Capsule c;
	MATH_SKIP_WORD(str, "Capsule(");
	MATH_SKIP_WORD(str, "a:(");
	MATH_SKIP_WORD(str, "LineSegment(");
	c.l.a = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " b:");
	c.l.b = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, ")");
	MATH_SKIP_WORD(str, ",");
	MATH_SKIP_WORD(str, " r:");
	c.r = DeserializeFloat(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return c;
}

bool Capsule::BitEquals(const Capsule &other) const
{
	return l.BitEquals(other.l) && ReinterpretAsU32(r) == ReinterpretAsU32(other.r);
}

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
