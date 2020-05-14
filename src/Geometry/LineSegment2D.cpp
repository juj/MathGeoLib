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

/** @file LineSegment2D.cpp
	@author Jukka Jylänki
	@brief Implementation for the LineSegment2D geometry object. */
#include "LineSegment2D.h"
#include "../Math/MathFunc.h"
#include "AABB2D.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "OBB2D.h"
#include "../Math/Quat.h"
#include "../Math/Swap.h"
#if 0
#include "Ray2D.h"
#include "Line2D.h"
#include "Polygon2D.h"
#include "Sphere2D.h"
#include "Capsule2D.h"
#endif
#include "Triangle2D.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

int IntervalIntersection(float u0, float u1, float v0, float v1, float &s, float &t)
{
	if (u1 < v0 || u0 > v1)
	{
		s = t = FLOAT_NAN;
		return 0;
	}

	if (u1 > v0)
	{
		if (u0 < v1)
		{
			s = Max(u0, v0);
			t = Min(u1, v1);
			return 2;
		}
		else // u0 == v1
		{
			s = t = u0;
			return 1;
		}
	}
	else
	{
		// u1 == v0
		s = t = u1;
		return 1;
	}
}

/// 2D line segment-line segment intersection from Geometric Tools for Computer Graphics, pp. 807-813, by Schneider and Eberly.
bool LineSegment2DLineSegment2DIntersect(const float2 &p0, const float2 &dir0, const float2 &p1, const float2 &dir1, float &s, float &t)
{
	float2 e = p1 - p0;
	float cross = dir0.PerpDot(dir1);
	float sqrCross = cross*cross;
	float sqrLen0 = dir0.LengthSq();
	float sqrLen1 = dir1.LengthSq();
	const float sqrEpsilon = 1e-8f;
	if (sqrCross > sqrEpsilon * sqrLen0 * sqrLen1)
	{
		// The lines are not parallel
		s = e.PerpDot(dir1) / cross;
		t = e.PerpDot(dir0) / cross;
		// The intersection point is p0 + s * dir0;
		return s >= 0.f && s <= 1.f && t >= 0.f && t <= 1.f;
	}

	// The line segments are parallel
	float sqrLenE = e.LengthSq();
	cross = e.PerpDot(dir0);
	sqrCross = cross*cross;
	if (sqrCross > sqrEpsilon * sqrLen0 * sqrLenE)
	{
		// The lines are at a positive distance, no intersection.
		s = t = FLOAT_NAN;
		return false;
	}

	// The line segments are along the same line. Do an overlap test.
	float s0 = Dot(dir0, e) / sqrLen0;
	float s1 = s0 + Dot(dir0, dir1) / sqrLen0;
	float smin = Min(s0, s1);
	float smax = Max(s0, s1);

	int nIntersections = IntervalIntersection(0.f, 1.f, smin, smax, s, t);
	return nIntersections > 0.f;
}

bool RangesOverlap(float start1, float end1, float start2, float end2)
{
	return end1 >= start2 && end2 >= start1;
}

LineSegment2D::LineSegment2D(const vec2d &a_, const vec2d &b_)
:a(a_), b(b_)
{
}
#if 0
LineSegment2D::LineSegment2D(const Ray2D &ray, float d)
:a(ray.pos), b(ray.GetPoint(d))
{
}

LineSegment2D::LineSegment2D(const Line2D &line, float d)
:a(line.pos), b(line.GetPoint(d))
{
}
#endif
vec2d LineSegment2D::GetPoint(float d) const
{
	return (1.f - d) * a + d * b;
}

vec2d LineSegment2D::CenterPoint() const
{
	return (a + b) * 0.5f;
}

void LineSegment2D::Reverse()
{
	Swap(a, b);
}

vec2d LineSegment2D::Dir() const
{
	return (b - a).Normalized();
}

vec2d LineSegment2D::ExtremePoint(const vec2d &direction) const
{
	return Dot(direction, b-a) >= 0.f ? b : a;
}

vec2d LineSegment2D::ExtremePoint(const vec2d &direction, float &projectionDistance) const
{
	vec2d extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void LineSegment2D::Translate(const vec2d &offset)
{
	a += offset;
	b += offset;
}
#if 0
void LineSegment2D::Transform(const float3x3 &transform)
{
	a = transform * a;
	b = transform * b;
}
#endif
void LineSegment2D::Transform(const float3x4 &transform)
{
	a = transform.MulPos(a);
	b = transform.MulPos(b);
}
#if 0
void LineSegment2D::Transform(const float4x4 &transform)
{
	a = transform.MulPos(a);
	b = transform.MulPos(b);
}

void LineSegment2D::Transform(const Quat &transform)
{
	a = transform * a;
	b = transform * b;
}
#endif
float LineSegment2D::Length() const
{
	return a.Distance(b);
}

float LineSegment2D::LengthSq() const
{
	return a.DistanceSq(b);
}

bool LineSegment2D::IsFinite() const
{
	return a.IsFinite() && b.IsFinite();
}

bool LineSegment2D::Contains(const vec2d &point, float distanceThreshold) const
{
	return ClosestPoint(point).DistanceSq(point) <= distanceThreshold;
}

bool LineSegment2D::Contains(const LineSegment2D &rhs, float distanceThreshold) const
{
	return Contains(rhs.a, distanceThreshold) && Contains(rhs.b, distanceThreshold);
}

bool LineSegment2D::Equals(const LineSegment2D &rhs, float e) const
{
	return (a.Equals(rhs.a, e) && b.Equals(rhs.b, e)) || (a.Equals(rhs.b, e) && b.Equals(rhs.a, e));
}

vec2d LineSegment2D::ClosestPoint(const vec2d &point, float &d) const
{
	vec2d dir = b - a;
	d = Clamp01(Dot(point - a, dir) / dir.LengthSq());
	return a + d * dir;
}
#if 0
vec2d LineSegment2D::ClosestPoint(const Ray2D &other, float &d, float &d2) const
{
	other.ClosestPoint(*this, d2, d);
	return GetPoint(d);
}

vec2d LineSegment2D::ClosestPoint(const Line2D &other, float &d, float &d2) const
{
	Line2D::ClosestPointLineLine(other.pos, other.dir, a, b - a, d2, d);
	if (d < 0.f)
	{
		d = 0.f;
		other.ClosestPoint(a, d2);
		return a;
	}
	else if (d > 1.f)
	{
		d = 1.f;
		other.ClosestPoint(b, d2);
		return b;
	}
	else
		return GetPoint(d);
}
#endif

/// Computes the closest point pair on two lines.
/** The first line is specified by two points start0 and end0. The second line is specified by
	two points start1 and end1.
	The implementation of this function follows http://paulbourke.net/geometry/lineline3d/ .
	@param v0 The starting point of the first line.
	@param v10 The direction vector of the first line. This can be unnormalized.
	@param v2 The starting point of the second line.
	@param v32 The direction vector of the second line. This can be unnormalized.
	@param d [out] Receives the normalized distance of the closest point along the first line.
	@param d2 [out] Receives the normalized distance of the closest point along the second line.
	@return Returns the closest point on line start0<->end0 to the second line.
	@note This is a low-level utility function. You probably want to use ClosestPoint() or Distance() instead.
	@see ClosestPoint(), Distance(). */
void Line2DClosestPointLineLine(const vec2d &v0, const vec2d &v10, const vec2d &v2, const vec2d &v32, float &d, float &d2) // TODO: Move to Line2D when that exists
{
	assume(!v10.IsZero());
	assume(!v32.IsZero());
	vec2d v02 = v0 - v2;
	float d0232 = v02.Dot(v32);
	float d3210 = v32.Dot(v10);
	float d3232 = v32.Dot(v32);
	assume(d3232 != 0.f); // Don't call with a zero direction vector.
	float d0210 = v02.Dot(v10);
	float d1010 = v10.Dot(v10);
	float denom = d1010*d3232 - d3210*d3210;
	if (denom != 0.f)
		d = (d0232*d3210 - d0210*d3232) / denom;
	else
		d = 0.f;
	d2 = (d0232 + d * d3210) / d3232;
}

vec2d LineSegment2D::ClosestPoint(const LineSegment2D &other, float &d, float &d2) const
{
	vec2d dir = b - a;
	Line2DClosestPointLineLine(a, b - a, other.a, other.b - other.a, d, d2);
	if (d >= 0.f && d <= 1.f && d2 >= 0.f && d2 <= 1.f)
		return a + d * dir;
	else if (d >= 0.f && d <= 1.f) // Only d2 is out of bounds.
	{
		vec2d p;
		if (d2 < 0.f)
		{
			d2 = 0.f;
			p = other.a;
		}
		else
		{
			d2 = 1.f;
			p = other.b;
		}
		return ClosestPoint(p, d);
	}
	else if (d2 >= 0.f && d2 <= 1.f) // Only d is out of bounds.
	{
		vec2d p;
		if (d < 0.f)
		{
			d = 0.f;
			p = a;
		}
		else
		{
			d = 1.f;
			p = b;
		}

		other.ClosestPoint(p, d2);
		return p;
	}
	else // Both u and u2 are out of bounds.
	{
		vec2d p;
		if (d < 0.f)
		{
			p = a;
			d = 0.f;
		}
		else
		{
			p = b;
			d = 1.f;
		}

		vec2d p2;
		if (d2 < 0.f)
		{
			p2 = other.a;
			d2 = 0.f;
		}
		else
		{
			p2 = other.b;
			d2 = 1.f;
		}

		float T, T2;
		vec2d closestPoint = ClosestPoint(p2, T);
		vec2d closestPoint2 = other.ClosestPoint(p, T2);

		if (closestPoint.DistanceSq(p2) <= closestPoint2.DistanceSq(p))
		{
			d = T;
			return closestPoint;
		}
		else
		{
			d2 = T2;
			return p;
		}
	}
}

float LineSegment2D::Distance(const vec2d &point, float &d) const
{
	/// See Christer Ericson's Real-Time Collision Detection, p.130.
	vec2d closestPoint = ClosestPoint(point, d);
	return closestPoint.Distance(point);
}

float LineSegment2D::DistanceSq(const vec2d &point) const
{
	float d;
	/// See Christer Ericson's Real-Time Collision Detection, p.130.
	vec2d closestPoint = ClosestPoint(point, d);
	return closestPoint.DistanceSq(point);
}
#if 0
float LineSegment2D::Distance(const Ray2D &other, float &d, float &d2) const
{
	ClosestPoint(other, d, d2);
	return GetPoint(d).Distance(other.GetPoint(d2));
}

float LineSegment2D::Distance(const Line2D &other, float &d, float &d2) const
{
	vec2d closestPoint2 = other.ClosestPoint(*this, d, d2);
	vec2d closestPoint = GetPoint(d2);
	return closestPoint.Distance(closestPoint2);
}
#endif
float LineSegment2D::Distance(const LineSegment2D &other, float &d, float &d2) const
{
	ClosestPoint(other, d, d2);
	return GetPoint(d).Distance(other.GetPoint(d2));
}

float LineSegment2D::DistanceSq(const LineSegment2D &other) const
{
	float d, d2;
	ClosestPoint(other, d, d2);
	return GetPoint(d).DistanceSq(other.GetPoint(d2));
}
#if 0
float LineSegment2D::Distance(const Sphere2D &other) const
{
	return Max(0.f, Distance(other.pos) - other.r);
}

float LineSegment2D::Distance(const Capsule2D &other) const
{
	return Max(0.f, Distance(other.l) - other.r);
}

bool LineSegment2D::Intersects(const Capsule2D &capsule) const
{
	return capsule.Intersects(*this);
}

bool LineSegment2D::Intersects(const Triangle2D &triangle, float *d, vec2d *intersectionPoint) const
{
	return triangle.Intersects(*this, d, intersectionPoint);
}

bool LineSegment2D::Intersects(const Sphere2D &s, vec2d *intersectionPoint, vec2d *intersectionNormal, float *d) const
{
	return s.Intersects(*this, intersectionPoint, intersectionNormal, d) > 0;
}

bool LineSegment2D::Intersects(const AABB2D &aabb) const
{
	return aabb.Intersects(*this);
}

bool LineSegment2D::Intersects(const AABB2D &aabb, float &dNear, float &dFar) const
{
	return aabb.Intersects(*this, dNear, dFar);
}

bool LineSegment2D::Intersects(const OBB2D &obb) const
{
	return obb.Intersects(*this);
}

bool LineSegment2D::Intersects(const OBB2D &obb, float &dNear, float &dFar) const
{
	return obb.Intersects(*this, dNear, dFar);
}
#endif
bool LineSegment2D::Intersects(const LineSegment2D &lineSegment, float epsilon) const
{
	return Distance(lineSegment) <= epsilon;
}
#if 0
bool LineSegment2D::Intersects(const Polygon2D &polygon) const
{
	return polygon.Intersects(*this);
}

Ray2D LineSegment2D::ToRay() const
{
	return Ray2D(a, Dir());
}

Line2D LineSegment2D::ToLine() const
{
	return Line2D(a, Dir());
}
#endif
void LineSegment2D::ProjectToAxis(const vec2d &direction, float &outMin, float &outMax) const
{
	outMin = Dot(direction, a);
	outMax = Dot(direction, b);
	if (outMax < outMin)
		Swap(outMin, outMax);
}
#if 0
LineSegment2D operator *(const float3x3 &transform, const LineSegment2D &l)
{
	return LineSegment2D(transform * l.a, transform * l.b);
}
#endif
LineSegment2D operator *(const float3x4 &transform, const LineSegment2D &l)
{
	return LineSegment2D(transform.MulPos(l.a), transform.MulPos(l.b));
}
#if 0
LineSegment2D operator *(const float4x4 &transform, const LineSegment2D &l)
{
	return LineSegment2D(transform.MulPos(l.a), transform.MulPos(l.b));
}

LineSegment2D operator *(const Quat &transform, const LineSegment2D &l)
{
	return LineSegment2D(transform * l.a, transform * l.b);
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT LineSegment2D::ToString() const
{
	char str[256];
	sprintf(str, "LineSegment2D(a:(%.2f, %.2f) b:(%.2f, %.2f))",
		a.x, a.y, b.x, b.y);
	return str;
}

StringT LineSegment2D::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(a.x, str); *s = ','; ++s;
	s = SerializeFloat(a.y, s); *s = ','; ++s;
	s = SerializeFloat(b.x, s); *s = ','; ++s;
	s = SerializeFloat(b.y, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT LineSegment2D::SerializeToCodeString() const
{
	return "LineSegment2D(" + a.SerializeToCodeString() + "," + b.SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const LineSegment2D &lineSegment)
{
	o << lineSegment.ToString();
	return o;
}

#endif

LineSegment2D LineSegment2D::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return LineSegment2D(vec2d::nan, vec2d::nan);
	LineSegment2D l;
	MATH_SKIP_WORD(str, "LineSegment2D(");
	MATH_SKIP_WORD(str, "a:(");
	l.a = POINT_TO_FLOAT4(PointVecFromString(str, &str)).ToVec2D();
	MATH_SKIP_WORD(str, " b:(");
	l.b = POINT_TO_FLOAT4(PointVecFromString(str, &str)).ToVec2D();
	if (outEndStr)
		*outEndStr = str;
	return l;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP

void LineSegment2D::ToLineList(VertexBuffer &vb) const
{
	int startIndex = vb.AppendVertices(2);
	vb.Set(startIndex, VDPosition, float4(a.x, a.y, 0.f, 1.f));
	vb.Set(startIndex+1, VDPosition, float4(b.x, b.y, 0.f, 1.f));
}

#endif

MATH_END_NAMESPACE
