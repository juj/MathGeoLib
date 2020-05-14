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

/** @file Triangle2D.cpp
	@author Jukka Jylänki
	@brief Implementation for the Triangle2D geometry object. */
#include "Triangle2D.h"
#include "../Math/MathFunc.h"
#include "../Math/Swap.h"
#include "../Math/float2.h"
#include "../Math/float3.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "LineSegment2D.h"
#if 0
#include "Capsule2D.h"
#include "Polygon2D.h"
#include "Line2D.h"
#include "Ray2D.h"
#include "Sphere2D.h"
#endif
#include "AABB2D.h"
#include "OBB2D.h"
#include "../Algorithm/Random/LCG.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
#include "../Math/float4_sse.h"
#include "../Math/float4_neon.h"
#endif

MATH_BEGIN_NAMESPACE

Triangle2D::Triangle2D(const vec2d &a_, const vec2d &b_, const vec2d &c_)
:a(a_), b(b_), c(c_)
{
}

void Triangle2D::Translate(const vec2d &offset)
{
	a += offset;
	b += offset;
	c += offset;
}

void Triangle2D::Transform(const float3x3 &transform)
{
	a = Mul2D(transform, a);
	b = Mul2D(transform, b);
	c = Mul2D(transform, c);
}

void Triangle2D::Transform(const float3x4 &transform)
{
	a = MulPos2D(transform, a);
	b = MulPos2D(transform, b);
	c = MulPos2D(transform, c);
}

void Triangle2D::Transform(const float4x4 &transform)
{
	a = MulPos2D(transform, a);
	b = MulPos2D(transform, b);
	c = MulPos2D(transform, c);
}

/// Implementation from Christer Ericson's Real-Time Collision Detection, pp. 51-52.
inline float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
{
	return (x1-x2)*(y2-y3) - (x2-x3)*(y1-y2);
}

float3 Triangle2D::BarycentricUVW(const vec2d &point) const
{
	// Implementation from Christer Ericson's Real-Time Collision Detection, pp. 51-52, adapted for 2D.

	// Nominators for u and v ratios.
	float nu, nv;

	nu = TriArea2D(point.x, point.y, b.x, b.y, c.x, c.y);
	nv = TriArea2D(point.x, point.y, c.x, c.y, a.x, a.y);
	float u = nu;
	float v = nv;
	float w = 1.f - u - v;
	return float3(u,v,w);
}

float2 Triangle2D::BarycentricUV(const vec2d &point) const
{
	float3 uvw = BarycentricUVW(point);
	return float2(uvw.y, uvw.z);
}

bool Triangle2D::BarycentricInsideTriangle(const float3 &barycentric)
{
	return barycentric.x >= 0.f && barycentric.y >= 0.f && barycentric.z >= 0.f &&
		EqualAbs(barycentric.x + barycentric.y + barycentric.z, 1.f);
}

vec2d Triangle2D::Point(float u, float v) const
{
	// In case the triangle is far away from the origin but is small in size, the elements of 'a' will have large magnitudes,
	// and the elements of (b-a) and (c-a) will be much smaller quantities. Therefore be extra careful with the
	// parentheses and first sum the small floats together before adding it to the large one.
	return a + ((b-a) * u + (c-a) * v);
}

vec2d Triangle2D::Point(float u, float v, float w) const
{
	return u * a + v * b + w * c;
}

vec2d Triangle2D::Point(const float3 &uvw) const
{
	return Point(uvw.x, uvw.y, uvw.z);
}

vec2d Triangle2D::Point(const float2 &uv) const
{
	return Point(uv.x, uv.y);
}

vec2d Triangle2D::Centroid() const
{
	return (a + b + c) * (1.f/3.f);
}
#if 0
float Triangle2D::Area() const
{
	return 0.5f * Cross(b-a, c-a).Length();
}
#endif
float Triangle2D::Perimeter() const
{
	return a.Distance(b) + b.Distance(c) + c.Distance(a);
}

LineSegment2D Triangle2D::Edge(int i) const
{
	assume(0 <= i);
	assume(i <= 2);
	if (i == 0)
		return LineSegment2D(a, b);
	else if (i == 1)
		return LineSegment2D(b, c);
	else if (i == 2)
		return LineSegment2D(c, a);
	else
		return LineSegment2D(vec2d::nan, vec2d::nan);
}

vec2d Triangle2D::Vertex(int i) const
{
	assume(0 <= i);
	assume(i <= 2);
	if (i == 0)
		return a;
	else if (i == 1)
		return b;
	else if (i == 2)
		return c;
	else
		return vec2d::nan;
}

vec2d Triangle2D::ExtremePoint(const vec2d &direction) const
{
	vec2d mostExtreme = vec2d::nan;
	float mostExtremeDist = -FLT_MAX;
	for(int i = 0; i < 3; ++i)
	{
		vec2d pt = Vertex(i);
		float d = Dot(direction, pt);
		if (d > mostExtremeDist)
		{
			mostExtremeDist = d;
			mostExtreme = pt;
		}
	}
	return mostExtreme;
}

vec2d Triangle2D::ExtremePoint(const vec2d &direction, float &projectionDistance) const
{
	vec2d extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}
#if 0
Polygon2D Triangle2D::ToPolygon() const
{
	Polygon2D p;
	p.p.push_back(a);
	p.p.push_back(b);
	p.p.push_back(c);
	return p;
}
#endif
AABB2D Triangle2D::BoundingAABB() const
{
	AABB2D aabb;
	aabb.minPoint = Min(a, b, c);
	aabb.maxPoint = Max(a, b, c);
	return aabb;
}

float Triangle2D::Area2D(const float2 &p1, const float2 &p2, const float2 &p3)
{
	return (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
}
#if 0
float Triangle2D::SignedArea(const vec2d &pt, const vec2d &a, const vec2d &b, const vec2d &c)
{
	return Dot(Cross(b-pt, c-pt), Cross(b-a, c-a).Normalized());
}
#endif
bool Triangle2D::IsFinite() const
{
	return a.IsFinite() && b.IsFinite() && c.IsFinite();
}

bool Triangle2D::IsDegenerate(float epsilon) const
{
	return IsDegenerate(a, b, c, epsilon);
}

bool Triangle2D::IsDegenerate(const vec2d &a, const vec2d &b, const vec2d &c, float epsilon)
{
	return a.Equals(b, epsilon) || a.Equals(c, epsilon) || b.Equals(c, epsilon);
}
#if 0
bool Triangle2D::Contains(const vec2d &point, float triangleThicknessSq) const
{
	vec2d normal = (b-a).Cross(c-a);
	float lenSq = normal.LengthSq();
	float d = normal.Dot(b - point);
	if (d*d > triangleThicknessSq * lenSq)
		return false; ///@todo The plane-point distance test is omitted in Real-Time Collision Detection. p. 25. A bug in the book?

	float3 br = BarycentricUVW(point);
	return br.x >= -1e-3f && br.y >= -1e-3f && br.z >= -1e-3f; // Allow for a small epsilon to properly account for points very near the edges of the triangle.
}

bool Triangle2D::Contains(const LineSegment2D &lineSegment, float triangleThickness) const
{
	return Contains(lineSegment.a, triangleThickness) && Contains(lineSegment.b, triangleThickness);
}

bool Triangle2D::Contains(const Triangle2D &triangle, float triangleThickness) const
{
	return Contains(triangle.a, triangleThickness) && Contains(triangle.b, triangleThickness)
	  && Contains(triangle.c, triangleThickness);
}
#endif
/*
bool Triangle2D::Contains(const Polygon2D &polygon, float triangleThickness) const
{
	if (polygon.points.size() == 0)
		return false;
	for(int i = 0; i < polygon.points.size(); ++i)
		if (!Contains(polygon.points[i], triangleThickness))
			return false;
	return true;
}
*/
float Triangle2D::Distance(const vec2d &point) const
{
	return ClosestPoint(point).Distance(point);
}

float Triangle2D::DistanceSq(const vec2d &point) const
{
	return ClosestPoint(point).DistanceSq(point);
}
#if 0
float Triangle2D::Distance(const Sphere2D &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Triangle2D::Distance(const Capsule2D &capsule) const
{
	vec2d otherPt;
	vec2d thisPt = ClosestPoint(capsule.l, &otherPt);
	return Max(0.f, thisPt.Distance(otherPt) - capsule.r);
}

/** Calculates the intersection between a line and a triangle. The facing is not accounted for, so
	rays are reported to intersect triangles that are both front and backfacing.
	According to "T. M&ouml;ller, B. Trumbore. Fast, Minimum Storage Ray2D/Triangle2D Intersection. 2005."
	http://jgt.akpeters.com/papers/MollerTrumbore97/
	@param linePos The starting point of the line.
	@param lineDir The direction vector of the line. This does not need to be normalized.
	@param v0 Vertex 0 of the triangle.
	@param v1 Vertex 1 of the triangle.
	@param v2 Vertex 2 of the triangle.
	@param u [out] The barycentric u coordinate is returned here if an intersection occurred.
	@param v [out] The barycentric v coordinate is returned here if an intersection occurred.
	@return The distance along the ray to the point of intersection, or +inf if no intersection occurred.
		If no intersection, then u and v and t will contain undefined values. If lineDir was not normalized, then to get the
		real world-space distance, one must scale the returned value with lineDir.Length(). If the returned value is negative,
		then the intersection occurs 'behind' the line starting position, with respect to the direction vector lineDir. */
float Triangle2D::IntersectLineTri(const vec2d &linePos, const vec2d &lineDir,
		const vec2d &v0, const vec2d &v1, const vec2d &v2,
		float &u, float &v)
{
	const float epsilon = 1e-4f;

	// Edge vectors
	vec2d vE1 = v1 - v0;
	vec2d vE2 = v2 - v0;

	// begin calculating determinant - also used to calculate U parameter
	vec2d vP = lineDir.Cross(vE2);

	// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
	const float det = vE1.Dot(vP);

	// If determinant is near zero, ray lies in plane of triangle.
	if (Abs(det) <= epsilon)
		return FLOAT_INF;
	const float recipDet = 1.f / det;

	// Calculate distance from v0 to ray origin
	vec2d vT = linePos - v0;

	// Output barycentric u
	u = vT.Dot(vP) * recipDet;
	if (u < -epsilon || u > 1.f + epsilon)
		return FLOAT_INF; // Barycentric U is outside the triangle - early out.

	// Prepare to test V parameter
	vec2d vQ = vT.Cross(vE1);

	// Output barycentric v
	v = lineDir.Dot(vQ) * recipDet;
	if (v < -epsilon || u + v > 1.f + epsilon) // Barycentric V or the combination of U and V are outside the triangle - no intersection.
		return FLOAT_INF;

	// Barycentric u and v are in limits, the ray intersects the triangle.
	
	// Output signed distance from ray to triangle.
	return vE2.Dot(vQ) * recipDet;
//	return (det < 0.f) ? IntersectBackface : IntersectFrontface;
}

/// [groupSyntax]
bool Triangle2D::Intersects(const LineSegment2D &l, float *d, vec2d *intersectionPoint) const
{
	/** The Triangle2D-Line2D/LineSegment2D/Ray2D intersection tests are based on M&ouml;ller-Trumbore method:
		"T. M&ouml;ller, B. Trumbore. Fast, Minimum Storage Ray2D/Triangle2D Intersection. 2005."
		http://jgt.akpeters.com/papers/MollerTrumbore97/. */
	float u, v;
	float t = IntersectLineTri(l.a, l.b - l.a, a, b, c, u, v);
	bool success = (t >= 0.0f && t <= 1.0f);
	if (!success)
		return false;
	if (d)
		*d = t;
	if (intersectionPoint)
		*intersectionPoint = l.GetPoint(t);
	return true;
}

bool Triangle2D::Intersects(const Line2D &l, float *d, vec2d *intersectionPoint) const
{
	float u, v;
	float t = IntersectLineTri(l.pos, l.dir, a, b, c, u, v);
	bool success = (t != FLOAT_INF);
	if (!success)
		return false;
	if (d)
		*d = t;
	if (intersectionPoint)
		*intersectionPoint = l.GetPoint(t);
	return success;
}

bool Triangle2D::Intersects(const Ray2D &r, float *d, vec2d *intersectionPoint) const
{
	float u, v;
	float t = IntersectLineTri(r.pos, r.dir, a, b, c, u, v);
	bool success = (t >= 0 && t != FLOAT_INF);
	if (!success)
		return false;
	if (d)
		*d = t;
	if (intersectionPoint)
		*intersectionPoint = r.GetPoint(t);
	return success;
}

/// [groupSyntax]
/** For Triangle2D-Sphere2D intersection code, see Christer Ericson's Real-Time Collision Detection, p.167. */
bool Triangle2D::Intersects(const Sphere2D &sphere, vec2d *closestPointOnTriangle) const
{
	vec2d pt = ClosestPoint(sphere.pos);

	if (closestPointOnTriangle)
		*closestPointOnTriangle = pt;

	return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool Triangle2D::Intersects(const Sphere2D &sphere) const
{
	return Intersects(sphere, 0);
}
#endif
#if 0
static void FindIntersectingLineSegments(const Triangle2D &t, float da, float db, float dc, LineSegment2D &l1, LineSegment2D &l2)
{
	if (da*db > 0.f)
	{
		l1 = LineSegment2D(t.a, t.c);
		l2 = LineSegment2D(t.b, t.c);
	}
	else if (db*dc > 0.f)
	{
		l1 = LineSegment2D(t.a, t.b);
		l2 = LineSegment2D(t.a, t.c);
	}
	else
	{
		l1 = LineSegment2D(t.a, t.b);
		l2 = LineSegment2D(t.b, t.c);
	}
}
#endif

#if 0
/// [groupSyntax]
bool Triangle2D::Intersects(const AABB2D &aabb) const
{
/** The AABB2D-Triangle2D test implementation is based on the pseudo-code in
	Christer Ericson's Real-Time Collision Detection, pp. 169-172. It is
	practically a standard SAT test. */
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Benchmark 'Triangle_intersects_AABB': Triangle2D::Intersects(AABB2D)
	//    Best: 8.065 nsecs / 21.664 ticks, Avg: 8.207 nsecs, Worst: 9.985 nsecs
	simd4f tMin = min_ps(a, min_ps(b, c));
	simd4f tMax = max_ps(a, max_ps(b, c));

	simd4f cmp = cmpge_ps(tMin, aabb.maxPoint.v);
	cmp = or_ps(cmp, cmple_ps(tMax, aabb.minPoint.v));
	// Mask off results from the W channel and test if all were zero.
	if (!a_and_b_allzero_ps(cmp, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU))) return false;

	simd4f center = mul_ps(add_ps(aabb.minPoint.v, aabb.maxPoint.v), set1_ps(0.5f));
	simd4f h = sub_ps(aabb.maxPoint.v, center);

	simd4f t0 = sub_ps(b, a);
	simd4f t1 = sub_ps(c, a);
	simd4f ac = sub_ps(a, center);
	simd4f n = cross_ps(t0, t1);
	if (_mm_ucomige_ss(abs_ps(dot4_ps(n, ac)), abs_ps(dot4_ps(h, abs_ps(n))))) return false;

	// {eX, eY, eZ} cross t1
	simd4f ac_wyxz  = zxyw_ps(ac);
	simd4f h_wyxz   = zxyw_ps(h);
	simd4f ac_wxzy  = yzxw_ps(ac);
	simd4f h_wxzy   = yzxw_ps(h);
	simd4f bc = sub_ps(b, center);
	simd4f bc_wyxz  = zxyw_ps(bc);
	simd4f at1 = abs_ps(t1);
	simd4f t1_wyxz  = zxyw_ps(t1);
	simd4f at1_wyxz = zxyw_ps(at1);
	simd4f bc_wxzy  = yzxw_ps(bc);
	simd4f t1_wxzy  = yzxw_ps(t1);
	simd4f at1_wxzy = yzxw_ps(at1);

	simd4f d1 = msub_ps(t1_wxzy, ac_wyxz, mul_ps(t1_wyxz, ac_wxzy));
	simd4f d2 = msub_ps(t1_wxzy, bc_wyxz, mul_ps(t1_wyxz, bc_wxzy));
	simd4f tc = mul_ps(add_ps(d1, d2), set1_ps(0.5f));
	simd4f r = abs_ps(madd_ps(h_wyxz, at1_wxzy, mul_ps(h_wxzy, at1_wyxz)));
	cmp = cmple_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	// Note: The three masks of W channel could be omitted if cmplt_ps was used instead of cmple_ps, but
	// want to be strict here and define that AABB2D and Triangle2D which touch at a vertex should not intersect.
	// Mask off results from the W channel and test if all were zero.
	if (!a_and_b_allzero_ps(cmp, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU))) return false;

	// {eX, eY, eZ} cross t2
	simd4f t2 = sub_ps(c, b);
	simd4f at2 = abs_ps(t2);
	simd4f t2_wyxz  = zxyw_ps(t2);
	simd4f at2_wyxz = zxyw_ps(at2);
	simd4f t2_wxzy  = yzxw_ps(t2);
	simd4f at2_wxzy = yzxw_ps(at2);

	d1 = msub_ps(t2_wxzy, ac_wyxz, mul_ps(t2_wyxz, ac_wxzy));
	d2 = msub_ps(t2_wxzy, bc_wyxz, mul_ps(t2_wyxz, bc_wxzy));
	tc = mul_ps(add_ps(d1, d2), set1_ps(0.5f));
	r = abs_ps(madd_ps(h_wyxz, at2_wxzy, mul_ps(h_wxzy, at2_wyxz)));
	cmp = cmple_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	// Mask off results from the W channel and test if all were zero.
	if (!a_and_b_allzero_ps(cmp, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU))) return false;

	// {eX, eY, eZ} cross t0
	simd4f cc = sub_ps(c, center);
	simd4f cc_wyxz  = zxyw_ps(cc);
	simd4f t0_wyxz  = zxyw_ps(t0);
	simd4f at0 = abs_ps(t0);
	simd4f at0_wyxz = zxyw_ps(at0);
	simd4f at0_wxzy = yzxw_ps(at0);
	simd4f t0_wxzy  = yzxw_ps(t0);
	simd4f cc_wxzy  = yzxw_ps(cc);

	d1 = msub_ps(t0_wxzy, ac_wyxz, mul_ps(t0_wyxz, ac_wxzy));
	d2 = msub_ps(t0_wxzy, cc_wyxz, mul_ps(t0_wyxz, cc_wxzy));
	tc = mul_ps(add_ps(d1, d2), set1_ps(0.5f));
	r = abs_ps(madd_ps(h_wyxz, at0_wxzy, mul_ps(h_wxzy, at0_wyxz)));
	cmp = cmple_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	// Mask off results from the W channel and test if all were zero.
	return a_and_b_allzero_ps(cmp, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU)) != 0;
#else
	// Benchmark 'Triangle_intersects_AABB': Triangle2D::Intersects(AABB2D)
	//    Best: 17.282 nsecs / 46.496 ticks, Avg: 17.804 nsecs, Worst: 18.434 nsecs
	vec2d tMin = a.Min(b.Min(c));
	vec2d tMax = a.Max(b.Max(c));

	if (tMin.x >= aabb.maxPoint.x || tMax.x <= aabb.minPoint.x
		|| tMin.y >= aabb.maxPoint.y || tMax.y <= aabb.minPoint.y
		|| tMin.z >= aabb.maxPoint.z || tMax.z <= aabb.minPoint.z)
		return false;

	vec2d center = (aabb.minPoint + aabb.maxPoint) * 0.5f;
	vec2d h = aabb.maxPoint - center;

	const vec2d t[3] = { b-a, c-a, c-b };

	vec2d ac = a-center;

	vec2d n = Cross(t[0], t[1]);
	float s = n.Dot(ac);
	float r = Abs(h.Dot(n.Abs()));
	if (Abs(s) >= r)
		return false;

	const vec2d at[3] = { Abs(t[0]), Abs(t[1]), Abs(t[2]) };

	vec2d bc = b-center;
	vec2d cc = c-center;

	// SAT test all cross-axes.
	// The following is a fully unrolled loop of this code, stored here for reference:
	/*
	float d1, d2, a1, a2;
	const vec2d e[3] = { DIR_VEC(1, 0, 0), DIR_VEC(0, 1, 0), DIR_VEC(0, 0, 1) };
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			vec2d axis = Cross(e[i], t[j]);
			ProjectToAxis(axis, d1, d2);
			aabb.ProjectToAxis(axis, a1, a2);
			if (d2 <= a1 || d1 >= a2) return false;
		}
	*/

	// eX <cross> t[0]
	float d1 = t[0].y * ac.z - t[0].z * ac.y;
	float d2 = t[0].y * cc.z - t[0].z * cc.y;
	float tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[0].z + h.z * at[0].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eX <cross> t[1]
	d1 = t[1].y * ac.z - t[1].z * ac.y;
	d2 = t[1].y * bc.z - t[1].z * bc.y;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[1].z + h.z * at[1].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eX <cross> t[2]
	d1 = t[2].y * ac.z - t[2].z * ac.y;
	d2 = t[2].y * bc.z - t[2].z * bc.y;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[2].z + h.z * at[2].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eY <cross> t[0]
	d1 = t[0].z * ac.x - t[0].x * ac.z;
	d2 = t[0].z * cc.x - t[0].x * cc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[0].z + h.z * at[0].x);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eY <cross> t[1]
	d1 = t[1].z * ac.x - t[1].x * ac.z;
	d2 = t[1].z * bc.x - t[1].x * bc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[1].z + h.z * at[1].x);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eY <cross> t[2]
	d1 = t[2].z * ac.x - t[2].x * ac.z;
	d2 = t[2].z * bc.x - t[2].x * bc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[2].z + h.z * at[2].x);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eZ <cross> t[0]
	d1 = t[0].x * ac.y - t[0].y * ac.x;
	d2 = t[0].x * cc.y - t[0].y * cc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[0].x + h.x * at[0].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eZ <cross> t[1]
	d1 = t[1].x * ac.y - t[1].y * ac.x;
	d2 = t[1].x * bc.y - t[1].y * bc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[1].x + h.x * at[1].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// eZ <cross> t[2]
	d1 = t[2].x * ac.y - t[2].y * ac.x;
	d2 = t[2].x * bc.y - t[2].y * bc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[2].x + h.x * at[2].y);
	if (r + Abs(tc - d1) <= Abs(tc))
		return false;

	// No separating axis exists, the AABB2D and triangle intersect.
	return true;
#endif
}
#endif
#if 0
bool Triangle2D::Intersects(const OBB2D &obb) const
{
	return obb.Intersects(*this);
}

bool Triangle2D::Intersects(const Polygon2D &polygon) const
{
	return polygon.Intersects(*this);
}

bool Triangle2D::Intersects(const Capsule2D &capsule) const
{
	return capsule.Intersects(*this);
}

#endif
void Triangle2D::ProjectToAxis(const vec2d &axis, float &dMin, float &dMax) const
{
	dMin = dMax = Dot(axis, a);
	float t = Dot(axis, b);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
	t = Dot(axis, c);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
}
#if 0
int Triangle2D::UniqueFaceNormals(vec2d *out) const
{
	out[0] = Cross(b-a, c-a);
	// If testing a pair of coplanar triangles for SAT intersection,
	// the common face normal will not be a separating axis, and neither will
	// the cross products of the pairwise edges, since those all coincide with the face normals.
	// Therefore to make SAT test work properly in that case, also report all the edge normals
	// as possible separation test directions, which fixes the coplanar case.
	// For more info, see Geometric Tools for Computer Graphics, 11.11.1, p. 612.
	out[1] = Cross(out[0], b-a);
	out[2] = Cross(out[0], c-a);
	out[3] = Cross(out[0], c-b);
	return 4;
}
#endif
int Triangle2D::UniqueEdgeDirections(vec2d *out) const
{
	out[0] = b-a;
	out[1] = c-a;
	out[2] = c-b;
	return 3;
}

/// [groupSyntax]
vec2d Triangle2D::ClosestPoint(const vec2d &p) const
{
	/** The code for Triangle2D-float3 test is from Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

	// Check if P is in vertex region outside A.
	vec2d ab = b - a;
	vec2d ac = c - a;
	vec2d ap = p - a;
	float d1 = Dot(ab, ap);
	float d2 = Dot(ac, ap);
	if (d1 <= 0.f && d2 <= 0.f)
		return a; // Barycentric coordinates are (1,0,0).

	// Check if P is in vertex region outside B.
	vec2d bp = p - b;
	float d3 = Dot(ab, bp);
	float d4 = Dot(ac, bp);
	if (d3 >= 0.f && d4 <= d3)
		return b; // Barycentric coordinates are (0,1,0).

	// Check if P is in edge region of AB, and if so, return the projection of P onto AB.
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
	{
		float v = d1 / (d1 - d3);
		return a + v * ab; // The barycentric coordinates are (1-v, v, 0).
	}

	// Check if P is in vertex region outside C.
	vec2d cp = p - c;
	float d5 = Dot(ab, cp);
	float d6 = Dot(ac, cp);
	if (d6 >= 0.f && d5 <= d6)
		return c; // The barycentric coordinates are (0,0,1).

	// Check if P is in edge region of AC, and if so, return the projection of P onto AC.
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
	{
		float w = d2 / (d2 - d6);
		return a + w * ac; // The barycentric coordinates are (1-w, 0, w).
	}

	// Check if P is in edge region of BC, and if so, return the projection of P onto BC.
	float va = d3*d6 - d5*d4;
	if (va <= 0.f && d4 - d3 >= 0.f && d5 - d6 >= 0.f)
	{
		float w = (d4 - d3) / (d4 - d3 + d5 - d6);
		return b + w * (c - b); // The barycentric coordinates are (0, 1-w, w).
	}

	// P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
	float denom = 1.f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + ab * v + ac * w;
}
#if 0
vec2d Triangle2D::ClosestPoint(const LineSegment2D &lineSegment, vec2d *otherPt) const
{
	///\todo Optimize.
	float u, v;
	float t = IntersectLineTri(lineSegment.a, lineSegment.b - lineSegment.a, a, b, c, u, v);
	bool intersects = (t >= 0.0f && t <= 1.0f);
	if (intersects)
	{
//		assume3(lineSegment.GetPoint(t).Equals(this->Point(u, v)), lineSegment.GetPoint(t).SerializeToCodeString(), this->Point(u, v).SerializeToCodeString(), lineSegment.GetPoint(t).Distance(this->Point(u, v)));
		if (otherPt)
			*otherPt = lineSegment.GetPoint(t);
		return this->Point(u, v);
	}

	float u1,v1,d1;
	vec2d pt1 = ClosestPointToTriangleEdge(lineSegment, &u1, &v1, &d1);

	vec2d pt2 = ClosestPoint(lineSegment.a);
	vec2d pt3 = ClosestPoint(lineSegment.b);
	
	float D1 = pt1.DistanceSq(lineSegment.GetPoint(d1));
	float D2 = pt2.DistanceSq(lineSegment.a);
	float D3 = pt3.DistanceSq(lineSegment.b);

	if (D1 <= D2 && D1 <= D3)
	{
		if (otherPt)
			*otherPt = lineSegment.GetPoint(d1);
		return pt1;
	}
	else if (D2 <= D3)
	{
		if (otherPt)
			*otherPt = lineSegment.a;
		return pt2;
	}
	else
	{
		if (otherPt)
			*otherPt = lineSegment.b;
		return pt3;
	}
}
#endif
#if 0
///\todo Enable this codepath. This if rom Geometric Tools for Computer Graphics,
/// but the algorithm in the book is broken and does not take into account the
/// direction of the gradient to determine the proper region of intersection.
/// Instead using a slower code path above.
/// [groupSyntax]
vec2d Triangle2D::ClosestPoint(const LineSegment2D &lineSegment, vec2d *otherPt) const
{
	vec2d e0 = b - a;
	vec2d e1 = c - a;
	vec2d v_p = a - lineSegment.a;
	vec2d d = lineSegment.b - lineSegment.a;

	// Q(u,v) = a + u*e0 + v*e1
	// L(t)   = ls.a + t*d
	// Minimize the distance |Q(u,v) - L(t)|^2 under u >= 0, v >= 0, u+v <= 1, t >= 0, t <= 1.

	float v_p_dot_e0 = Dot(v_p, e0);
	float v_p_dot_e1 = Dot(v_p, e1);
	float v_p_dot_d = Dot(v_p, d);

	float3x3 m;
	m[0][0] = Dot(e0, e0); m[0][1] = Dot(e0, e1); m[0][2] = -Dot(e0, d);
	m[1][0] =     m[0][1]; m[1][1] = Dot(e1, e1); m[1][2] = -Dot(e1, d);
	m[2][0] =     m[0][2]; m[2][1] =     m[1][2]; m[2][2] =  Dot(d, d);

	float3 B(-v_p_dot_e0, -v_p_dot_e1, v_p_dot_d);

	float3 uvt;
	bool success = m.SolveAxb(B, uvt);
	if (!success)
	{
		float t1, t2, t3;
		float s1, s2, s3;
		LineSegment2D e1 = Edge(0);
		LineSegment2D e2 = Edge(1);
		LineSegment2D e3 = Edge(2);
		float d1 = e1.Distance(lineSegment, &t1, &s1);
		float d2 = e2.Distance(lineSegment, &t2, &s2);
		float d3 = e3.Distance(lineSegment, &t3, &s3);
		if (d1 < d2 && d1 < d3)
		{
			if (otherPt)
				*otherPt = lineSegment.GetPoint(s1);
			return e1.GetPoint(t1);
		}
		else if (d2 < d3)
		{
			if (otherPt)
				*otherPt = lineSegment.GetPoint(s2);
			return e2.GetPoint(t2);
		}
		else
		{
			if (otherPt)
				*otherPt = lineSegment.GetPoint(s3);
			return e3.GetPoint(t3);
		}
	}

	if (uvt.x < 0.f)
	{
		// Clamp to u == 0 and solve again.
		float m_00 = m[2][2];
		float m_01 = -m[1][2];
		float m_10 = -m[2][1];
		float m_11 = m[1][1];
		float det = m_00 * m_11 - m_01 * m_10;
		float v = m_00 * B[1] + m_01 * B[2];
		float t = m_10 * B[1] + m_11 * B[2];
		v /= det;
		t /= det;
		if (v < 0.f)
		{
			// Clamp to v == 0 and solve for t.
			t = B[2] / m[2][2];
			t = Clamp01(t); // The solution for t must also be in the range [0,1].
			// The solution is (u,v,t)=(0,0,t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return a;
		}
		else if (v > 1.f)
		{
			// Clamp to v == 1 and solve for t.
			t = (B[2] - m[2][1]) / m[2][2];
			t = Clamp01(t);
			// The solution is (u,v,t)=(0,1,t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return c; // == a + v*e1
		}
		else if (t < 0.f)
		{
			// Clamp to t == 0 and solve for v.
			v = B[1] / m[1][1];
//			mathassert(EqualAbs(v, Clamp01(v)));
			v = Clamp01(v); // The solution for v must also be in the range [0,1]. TODO: Is this guaranteed by the above?
			// The solution is (u,v,t)=(0,v,0).
			if (otherPt)
				*otherPt = lineSegment.a;
			return a + v * e1;
		}
		else if (t > 1.f)
		{
			// Clamp to t == 1 and solve for v.
			v = (B[1] - m[1][2]) / m[1][1];
//			mathassert(EqualAbs(v, Clamp01(v)));
			v = Clamp01(v); // The solution for v must also be in the range [0,1]. TODO: Is this guaranteed by the above?
			// The solution is (u,v,t)=(0,v,1).
			if (otherPt)
				*otherPt = lineSegment.b;
			return a + v * e1;
		}
		else
		{
			// The solution is (u,v,t)=(0,v,t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return a + v * e1;
		}
	}
	else if (uvt.y < 0.f)
	{
		// Clamp to v == 0 and solve again.
		float m_00 = m[2][2];
		float m_01 = -m[0][2];
		float m_10 = -m[2][0];
		float m_11 = m[0][0];
		float det = m_00 * m_11 - m_01 * m_10;
		float u = m_00 * B[0] + m_01 * B[2];
		float t = m_10 * B[0] + m_11 * B[2];
		u /= det;
		t /= det;

		if (u < 0.f)
		{
			// Clamp to u == 0 and solve for t.
			t = B[2] / m[2][2];
			t = Clamp01(t); // The solution for t must also be in the range [0,1].
			// The solution is (u,v,t)=(0,0,t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return a;
		}
		else if (u > 1.f)
		{
			// Clamp to u == 1 and solve for t.
			t = (B[2] - m[2][0]) / m[2][2];
			t = Clamp01(t); // The solution for t must also be in the range [0,1].
			// The solution is (u,v,t)=(1,0,t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return b;
		}
		else if (t < 0.f)
		{
			// Clamp to t == 0 and solve for u.
			u = B[0] / m[0][0];
//			mathassert(EqualAbs(u, Clamp01(u)));
			u = Clamp01(u); // The solution for u must also be in the range [0,1].
			if (otherPt)
				*otherPt = lineSegment.a;
			return a + u * e0;
		}
		else if (t > 1.f)
		{
			// Clamp to t == 1 and solve for u.
			u = (B[0] - m[0][2]) / m[0][0];
//			mathassert(EqualAbs(u, Clamp01(u)));
			u = Clamp01(u); // The solution for u must also be in the range [0,1].
			if (otherPt)
				*otherPt = lineSegment.b;
			return a + u * e0;
		}
		else
		{
			// The solution is (u, 0, t).
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return a + u * e0;
		}
	}
	else if (uvt.z < 0.f)
	{
		if (otherPt)
			*otherPt = lineSegment.a;
		// Clamp to t == 0 and solve again.
		float m_00 = m[1][1];
		float m_01 = -m[0][1];
		float m_10 = -m[1][0];
		float m_11 = m[0][0];
		float det = m_00 * m_11 - m_01 * m_10;
		float u = m_00 * B[0] + m_01 * B[1];
		float v = m_10 * B[0] + m_11 * B[1];
		u /= det;
		v /= det;
		if (u < 0.f)
		{
			// Clamp to u == 0 and solve for v.
			v = B[1] / m[1][1];
			v = Clamp01(v);
			return a + v*e1;
		}
		else if (v < 0.f)
		{
			// Clamp to v == 0 and solve for u.
			u = B[0] / m[0][0];
			u = Clamp01(u);
			return a + u*e0;
		}
		else if (u+v > 1.f)
		{
			// Set v = 1-u and solve again.
//			u = (B[0] - m[0][0]) / (m[0][0] - m[0][1]);
//			mathassert(EqualAbs(u, Clamp01(u)));
//			u = Clamp01(u); // The solution for u must also be in the range [0,1].
//			return a + u*e0;

			// Clamp to v = 1-u and solve again.
			float m_00 = m[2][2];
			float m_01 = m[1][2] - m[0][2];
			float m_10 = m_01;
			float m_11 = m[0][0] + m[1][1] - 2.f * m[0][1];
			float det = m_00 * m_11 - m_01 * m_10;
			float b0 = m[1][1] - m[0][1] + v_p_dot_e1 - v_p_dot_e0;
			float b1 = -m[1][2] + v_p_dot_d;
			float u = m_00 * b0 + m_01 * b1;
			u /= det;
			u = Clamp01(u);

			float t = m_10 * b0 + m_11 * b1;
			t /= det;
			t = Clamp01(t);
			if (otherPt)
				*otherPt = lineSegment.GetPoint(t);
			return a + u*e0 + (1.f-u)*e1;
		}
		else
		{
			// The solution is (u, v, 0)
			return a + u * e0 + v * e1;
		}
	}
	else if (uvt.z > 1.f)
	{
		if (otherPt)
			*otherPt = lineSegment.b;
		// Clamp to t == 1 and solve again.
		float m_00 = m[1][1];
		float m_01 = -m[0][1];
		float m_10 = -m[1][0];
		float m_11 = m[0][0];
		float det = m_00 * m_11 - m_01 * m_10;
		float u = m_00 * (B[0]-m[0][2]) + m_01 * (B[1]-m[1][2]);
		float v = m_10 * (B[0]-m[0][2]) + m_11 * (B[1]-m[1][2]);
		u /= det;
		v /= det;
		if (u < 0.f)
		{
			// Clamp to u == 0 and solve again.
			v = (B[1] - m[1][2]) / m[1][1];
			v = Clamp01(v);
			return a + v*e1;
		}
		else if (u > 1.f)
		{
			// Clamp to u == 1 and solve again.
			v = (B[1] - m[1][0] - m[1][2]) / m[1][1];
			v = Clamp01(v); // The solution for v must also be in the range [0,1]. TODO: Is this guaranteed by the above?
			// The solution is (u,v,t)=(1,v,1).
			return a + e0 + v*e1;
		}
		else if (u+v > 1.f)
		{
			// Set v = 1-u and solve again.

			// Q(u,1-u) = a + u*e0 + e1 - u*e1 = a+e1 + u*(e0-e1)
			// L(1)   = ls.a + t*d = ls.b
			// Minimize the distance |Q(u,1-u) - L(1)| = |a+e1+ls.b + u*(e0-e1)|

			// |K + u*(e0-e1)|^2 = (K,K) + 2*u(K,e0-e1) + u^2 * (e0-e1,e0-e1)

			// grad = 2*(K,e0-e1) + 2*u*(e0-e1,e0-e1) == 0
			//                                      u == (K,e1-e0) / (e0-e1,e0-e1)

			u = (B[0] - m[0][1] - m[0][2]) / (m[0][0] - m[0][1]);
//			u = Dot(a + e1 + lineSegment.b, e1 - e0) / Dot(e0-e1, e0-e1);

//			mathassert(EqualAbs(u, Clamp01(u)));
			u = Clamp01(u);
			return a + u*e0 + (1-u)*e1;
		}
		else
		{
			// The solution is (u, v, 1)
			return a + u*e0 + v*e1;
		}
	}
	else if (uvt.x + uvt.y > 1.f)
	{
		// Clamp to v = 1-u and solve again.
		float m_00 = m[2][2];
		float m_01 = m[1][2] - m[0][2];
		float m_10 = m_01;
		float m_11 = m[0][0] + m[1][1] - 2.f * m[0][1];
		float det = m_00 * m_11 - m_01 * m_10;
		float b0 = m[1][1] - m[0][1] + v_p_dot_e1 - v_p_dot_e0;
		float b1 = -m[1][2] + v_p_dot_d;
		float u = m_00 * b0 + m_01 * b1;
		float t = m_10 * b0 + m_11 * b1;
		u /= det;
		t /= det;

		t = Clamp01(t);
		if (otherPt)
			*otherPt = lineSegment.GetPoint(t);

		if (u < 0.f)
		{
			// The solution is (u,v,t)=(0,1,t)
			return c;
		}
		if (u > 1.f)
		{
			// The solution is (u,v,t)=(1,0,t)
			return b;
		}
		mathassert(t >= 0.f);
		mathassert(t <= 1.f);
		return a + u*e0 + (1.f-u)*e1;
	}
	else // All parameters are within range, so the triangle and the line segment intersect, and the intersection point is the closest point.
	{
		if (otherPt)
			*otherPt = lineSegment.GetPoint(uvt.z);
		return a + uvt.x * e0 + uvt.y * e1;
	}
}
#endif
#if 0
vec2d Triangle2D::ClosestPointToTriangleEdge(const Line2D &other, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float unused1, unused2, unused3;
	float d1, d2, d3;
	vec2d pt1 = Edge(0).ClosestPoint(other, unused1, d1);
	vec2d pt2 = Edge(1).ClosestPoint(other, unused2, d2);
	vec2d pt3 = Edge(2).ClosestPoint(other, unused3, d3);
	float dist1 = pt1.DistanceSq(other.GetPoint(d1));
	float dist2 = pt2.DistanceSq(other.GetPoint(d2));
	float dist3 = pt3.DistanceSq(other.GetPoint(d3));
	if (dist1 <= dist2 && dist1 <= dist3)
	{
		if (outU) *outU = BarycentricUV(pt1).x;
		if (outV) *outV = BarycentricUV(pt1).y;
		if (outD) *outD = d1;
		return pt1;
	}
	else if (dist2 <= dist3)
	{
		if (outU) *outU = BarycentricUV(pt2).x;
		if (outV) *outV = BarycentricUV(pt2).y;
		if (outD) *outD = d2;
		return pt2;
	}
	else
	{
		if (outU) *outU = BarycentricUV(pt3).x;
		if (outV) *outV = BarycentricUV(pt3).y;
		if (outD) *outD = d3;
		return pt3;
	}
}

vec2d Triangle2D::ClosestPointToTriangleEdge(const LineSegment2D &lineSegment, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float unused1, unused2, unused3;
	float d1, d2, d3;
	vec2d pt1 = Edge(0).ClosestPoint(lineSegment, unused1, d1);
	vec2d pt2 = Edge(1).ClosestPoint(lineSegment, unused2, d2);
	vec2d pt3 = Edge(2).ClosestPoint(lineSegment, unused3, d3);
	float dist1 = pt1.DistanceSq(lineSegment.GetPoint(d1));
	float dist2 = pt2.DistanceSq(lineSegment.GetPoint(d2));
	float dist3 = pt3.DistanceSq(lineSegment.GetPoint(d3));
	if (dist1 <= dist2 && dist1 <= dist3)
	{
		if (outU) *outU = BarycentricUV(pt1).x;
		if (outV) *outV = BarycentricUV(pt1).y;
		if (outD) *outD = d1;
		return pt1;
	}
	else if (dist2 <= dist3)
	{
		if (outU) *outU = BarycentricUV(pt2).x;
		if (outV) *outV = BarycentricUV(pt2).y;
		if (outD) *outD = d2;
		return pt2;
	}
	else
	{
		if (outU) *outU = BarycentricUV(pt3).x;
		if (outV) *outV = BarycentricUV(pt3).y;
		if (outD) *outD = d3;
		return pt3;
	}
}

vec2d Triangle2D::ClosestPoint(const Line2D &line, vec2d *otherPt) const
{
	///\todo Optimize this function.
	vec2d intersectionPoint;
	if (Intersects(line, 0, &intersectionPoint))
	{
		if (otherPt)
			*otherPt = intersectionPoint;
		return intersectionPoint;
	}

	float u1,v1,d1;
	vec2d pt1 = ClosestPointToTriangleEdge(line, &u1, &v1, &d1);
	if (otherPt)
		*otherPt = line.GetPoint(d1);
	return pt1;
}
#endif
#if 0
///\todo Enable this codepath. This if rom Geometric Tools for Computer Graphics,
/// but the algorithm in the book is broken and does not take into account the
/// direction of the gradient to determine the proper region of intersection.
/// Instead using a slower code path above.
vec2d Triangle2D::ClosestPoint(const Line2D &line, vec2d *otherPt) const
{
	vec2d e0 = b - a;
	vec2d e1 = c - a;
	vec2d v_p = a - line.pos;
	vec2d d = line.dir;

	float v_p_dot_e0 = Dot(v_p, e0);
	float v_p_dot_e1 = Dot(v_p, e1);
	float v_p_dot_d = Dot(v_p, d);

	float3x3 m;
	m[0][0] = Dot(e0, e0); m[0][1] = Dot(e0, e1); m[0][2] = -Dot(e0, d);
	m[1][0] =     m[0][1]; m[1][1] = Dot(e1, e1); m[1][2] = -Dot(e1, d);
	m[2][0] =     m[0][2]; m[2][1] =     m[1][2]; m[2][2] =  Dot(d, d);

	float3 B(-v_p_dot_e0, -v_p_dot_e1, v_p_dot_d);

	float3 uvt;
	bool success = m.SolveAxb(B, uvt);
	if (!success)
	{
		float t1, t2, t3;
		float s1, s2, s3;
		LineSegment2D e1 = Edge(0);
		LineSegment2D e2 = Edge(1);
		LineSegment2D e3 = Edge(2);
		float d1 = e1.Distance(line, &t1, &s1);
		float d2 = e2.Distance(line, &t2, &s2);
		float d3 = e3.Distance(line, &t3, &s3);
		if (d1 < d2 && d1 < d3)
		{
			if (otherPt)
				*otherPt = line.GetPoint(s1);
			return e1.GetPoint(t1);
		}
		else if (d2 < d3)
		{
			if (otherPt)
				*otherPt = line.GetPoint(s2);
			return e2.GetPoint(t2);
		}
		else
		{
			if (otherPt)
				*otherPt = line.GetPoint(s3);
			return e3.GetPoint(t3);
		}
	}

	if (uvt.x < 0.f)
	{
		// Clamp to u == 0 and solve again.
		float m_00 = m[2][2];
		float m_01 = -m[1][2];
		float m_10 = -m[2][1];
		float m_11 = m[1][1];
		float det = m_00 * m_11 - m_01 * m_10;
		float v = m_00 * B[1] + m_01 * B[2];
		float t = m_10 * B[1] + m_11 * B[2];
		v /= det;
		t /= det;
		if (v < 0.f)
		{
			// Clamp to v == 0 and solve for t.
			t = B[2] / m[2][2];
			// The solution is (u,v,t)=(0,0,t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return a;
		}
		else if (v > 1.f)
		{
			// Clamp to v == 1 and solve for t.
			t = (B[2] - m[2][1]) / m[2][2];
			// The solution is (u,v,t)=(0,1,t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return c; // == a + v*e1
		}
		else
		{
			// The solution is (u,v,t)=(0,v,t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return a + v * e1;
		}
	}
	else if (uvt.y < 0.f)
	{
		// Clamp to v == 0 and solve again.
		float m_00 = m[2][2];
		float m_01 = -m[0][2];
		float m_10 = -m[2][0];
		float m_11 = m[0][0];
		float det = m_00 * m_11 - m_01 * m_10;
		float u = m_00 * B[0] + m_01 * B[2];
		float t = m_10 * B[0] + m_11 * B[2];
		u /= det;
		t /= det;

		if (u < 0.f)
		{
			// Clamp to u == 0 and solve for t.
			t = B[2] / m[2][2];
			// The solution is (u,v,t)=(0,0,t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return a;
		}
		else if (u > 1.f)
		{
			// Clamp to u == 1 and solve for t.
			t = (B[2] - m[2][0]) / m[2][2];
			// The solution is (u,v,t)=(1,0,t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return b;
		}
		else
		{
			// The solution is (u, 0, t).
			if (otherPt)
				*otherPt = line.GetPoint(t);
			return a + u * e0;
		}
	}
	else if (uvt.x + uvt.y > 1.f)
	{
		// Clamp to v = 1-u and solve again.
		float m_00 = m[2][2];
		float m_01 = m[1][2] - m[0][2];
		float m_10 = m_01;
		float m_11 = m[0][0] + m[1][1] - 2.f * m[0][1];
		float det = m_00 * m_11 - m_01 * m_10;
		float b0 = m[1][1] - m[0][1] + v_p_dot_e1 - v_p_dot_e0;
		float b1 = -m[1][2] + v_p_dot_d;
		float u = m_00 * b0 + m_01 * b1;
		float t = m_10 * b0 + m_11 * b1;
		u /= det;
		t /= det;

		if (otherPt)
			*otherPt = line.GetPoint(t);

		if (u < 0.f)
		{
			// The solution is (u,v,t)=(0,1,t)
			return c;
		}
		if (u > 1.f)
		{
			// The solution is (u,v,t)=(1,0,t)
			return b;
		}
		return a + u*e0 + (1.f-u)*e1;
	}
	else // All parameters are within range, so the triangle and the line segment intersect, and the intersection point is the closest point.
	{
		if (otherPt)
			*otherPt = line.GetPoint(uvt.z);
		return a + uvt.x * e0 + uvt.y * e1;
	}
}
#endif

#if 0
/// [groupSyntax]
vec2d Triangle2D::ClosestPoint(const Line2D &other, float *outU, float *outV, float *outD) const
{
	/** The implementation of the Triangle2D-Line2D test is based on the pseudo-code in
		Schneider, Eberly. Geometric Tools for Computer Graphics pp. 433 - 441. */
	///@todo The Triangle2D-Line2D code is currently untested. Run tests to ensure the following code works properly.

	// Point on triangle: T(u,v) = a + u*b + v*c;
	// Point on line:  L(t) = p + t*d;
	// Minimize the function Q(u,v,t) = ||T(u,v) - L(t)||.

	vec2d e0 = b-a;
	vec2d e1 = c-a;
	vec2d d = other.dir;

	const float d_e0e0 = Dot(e0, e0);
	const float d_e0e1 = Dot(e0, e1);
	const float d_e0d = Dot(e0, d);
	const float d_e1e1 = Dot(e1, e1);
	const float d_e1d = Dot(e1, d);
	const float d_dd = Dot(d, d);

	float3x3 m;
	m[0][0] = d_e0e0;  m[0][1] = d_e0e1;  m[0][2] = -d_e0d;
	m[1][0] = d_e0e1;  m[1][1] = d_e1e1;  m[1][2] = -d_e1d;
	m[2][0] = -d_e0d;  m[2][1] = -d_e1d;  m[2][2] =   d_dd;

	///@todo Add optimized float3x3::InverseSymmetric().
	bool inv = m.Inverse();
	if (!inv)
		return ClosestPointToTriangleEdge(other, outU, outV, outD);

	vec2d v_m_p = a - other.pos;
	float v_m_p_e0 = v_m_p.Dot(e0);
	float v_m_p_e1 = v_m_p.Dot(e1);
	float v_m_p_d = v_m_p.Dot(d);
	float3 b = float3(-v_m_p_e0, -v_m_p_e1, v_m_p_d);
	float3 uvt = m * b;
	// We cannot simply clamp the solution to (uv) inside the constraints, since the expression we
	// are minimizing is quadratic.
	// So, examine case-by-case which part of the space the solution lies in. Because the function is convex,
	// we can clamp the search space to the boundary planes.
	float u = uvt.x;
	float v = uvt.y;
	float t = uvt.z;
	if (u <= 0)
	{
		if (outU) *outU = 0;

		// Solve 2x2 matrix for the (v,t) solution when u == 0.
		v = m[1][1]*b[1] + m[1][2]*b[2];
		t = m[2][1]*b[1] + m[2][2]*b[2];

		// Check if the solution is still out of bounds.
		if (v <= 0)
		{
			if (outV) *outV = 0;
			if (outD) *outD = v_m_p_d / d_dd;
			return Point(0, 0);
		}
		else if (v >= 1)
		{
			if (outV) *outV = 1;
			if (outD) *outD = (v_m_p_d - d_e1d) / d_dd;
			return Point(0, 1);
		}
		else // (0 <= v <= 1).
		{
			if (outV) *outV = v;
			if (outD) *outD = t;
			return Point(0, v);
		}
	}
	else if (v <= 0)
	{
		if (outV) *outV = 0;

		// Solve 2x2 matrix for the (u,t) solution when v == 0.
		u = m[0][0]*b[0] + m[0][2]*b[2];
		t = m[2][0]*b[0] + m[2][2]*b[2];

		// Check if the solution is still out of bounds.
		if (u <= 0)
		{
			if (outU) *outU = 0;
			if (outD) *outD = v_m_p_d / d_dd;
			return Point(0, 0);
		}
		else if (u >= 1)
		{
			if (outU) *outU = 1;
			if (outD) *outD = (v_m_p_d - d_e0d) / d_dd;
			return Point(1, 0);
		}
		else // (0 <= u <= 1).
		{
			if (outU) *outU = u;
			if (outD) *outD = t;
			return Point(u, 0);
		}
	}
	else if (u + v >= 1.f)
	{
		// Set v = 1-u.
#if 0
		float m00 = d_e0e0 + d_e1e1 - 2.f * d_e0e1;
		float m01 = -d_e0d + d_e1d;
		float m10 = -d_e0d + d_e1d;
		float m11 = d_dd;
//		float det = 1.f / (m00*m11 - m01*m10);

		float b0 = d_e1e1 - d_e0e1 + v_m_p_e0 - v_m_p_e1;
		float b1 = d_e1d + v_m_p_d;
		/*
		// Inverse 2x2 matrix.
		Swap(m00, m11);
		Swap(m01, m10);
		m01 = -m01;
		m10 = -m10;
		*/
		// 2x2 * 2 matrix*vec2d mul.
		u = (m00*b0 + m01*b1);// * det;
		t = (m10*b0 + m11*b1);// * det;
#endif
//		u = m[0][0]*b[0] +

		// Check if the solution is still out of bounds.
		if (u <= 0)
		{
			if (outU) *outU = 0;
			if (outV) *outV = 1;
			if (outD) *outD = (d_e1d + v_m_p_d) / d_dd;
			return Point(0, 1);
		}
		else if (u >= 1)
		{
			if (outU) *outU = 1;
			if (outV) *outV = 0;
			if (outD) *outD = (v_m_p_d + d_e0d) / d_dd;
			return Point(1, 0);
		}
		else // (0 <= u <= 1).
		{
			if (outU) *outU = u;
			if (outV) *outV = 1.f - u;
			if (outD) *outD = t;
			return Point(u, 1.f - u);
		}
	}
	else // Each u, v and t are in appropriate range.
	{
		if (outU) *outU = u;
		if (outV) *outV = v;
		if (outD) *outD = t;
		return Point(u, v);
	}
}
#endif
#if 0
/// [groupSyntax]
vec2d Triangle2D::ClosestPoint(const Triangle2D &other, vec2d *otherPt) const
{
	/** The code for computing the closest point pair on two Triangles is based
		on pseudo-code from Christer Ericson's Real-Time Collision Detection, pp. 155-156. */

	// First detect if the two triangles are intersecting.
	LineSegment2D l;
	bool success = this->Intersects(other, &l);
	if (success)
	{
		vec2d cp = l.CenterPoint();
		if (otherPt)
			*otherPt = cp;
		return cp;
	}

	vec2d closestThis = this->ClosestPoint(other.a);
	vec2d closestOther = other.a;
	float closestDSq = closestThis.DistanceSq(closestOther);

	vec2d pt = this->ClosestPoint(other.b);
	float dSq = pt.DistanceSq(other.b);
	if (dSq < closestDSq) closestThis = pt, closestOther = other.b, closestDSq = dSq;

	pt = this->ClosestPoint(other.c);
	dSq = pt.DistanceSq(other.c);
	if (dSq < closestDSq) closestThis = pt, closestOther = other.c, closestDSq = dSq;

	pt = other.ClosestPoint(this->a);
	dSq = pt.DistanceSq(this->a);
	if (dSq < closestDSq) closestOther = pt, closestThis = this->a, closestDSq = dSq;

	pt = other.ClosestPoint(this->b);
	dSq = pt.DistanceSq(this->b);
	if (dSq < closestDSq) closestOther = pt, closestThis = this->b, closestDSq = dSq;

	pt = other.ClosestPoint(this->c);
	dSq = pt.DistanceSq(this->c);
	if (dSq < closestDSq) closestOther = pt, closestThis = this->c, closestDSq = dSq;

	LineSegment2D l1[3] = { LineSegment2D(a,b), LineSegment2D(a,c), LineSegment2D(b,c) };
	LineSegment2D l2[3] = { LineSegment2D(other.a,other.b), LineSegment2D(other.a,other.c), LineSegment2D(other.b,other.c) };
	float d, d2;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			float dist = l1[i].Distance(l2[j], d, d2);
			if (dist*dist < closestDSq)
			{
				closestThis = l1[i].GetPoint(d);
				closestOther = l2[j].GetPoint(d2);
				closestDSq = dist*dist;
			}
		}

	if (otherPt)
		*otherPt = closestOther;
	return closestThis;
}
#endif
vec2d Triangle2D::RandomPointInside(LCG &rng) const
{
	float epsilon = 1e-3f;
	///@todo rng.Float() returns [0,1[, but to be completely uniform, we'd need [0,1] here.
	float s = rng.Float(epsilon, 1.f - epsilon);//1e-2f, 1.f - 1e-2f);
	float t = rng.Float(epsilon, 1.f - epsilon);//1e-2f, 1.f - 1e-2f
	if (s + t >= 1.f)
	{
		s = 1.f - s;
		t = 1.f - t;
	}
#ifdef MATH_ASSERT_CORRECTNESS
	vec2d pt = Point(s, t);
	float2 uv = BarycentricUV(pt);
	assert1(uv.x >= 0.f, uv.x);
	assert1(uv.y >= 0.f, uv.y);
	assert3(uv.x + uv.y <= 1.f, uv.x, uv.y, uv.x + uv.y);
	float3 uvw = BarycentricUVW(pt);
	assert1(uvw.x >= 0.f, uvw.x);
	assert1(uvw.y >= 0.f, uvw.y);
	assert1(uvw.z >= 0.f, uvw.z);
	assert4(EqualAbs(uvw.x + uvw.y + uvw.z, 1.f), uvw.x, uvw.y, uvw.z, uvw.x + uvw.y + uvw.z);
#endif
	return Point(s, t);
}

vec2d Triangle2D::RandomVertex(LCG &rng) const
{
	return Vertex(rng.Int(0, 2));
}

vec2d Triangle2D::RandomPointOnEdge(LCG &rng) const
{
	assume(!IsDegenerate());
	float ab = a.Distance(b);
	float bc = b.Distance(c);
	float ca = c.Distance(a);
	float r = rng.Float(0, ab + bc + ca);
	if (r < ab)
		return a + (b-a) * r / ab;
	r -= ab;
	if (r < bc)
		return b + (c-b) * r / bc;
	r -= bc;
	return c + (a-c) * r / ca;
}

Triangle2D operator *(const float3x3 &transform, const Triangle2D &triangle)
{
	Triangle2D t(triangle);
	t.Transform(transform);
	return t;
}

Triangle2D operator *(const float3x4 &transform, const Triangle2D &triangle)
{
	Triangle2D t(triangle);
	t.Transform(transform);
	return t;
}

Triangle2D operator *(const float4x4 &transform, const Triangle2D &triangle)
{
	Triangle2D t(triangle);
	t.Transform(transform);
	return t;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT Triangle2D::ToString() const
{
	char str[256];
	sprintf(str, "Triangle2D(a:(%.2f, %.2f) b:(%.2f, %.2f) c:(%.2f, %.2f))",
		a.x, a.y, b.x, b.y, c.x, c.y);
	return str;
}

StringT Triangle2D::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(a.x, str); *s = ','; ++s;
	s = SerializeFloat(a.y, s); *s = ','; ++s;
	s = SerializeFloat(b.x, s); *s = ','; ++s;
	s = SerializeFloat(b.y, s); *s = ','; ++s;
	s = SerializeFloat(c.x, s); *s = ','; ++s;
	s = SerializeFloat(c.y, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT Triangle2D::SerializeToCodeString() const
{
	return "Triangle2D(" + a.SerializeToCodeString() + "," + b.SerializeToCodeString() + "," + c.SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const Triangle2D &triangle)
{
	o << triangle.ToString();
	return o;
}

#endif

Triangle2D Triangle2D::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return Triangle2D(vec2d::nan, vec2d::nan, vec2d::nan);
	Triangle2D t;
	MATH_SKIP_WORD(str, "Triangle2D(");
	MATH_SKIP_WORD(str, "a:(");
	t.a = POINT_TO_FLOAT4(PointVecFromString(str, &str)).ToVec2D();
	MATH_SKIP_WORD(str, " b:(");
	t.b = POINT_TO_FLOAT4(PointVecFromString(str, &str)).ToVec2D();
	MATH_SKIP_WORD(str, " c:(");
	t.c = POINT_TO_FLOAT4(PointVecFromString(str, &str)).ToVec2D();
	if (outEndStr)
		*outEndStr = str;
	return t;
}

MATH_END_NAMESPACE
