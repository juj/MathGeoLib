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

/** @file Triangle.cpp
	@author Jukka Jylänki
	@brief Implementation for the Triangle geometry object. */
#include "Triangle.h"
#include "../Math/MathFunc.h"
#include "../Math/Swap.h"
#include "../Math/float2.h"
#include "../Math/float3.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "../Math/float4d.h"
#include "Capsule.h"
#include "Frustum.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Line.h"
#include "LineSegment.h"
#include "Ray.h"
#include "Sphere.h"
#include "AABB.h"
#include "OBB.h"
#include "../Algorithm/Random/LCG.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
#include "../Math/float4_sse.h"
#include "../Math/float4_neon.h"
#endif

MATH_BEGIN_NAMESPACE

Triangle::Triangle(const vec &a_, const vec &b_, const vec &c_)
:a(a_), b(b_), c(c_)
{
}

void Triangle::Translate(const vec &offset)
{
	a += offset;
	b += offset;
	c += offset;
}

void Triangle::Transform(const float3x3 &transform)
{
	transform.BatchTransform(&a, 3);
}

void Triangle::Transform(const float3x4 &transform)
{
	transform.BatchTransformPos(&a, 3);
}

void Triangle::Transform(const float4x4 &transform)
{
	a = transform.MulPos(a);
	b = transform.MulPos(b);
	c = transform.MulPos(c);
}

void Triangle::Transform(const Quat &transform)
{
	a = transform * a;
	b = transform * b;
	c = transform * c;
}

/// Implementation from Christer Ericson's Real-Time Collision Detection, pp. 51-52.
inline float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3)
{
	return (x1-x2)*(y2-y3) - (x2-x3)*(y1-y2);
}

float3 Triangle::BarycentricUVW(const vec &point) const
{
	// Implementation from Christer Ericson's Real-Time Collision Detection, pp. 51-52.

	// Unnormalized triangle normal.
	vec m = Cross(b-a, c-a);

	// Nominators and one-over-denominator for u and v ratios.
	float nu, nv, ood;

	// Absolute components for determining projection plane.
	float x = Abs(m.x);
	float y = Abs(m.y);
	float z = Abs(m.z);

	if (x >= y && x >= z)
	{
		// Project to the yz plane.
		nu = TriArea2D(point.y, point.z, b.y, b.z, c.y, c.z); // Area of PBC in yz-plane.
		nv = TriArea2D(point.y, point.z, c.y, c.z, a.y, a.z); // Area OF PCA in yz-plane.
		ood = 1.f / m.x; // 1 / (2*area of ABC in yz plane)
	}
	else if (y >= z) // Note: The book has a redundant 'if (y >= x)' comparison
	{
		// y is largest, project to the xz-plane.
		nu = TriArea2D(point.x, point.z, b.x, b.z, c.x, c.z);
		nv = TriArea2D(point.x, point.z, c.x, c.z, a.x, a.z);
		ood = 1.f / -m.y;
	}
	else // z is largest, project to the xy-plane.
	{
		nu = TriArea2D(point.x, point.y, b.x, b.y, c.x, c.y);
		nv = TriArea2D(point.x, point.y, c.x, c.y, a.x, a.y);
		ood = 1.f / m.z;
	}
	float u = nu * ood;
	float v = nv * ood;
	float w = 1.f - u - v;
	return float3(u,v,w);
#if 0 // TODO: This version should be more SIMD-friendly, but for some reason, it doesn't return good values for all points inside the triangle.
	vec v0 = b - a;
	vec v1 = c - a;
	vec v2 = point - a;
	float d00 = Dot(v0, v0);
	float d01 = Dot(v0, v1);
	float d02 = Dot(v0, v2);
	float d11 = Dot(v1, v1);
	float d12 = Dot(v1, v2);
	float denom = 1.f / (d00 * d11 - d01 * d01);
	float v = (d11 * d02 - d01 * d12) * denom;
	float w = (d00 * d12 - d01 * d02) * denom;
	float u = 1.0f - v - w;
	return vec(u, v, w);
#endif
}

float2 Triangle::BarycentricUV(const vec &point) const
{
	float3 uvw = BarycentricUVW(point);
	return float2(uvw.y, uvw.z);
}

bool Triangle::BarycentricInsideTriangle(const float3 &barycentric)
{
	return barycentric.x >= 0.f && barycentric.y >= 0.f && barycentric.z >= 0.f &&
		EqualAbs(barycentric.x + barycentric.y + barycentric.z, 1.f);
}

vec Triangle::Point(float u, float v) const
{
	// In case the triangle is far away from the origin but is small in size, the elements of 'a' will have large magnitudes,
	// and the elements of (b-a) and (c-a) will be much smaller quantities. Therefore be extra careful with the
	// parentheses and first sum the small floats together before adding it to the large one.
	return a + ((b-a) * u + (c-a) * v);
}

vec Triangle::Point(float u, float v, float w) const
{
	return u * a + v * b + w * c;
}

vec Triangle::Point(const float3 &uvw) const
{
	return Point(uvw.x, uvw.y, uvw.z);
}

vec Triangle::Point(const float2 &uv) const
{
	return Point(uv.x, uv.y);
}

vec Triangle::Centroid() const
{
	return (a + b + c) * (1.f/3.f);
}

float Triangle::Area() const
{
	return 0.5f * Cross(b-a, c-a).Length();
}

float Triangle::Perimeter() const
{
	return a.Distance(b) + b.Distance(c) + c.Distance(a);
}

LineSegment Triangle::Edge(int i) const
{
	assume(0 <= i);
	assume(i <= 2);
	if (i == 0)
		return LineSegment(a, b);
	else if (i == 1)
		return LineSegment(b, c);
	else if (i == 2)
		return LineSegment(c, a);
	else
		return LineSegment(vec::nan, vec::nan);
}

vec Triangle::Vertex(int i) const
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
		return vec::nan;
}

Plane Triangle::PlaneCCW() const
{
	return Plane(a, b, c);
}

Plane Triangle::PlaneCW() const
{
	return Plane(a, c, b);
}

vec Triangle::NormalCCW() const
{
	return UnnormalizedNormalCCW().Normalized();
}

vec Triangle::NormalCW() const
{
	return UnnormalizedNormalCW().Normalized();
}

vec Triangle::UnnormalizedNormalCCW() const
{
	return Cross(b-a, c-a);
}

vec Triangle::UnnormalizedNormalCW() const
{
	return Cross(c-a, b-a);
}

vec Triangle::ExtremePoint(const vec &direction) const
{
	vec mostExtreme = vec::nan;
	float mostExtremeDist = -FLT_MAX;
	for(int i = 0; i < 3; ++i)
	{
		vec pt = Vertex(i);
		float d = Dot(direction, pt);
		if (d > mostExtremeDist)
		{
			mostExtremeDist = d;
			mostExtreme = pt;
		}
	}
	return mostExtreme;
}

vec Triangle::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

Polygon Triangle::ToPolygon() const
{
	Polygon p;
	p.p.push_back(a);
	p.p.push_back(b);
	p.p.push_back(c);
	return p;
}

Polyhedron Triangle::ToPolyhedron() const
{
	return ToPolygon().ToPolyhedron();
}

AABB Triangle::BoundingAABB() const
{
	AABB aabb;
	aabb.minPoint = Min(a, b, c);
	aabb.maxPoint = Max(a, b, c);
	return aabb;
}

float Triangle::Area2D(const float2 &p1, const float2 &p2, const float2 &p3)
{
	return (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
}

float Triangle::SignedArea(const vec &pt, const vec &a, const vec &b, const vec &c)
{
	return Dot(Cross(b-pt, c-pt), Cross(b-a, c-a).Normalized());
}

bool Triangle::IsFinite() const
{
	return a.IsFinite() && b.IsFinite() && c.IsFinite();
}

bool Triangle::IsDegenerate(float epsilon) const
{
	return IsDegenerate(a, b, c, epsilon);
}

bool Triangle::IsDegenerate(const vec &a, const vec &b, const vec &c, float epsilon)
{
	return a.Equals(b, epsilon) || a.Equals(c, epsilon) || b.Equals(c, epsilon);
}

bool Triangle::Contains(const vec &point, float triangleThicknessSq) const
{
	vec normal = (b-a).Cross(c-a);
	float lenSq = normal.LengthSq();
	float d = normal.Dot(b - point);
	if (d*d > triangleThicknessSq * lenSq)
		return false; ///@todo The plane-point distance test is omitted in Real-Time Collision Detection. p. 25. A bug in the book?

	float3 br = BarycentricUVW(point);
	return br.x >= -1e-3f && br.y >= -1e-3f && br.z >= -1e-3f; // Allow for a small epsilon to properly account for points very near the edges of the triangle.
}

bool Triangle::Contains(const LineSegment &lineSegment, float triangleThickness) const
{
	return Contains(lineSegment.a, triangleThickness) && Contains(lineSegment.b, triangleThickness);
}

bool Triangle::Contains(const Triangle &triangle, float triangleThickness) const
{
	return Contains(triangle.a, triangleThickness) && Contains(triangle.b, triangleThickness)
	  && Contains(triangle.c, triangleThickness);
}

/*
bool Triangle::Contains(const Polygon &polygon, float triangleThickness) const
{
	if (polygon.points.size() == 0)
		return false;
	for(int i = 0; i < polygon.points.size(); ++i)
		if (!Contains(polygon.points[i], triangleThickness))
			return false;
	return true;
}
*/
float Triangle::Distance(const vec &point) const
{
	return ClosestPoint(point).Distance(point);
}

float Triangle::DistanceSq(const vec &point) const
{
	return ClosestPoint(point).DistanceSq(point);
}

double Triangle::DistanceSqD(const vec &point) const
{
    return ClosestPointD(point).DistanceSq(point);
}

float Triangle::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Triangle::Distance(const Capsule &capsule) const
{
	vec otherPt;
	vec thisPt = ClosestPoint(capsule.l, &otherPt);
	return Max(0.f, thisPt.Distance(otherPt) - capsule.r);
}

/** Calculates the intersection between a line and a triangle. The facing is not accounted for, so
	rays are reported to intersect triangles that are both front and backfacing.
	According to "T. M&ouml;ller, B. Trumbore. Fast, Minimum Storage Ray/Triangle Intersection. 2005."
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
float Triangle::IntersectLineTri(const vec &linePos, const vec &lineDir,
		const vec &v0, const vec &v1, const vec &v2,
		float &u, float &v)
{
	const float epsilon = 1e-4f;

	// Edge vectors
	vec vE1 = v1 - v0;
	vec vE2 = v2 - v0;

	// begin calculating determinant - also used to calculate U parameter
	vec vP = lineDir.Cross(vE2);

	// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
	const float det = vE1.Dot(vP);

	// If determinant is near zero, ray lies in plane of triangle.
	if (Abs(det) <= epsilon)
		return FLOAT_INF;
	const float recipDet = 1.f / det;

	// Calculate distance from v0 to ray origin
	vec vT = linePos - v0;

	// Output barycentric u
	u = vT.Dot(vP) * recipDet;
	if (u < -epsilon || u > 1.f + epsilon)
		return FLOAT_INF; // Barycentric U is outside the triangle - early out.

	// Prepare to test V parameter
	vec vQ = vT.Cross(vE1);

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
bool Triangle::Intersects(const LineSegment &l, float *d, vec *intersectionPoint) const
{
	/** The Triangle-Line/LineSegment/Ray intersection tests are based on M&ouml;ller-Trumbore method:
		"T. M&ouml;ller, B. Trumbore. Fast, Minimum Storage Ray/Triangle Intersection. 2005."
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

bool Triangle::Intersects(const Line &l, float *d, vec *intersectionPoint) const
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

bool Triangle::Intersects(const Ray &r, float *d, vec *intersectionPoint) const
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

bool Triangle::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

/// [groupSyntax]
/** For Triangle-Sphere intersection code, see Christer Ericson's Real-Time Collision Detection, p.167. */
bool Triangle::Intersects(const Sphere &sphere, vec *closestPointOnTriangle) const
{
	vec pt = ClosestPoint(sphere.pos);

	if (closestPointOnTriangle)
		*closestPointOnTriangle = pt;

	return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool Triangle::Intersects(const Sphere &sphere) const
{
	return Intersects(sphere, 0);
}

static void FindIntersectingLineSegments(const Triangle &t, float da, float db, float dc, LineSegment &l1, LineSegment &l2)
{
	if (da*db > 0.f)
	{
		l1 = LineSegment(t.a, t.c);
		l2 = LineSegment(t.b, t.c);
	}
	else if (db*dc > 0.f)
	{
		l1 = LineSegment(t.a, t.b);
		l2 = LineSegment(t.a, t.c);
	}
	else
	{
		l1 = LineSegment(t.a, t.b);
		l2 = LineSegment(t.b, t.c);
	}
}

/// [groupSyntax]
/** The Triangle-Triangle test implementation is based on pseudo-code from Tomas M&ouml;ller's
	"A Fast Triangle-Triangle Intersection Test": http://jgt.akpeters.com/papers/Moller97/.
	See also Christer Ericson's Real-Time Collision Detection, p. 172. */
bool Triangle::Intersects(const Triangle &t2, LineSegment *outLine) const
{
	const float triangleThickness = 1e-4f;
	// Is the triangle t2 completely on one side of the plane of this triangle?
	Plane p1 = this->PlaneCCW();
	Plane p2 = t2.PlaneCCW();
	if (!p1.normal.Cross(p2.normal).IsZero())
	{
		float t2da = p1.SignedDistance(t2.a);
		float t2db = p1.SignedDistance(t2.b);
		float t2dc = p1.SignedDistance(t2.c);
		float t2min = Min(Min(t2da, t2db), t2dc);
		float t2max = Max(Max(t2da, t2db), t2dc);
		if (t2max < -triangleThickness || t2min > triangleThickness)
			return false;
		// Is this triangle completely on one side of the plane of the triangle t2?
		float t1da = p2.SignedDistance(this->a);
		float t1db = p2.SignedDistance(this->b);
		float t1dc = p2.SignedDistance(this->c);
		float t1min = Min(Min(t1da, t1db), t1dc);
		float t1max = Max(Max(t1da, t1db), t1dc);
		if (t1max < -triangleThickness || t1min > triangleThickness)
			return false;

		// Find the intersection line of the two planes.
		Line l;
		bool success = p1.Intersects(p2, &l);
		assume(success); // We already determined the two triangles have intersecting planes, so this should always succeed.
		if (!success)
			return false;

		// Find the two line segments of both triangles which straddle the intersection line.
		LineSegment l1a, l1b;
		LineSegment l2a, l2b;
		FindIntersectingLineSegments(*this, t1da, t1db, t1dc, l1a, l1b);
		FindIntersectingLineSegments(t2, t2da, t2db, t2dc, l2a, l2b);

		// Find the projection intervals on the intersection line.
		float d1a, d1b, d2a, d2b;
		l.Distance(l1a, d1a);
		l.Distance(l1b, d1b);
		l.Distance(l2a, d2a);
		l.Distance(l2b, d2b);
		if (d1a > d1b)
			Swap(d1a, d1b);
		if (d2a > d2b)
			Swap(d2a, d2b);
		float rStart = Max(d1a, d2a);
		float rEnd = Min(d1b, d2b);
		if (rStart <= rEnd)
		{
			if (outLine)
				*outLine = LineSegment(l.GetPoint(rStart), l.GetPoint(rEnd));
			return true;
		}
		return false;
	}
	else // The two triangles lie in the same plane. Perform the intersection test in 2D.
	{
		float tri0Pos = p1.normal.Dot(a);
		float tri1Pos = p1.normal.Dot(t2.a);
		if (Abs(tri0Pos-tri1Pos) > triangleThickness)
			return false;

		if (t2.Contains(a, FLOAT_INF) || this->Contains(t2.a, FLOAT_INF))
			return true;

		vec basisU, basisV;
		p1.normal.PerpendicularBasis(basisU, basisV);
		float2 a1 = float2::zero;
		float2 a2 = float2(basisU.Dot(b-a), basisV.Dot(b-a));
		float2 a3 = float2(basisU.Dot(c-a), basisV.Dot(c-a));
		float2 b1 = float2(basisU.Dot(t2.a-a), basisV.Dot(t2.a-a));
		float2 b2 = float2(basisU.Dot(t2.b-a), basisV.Dot(t2.b-a));
		float2 b3 = float2(basisU.Dot(t2.c-a), basisV.Dot(t2.c-a));
		float s, t;
		if (LineSegment2DLineSegment2DIntersect(a1, a2-a1, b1, b2-b1, s, t)) { if (outLine) { float2 pt = s*(a2-a1); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a1, a2-a1, b2, b3-b2, s, t)) { if (outLine) { float2 pt = s*(a2-a1); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a1, a2-a1, b3, b1-b3, s, t)) { if (outLine) { float2 pt = s*(a2-a1); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a2, a3-a2, b1, b2-b1, s, t)) { if (outLine) { float2 pt = s*(a3-a2); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a2, a3-a2, b2, b3-b2, s, t)) { if (outLine) { float2 pt = s*(a3-a2); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a2, a3-a2, b3, b1-b3, s, t)) { if (outLine) { float2 pt = s*(a3-a2); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a3, a1-a3, b1, b2-b1, s, t)) { if (outLine) { float2 pt = s*(a1-a3); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a3, a1-a3, b2, b3-b2, s, t)) { if (outLine) { float2 pt = s*(a1-a3); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		if (LineSegment2DLineSegment2DIntersect(a3, a1-a3, b3, b1-b3, s, t)) { if (outLine) { float2 pt = s*(a1-a3); vec pt3d = a+pt.x*basisU+pt.y*basisV; *outLine = LineSegment(pt3d, pt3d); } return true; }
		return false;
	}
}

/// [groupSyntax]
bool Triangle::Intersects(const AABB &aabb) const
{
/** The AABB-Triangle test implementation is based on the pseudo-code in
	Christer Ericson's Real-Time Collision Detection, pp. 169-172. It is
	practically a standard SAT test. */
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// Benchmark 'Triangle_intersects_AABB': Triangle::Intersects(AABB)
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
	cmp = cmplt_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	if (!allzero_ps(cmp)) return false;

	// Note: the above semantics with cmplt_ps() vs cmple_ps() is quite delicate. cmple_ps might be usable
	// to define AABB and Triangle touching at a vertex to intersect, but it has troubles when the Triangle
	// is parallel to one of the cardinal axes. (see Triangle_Intersects_AABB_Case* tests)
	// To do the cmple_ps approach, would mask off results from the W channel and test if all were zero.
	// cmp = cmple_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	// if (!a_and_b_allzero_ps(cmp, set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU))) return false;

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
	cmp = cmplt_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	if (!allzero_ps(cmp)) return false;

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
	cmp = cmplt_ps(add_ps(r, abs_ps(sub_ps(tc, d1))), abs_ps(tc));
	return allzero_ps(cmp) != 0;
#else
	// Benchmark 'Triangle_intersects_AABB': Triangle::Intersects(AABB)
	//    Best: 17.282 nsecs / 46.496 ticks, Avg: 17.804 nsecs, Worst: 18.434 nsecs
	vec tMin = a.Min(b.Min(c));
	vec tMax = a.Max(b.Max(c));

	if (tMin.x >= aabb.maxPoint.x || tMax.x <= aabb.minPoint.x
		|| tMin.y >= aabb.maxPoint.y || tMax.y <= aabb.minPoint.y
		|| tMin.z >= aabb.maxPoint.z || tMax.z <= aabb.minPoint.z)
		return false;

	vec center = (aabb.minPoint + aabb.maxPoint) * 0.5f;
	vec h = aabb.maxPoint - center;

	const vec t[3] = { b-a, c-a, c-b };

	vec ac = a-center;

	vec n = Cross(t[0], t[1]);
	float s = n.Dot(ac);
	float r = Abs(h.Dot(n.Abs()));
	if (Abs(s) >= r)
		return false;

	const vec at[3] = { Abs(t[0]), Abs(t[1]), Abs(t[2]) };

	vec bc = b-center;
	vec cc = c-center;

	// SAT test all cross-axes.
	// The following is a fully unrolled loop of this code, stored here for reference:
	/*
	float d1, d2, a1, a2;
	const vec e[3] = { DIR_VEC(1, 0, 0), DIR_VEC(0, 1, 0), DIR_VEC(0, 0, 1) };
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			vec axis = Cross(e[i], t[j]);
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
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eX <cross> t[1]
	d1 = t[1].y * ac.z - t[1].z * ac.y;
	d2 = t[1].y * bc.z - t[1].z * bc.y;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[1].z + h.z * at[1].y);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eX <cross> t[2]
	d1 = t[2].y * ac.z - t[2].z * ac.y;
	d2 = t[2].y * bc.z - t[2].z * bc.y;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[2].z + h.z * at[2].y);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eY <cross> t[0]
	d1 = t[0].z * ac.x - t[0].x * ac.z;
	d2 = t[0].z * cc.x - t[0].x * cc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[0].z + h.z * at[0].x);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eY <cross> t[1]
	d1 = t[1].z * ac.x - t[1].x * ac.z;
	d2 = t[1].z * bc.x - t[1].x * bc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[1].z + h.z * at[1].x);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eY <cross> t[2]
	d1 = t[2].z * ac.x - t[2].x * ac.z;
	d2 = t[2].z * bc.x - t[2].x * bc.z;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.x * at[2].z + h.z * at[2].x);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eZ <cross> t[0]
	d1 = t[0].x * ac.y - t[0].y * ac.x;
	d2 = t[0].x * cc.y - t[0].y * cc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[0].x + h.x * at[0].y);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eZ <cross> t[1]
	d1 = t[1].x * ac.y - t[1].y * ac.x;
	d2 = t[1].x * bc.y - t[1].y * bc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[1].x + h.x * at[1].y);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// eZ <cross> t[2]
	d1 = t[2].x * ac.y - t[2].y * ac.x;
	d2 = t[2].x * bc.y - t[2].y * bc.x;
	tc = (d1 + d2) * 0.5f;
	r = Abs(h.y * at[2].x + h.x * at[2].y);
	if (r + Abs(tc - d1) < Abs(tc))
		return false;

	// No separating axis exists, the AABB and triangle intersect.
	return true;
#endif
}

bool Triangle::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool Triangle::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Triangle::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Triangle::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

bool Triangle::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

void Triangle::ProjectToAxis(const vec &axis, float &dMin, float &dMax) const
{
	dMin = dMax = Dot(axis, a);
	float t = Dot(axis, b);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
	t = Dot(axis, c);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
}

int Triangle::UniqueFaceNormals(vec *out) const
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

int Triangle::UniqueEdgeDirections(vec *out) const
{
	out[0] = b-a;
	out[1] = c-a;
	out[2] = c-b;
	return 3;
}

/// [groupSyntax]
vec Triangle::ClosestPoint(const vec &p) const
{
	/** The code for Triangle-float3 test is from Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

	// Check if P is in vertex region outside A.
	vec ab = b - a;
	vec ac = c - a;
	vec ap = p - a;
	float d1 = Dot(ab, ap);
	float d2 = Dot(ac, ap);
	if (d1 <= 0.f && d2 <= 0.f)
		return a; // Barycentric coordinates are (1,0,0).

	// Check if P is in vertex region outside B.
	vec bp = p - b;
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
	vec cp = p - c;
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

// This is a copy of Triangle::ClosestPoint(), except intermediate computation is done in doubles instead of floats
// for extra precision.
vec Triangle::ClosestPointD(const vec &p) const
{
	/** The code for Triangle-float3 test is from Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

	// Check if P is in vertex region outside A.
	float4d ab = POINT_TO_FLOAT4D(b) - POINT_TO_FLOAT4D(a);
	float4d ac = POINT_TO_FLOAT4D(c) - POINT_TO_FLOAT4D(a);
	float4d ap = POINT_TO_FLOAT4D(p) - POINT_TO_FLOAT4D(a);
	double d1 = ab.Dot(ap);
	double d2 = ac.Dot(ap);
	if (d1 <= 0.0 && d2 <= 0.0)
		return a; // Barycentric coordinates are (1,0,0).

	// Check if P is in vertex region outside B.
	float4d bp = POINT_TO_FLOAT4D(p) - POINT_TO_FLOAT4D(b);
	double d3 = ab.Dot(bp);
	double d4 = ac.Dot(bp);
	if (d3 >= 0.0 && d4 <= d3)
		return b; // Barycentric coordinates are (0,1,0).

	// Check if P is in edge region of AB, and if so, return the projection of P onto AB.
	double vc = d1*d4 - d3*d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
	{
		double v = d1 / (d1 - d3);
		return (POINT_TO_FLOAT4D(a) + v * ab).ToPointVec(); // The barycentric coordinates are (1-v, v, 0).
	}

	// Check if P is in vertex region outside C.
	float4d cp = POINT_TO_FLOAT4D(p) - POINT_TO_FLOAT4D(c);
	double d5 = ab.Dot(cp);
	double d6 = ac.Dot(cp);
	if (d6 >= 0.0 && d5 <= d6)
		return c; // The barycentric coordinates are (0,0,1).

	// Check if P is in edge region of AC, and if so, return the projection of P onto AC.
	double vb = d5*d2 - d1*d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
	{
		double w = d2 / (d2 - d6);
		return (POINT_TO_FLOAT4D(a) + w * ac).ToPointVec(); // The barycentric coordinates are (1-w, 0, w).
	}

	// Check if P is in edge region of BC, and if so, return the projection of P onto BC.
	double va = d3*d6 - d5*d4;
	if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0)
	{
		double w = (d4 - d3) / (d4 - d3 + d5 - d6);
		return (POINT_TO_FLOAT4D(b) + w * (POINT_TO_FLOAT4D(c) - POINT_TO_FLOAT4D(b))).ToPointVec(); // The barycentric coordinates are (0, 1-w, w).
	}

	// P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
	double denom = 1.f / (va + vb + vc);
	double v = vb * denom;
	double w = vc * denom;
	return (POINT_TO_FLOAT4D(a) + ab * v + ac * w).ToPointVec();
}

vec Triangle::ClosestPoint(const LineSegment &lineSegment, vec *otherPt) const
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
	vec pt1 = ClosestPointToTriangleEdge(lineSegment, &u1, &v1, &d1);

	vec pt2 = ClosestPoint(lineSegment.a);
	vec pt3 = ClosestPoint(lineSegment.b);
	
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

#if 0
///\todo Enable this codepath. This if rom Geometric Tools for Computer Graphics,
/// but the algorithm in the book is broken and does not take into account the
/// direction of the gradient to determine the proper region of intersection.
/// Instead using a slower code path above.
/// [groupSyntax]
vec Triangle::ClosestPoint(const LineSegment &lineSegment, vec *otherPt) const
{
	vec e0 = b - a;
	vec e1 = c - a;
	vec v_p = a - lineSegment.a;
	vec d = lineSegment.b - lineSegment.a;

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
		LineSegment e1 = Edge(0);
		LineSegment e2 = Edge(1);
		LineSegment e3 = Edge(2);
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
vec Triangle::ClosestPointToTriangleEdge(const Line &other, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float unused1, unused2, unused3;
	float d1, d2, d3;
	vec pt1 = Edge(0).ClosestPoint(other, unused1, d1);
	vec pt2 = Edge(1).ClosestPoint(other, unused2, d2);
	vec pt3 = Edge(2).ClosestPoint(other, unused3, d3);
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

vec Triangle::ClosestPointToTriangleEdge(const LineSegment &lineSegment, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float unused1, unused2, unused3;
	float d1, d2, d3;
	vec pt1 = Edge(0).ClosestPoint(lineSegment, unused1, d1);
	vec pt2 = Edge(1).ClosestPoint(lineSegment, unused2, d2);
	vec pt3 = Edge(2).ClosestPoint(lineSegment, unused3, d3);
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

vec Triangle::ClosestPoint(const Line &line, vec *otherPt) const
{
	///\todo Optimize this function.
	vec intersectionPoint;
	if (Intersects(line, 0, &intersectionPoint))
	{
		if (otherPt)
			*otherPt = intersectionPoint;
		return intersectionPoint;
	}

	float u1,v1,d1;
	vec pt1 = ClosestPointToTriangleEdge(line, &u1, &v1, &d1);
	if (otherPt)
		*otherPt = line.GetPoint(d1);
	return pt1;
}

#if 0
///\todo Enable this codepath. This if rom Geometric Tools for Computer Graphics,
/// but the algorithm in the book is broken and does not take into account the
/// direction of the gradient to determine the proper region of intersection.
/// Instead using a slower code path above.
vec Triangle::ClosestPoint(const Line &line, vec *otherPt) const
{
	vec e0 = b - a;
	vec e1 = c - a;
	vec v_p = a - line.pos;
	vec d = line.dir;

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
		LineSegment e1 = Edge(0);
		LineSegment e2 = Edge(1);
		LineSegment e3 = Edge(2);
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
vec Triangle::ClosestPoint(const Line &other, float *outU, float *outV, float *outD) const
{
	/** The implementation of the Triangle-Line test is based on the pseudo-code in
		Schneider, Eberly. Geometric Tools for Computer Graphics pp. 433 - 441. */
	///@todo The Triangle-Line code is currently untested. Run tests to ensure the following code works properly.

	// Point on triangle: T(u,v) = a + u*b + v*c;
	// Point on line:  L(t) = p + t*d;
	// Minimize the function Q(u,v,t) = ||T(u,v) - L(t)||.

	vec e0 = b-a;
	vec e1 = c-a;
	vec d = other.dir;

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

	vec v_m_p = a - other.pos;
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
		// 2x2 * 2 matrix*vec mul.
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

/// [groupSyntax]
vec Triangle::ClosestPoint(const Triangle &other, vec *otherPt) const
{
	/** The code for computing the closest point pair on two Triangles is based
		on pseudo-code from Christer Ericson's Real-Time Collision Detection, pp. 155-156. */

	// First detect if the two triangles are intersecting.
	LineSegment l;
	bool success = this->Intersects(other, &l);
	if (success)
	{
		vec cp = l.CenterPoint();
		if (otherPt)
			*otherPt = cp;
		return cp;
	}

	vec closestThis = this->ClosestPoint(other.a);
	vec closestOther = other.a;
	float closestDSq = closestThis.DistanceSq(closestOther);

	vec pt = this->ClosestPoint(other.b);
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

	LineSegment l1[3] = { LineSegment(a,b), LineSegment(a,c), LineSegment(b,c) };
	LineSegment l2[3] = { LineSegment(other.a,other.b), LineSegment(other.a,other.c), LineSegment(other.b,other.c) };
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

vec Triangle::RandomPointInside(LCG &rng) const
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
	vec pt = Point(s, t);
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

vec Triangle::RandomVertex(LCG &rng) const
{
	return Vertex(rng.Int(0, 2));
}

vec Triangle::RandomPointOnEdge(LCG &rng) const
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

Triangle operator *(const float3x3 &transform, const Triangle &triangle)
{
	Triangle t(triangle);
	t.Transform(transform);
	return t;
}

Triangle operator *(const float3x4 &transform, const Triangle &triangle)
{
	Triangle t(triangle);
	t.Transform(transform);
	return t;
}

Triangle operator *(const float4x4 &transform, const Triangle &triangle)
{
	Triangle t(triangle);
	t.Transform(transform);
	return t;
}

Triangle operator *(const Quat &transform, const Triangle &triangle)
{
	Triangle t(triangle);
	t.Transform(transform);
	return t;
}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT Triangle::ToString() const
{
	char str[256];
	sprintf(str, "Triangle(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f) c:(%.2f, %.2f, %.2f))",
		a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
	return str;
}

StringT Triangle::SerializeToString() const
{
	char str[256];
	char *s = SerializeFloat(a.x, str); *s = ','; ++s;
	s = SerializeFloat(a.y, s); *s = ','; ++s;
	s = SerializeFloat(a.z, s); *s = ','; ++s;
	s = SerializeFloat(b.x, s); *s = ','; ++s;
	s = SerializeFloat(b.y, s); *s = ','; ++s;
	s = SerializeFloat(b.z, s); *s = ','; ++s;
	s = SerializeFloat(c.x, s); *s = ','; ++s;
	s = SerializeFloat(c.y, s); *s = ','; ++s;
	s = SerializeFloat(c.z, s);
	assert(s+1 - str < 256);
	MARK_UNUSED(s);
	return str;
}

StringT Triangle::SerializeToCodeString() const
{
	return "Triangle(" + a.SerializeToCodeString() + "," + b.SerializeToCodeString() + "," + c.SerializeToCodeString() + ")";
}
#endif

#if defined(MATH_ENABLE_STL_SUPPORT)

std::ostream &operator <<(std::ostream &o, const Triangle &triangle)
{
	o << triangle.ToString();
	return o;
}

#endif

Triangle Triangle::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return Triangle(vec::nan, vec::nan, vec::nan);
	Triangle t;
	MATH_SKIP_WORD(str, "Triangle(");
	MATH_SKIP_WORD(str, "a:(");
	t.a = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " b:(");
	t.b = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " c:(");
	t.c = PointVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return t;
}

MATH_END_NAMESPACE
