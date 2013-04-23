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
#include "../Math/float2.h"
#include "../Math/float3.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
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

MATH_BEGIN_NAMESPACE

Triangle::Triangle(const float3 &a_, const float3 &b_, const float3 &c_)
:a(a_), b(b_), c(c_)
{
}

void Triangle::Translate(const float3 &offset)
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

float3 Triangle::BarycentricUVW(const float3 &point) const
{
	// Implementation from Christer Ericson's Real-Time Collision Detection, pp. 51-52.

	// Unnormalized triangle normal.
	float3 m = Cross(b-a, c-a);

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
	float3 v0 = b - a;
	float3 v1 = c - a;
	float3 v2 = point - a;
	float d00 = Dot(v0, v0);
	float d01 = Dot(v0, v1);
	float d02 = Dot(v0, v2);
	float d11 = Dot(v1, v1);
	float d12 = Dot(v1, v2);
	float denom = 1.f / (d00 * d11 - d01 * d01);
	float v = (d11 * d02 - d01 * d12) * denom;
	float w = (d00 * d12 - d01 * d02) * denom;
	float u = 1.0f - v - w;
	return float3(u, v, w);
#endif
}

float2 Triangle::BarycentricUV(const float3 &point) const
{
	float3 uvw = BarycentricUVW(point);
	return float2(uvw.y, uvw.z);
}

bool Triangle::BarycentricInsideTriangle(const float3 &barycentric)
{
	return barycentric.x >= 0.f && barycentric.y >= 0.f && barycentric.z >= 0.f &&
		EqualAbs(barycentric.x + barycentric.y + barycentric.z, 1.f);
}

float3 Triangle::Point(float u, float v) const
{
	return a + (b-a) * u + (c-a) * v;
}

float3 Triangle::Point(float u, float v, float w) const
{
	return u * a + v * b + w * c;
}

float3 Triangle::Point(const float3 &b) const
{
	return Point(b.x, b.y, b.z);
}

float3 Triangle::Point(const float2 &b) const
{
	return Point(b.x, b.y);
}

float3 Triangle::Centroid() const
{
	return (a + b + c) / 3.f;
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
		return LineSegment(float3::nan, float3::nan);
}

float3 Triangle::Vertex(int i) const
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
		return float3::nan;
}

Plane Triangle::PlaneCCW() const
{
	return Plane(a, b, c);
}

Plane Triangle::PlaneCW() const
{
	return Plane(a, c, b);
}

float3 Triangle::NormalCCW() const
{
	return UnnormalizedNormalCCW().Normalized();
}

float3 Triangle::NormalCW() const
{
	return UnnormalizedNormalCW().Normalized();
}

float3 Triangle::UnnormalizedNormalCCW() const
{
	return Cross(b-a, c-a);
}

float3 Triangle::UnnormalizedNormalCW() const
{
	return Cross(c-a, b-a);
}

float3 Triangle::ExtremePoint(const float3 &direction) const
{
	float3 mostExtreme = float3::nan;
	float mostExtremeDist = -FLT_MAX;
	for(int i = 0; i < 3; ++i)
	{
		float3 pt = Vertex(i);
		float d = Dot(direction, pt);
		if (d > mostExtremeDist)
		{
			mostExtremeDist = d;
			mostExtreme = pt;
		}
	}
	return mostExtreme;
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
	aabb.SetNegativeInfinity();
	aabb.Enclose(a);
	aabb.Enclose(b);
	aabb.Enclose(c);
	return aabb;
}

float Triangle::Area2D(const float2 &p1, const float2 &p2, const float2 &p3)
{
	return (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
}

float Triangle::SignedArea(const float3 &pt, const float3 &a, const float3 &b, const float3 &c)
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

bool Triangle::IsDegenerate(const float3 &a, const float3 &b, const float3 &c, float epsilon)
{
	return a.Equals(b, epsilon) || a.Equals(c, epsilon) || b.Equals(c, epsilon);
}

bool Triangle::Contains(const float3 &point, float triangleThickness) const
{
	if (PlaneCCW().Distance(point) > triangleThickness) // The winding order of the triangle plane does not matter.
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
float Triangle::Distance(const float3 &point) const
{
	return ClosestPoint(point).Distance(point);
}

float Triangle::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Triangle::Distance(const Capsule &capsule) const
{
	float3 otherPt;
	float3 thisPt = ClosestPoint(capsule.l, &otherPt);
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
float Triangle::IntersectLineTri(const float3 &linePos, const float3 &lineDir,
		const float3 &v0, const float3 &v1, const float3 &v2,
		float &u, float &v)
{
	float3 vE1, vE2;
	float3 vT, vP, vQ;

	const float epsilon = 1e-4f;

	// Edge vectors
	vE1 = v1 - v0;
	vE2 = v2 - v0;

	// begin calculating determinant - also used to calculate U parameter
	vP = lineDir.Cross(vE2);

	// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
	const float det = vE1.Dot(vP);

	// If determinant is near zero, ray lies in plane of triangle.
	if (fabs(det) <= epsilon)
		return FLOAT_INF;
	const float recipDet = 1.f / det;

	// Calculate distance from v0 to ray origin
	vT = linePos - v0;

	// Output barycentric u
	u = vT.Dot(vP) * recipDet;
	if (u < -epsilon || u > 1.f + epsilon)
		return FLOAT_INF; // Barycentric U is outside the triangle - early out.

	// Prepare to test V parameter
	vQ = vT.Cross(vE1);

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
bool Triangle::Intersects(const LineSegment &l, float *d, float3 *intersectionPoint) const
{
	/** The Triangle-Line/LineSegment/Ray intersection tests are based on M&ouml;ller-Trumbore method:
		"T. M&ouml;ller, B. Trumbore. Fast, Minimum Storage Ray/Triangle Intersection. 2005."
		http://jgt.akpeters.com/papers/MollerTrumbore97/. */
	float u, v;
	float t = IntersectLineTri(l.a, l.Dir(), a, b, c, u, v);
	bool success = (t >= 0 && t != FLOAT_INF);
	if (!success)
		return false;
	float length = l.LengthSq();
	if (t < 0.f || t*t >= length)
		return false;
	length = Sqrt(length);
	if (d)
	{
		float len = t / length;
		*d = len;
		if (intersectionPoint)
			*intersectionPoint = l.GetPoint(len);
	}
	else if (intersectionPoint)
		*intersectionPoint = l.GetPoint(t / length);
	return true;
}

bool Triangle::Intersects(const Line &l, float *d, float3 *intersectionPoint) const
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

bool Triangle::Intersects(const Ray &r, float *d, float3 *intersectionPoint) const
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
bool Triangle::Intersects(const Sphere &sphere, float3 *closestPointOnTriangle) const
{
	float3 pt = ClosestPoint(sphere.pos);

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
	// Is the triangle t2 completely on one side of the plane of this triangle?
	Plane p1 = this->PlaneCCW();
	float t2da = p1.SignedDistance(t2.a);
	float t2db = p1.SignedDistance(t2.b);
	float t2dc = p1.SignedDistance(t2.c);
	if (t2da*t2db > 0.f && t2da*t2dc > 0.f)
		return false;
	// Is this triangle completely on one side of the plane of the triangle t2?
	Plane p2 = t2.PlaneCCW();
	float t1da = p2.SignedDistance(this->a);
	float t1db = p2.SignedDistance(this->b);
	float t1dc = p2.SignedDistance(this->c);
	if (t1da*t1db > 0.f && t1da*t1dc > 0.f)
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
	l.Distance(l1a, &d1a);
	l.Distance(l1b, &d1b);
	l.Distance(l2a, &d2a);
	l.Distance(l2b, &d2b);
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

bool RangesOverlap(float start1, float end1, float start2, float end2)
{
	return end1 >= start2 && end2 >= start1;
}

/// [groupSyntax]
bool Triangle::Intersects(const AABB &aabb) const
{
/** The AABB-Triangle test implementation is based on the pseudo-code in
	Christer Ericson's Real-Time Collision Detection, pp. 169-172. */
	///@todo The Triangle-AABB intersection test can be greatly optimized by manually unrolling loops, trivial math and by avoiding
	/// unnecessary copying.
	float t1, t2, a1, a2;
	const float3 e[3] = { float3(1,0,0), float3(0,1,0), float3(0,0,1) };

	for(int i = 0; i < 3; ++i)
	{
		ProjectToAxis(e[i], t1, t2);
		aabb.ProjectToAxis(e[i], a1, a2);
		if (!RangesOverlap(t1, t2, a1, a2))
			return false;
	}

	float3 n = UnnormalizedNormalCCW();
	ProjectToAxis(n, t1, t2);
	aabb.ProjectToAxis(n, a1, a2);
	if (!RangesOverlap(t1, t2, a1, a2))
		return false;

	const float3 t[3] = { b-a, c-a, c-b };

	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			float3 axis = Cross(e[i], t[j]);
			float len = axis.LengthSq();
			if (len <= 1e-4f)
				continue; // Ignore tests on degenerate axes.

			ProjectToAxis(axis, t1, t2);
			aabb.ProjectToAxis(axis, a1, a2);
			if (!RangesOverlap(t1, t2, a1, a2))
				return false;
		}

	// No separating axis exists, the AABB and triangle intersect.
	return true;
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

void Triangle::ProjectToAxis(const float3 &axis, float &dMin, float &dMax) const
{
	dMin = dMax = Dot(axis, a);
	float t = Dot(axis, b);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
	t = Dot(axis, c);
	dMin = Min(t, dMin);
	dMax = Max(t, dMax);
}

/// [groupSyntax]
float3 Triangle::ClosestPoint(const float3 &p) const
{
	/** The code for Triangle-float3 test is from Christer Ericson's Real-Time Collision Detection, pp. 141-142. */

	// Check if P is in vertex region outside A.
	float3 ab = b - a;
	float3 ac = c - a;
	float3 ap = p - a;
	float d1 = Dot(ab, ap);
	float d2 = Dot(ac, ap);
	if (d1 <= 0.f && d2 <= 0.f)
		return a; // Barycentric coordinates are (1,0,0).

	// Check if P is in vertex region outside B.
	float3 bp = p - b;
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
	float3 cp = p - c;
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

float3 Triangle::ClosestPoint(const LineSegment &lineSegment, float3 *otherPt) const
{
	///\todo Optimize.
	float3 intersectionPoint;
	if (Intersects(lineSegment, 0, &intersectionPoint))
	{
		if (otherPt)
			*otherPt = intersectionPoint;
		return intersectionPoint;
	}

	float u1,v1,d1;
	float3 pt1 = ClosestPointToTriangleEdge(lineSegment, &u1, &v1, &d1);

	float3 pt2 = ClosestPoint(lineSegment.a);
	float3 pt3 = ClosestPoint(lineSegment.b);
	
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
float3 Triangle::ClosestPoint(const LineSegment &lineSegment, float3 *otherPt) const
{
	float3 e0 = b - a;
	float3 e1 = c - a;
	float3 v_p = a - lineSegment.a;
	float3 d = lineSegment.b - lineSegment.a;

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
float3 Triangle::ClosestPointToTriangleEdge(const Line &other, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float d1, d2, d3;
	float3 pt1 = Edge(0).ClosestPoint(other, 0, &d1);
	float3 pt2 = Edge(1).ClosestPoint(other, 0, &d2);
	float3 pt3 = Edge(2).ClosestPoint(other, 0, &d3);
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

float3 Triangle::ClosestPointToTriangleEdge(const LineSegment &lineSegment, float *outU, float *outV, float *outD) const
{
	///@todo Optimize!
	// The line is parallel to the triangle.
	float d1, d2, d3;
	float3 pt1 = Edge(0).ClosestPoint(lineSegment, 0, &d1);
	float3 pt2 = Edge(1).ClosestPoint(lineSegment, 0, &d2);
	float3 pt3 = Edge(2).ClosestPoint(lineSegment, 0, &d3);
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

float3 Triangle::ClosestPoint(const Line &line, float3 *otherPt) const
{
	///\todo Optimize this function.
	float3 intersectionPoint;
	if (Intersects(line, 0, &intersectionPoint))
	{
		if (otherPt)
			*otherPt = intersectionPoint;
		return intersectionPoint;
	}

	float u1,v1,d1;
	float3 pt1 = ClosestPointToTriangleEdge(line, &u1, &v1, &d1);
	if (otherPt)
		*otherPt = line.GetPoint(d1);
	return pt1;
}

#if 0
///\todo Enable this codepath. This if rom Geometric Tools for Computer Graphics,
/// but the algorithm in the book is broken and does not take into account the
/// direction of the gradient to determine the proper region of intersection.
/// Instead using a slower code path above.
float3 Triangle::ClosestPoint(const Line &line, float3 *otherPt) const
{
	float3 e0 = b - a;
	float3 e1 = c - a;
	float3 v_p = a - line.pos;
	float3 d = line.dir;

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
float3 Triangle::ClosestPoint(const Line &other, float *outU, float *outV, float *outD) const
{
	/** The implementation of the Triangle-Line test is based on the pseudo-code in
		Schneider, Eberly. Geometric Tools for Computer Graphics pp. 433 - 441. */
	///@todo The Triangle-Line code is currently untested. Run tests to ensure the following code works properly.

	// Point on triangle: T(u,v) = a + u*b + v*c;
	// Point on line:  L(t) = p + t*d;
	// Minimize the function Q(u,v,t) = ||T(u,v) - L(t)||.

	float3 e0 = b-a;
	float3 e1 = c-a;
	float3 d = other.dir;

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

	float3 v_m_p = a - other.pos;
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
float3 Triangle::ClosestPoint(const Triangle &other, float3 *otherPt) const
{
	/** The code for computing the closest point pair on two Triangles is based
		on pseudo-code from Christer Ericson's Real-Time Collision Detection, pp. 155-156. */

	// First detect if the two triangles are intersecting.
	LineSegment l;
	bool success = this->Intersects(other, &l);
	if (success)
	{
		float3 cp = l.CenterPoint();
		if (otherPt)
			*otherPt = cp;
		return cp;
	}

	float3 closestThis = this->ClosestPoint(other.a);
	float3 closestOther = other.a;
	float closestDSq = closestThis.DistanceSq(closestOther);

	float3 pt = this->ClosestPoint(other.b);
	float dSq = pt.DistanceSq(other.b);
	if (dSq < closestDSq) closestThis = pt, closestOther = other.b, closestDSq = dSq;

	pt = this->ClosestPoint(other.c);
	dSq = pt.DistanceSq(other.c);
	if (dSq < closestDSq) closestThis = pt, closestOther = other.c, closestDSq = dSq;

	LineSegment l1[3] = { LineSegment(a,b), LineSegment(a,c), LineSegment(b,c) };
	LineSegment l2[3] = { LineSegment(other.a,other.b), LineSegment(other.a,other.c), LineSegment(other.b,other.c) };
	float d, d2;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
		{
			float dist = l1[i].Distance(l2[j], &d, &d2);
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

float3 Triangle::RandomPointInside(LCG &rng) const
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
	float3 pt = Point(s, t);
	float2 uv = BarycentricUV(pt);
	assert(uv.x >= 0.f);
	assert(uv.y >= 0.f);
	assert(uv.x + uv.y <= 1.f);
	float3 uvw = BarycentricUVW(pt);
	assert(uvw.x >= 0.f);
	assert(uvw.y >= 0.f);
	assert(uvw.z >= 0.f);
	assert(EqualAbs(uvw.x + uvw.y + uvw.z, 1.f));
#endif
	return Point(s, t);
}

float3 Triangle::RandomVertex(LCG &rng) const
{
	return Vertex(rng.Int(0, 2));
}

float3 Triangle::RandomPointOnEdge(LCG &rng) const
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

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Triangle::ToString() const
{
	char str[256];
	sprintf(str, "Triangle(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f) c:(%.2f, %.2f, %.2f))",
		a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Triangle &triangle)
{
	o << triangle.ToString();
	return o;
}

#endif

MATH_END_NAMESPACE
