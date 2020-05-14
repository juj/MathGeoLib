/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Polygon.cpp
	@author Jukka Jyl�nki
	@brief Implementation for the Polygon geometry object. */
#include "Polygon.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include "../Math/myassert.h"
#include <utility>
#include <list>
#endif

#include "AABB.h"
#include "OBB.h"
#include "Frustum.h"
#include "Polyhedron.h"
#include "Plane.h"
#include "Line.h"
#include "Ray.h"
#include "LineSegment.h"
#include "Triangle.h"
#include "Sphere.h"
#include "../Algorithm/Random/LCG.h"
#include "../Math/MathFunc.h"
#include "../Math/Swap.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "../Math/float2.h"
#include "../Math/MathConstants.h"
#include "../Algorithm/GJK.h"

MATH_BEGIN_NAMESPACE

int Polygon::NumVertices() const
{
	return (int)p.size();
}

int Polygon::NumEdges() const
{
	return (int)p.size();
}

vec Polygon::Vertex(int vertexIndex) const
{
	assume(vertexIndex >= 0);
	assume(vertexIndex < (int)p.size());
	return p[vertexIndex];
}

LineSegment Polygon::Edge(int i) const
{
	if (p.empty())
		return LineSegment(vec::nan, vec::nan);
	if (p.size() == 1)
		return LineSegment(p[0], p[0]);
	return LineSegment(p[i], p[(i+1)%p.size()]);
}

LineSegment Polygon::Edge2D(int i) const
{
	if (p.empty())
		return LineSegment(vec::nan, vec::nan);
	if (p.size() == 1)
		return LineSegment(vec::zero, vec::zero);
	return LineSegment(POINT_VEC(MapTo2D(i), 0), POINT_VEC(MapTo2D((i+1)%p.size()), 0));
}

bool Polygon::DiagonalExists(int i, int j) const
{
	assume(p.size() >= 3);
	assume(i >= 0);
	assume(j >= 0);
	assume(i < (int)p.size());
	assume(j < (int)p.size());
	assume(IsPlanar());
	assume(i != j);
	if (i == j) // Degenerate if i == j.
		return false;
	if (i > j)
		Swap(i, j);
	assume(i+1 != j);
	if (i+1 == j) // Is this LineSegment an edge of this polygon?
		return false;

	Plane polygonPlane = PlaneCCW();
	LineSegment diagonal = polygonPlane.Project(LineSegment(p[i], p[j]));

	// First check that this diagonal line is not intersected by an edge of this polygon.
	for(int k = 0; k < (int)p.size(); ++k)
		if (!(k == i || k+1 == i || k == j))
			if (polygonPlane.Project(LineSegment(p[k], p[k+1])).Intersects(diagonal))
				return false;

	return IsConvex();
}

vec Polygon::BasisU() const
{
	if (p.size() < 2)
		return vec::unitX;
	vec u = (vec)p[1] - (vec)p[0];
	u.Normalize(); // Always succeeds, even if u was zero (generates (1,0,0)).
	return u;
}

vec Polygon::BasisV() const
{
	if (p.size() < 2)
		return vec::unitY;
	return Cross(PlaneCCW().normal, BasisU()).Normalized();
}

LineSegment Polygon::Diagonal(int i, int j) const
{
	assume(i >= 0);
	assume(j >= 0);
	assume(i < (int)p.size());
	assume(j < (int)p.size());
	return LineSegment(p[i], p[j]);
}

bool Polygon::IsConvex() const
{
	assume(IsPlanar());
	if (p.empty())
		return false;
	if (p.size() <= 3)
		return true;
	int i = (int)p.size()-2;
	int j = (int)p.size()-1;
	int k = 0;

	while(k < (int)p.size())
	{
		float2 a = MapTo2D(i);
		float2 b = MapTo2D(j);
		float2 c = MapTo2D(k);
		if (!float2::OrientedCCW(a, b, c))
			return false;
		i = j;
		j = k;
		++k;
	}
	return true;
}

float2 Polygon::MapTo2D(int i) const
{
	assume(i >= 0);
	assume(i < (int)p.size());
	return MapTo2D(p[i]);
}

float2 Polygon::MapTo2D(const vec &point) const
{
	assume(!p.empty());
	vec basisU = BasisU();
	vec basisV = BasisV();
	vec pt = point - p[0];
	return float2(Dot(pt, basisU), Dot(pt, basisV));
}

vec Polygon::MapFrom2D(const float2 &point) const
{
	assume(!p.empty());
	return (vec)p[0] + point.x * BasisU() + point.y * BasisV();
}

bool Polygon::IsPlanar(float epsilonSq) const
{
	if (p.empty())
		return false;
	if (p.size() <= 3)
		return true;
	vec normal = (vec(p[1])-vec(p[0])).Cross(vec(p[2])-vec(p[0]));
	float lenSq = normal.LengthSq();
	for(size_t i = 3; i < p.size(); ++i)
	{
		float d = normal.Dot(vec(p[i])-vec(p[0]));
		if (d*d > epsilonSq * lenSq)
			return false;
	}
	return true;
}

bool Polygon::IsSimple() const
{
	assume(IsPlanar());
	Plane plane = PlaneCCW();
	for(int i = 0; i < (int)p.size(); ++i)
	{
		LineSegment si = plane.Project(Edge(i));
		for(int j = i+2; j < (int)p.size(); ++j)
		{
			if (i == 0 && j == (int)p.size() - 1)
				continue; // These two edges are consecutive and share a vertex. Don't check that pair.
			LineSegment sj = plane.Project(Edge(j));
			if (si.Intersects(sj))
				return false;
		}
	}
	return true;
}

bool Polygon::IsNull() const
{
	return p.empty();
}

bool Polygon::IsFinite() const
{
	for(size_t i = 0; i < p.size(); ++i)
		if (!((vec)p[i]).IsFinite())
			return false;

	return true;
}

bool Polygon::IsDegenerate(float epsilon) const
{
	return p.size() < 3 || Area() <= epsilon;
}

vec Polygon::NormalCCW() const
{
	///@todo Optimize temporaries.
	return PlaneCCW().normal;
}

vec Polygon::NormalCW() const
{
	///@todo Optimize temporaries.
	return PlaneCW().normal;
}

Plane Polygon::PlaneCCW() const
{
	if (p.size() > 3)
	{
		Plane plane;
		for(size_t i = 0; i < p.size()-2; ++i)
			for(size_t j = i+1; j < p.size()-1; ++j)
			{
				vec pij = vec(p[j])-vec(p[i]);
				for(size_t k = j+1; k < p.size(); ++k)
				{
					plane.normal = pij.Cross(vec(p[k])-vec(p[i]));
					float lenSq = plane.normal.LengthSq();
					if (lenSq > 1e-8f)
					{
						plane.normal /= Sqrt(lenSq);
						plane.d = plane.normal.Dot(vec(p[i]));
						return plane;
					}
				}
			}

#ifndef MATH_SILENT_ASSUME
#ifdef MATH_ENABLE_STL_SUPPORT
		LOGW("Polygon contains %d points, but they are all collinear! Cannot form a plane for the Polygon using three points! %s", (int)p.size(), this->SerializeToString().c_str());
#else
		// TODO: enable above
		LOGW("Polygon contains %d points, but they are all collinear! Cannot form a plane for the Polygon using three points!", (int)p.size());
#endif
#endif
		// Polygon contains multiple points, but they are all collinear.
		// Pick an arbitrary plane along the line as the polygon plane (as if the polygon had only two points)
		vec dir = (vec(p[1])-vec(p[0])).Normalized();
		return Plane(Line(p[0], dir), dir.Perpendicular());
	}
	if (p.size() == 3)
		return Plane(p[0], p[1], p[2]);
	if (p.size() == 2)
	{
		vec dir = (vec(p[1])-vec(p[0])).Normalized();
		return Plane(Line(p[0], dir), dir.Perpendicular());
	}
	if (p.size() == 1)
		return Plane(p[0], DIR_VEC(0,1,0));
	return Plane();
}

Plane Polygon::PlaneCW() const
{
	Plane plane = PlaneCCW();
	plane.ReverseNormal();
	return plane;
}

void Polygon::Translate(const vec &offset)
{
	for(size_t i = 0; i < p.size(); ++i)
		p[i] = (vec)p[i] + offset;
}

void Polygon::Transform(const float3x3 &transform)
{
	if (!p.empty())
		transform.BatchTransform((vec*)&p[0], (int)p.size());
}

void Polygon::Transform(const float3x4 &transform)
{
	if (!p.empty())
		transform.BatchTransformPos((vec*)&p[0], (int)p.size());
}

void Polygon::Transform(const float4x4 &transform)
{
	for(size_t i = 0; i < p.size(); ++i)
		p[i] = transform.MulPos(p[i]);
}

void Polygon::Transform(const Quat &transform)
{
	for(size_t i = 0; i < p.size(); ++i)
		p[i] = transform * p[i];
}

bool Polygon::Contains(const Polygon &worldSpacePolygon, float polygonThickness) const
{
	for(int i = 0; i < worldSpacePolygon.NumVertices(); ++i)
		if (!Contains(worldSpacePolygon.Vertex(i), polygonThickness))
			return false;
	return true;
}

bool Polygon::Contains(const vec &worldSpacePoint, float polygonThicknessSq) const
{
	// Implementation based on the description from http://erich.realtimerendering.com/ptinpoly/

	if (p.size() < 3)
		return false;

	vec basisU = BasisU();
	vec basisV = BasisV();
	assert1(basisU.IsNormalized(), basisU);
	assert1(basisV.IsNormalized(), basisV);
	assert2(basisU.IsPerpendicular(basisV), basisU, basisV);
	assert3(basisU.IsPerpendicular(PlaneCCW().normal), basisU, PlaneCCW().normal, basisU.Dot(PlaneCCW().normal));
	assert3(basisV.IsPerpendicular(PlaneCCW().normal), basisV, PlaneCCW().normal, basisV.Dot(PlaneCCW().normal));

	vec normal = basisU.Cross(basisV);
//	float lenSq = normal.LengthSq(); ///\todo Could we treat basisU and basisV unnormalized here?
	float dot = normal.Dot(vec(p[0]) - worldSpacePoint);
	if (dot*dot > polygonThicknessSq)
		return false; // The point is not even within the plane of the polygon - can't be contained.

	int numIntersections = 0;

	const float epsilon = 1e-4f;

	// General strategy: transform all points on the polygon onto 2D face plane of the polygon, where the target query point is 
	// centered to lie in the origin.
	// If the test ray (0,0) -> (+inf, 0) intersects exactly an odd number of polygon edge segments, then the query point must have been
	// inside the polygon. The test ray is chosen like that to avoid all extra per-edge computations.
	// This method works for both simple and non-simple (self-intersecting) polygons.
	vec vt = vec(p.back()) - worldSpacePoint;
	float2 p0 = float2(Dot(vt, basisU), Dot(vt, basisV));
	if (Abs(p0.y) < epsilon)
		p0.y = -epsilon; // Robustness check - if the ray (0,0) -> (+inf, 0) would pass through a vertex, move the vertex slightly.

	for(int i = 0; i < (int)p.size(); ++i)
	{
		vt = vec(p[i]) - worldSpacePoint;
		float2 p1 = float2(Dot(vt, basisU), Dot(vt, basisV));
		if (Abs(p1.y) < epsilon)
			p1.y = -epsilon; // Robustness check - if the ray (0,0) -> (+inf, 0) would pass through a vertex, move the vertex slightly.

		if (p0.y * p1.y < 0.f) // If the line segment p0 -> p1 straddles the line x=0, it could intersect the ray (0,0) -> (+inf, 0)
		{
			if (Min(p0.x, p1.x) > 0.f) // If both x-coordinates are positive, then there certainly is an intersection with the ray.
				++numIntersections;
			else if (Max(p0.x, p1.x) > 0.f) // If one of them is positive, there could be an intersection. (otherwise both are negative and they can't intersect ray)
			{
				// P = p0 + t*(p1-p0) == (x,0)
				//     p0.x + t*(p1.x-p0.x) == x
				//     p0.y + t*(p1.y-p0.y) == 0
				//                 t == -p0.y / (p1.y - p0.y)

				// Test whether the lines (0,0) -> (+inf,0) and p0 -> p1 intersect at a positive X-coordinate?
				float2 d = p1 - p0;
				if (d.y != 0.f)
				{
					float t = -p0.y / d.y; // The line segment parameter, t \in [0,1] forms the line segment p0->p1.
					float x = p0.x + t * d.x; // The x-coordinate of intersection with the ray.
					if (t >= 0.f && t <= 1.f && x > 0.f)
						++numIntersections;
				}
			}
		}
		p0 = p1;
	}

	return numIntersections % 2 == 1;
}

bool Polygon::Contains(const LineSegment &worldSpaceLineSegment, float polygonThickness) const
{
	if (p.size() < 3)
		return false;

	Plane plane = PlaneCCW();
	if (plane.Distance(worldSpaceLineSegment.a) > polygonThickness ||
		plane.Distance(worldSpaceLineSegment.b) > polygonThickness)
		return false;

	// For robustness, project onto the polygon plane.
	LineSegment l = plane.Project(worldSpaceLineSegment);

	if (!Contains(l.a) || !Contains(l.b))
		return false;

	for(int i = 0; i < (int)p.size(); ++i)
		if (plane.Project(Edge(i)).Intersects(l))
			return false;

	return true;
}

bool Polygon::Contains(const Triangle &worldSpaceTriangle, float polygonThickness) const
{
	return Contains(worldSpaceTriangle.Edge(0), polygonThickness) &&
		Contains(worldSpaceTriangle.Edge(1), polygonThickness) &&
		Contains(worldSpaceTriangle.Edge(2), polygonThickness);
}

bool Polygon::Contains2D(const LineSegment &localSpaceLineSegment) const
{
	if (p.size() < 3)
		return false;

	const vec basisU = BasisU();
	const vec basisV = BasisV();
	const vec origin = p[0];

	LineSegment edge;
	edge.a = POINT_VEC(Dot(p.back(), basisU), Dot(p.back(), basisV), 0); // map to 2D
	for (int i = 0; i < (int)p.size(); ++i)
	{
		edge.b = POINT_VEC(Dot(p[i], basisU), Dot(p[i], basisV), 0); // map to 2D
		if (edge.Intersects(localSpaceLineSegment))
			return false;
		edge.a = edge.b;
	}

	// The line segment did not intersect with any of the polygon edges, so either the whole line segment is inside
	// the polygon, or it is fully outside the polygon. Test one point of the line segment to determine which.
	return Contains(MapFrom2D(localSpaceLineSegment.a.xy()));
}

bool Polygon::Intersects2D(const LineSegment &localSpaceLineSegment) const
{
	if (p.size() < 3)
		return false;

	const vec basisU = BasisU();
	const vec basisV = BasisV();
	const vec origin = p[0];

	LineSegment edge;
	edge.a = POINT_VEC(Dot(p.back(), basisU), Dot(p.back(), basisV), 0); // map to 2D
	for (int i = 0; i < (int)p.size(); ++i)
	{
		edge.b = POINT_VEC(Dot(p[i], basisU), Dot(p[i], basisV), 0); // map to 2D
		if (edge.Intersects(localSpaceLineSegment))
			return true;
		edge.a = edge.b;
	}

	// The line segment did not intersect with any of the polygon edges, so either the whole line segment is inside
	// the polygon, or it is fully outside the polygon. Test one point of the line segment to determine which.
	return Contains(MapFrom2D(localSpaceLineSegment.a.xy()));
}

bool Polygon::Intersects(const Line &line) const
{
	float d;
	if (!PlaneCCW().Intersects(line, &d))
		return false;
	return Contains(line.GetPoint(d));
}

bool Polygon::Intersects(const Ray &ray) const
{
	float d;
	if (!PlaneCCW().Intersects(ray, &d))
		return false;
	return Contains(ray.GetPoint(d));
}

bool Polygon::Intersects(const LineSegment &lineSegment) const
{
	Plane plane = PlaneCCW();

	// Compute line-plane intersection (unroll Plane::IntersectLinePlane())
	float denom = Dot(plane.normal, lineSegment.b - lineSegment.a);

	if (Abs(denom) < 1e-4f) // The plane of the polygon and the line are planar? Do the test in 2D.
		return Intersects2D(LineSegment(POINT_VEC(MapTo2D(lineSegment.a), 0), POINT_VEC(MapTo2D(lineSegment.b), 0)));

	// The line segment properly intersects the plane of the polygon, so there is exactly one
	// point of intersection between the plane of the polygon and the line segment. Test that intersection point against
	// the line segment end points.
	float t = (plane.d - Dot(plane.normal, lineSegment.a)) / denom;
	if (t < 0.f || t > 1.f)
		return false;

	return Contains(lineSegment.GetPoint(t));
}

bool Polygon::Intersects(const Plane &plane) const
{
	// Project the points of this polygon onto the 1D axis of the plane normal.
	// If there are points on both sides of the plane, then the polygon intersects the plane.
	float minD = inf;
	float maxD = -inf;
	for(size_t i = 0; i < p.size(); ++i)
	{
		float d = plane.SignedDistance(p[i]);
		minD = Min(minD, d);
		maxD = Max(maxD, d);
	}
	// Allow a very small epsilon tolerance.
	return minD <= 1e-4f && maxD >= -1e-4f;
}

bool Polygon::ConvexIntersects(const AABB &aabb) const
{
	return GJKIntersect(*this, aabb);
}

bool Polygon::ConvexIntersects(const OBB &obb) const
{
	return GJKIntersect(*this, obb);
}

bool Polygon::ConvexIntersects(const Frustum &frustum) const
{
	return GJKIntersect(*this, frustum);
}

template<typename Convex /* = AABB, OBB, Frustum. */>
bool Convex_Intersects_Polygon(const Convex &c, const Polygon &p)
{
	LineSegment l;
	l.a = p.p.back();
	for(size_t i = 0; i < p.p.size(); ++i)
	{
		l.b = p.p[i];
		if (c.Intersects(l))
			return true;
		l.a = l.b;
	}

	// Check all the edges of the convex shape against the polygon.
	for(int i = 0; i < c.NumEdges(); ++i)
	{
		l = c.Edge(i);
		if (p.Intersects(l))
			return true;
	}

	return false;
}

bool Polygon::Intersects(const AABB &aabb) const
{
	// Because GJK test is so fast, use that as an early-out. (computes intersection between the convex hull of this poly, and the aabb)
	bool convexIntersects = ConvexIntersects(aabb);
	if (!convexIntersects)
		return false;

	return Convex_Intersects_Polygon(aabb, *this);
}

bool Polygon::Intersects(const OBB &obb) const
{
	// Because GJK test is so fast, use that as an early-out. (computes intersection between the convex hull of this poly, and the obb)
	bool convexIntersects = ConvexIntersects(obb);
	if (!convexIntersects)
		return false;

	return Convex_Intersects_Polygon(obb, *this);
}

bool Polygon::Intersects(const Frustum &frustum) const
{
	// Because GJK test is so fast, use that as an early-out. (computes intersection between the convex hull of this poly, and the frustum)
	bool convexIntersects = ConvexIntersects(frustum);
	if (!convexIntersects)
		return false;

	return Convex_Intersects_Polygon(frustum, *this);
}

template<typename T /* = Polygon or Triangle */>
bool Polygon_Intersects_Polygon(const Polygon &poly, const T &other, float polygonThickness)
{
	Plane plane = poly.PlaneCCW();
	Plane plane2 = other.PlaneCCW();

	if (!plane.normal.Cross(plane2.normal).IsZero())
	{
		// General strategy: If two polygon/triangle objects intersect, one 
		// of them must have an edge that passes through the interior of the other object.
		// Test each edge of the this object against intersection of the interior of the other polygon,
		// and vice versa.
		for(int i = 0; i < other.NumEdges(); ++i)
		{
			LineSegment lineSegment = other.Edge(i);
			float t;
			bool intersects = Plane::IntersectLinePlane(plane.normal, plane.d, lineSegment.a, lineSegment.b - lineSegment.a, t);
			if (!intersects || t < 0.f || t > 1.f)
				continue;

			if (poly.Contains(lineSegment.GetPoint(t)))
				return true;
		}
	
		for(int i = 0; i < poly.NumEdges(); ++i)
		{
			LineSegment lineSegment = poly.Edge(i);
			float t;
			bool intersects = Plane::IntersectLinePlane(plane2.normal, plane2.d, lineSegment.a, lineSegment.b - lineSegment.a, t);
			if (!intersects || t < 0.f || t > 1.f)
				continue;

			if (other.Contains(lineSegment.GetPoint(t)))
				return true;
		}
		return false;
	}
	else // The two polygons are coplanar. Perform the intersection test in 2D.
	{
		float poly0Pos = plane.normal.Dot(poly.Vertex(0));
		float poly1Pos = plane.normal.Dot(other.Vertex(0));
		if (Abs(poly0Pos-poly1Pos) > polygonThickness)
			return false;

		if (other.Contains(poly.Vertex(0), FLOAT_INF) || poly.Contains(other.Vertex(0), FLOAT_INF))
			return true;

		vec basisU, basisV;
		plane.normal.PerpendicularBasis(basisU, basisV);

		vec pivot = poly.Vertex(0);
		vec pt = poly.Vertex(poly.NumVertices()-1)-pivot;
		float2 a1 = float2(basisU.Dot(pt), basisV.Dot(pt));
		for(int i = 0; i < poly.NumVertices(); ++i)
		{
			pt = poly.Vertex(i)-pivot;
			float2 a2 = float2(basisU.Dot(pt), basisV.Dot(pt));

			pt = other.Vertex(other.NumVertices()-1)-pivot;
			float2 b1 = float2(basisU.Dot(pt), basisV.Dot(pt));
			for(int j = 0; j < other.NumVertices(); ++j)
			{
				pt = other.Vertex(j)-pivot;
				float2 b2 = float2(basisU.Dot(pt), basisV.Dot(pt));

				float s, t;
				if (LineSegment2DLineSegment2DIntersect(a1, a2-a1, b1, b2-b1, s, t))
					return true;

				b1 = b2;
			}
			a1 = a2;
		}

		return false;
	}
}


bool Polygon::Intersects(const Triangle &triangle, float polygonThickness) const
{
	return Polygon_Intersects_Polygon(*this, triangle, polygonThickness);
}

bool Polygon::Intersects(const Polygon &polygon, float polygonThickness) const
{
	return Polygon_Intersects_Polygon(*this, polygon, polygonThickness);
}

bool Polygon::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

bool Polygon::Intersects(const Sphere &sphere) const
{
	///@todo Optimize.
	TriangleArray tris = Triangulate();
	for(size_t i = 0; i < tris.size(); ++i)
		if (TRIANGLE(tris[i]).Intersects(sphere))
			return true;

	return false;
}

bool Polygon::Intersects(const Capsule &capsule) const
{
	///@todo Optimize.
	TriangleArray tris = Triangulate();
	for(size_t i = 0; i < tris.size(); ++i)
		if (TRIANGLE(tris[i]).Intersects(capsule))
			return true;

	return false;
}

vec Polygon::ClosestPoint(const vec &point) const
{
	assume(IsPlanar());

	TriangleArray tris = Triangulate();
	vec closestPt = vec::nan;
	float closestDist = FLT_MAX;
	for(size_t i = 0; i < tris.size(); ++i)
	{
		vec pt = TRIANGLE(tris[i]).ClosestPoint(point);
		float d = pt.DistanceSq(point);
		if (d < closestDist)
		{
			closestPt = pt;
			closestDist = d;
		}
	}
	return closestPt;
}

vec Polygon::ClosestPoint(const LineSegment &lineSegment) const
{
	return ClosestPoint(lineSegment, 0);
}

vec Polygon::ClosestPoint(const LineSegment &lineSegment, vec *lineSegmentPt) const
{
	TriangleArray tris = Triangulate();
	vec closestPt = vec::nan;
	vec closestLineSegmentPt = vec::nan;
	float closestDist = FLT_MAX;
	for(size_t i = 0; i < tris.size(); ++i)
	{
		vec lineSegPt;
		vec pt = TRIANGLE(tris[i]).ClosestPoint(lineSegment, &lineSegPt);
		float d = pt.DistanceSq(lineSegPt);
		if (d < closestDist)
		{
			closestPt = pt;
			closestLineSegmentPt = lineSegPt;
			closestDist = d;
		}
	}
	if (lineSegmentPt)
		*lineSegmentPt = closestLineSegmentPt;
	return closestPt;
}

float Polygon::Distance(const vec &point) const
{
	vec pt = ClosestPoint(point);
	return pt.Distance(point);
}

vec Polygon::EdgeNormal(int edgeIndex) const
{
	return Cross(Edge(edgeIndex).Dir(), PlaneCCW().normal).Normalized();
}

Plane Polygon::EdgePlane(int edgeIndex) const
{
	return Plane(Edge(edgeIndex).a, EdgeNormal(edgeIndex));
}

vec Polygon::ExtremePoint(const vec &direction) const
{
	float projectionDistance;
	return ExtremePoint(direction, projectionDistance);
}

vec Polygon::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec mostExtreme = vec::nan;
	projectionDistance = -FLOAT_INF;
	for(int i = 0; i < NumVertices(); ++i)
	{
		vec pt = Vertex(i);
		float d = Dot(direction, pt);
		if (d > projectionDistance)
		{
			projectionDistance = d;
			mostExtreme = pt;
		}
	}
	return mostExtreme;
}

void Polygon::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	///\todo Optimize!
	vec minPt = ExtremePoint(-direction);
	vec maxPt = ExtremePoint(direction);
	outMin = Dot(minPt, direction);
	outMax = Dot(maxPt, direction);
}

/*
/// Returns true if the edges of this polygon self-intersect.
bool IsSelfIntersecting() const;

/// Projects all vertices of this polygon to the given plane.
void ProjectToPlane(const Plane &plane);

/// Returns true if the edges of this polygon self-intersect when viewed from the given direction.
bool IsSelfIntersecting(const vec &viewDirection) const;

bool Contains(const vec &point, const vec &viewDirection) const;

*/

/** Implementation based on Graphics Gems 2, p. 170: "IV.1. Area of Planar Polygons and Volume of Polyhedra." */
float Polygon::Area() const
{
	assume(IsPlanar());
	vec area = vec::zero;
	if (p.size() <= 2)
		return 0.f;

	int i = NumEdges()-1;
	for(int j = 0; j < NumEdges(); ++j)
	{
		area += Vertex(i).Cross(Vertex(j));
		i = j;
	}
	return 0.5f * Abs(NormalCCW().Dot(area));
}

float Polygon::Perimeter() const
{
	float perimeter = 0.f;
	for(int i = 0; i < NumEdges(); ++i)
		perimeter += Edge(i).Length();
	return perimeter;
}

///\bug This function does not properly compute the centroid.
vec Polygon::Centroid() const
{
	if (NumVertices() == 0)
		return vec::nan;
	vec centroid = vec::zero;
	for(int i = 0; i < NumVertices(); ++i)
		centroid += Vertex(i);
	return centroid / (float)NumVertices();
}

vec Polygon::PointOnEdge(float normalizedDistance) const
{
	if (p.empty())
		return vec::nan;
	if (p.size() < 2)
		return p[0];
	normalizedDistance = Frac(normalizedDistance); // Take modulo 1 so we have the range [0,1[.
	float perimeter = Perimeter();
	float d = normalizedDistance * perimeter;
	for(int i = 0; i < NumVertices(); ++i)
	{
		LineSegment edge = Edge(i);
		float len = edge.Length();
		assume(len != 0.f && "Degenerate Polygon detected!");
		if (d <= len)
			return edge.GetPoint(d / len);
		d -= len;
	}
	mathassert(false && "Polygon::PointOnEdge reached end of loop which shouldn't!");
	return p[0];
}

vec Polygon::RandomPointOnEdge(LCG &rng) const
{
	return PointOnEdge(rng.Float());
}

vec Polygon::FastRandomPointInside(LCG &rng) const
{
	TriangleArray tris = Triangulate();
	if (tris.empty())
		return vec::nan;
	int i = rng.Int(0, (int)tris.size()-1);
	return TRIANGLE(tris[i]).RandomPointInside(rng);
}

Polyhedron Polygon::ToPolyhedron() const
{
	Polyhedron poly;
	poly.v = p;
	poly.f.push_back(Polyhedron::Face());
	poly.f.push_back(Polyhedron::Face());
	for(int i = 0; i < NumVertices(); ++i)
	{
		poly.f[0].v.push_back(i);
		poly.f[1].v.push_back(NumVertices()-1-i);
	}
	return poly;
}

// A(u) = a1 + u * (a2-a1).
// B(v) = b1 + v * (b2-b1).
// Returns (u,v).
bool IntersectLineLine2D(const float2 &a1, const float2 &a2, const float2 &b1, const float2 &b2, float2 &out)
{
	float u = (b2.x - b1.x)*(a1.y - b1.y) - (b2.y - b1.y)*(a1.x - b1.x);
	float v = (a2.x - a1.x)*(a1.y - b1.y) - (a2.y - a1.y)*(a1.x - b1.x);

	float det = (b2.y - b1.y)*(a2.x - a1.x) - (b2.x - b1.x)*(a2.y - a1.y);
	if (Abs(det) < 1e-4f)
		return false;
	det = 1.f / det;
	out.x = u * det;
	out.y = v * det;

	return true;
}

bool IntersectLineSegmentLineSegment2D(const float2 &a1, const float2 &a2, const float2 &b1, const float2 &b2, float2 &out)
{
	bool ret = IntersectLineLine2D(a1, a2, b1, b2, out);
	return ret && out.x >= 0.f && out.x <= 1.f && out.y >= 0.f && out.y <= 1.f;
}

/// Returns true if poly[i+1] is an ear.
/// Precondition: i+2 == j (mod poly.size()).
bool IsAnEar(const std::vector<float2> &poly, int i, int j)
{
	float2 dummy;
	int x = (int)poly.size()-1;
	for(int y = 0; y < i; ++y)
	{
		if (IntersectLineSegmentLineSegment2D(poly[i], poly[j], poly[x], poly[y], dummy))
			return false;
		x = y;
	}
	x = j+1;
	for(int y = x+1; y < (int)poly.size(); ++y)
	{
		if (IntersectLineSegmentLineSegment2D(poly[i], poly[j], poly[x], poly[y], dummy))
			return false;
		x = y;
	}
	return true;
}

/** The implementation of this function is based on the paper
	"Kong, Everett, Toussant. The Graham Scan Triangulates Simple Polygons."
	See also p. 772-775 of Geometric Tools for Computer Graphics.
	The running time of this function is O(n^2). */
TriangleArray Polygon::Triangulate() const
{
	assume(IsPlanar());
//	assume1(IsPlanar(), this->SerializeToString()); // TODO: enable

	TriangleArray t;
	// Handle degenerate cases.
	if (NumVertices() < 3)
		return t;
	if (NumVertices() == 3)
	{
		t.push_back(Triangle(Vertex(0), Vertex(1), Vertex(2)));
		return t;
	}
	std::vector<float2> p2d;
	std::vector<int> polyIndices;
	for(int v = 0; v < NumVertices(); ++v)
	{
		p2d.push_back(MapTo2D(v));
		polyIndices.push_back(v);
	}

	// Clip ears of the polygon until it has been reduced to a triangle.
	int i = 0;
	int j = 1;
	int k = 2;
	size_t numTries = 0; // Avoid creating an infinite loop.
	while(p2d.size() > 3 && numTries < p2d.size())
	{
		if (float2::OrientedCCW(p2d[i], p2d[j], p2d[k]) && IsAnEar(p2d, i, k))
		{
			// The vertex j is an ear. Clip it off.
			t.push_back(Triangle(p[polyIndices[i]], p[polyIndices[j]], p[polyIndices[k]]));
			p2d.erase(p2d.begin() + j);
			polyIndices.erase(polyIndices.begin() + j);

			// The previous index might now have become an ear. Move back one index to see if so.
			if (i > 0)
			{
				i = (i + (int)p2d.size() - 1) % p2d.size();
				j = (j + (int)p2d.size() - 1) % p2d.size();
				k = (k + (int)p2d.size() - 1) % p2d.size();
			}
			numTries = 0;
		}
		else
		{
			// The vertex at j is not an ear. Move to test next vertex.
			i = j;
			j = k;
			k = (k+1) % p2d.size();
			++numTries;
		}
	}

	assume3(p2d.size() == 3, (int)p2d.size(), (int)polyIndices.size(), (int)NumVertices());
	if (p2d.size() > 3) // If this occurs, then the polygon is NOT counter-clockwise oriented.
		return t;
/*
	{
		// For conveniency, create a copy that has the winding order fixed, and triangulate that instead.
		// (Causes a large performance hit!)
		Polygon p2 = *this;
		for(size_t i = 0; i < p2.p.size()/2; ++i)
			std::swap(p2.p[i], p2.p[p2.p.size()-1-i]);
		return p2.Triangulate();
	}
*/
	// Add the last poly.
	t.push_back(Triangle(p[polyIndices[0]], p[polyIndices[1]], p[polyIndices[2]]));

	return t;
}

AABB Polygon::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetNegativeInfinity();
	for(int i = 0; i < NumVertices(); ++i)
		aabb.Enclose(Vertex(i));
	return aabb;
}

#if defined(MATH_ENABLE_STL_SUPPORT)
std::string Polygon::ToString() const
{
	if (p.empty())
		return "Polygon(0 vertices)";

	std::stringstream ss;
	ss << "Polygon(" << p.size() << " vertices:";
	for(size_t i = 0; i < p.size(); ++i)
	{
		if (i != 0)
			ss << ",";
		ss << p[i];
	}
	ss << ")";
	return ss.str();
}

std::string Polygon::SerializeToString() const
{
	std::stringstream ss;
	ss << "(";
	for(size_t i = 0; i < p.size(); ++i)
		ss << "(" << vec(p[i]).xyz().SerializeToString() + (i+1 != p.size() ? ")," : ")");
	ss << ")";
	return ss.str();
}
#endif

Polygon Polygon::FromString(const char *str, const char **outEndStr)
{
	MATH_SKIP_WORD(str, "Polygon");
	MATH_SKIP_WORD(str, "(");
	Polygon p;
	while(*str == '(' || *str == ',')
	{
		MATH_SKIP_WORD(str, ",");
		float3 pt = float3::FromString(str, &str);
		p.p.push_back(POINT_VEC(pt));
	}
	MATH_SKIP_WORD(str, ")");

	if (outEndStr)
		*outEndStr = str;

	return p;
}

bool Polygon::Equals(const Polygon &other) const
{
	if (p.size() != other.p.size())
		return false;

	for(size_t i = 0; i < p.size(); ++i)
		if (!Vertex((int)i).Equals(other.Vertex((int)i)))
			return false;

	return true;
}

bool Polygon::BitEquals(const Polygon &other) const
{
	if (p.size() != other.p.size())
		return false;

	for(size_t i = 0; i < p.size(); ++i)
		if (!Vertex((int)i).BitEquals(other.Vertex((int)i)))
			return false;

	return true;
}

/*
/// Returns true if the given vertex is a concave vertex. Otherwise the vertex is a convex vertex.
bool IsConcaveVertex(int i) const;

/// Computes the conves hull of this polygon.
Polygon ConvexHull() const;

bool IsSupportingPoint(int i) const;

bool IsSupportingPoint(const vec &point) const;

/// Returns true if the quadrilateral defined by the four points is convex (and not concave or bowtie).
static bool IsConvexQuad(const vec &pointA, const vec &pointB, const vec &pointC, const vec &pointD);
*/

Polygon operator *(const float3x3 &transform, const Polygon &polygon)
{
	Polygon p(polygon);
	p.Transform(transform);
	return p;
}

Polygon operator *(const float3x4 &transform, const Polygon &polygon)
{
	Polygon p(polygon);
	p.Transform(transform);
	return p;
}

Polygon operator *(const float4x4 &transform, const Polygon &polygon)
{
	Polygon p(polygon);
	p.Transform(transform);
	return p;
}

Polygon operator *(const Quat &transform, const Polygon &polygon)
{
	Polygon p(polygon);
	p.Transform(transform);
	return p;
}

MATH_END_NAMESPACE
