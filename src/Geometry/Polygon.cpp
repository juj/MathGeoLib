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
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "../Math/float2.h"

MATH_BEGIN_NAMESPACE

int Polygon::NumVertices() const
{
	return (int)p.size();
}

int Polygon::NumEdges() const
{
	return (int)p.size();
}

float3 Polygon::Vertex(int vertexIndex) const
{
	assume(vertexIndex >= 0);
	assume(vertexIndex < (int)p.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (vertexIndex < 0 || vertexIndex >= (int)p.size())
		return float3::nan;
#endif
	return p[vertexIndex];
}

LineSegment Polygon::Edge(int i) const
{
	if (p.empty())
		return LineSegment(float3::nan, float3::nan);
	if (p.size() == 1)
		return LineSegment(p[0], p[0]);
	return LineSegment(p[i], p[(i+1)%p.size()]);
}

LineSegment Polygon::Edge2D(int i) const
{
	if (p.empty())
		return LineSegment(float3::nan, float3::nan);
	if (p.size() == 1)
		return LineSegment(float3::zero, float3::zero);
	return LineSegment(float3(MapTo2D(i), 0), float3(MapTo2D((i+1)%p.size()), 0));
}

bool Polygon::DiagonalExists(int i, int j) const
{
	assume(p.size() >= 3);
	assume(i >= 0);
	assume(j >= 0);
	assume(i < (int)p.size());
	assume(j < (int)p.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (p.size() < 3 || i < 0 || j < 0 || i >= (int)p.size() || j >= (int)p.size())
		return false;
#endif
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

float3 Polygon::BasisU() const
{
	if (p.size() < 2)
		return float3::unitX;
	float3 u = p[1] - p[0];
	u.Normalize(); // Always succeeds, even if u was zero (generates (1,0,0)).
	return u;
}

float3 Polygon::BasisV() const
{
	if (p.size() < 2)
		return float3::unitY;
	return Cross(PlaneCCW().normal, BasisU()).Normalized();
}

LineSegment Polygon::Diagonal(int i, int j) const
{
	assume(i >= 0);
	assume(j >= 0);
	assume(i < (int)p.size());
	assume(j < (int)p.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (i < 0 || j < 0 || i >= (int)p.size() || j >= (int)p.size())
		return LineSegment(float3::nan, float3::nan);
#endif
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
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (i < 0 || i >= (int)p.size())
		return float2::nan;
#endif
	return MapTo2D(p[i]);
}

float2 Polygon::MapTo2D(const float3 &point) const
{
	assume(!p.empty());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (p.empty())
		return float2::nan;
#endif
	float3 basisU = BasisU();
	float3 basisV = BasisV();
	float3 pt = point - p[0];
	return float2(Dot(pt, basisU), Dot(pt, basisV));
}

float3 Polygon::MapFrom2D(const float2 &point) const
{
	assume(!p.empty());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (p.empty())
		return float3::nan;
#endif
	return p[0] + point.x * BasisU() + point.y * BasisV();
}

bool Polygon::IsPlanar(float epsilon) const
{
	if (p.empty())
		return false;
	if (p.size() <= 3)
		return true;
	Plane plane(p[0], p[1], p[2]);
	for(size_t i = 3; i < p.size(); ++i)
		if (plane.Distance(p[i]) > epsilon)
			return false;
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
		if (!p[i].IsFinite())
			return false;

	return true;
}

bool Polygon::IsDegenerate(float epsilon) const
{
	return p.size() < 3 || Area() <= epsilon;
}

float3 Polygon::NormalCCW() const
{
	///@todo Optimize temporaries.
	return PlaneCCW().normal;
}

float3 Polygon::NormalCW() const
{
	///@todo Optimize temporaries.
	return PlaneCW().normal;
}

Plane Polygon::PlaneCCW() const
{
	if (p.size() >= 3)
		return Plane(p[0], p[1], p[2]);
	if (p.size() == 2)
		return Plane(Line(p[0], p[1]), (p[0]-p[1]).Perpendicular());
	if (p.size() == 1)
		return Plane(p[0], float3(0,1,0));
	return Plane();
}

Plane Polygon::PlaneCW() const
{
	if (p.size() >= 3)
		return Plane(p[0], p[2], p[1]);
	if (p.size() == 2)
		return Plane(Line(p[0], p[1]), (p[0]-p[1]).Perpendicular());
	if (p.size() == 1)
		return Plane(p[0], float3(0,1,0));
	return Plane();
}

void Polygon::Translate(const float3 &offset)
{
	for(size_t i = 0; i < p.size(); ++i)
		p[i] += offset;
}

void Polygon::Transform(const float3x3 &transform)
{
	if (!p.empty())
		transform.BatchTransform(&p[0], (int)p.size());
}

void Polygon::Transform(const float3x4 &transform)
{
	if (!p.empty())
		transform.BatchTransformPos(&p[0], (int)p.size());
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

bool Polygon::Contains(const float3 &worldSpacePoint, float polygonThickness) const
{
	// Implementation based on the description from http://erich.realtimerendering.com/ptinpoly/

	if (p.size() < 3)
		return false;

	if (PlaneCCW().Distance(worldSpacePoint) > polygonThickness)
		return false;

	int numIntersections = 0;

	float3 basisU = BasisU();
	float3 basisV = BasisV();
	mathassert(basisU.IsNormalized());
	mathassert(basisV.IsNormalized());
	mathassert(basisU.IsPerpendicular(basisV));
	mathassert(basisU.IsPerpendicular(PlaneCCW().normal));
	mathassert(basisV.IsPerpendicular(PlaneCCW().normal));

	float2 localSpacePoint = float2(Dot(worldSpacePoint, basisU), Dot(worldSpacePoint, basisV));

	const float epsilon = 1e-4f;

	float2 p0 = float2(Dot(p[p.size()-1], basisU), Dot(p[p.size()-1], basisV)) - localSpacePoint;
	if (Abs(p0.y) < epsilon)
		p0.y = -epsilon; // Robustness check - if the ray (0,0) -> (+inf, 0) would pass through a vertex, move the vertex slightly.
	for(int i = 0; i < (int)p.size(); ++i)
	{
		float2 p1 = float2(Dot(p[i], basisU), Dot(p[i], basisV)) - localSpacePoint;
		if (Abs(p1.y) < epsilon)
			p0.y = -epsilon; // Robustness check - if the ray (0,0) -> (+inf, 0) would pass through a vertex, move the vertex slightly.

		if (p0.y * p1.y < 0.f)
		{
			if (p0.x > 1e-3f && p1.x > 1e-3f)
				++numIntersections;
			else
			{
				// P = p0 + t*(p1-p0) == (x,0)
				//     p0.x + t*(p1.x-p0.x) == x
				//     p0.y + t*(p1.y-p0.y) == 0
				//                 t == -p0.y / (p1.y - p0.y)

				// Test whether the lines (0,0) -> (+inf,0) and p0 -> p1 intersect at a positive X-coordinate.
				float2 d = p1 - p0;
				if (Abs(d.y) > 1e-5f)
				{
					float t = -p0.y / d.y;
					float x = p0.x + t * d.x;
					if (t >= 0.f && t <= 1.f && x > 1e-3f)
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

///\todo Reimplement this!
//	if (!Contains2D(localSpaceLineSegment.a.xy()) || !Contains2D(localSpaceLineSegment.b.xy()))
//		return false;

	for(int i = 0; i < (int)p.size(); ++i)
		if (Edge2D(i).Intersects(localSpaceLineSegment))
			return false;

	return true;
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
	float t;
	bool intersects = Plane::IntersectLinePlane(plane.normal, plane.d, lineSegment.a, lineSegment.b - lineSegment.a, t);
	if (!intersects || t < 0.f || t > 1.f)
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

bool Polygon::Intersects(const AABB &aabb) const
{
	return aabb.Intersects(*this);
}

bool Polygon::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool Polygon::Intersects(const Triangle &triangle) const
{
	return ToPolyhedron().Intersects(triangle);
}

bool Polygon::Intersects(const Polygon &polygon) const
{
	return ToPolyhedron().Intersects(polygon);
}

bool Polygon::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Polygon::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

bool Polygon::Intersects(const Sphere &sphere) const
{
	///@todo Optimize.
	std::vector<Triangle> tris = Triangulate();
	for(size_t i = 0; i < tris.size(); ++i)
		if (tris[i].Intersects(sphere))
			return true;

	return false;
}

bool Polygon::Intersects(const Capsule &capsule) const
{
	///@todo Optimize.
	std::vector<Triangle> tris = Triangulate();
	for(size_t i = 0; i < tris.size(); ++i)
		if (tris[i].Intersects(capsule))
			return true;

	return false;
}

float3 Polygon::ClosestPoint(const float3 &point) const
{
	assume(IsPlanar());

	std::vector<Triangle> tris = Triangulate();
	float3 closestPt = float3::nan;
	float closestDist = FLT_MAX;
	for(size_t i = 0; i < tris.size(); ++i)
	{
		float3 pt = tris[i].ClosestPoint(point);
		float d = pt.DistanceSq(point);
		if (d < closestDist)
		{
			closestPt = pt;
			closestDist = d;
		}
	}
	return closestPt;
}

float3 Polygon::ClosestPoint(const LineSegment &lineSegment) const
{
	return ClosestPoint(lineSegment, 0);
}

float3 Polygon::ClosestPoint(const LineSegment &lineSegment, float3 *lineSegmentPt) const
{
	std::vector<Triangle> tris = Triangulate();
	float3 closestPt = float3::nan;
	float3 closestLineSegmentPt = float3::nan;
	float closestDist = FLT_MAX;
	for(size_t i = 0; i < tris.size(); ++i)
	{
		float3 lineSegPt;
		float3 pt = tris[i].ClosestPoint(lineSegment, &lineSegPt);
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

float Polygon::Distance(const float3 &point) const
{
	float3 pt = ClosestPoint(point);
	return pt.Distance(point);
}

float3 Polygon::EdgeNormal(int edgeIndex) const
{
	return Cross(Edge(edgeIndex).Dir(), PlaneCCW().normal).Normalized();
}

Plane Polygon::EdgePlane(int edgeIndex) const
{
	return Plane(Edge(edgeIndex).a, EdgeNormal(edgeIndex));
}

float3 Polygon::ExtremePoint(const float3 &direction) const
{
	float3 mostExtreme = float3::nan;
	float mostExtremeDist = -FLT_MAX;
	for(int i = 0; i < NumVertices(); ++i)
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

void Polygon::ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const
{
	///\todo Optimize!
	float3 minPt = ExtremePoint(-direction);
	float3 maxPt = ExtremePoint(direction);
	outMin = Dot(minPt, direction);
	outMax = Dot(maxPt, direction);
}

/*
/// Returns true if the edges of this polygon self-intersect.
bool IsSelfIntersecting() const;

/// Projects all vertices of this polygon to the given plane.
void ProjectToPlane(const Plane &plane);

/// Returns true if the edges of this polygon self-intersect when viewed from the given direction.
bool IsSelfIntersecting(const float3 &viewDirection) const;

bool Contains(const float3 &point, const float3 &viewDirection) const;

*/

/** Implementation based on Graphics Gems 2, p. 170: "IV.1. Area of Planar Polygons and Volume of Polyhedra." */
float Polygon::Area() const
{
	assume(IsPlanar());
	float3 area = float3::zero;
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
float3 Polygon::Centroid() const
{
	if (NumVertices() == 0)
		return float3::nan;
	float3 centroid = float3::zero;
	for(int i = 0; i < NumVertices(); ++i)
		centroid += Vertex(i);
	return centroid / (float)NumVertices();
}

float3 Polygon::PointOnEdge(float normalizedDistance) const
{
	if (p.empty())
		return float3::nan;
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

float3 Polygon::RandomPointOnEdge(LCG &rng) const
{
	return PointOnEdge(rng.Float());
}

float3 Polygon::FastRandomPointInside(LCG &rng) const
{
	std::vector<Triangle> tris = Triangulate();
	if (tris.empty())
		return float3::nan;
	return tris[rng.Int(0, (int)tris.size()-1)].RandomPointInside(rng);
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
std::vector<Triangle> Polygon::Triangulate() const
{
	assume(IsPlanar());

	std::vector<Triangle> t;
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

	assume(p2d.size() == 3);
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

/*
/// Returns true if the given vertex is a concave vertex. Otherwise the vertex is a convex vertex.
bool IsConcaveVertex(int i) const;

/// Computes the conves hull of this polygon.
Polygon ConvexHull() const;

bool IsSupportingPoint(int i) const;

bool IsSupportingPoint(const float3 &point) const;

/// Returns true if the quadrilateral defined by the four points is convex (and not concave or bowtie).
static bool IsConvexQuad(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD);
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
