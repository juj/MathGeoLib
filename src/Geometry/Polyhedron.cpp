/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Polyhedron.cpp
	@author Jukka Jylänki
	@brief Implementation for the Polyhedron geometry object. */
#include <set>
#include <utility>
#include "assume.h"
#include "Math/MathFunc.h"
#include "Geometry/AABB.h"
#include "Geometry/OBB.h"
#include "Geometry/Frustum.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Triangle.h"
#include "Geometry/Sphere.h"
#include "Geometry/Capsule.h"

MATH_BEGIN_NAMESPACE

int Polyhedron::NumEdges() const
{
	return EdgeIndices().size();
}

float3 Polyhedron::Vertex(int vertexIndex) const
{
	assume(vertexIndex >= 0);
	assume(vertexIndex < (int)v.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (vertexIndex < 0 || vertexIndex >= (int)v.size())
		return float3::nan;
#endif
	
	return v[vertexIndex];
}

LineSegment Polyhedron::Edge(int edgeIndex) const
{
	assume(edgeIndex >= 0);
	std::vector<LineSegment> edges = Edges();
	assume(edgeIndex < (int)edges.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (edgeIndex < 0 || edgeIndex >= (int)edges.size())
		return LineSegment(float3::nan, float3::nan);
#endif
	return edges[edgeIndex];
}

std::vector<LineSegment> Polyhedron::Edges() const
{
	std::vector<std::pair<int, int> > edges = EdgeIndices();
	std::vector<LineSegment> edgeLines;
	edgeLines.reserve(edges.size());
	for(size_t i = 0; i < edges.size(); ++i)
		edgeLines.push_back(LineSegment(Vertex(edges[i].first), Vertex(edges[i].second)));

	return edgeLines;
}

std::vector<std::pair<int, int> > Polyhedron::EdgeIndices() const
{
	std::set<std::pair<int, int> > uniqueEdges;
	for(int i = 0; i < NumFaces(); ++i)
	{
		assume(f[i].v.size() >= 3);
		if (f[i].v.size() < 3)
			continue; // Degenerate face with less than three vertices, skip!
		int x = f[i].v.back();
		for(size_t j = 0; j < f[i].v.size(); ++j)
		{
			int y = f[i].v[j];
			uniqueEdges.insert(std::make_pair(std::min(x, y), std::max(x, y)));
			x = y;
		}
	}

	std::vector<std::pair<int, int> >edges;
	edges.insert(edges.end(), uniqueEdges.begin(), uniqueEdges.end());
	return edges;
}

Polygon Polyhedron::FacePolygon(int faceIndex) const
{
	Polygon p;
	assume(faceIndex >= 0);
	assume(faceIndex < (int)f.size());
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (faceIndex < 0 || faceIndex >= (int)f.size())
		return Polygon();
#endif

	p.p.reserve(f[faceIndex].v.size());
	for(size_t v = 0; v < f[faceIndex].v.size(); ++v)
		p.p.push_back(Vertex(f[faceIndex].v[v]));
	return p;
}

Plane Polyhedron::FacePlane(int faceIndex) const
{
	return FacePolygon(faceIndex).PlaneCCW();
}

int Polyhedron::ExtremeVertex(const float3 &direction) const
{
	int mostExtreme = -1;
	float mostExtremeDist = -FLOAT_MAX;
	for(int i = 0; i < NumVertices(); ++i)
	{
		float d = Dot(direction, Vertex(i));
		if (d > mostExtremeDist)
		{
			mostExtremeDist = d;
			mostExtreme = i;
		}
	}
	return mostExtreme;
}

float3 Polyhedron::ExtremePoint(const float3 &direction) const
{
	return Vertex(ExtremeVertex(direction));
}

float3 Polyhedron::Centroid() const
{
	float3 centroid = float3::zero;
	for(int i = 0; i < NumVertices(); ++i)
		centroid += Vertex(i);
	return centroid / (float)NumVertices();
}

float Polyhedron::SurfaceArea() const
{
	float area = 0.f;
	for(int i = 0; i < NumFaces(); ++i)
		area += FacePolygon(i).Area(); ///@todo Optimize temporary copies.
	return area;
}

/** The implementation of this function is based on Graphics Gems 2, p. 170: "IV.1. Area of Planar Polygons and Volume of Polyhedra." */
float Polyhedron::Volume() const
{
	float volume = 0.f;
	for(int i = 0; i < NumFaces(); ++i)
	{
		Polygon face = FacePolygon(i); ///@todo Optimize temporary copies.
		volume += face.Vertex(0).Dot(face.NormalCCW()) * face.Area();
	}
	return Abs(volume) / 3.f;
}

AABB Polyhedron::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetNegativeInfinity();
	for(int i = 0; i < NumVertices(); ++i)
		aabb.Enclose(Vertex(i));
	return aabb;
}

bool Polyhedron::FaceIndicesValid() const
{
	// Test condition 1: Face indices in proper range.
	for(int i = 0; i < NumFaces(); ++i)
		for(int j = 0; j < (int)f[i].v.size(); ++j)
			if (f[i].v[j] < 0 || f[i].v[j] >= (int)v.size())
				return false;

	// Test condition 2: Each face has at least three vertices.
	for(int i = 0; i < NumFaces(); ++i)
		if (f[i].v.size() < 3)
			return false;

	// Test condition 3: Each face may refer to a vertex at most once. (Complexity O(n^2)).
	for(int i = 0; i < NumFaces(); ++i)
		for(int j = 0; j < (int)f[i].v.size(); ++j)
			for(size_t k = j+1; k < f[i].v.size(); ++k)
				if (f[i].v[j] == f[i].v[k])
					return false;

	return true;
}

bool Polyhedron::IsClosed() const
{
	std::set<std::pair<int, int> > uniqueEdges;
	for(int i = 0; i < NumFaces(); ++i)
	{
		assume(FacePolygon(i).IsPlanar());
		assume(FacePolygon(i).IsSimple());
		int x = f[i].v.back();
		for(size_t j = 0; j < f[i].v.size(); ++j)
		{
			int y = f[i].v[j];
			if (uniqueEdges.find(std::make_pair(x, y)) != uniqueEdges.end())
				return false; // This edge is being used twice! Cannot be simple and closed.
			uniqueEdges.insert(std::make_pair(x, y));
			x = y;
		}
	}

	for(std::set<std::pair<int, int> >::iterator iter = uniqueEdges.begin(); 
		iter != uniqueEdges.end(); ++iter)
	{
		std::pair<int, int> reverse = std::make_pair(iter->second, iter->first);
		if (uniqueEdges.find(reverse) == uniqueEdges.end())
			return false;
	}

	return true;
}

bool Polyhedron::IsConvex() const
{
	// This function is O(n^2).
	/** @todo Real-Time Collision Detection, p. 64:
		A faster O(n) approach is to compute for each face F of P the centroid C of F,
		and for all neighboring faces G of F test if C lies behind the supporting plane of
		G. If some C fails to lie behind the supporting plane of one or more neighboring
		faces, P is concave, and is otherwise assumed convex. However, note that just as the
		corresponding polygonal convexity test may fail for a pentagram this test may fail for,
		for example, a pentagram extruded out of its plane and capped at the ends. */

	for(int f = 0; f < NumFaces(); ++f)
	{
		Plane p = FacePlane(f);
		for(int i = 0; i < NumVertices(); ++i)
			if (p.SignedDistance(Vertex(i)) > 1e-3f) // Tolerate a small epsilon error.
				return false;
	}
	return true;
}

bool Polyhedron::EulerFormulaHolds() const
{
	return NumVertices() + NumFaces() - NumEdges() == 2;
}

bool Polyhedron::Contains(const float3 &point) const
{
	Ray r(point, float3::unitX);
	int numIntersections = 0;
	for(int i = 0; i < NumFaces(); ++i)
		if (FacePolygon(i).Intersects(r))
			++numIntersections;

	return numIntersections % 2 == 1;
}

bool Polyhedron::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool Polyhedron::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool Polyhedron::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool Polyhedron::Contains(const AABB &aabb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool Polyhedron::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool Polyhedron::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool Polyhedron::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

bool Polyhedron::ContainsConvex(const float3 &point) const
{
	assume(IsConvex());
	for(int i = 0; i < NumFaces(); ++i)
		if (FacePlane(i).SignedDistance(point) > 0.f)
			return false;

	return true;
}

bool Polyhedron::ContainsConvex(const LineSegment &lineSegment) const
{
	return ContainsConvex(lineSegment.a) && ContainsConvex(lineSegment.b);
}

bool Polyhedron::ContainsConvex(const Triangle &triangle) const
{
	return ContainsConvex(triangle.a) && ContainsConvex(triangle.b) && ContainsConvex(triangle.c);
}

float3 Polyhedron::ClosestPointConvex(const float3 &point) const
{
	assume(IsConvex());
	if (ContainsConvex(point))
		return point;
	float3 closestPoint;
	float closestDistance = FLOAT_MAX;
	for(int i = 0; i < NumFaces(); ++i)
	{
		float3 closestOnPoly = FacePolygon(i).ClosestPoint(point);
		float d = closestOnPoly.DistanceSq(point);
		if (d < closestDistance)
		{
			closestPoint = closestOnPoly;
			closestDistance = d;
		}
	}
	return closestPoint;
}

float3 Polyhedron::ClosestPoint(const float3 &point) const
{
	if (Contains(point))
		return point;
	float3 closestPoint;
	float closestDistance = FLOAT_MAX;
	for(int i = 0; i < NumFaces(); ++i)
	{
		float3 closestOnPoly = FacePolygon(i).ClosestPoint(point);
		float d = closestOnPoly.DistanceSq(point);
		if (d < closestDistance)
		{
			closestPoint = closestOnPoly;
			closestDistance = d;
		}
	}
	return closestPoint;
}

float3 Polyhedron::ClosestPoint(const LineSegment &lineSegment) const
{
	return ClosestPoint(lineSegment, 0);
}

float3 Polyhedron::ClosestPoint(const LineSegment &lineSegment, float3 *lineSegmentPt) const
{
	if (Contains(lineSegment.a))
	{
		if (lineSegmentPt)
			*lineSegmentPt = lineSegment.a;
		return lineSegment.a;
	}
	if (Contains(lineSegment.b))
	{
		if (lineSegmentPt)
			*lineSegmentPt = lineSegment.b;
		return lineSegment.b;
	}
	float3 closestPt;
	float closestDistance = FLOAT_MAX;
	float3 closestLineSegmentPt;
	for(int i = 0; i < NumFaces(); ++i)
	{
		float3 lineSegmentPt;
		float3 pt = FacePolygon(i).ClosestPoint(lineSegment, &lineSegmentPt);
		float d = pt.DistanceSq(lineSegmentPt);
		if (d < closestDistance)
		{
			closestDistance = d;
			closestPt = pt;
			closestLineSegmentPt = lineSegmentPt;
		}
	}
	if (lineSegmentPt)
		*lineSegmentPt = closestLineSegmentPt;
	return closestPt;
}

float Polyhedron::Distance(const float3 &point) const
{
	float3 pt = ClosestPoint(point);
	return pt.Distance(point);
}

bool Polyhedron::ClipLineSegmentToConvexPolyhedron(const float3 &ptA, const float3 &dir, 
												   float &tFirst, float &tLast) const
{
	assume(IsConvex());
	// Intersect line segment against each plane.
	for(int i = 0; i < NumFaces(); ++i)
	{
		Plane p = FacePlane(i);
		float denom = Dot(p.normal, dir);
		float dist = p.d - Dot(p.normal, ptA);
		// Test if segment runs parallel to the plane.
		if (Abs(denom) < 1e-5f)
		{
			if (dist > 0.f) return false; 
		}
		else
		{
			float t = dist / denom;
			if (denom < 0.f) // When entering halfspace, update tFirst if t is larger.
				tFirst = Max(t, tFirst);
			else // When exiting halfspace, updeate tLast if t is smaller.
				tLast = Min(t, tLast);

			if (tFirst > tLast)
				return false; // We clipped the whole line segment.
		}
	}
	return true;
}

bool Polyhedron::Intersects(const LineSegment &lineSegment) const
{
	if (Contains(lineSegment))
		return true;
	for(int i = 0; i < NumFaces(); ++i)
		if (FacePolygon(i).Intersects(lineSegment))
			return true;

	return false;
}

bool Polyhedron::Intersects(const Line &line) const
{
	for(int i = 0; i < NumFaces(); ++i)
		if (FacePolygon(i).Intersects(line))
			return true;

	return false;
}

bool Polyhedron::Intersects(const Ray &ray) const
{
	for(int i = 0; i < NumFaces(); ++i)
		if (FacePolygon(i).Intersects(ray))
			return true;

	return false;
}

bool Polyhedron::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

/** The algorithm for Polyhedron-Polyhedron intersection is from Christer Ericson's Real-Time Collision Detection, p. 384.
	As noted by the author, the algorithm is very naive (and here unoptimized), and better methods exist. [groupSyntax] */
bool Polyhedron::Intersects(const Polyhedron &polyhedron) const
{
	///@todo Implement a more efficient algorithm.
	if (polyhedron.Contains(this->Centroid()))
		return true;
	if (this->Contains(polyhedron.Centroid()))
		return true;

	std::vector<LineSegment> edges = this->Edges();
	for(size_t i = 0; i < edges.size(); ++i)
		if (polyhedron.Intersects(edges[i]))
			return true;

	edges = polyhedron.Edges();
	for(size_t i = 0; i < edges.size(); ++i)
		if (this->Intersects(edges[i]))
			return true;

	return false;
}

bool Polyhedron::Intersects(const AABB &aabb) const
{
	///@todo This is a naive test. Implement a faster version.
	return Intersects(aabb.ToPolyhedron());
}

bool Polyhedron::Intersects(const OBB &obb) const
{
	///@todo This is a naive test. Implement a faster version.
	return Intersects(obb.ToPolyhedron());
}

bool Polyhedron::Intersects(const Triangle &triangle) const
{
	return Intersects(triangle.ToPolyhedron());
}

bool Polyhedron::Intersects(const Polygon &polygon) const
{
	return Intersects(polygon.ToPolyhedron());
}

bool Polyhedron::Intersects(const Frustum &frustum) const
{
	///@todo This is a naive test. Implement a faster version.
	return Intersects(frustum.ToPolyhedron());
}

bool Polyhedron::Intersects(const Sphere &sphere) const
{
	float3 closestPt = ClosestPoint(sphere.pos);
	return closestPt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool Polyhedron::Intersects(const Capsule &capsule) const
{
	float3 pt, ptOnLineSegment;
	pt = ClosestPoint(capsule.l, &ptOnLineSegment);
	return pt.DistanceSq(ptOnLineSegment) <= capsule.r;
}

bool Polyhedron::IntersectsConvex(const Line &line) const
{
	float tFirst = -FLOAT_MAX;
	float tLast = FLOAT_MAX;
	return ClipLineSegmentToConvexPolyhedron(line.pos, line.dir, tFirst, tLast);
}

bool Polyhedron::IntersectsConvex(const Ray &ray) const
{
	float tFirst = 0.f;
	float tLast = FLOAT_MAX;
	return ClipLineSegmentToConvexPolyhedron(ray.pos, ray.dir, tFirst, tLast);
}

bool Polyhedron::IntersectsConvex(const LineSegment &lineSegment) const
{
	float tFirst = 0.f;
	float tLast = 1.f;
	return ClipLineSegmentToConvexPolyhedron(lineSegment.a, lineSegment.b - lineSegment.a, tFirst, tLast);
}

MATH_END_NAMESPACE
