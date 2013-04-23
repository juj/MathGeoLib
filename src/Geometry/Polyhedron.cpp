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

/** @file Polyhedron.cpp
	@author Jukka Jylänki
	@brief Implementation for the Polyhedron geometry object. */
#include "Polyhedron.h"
#include <set>
#include <map>
#include <utility>
#include <list>
#include <sstream>
#include "../Math/assume.h"
#include "../Math/MathFunc.h"
#include "../Math/float3x4.h"
#include "../Math/Quat.h"
#include "AABB.h"
#include "OBB.h"
#include "Frustum.h"
#include "Plane.h"
#include "Polygon.h"
#include "Line.h"
#include "Ray.h"
#include "LineSegment.h"
#include "Triangle.h"
#include "Sphere.h"
#include "Capsule.h"

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

void Polyhedron::Face::FlipWindingOrder()
{
	for(size_t i = 0; i < v.size()/2; ++i)
		Swap(v[i], v[v.size()-1-i]);
}

std::string Polyhedron::Face::ToString() const
{
	std::stringstream ss;
	for(size_t i = 0; i < v.size(); ++i)
		ss << v[i] << ((i!=v.size()-1) ? ", " : "");
	return ss.str();
}

int Polyhedron::NumEdges() const
{
	return (int)EdgeIndices().size();
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
			uniqueEdges.insert(std::make_pair(Min(x, y), Max(x, y)));
			x = y;
		}
	}

	std::vector<std::pair<int, int> >edges;
	edges.insert(edges.end(), uniqueEdges.begin(), uniqueEdges.end());
	return edges;
}

std::vector<Polygon> Polyhedron::Faces() const
{
	std::vector<Polygon> faces;
	faces.reserve(NumFaces());
	for(int i = 0; i < NumFaces(); ++i)
		faces.push_back(FacePolygon(i));
	return faces;
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
	const Face &face = f[faceIndex];
	if (face.v.size() >= 3)
		return Plane(v[face.v[0]], v[face.v[1]], v[face.v[2]]);
	else if (face.v.size() == 2)
		return Plane(Line(v[face.v[0]], v[face.v[1]]), (v[face.v[0]]-v[face.v[1]]).Perpendicular());
	else if (face.v.size() == 1)
		return Plane(v[face.v[0]], float3(0,1,0));
	else
		return Plane();
}

float3 Polyhedron::FaceNormal(int faceIndex) const
{
	const Face &face = f[faceIndex];
	if (face.v.size() >= 3)
		return (v[face.v[1]]-v[face.v[0]]).Cross(v[face.v[2]]-v[face.v[0]]).Normalized();
	else if (face.v.size() == 2)
		return (v[face.v[1]]-v[face.v[0]]).Cross((v[face.v[0]]-v[face.v[1]]).Perpendicular()-v[face.v[0]]).Normalized();
	else if (face.v.size() == 1)
		return float3(0,1,0);
	else
		return float3::nan;
}

int Polyhedron::ExtremeVertex(const float3 &direction) const
{
	int mostExtreme = -1;
	float mostExtremeDist = -FLT_MAX;
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

void Polyhedron::ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const
{
	///\todo Optimize!
	float3 minPt = ExtremePoint(-direction);
	float3 maxPt = ExtremePoint(direction);
	outMin = Dot(minPt, direction);
	outMax = Dot(maxPt, direction);
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

#ifdef MATH_CONTAINERLIB_SUPPORT
OBB Polyhedron::MinimalEnclosingOBB() const
{
	return OBB::OptimalEnclosingOBB(&v[0], (int)v.size());
}
#endif

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

void Polyhedron::FlipWindingOrder()
{
	for(size_t i = 0; i < f.size(); ++i)
		f[i].FlipWindingOrder();
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
			{
				LOGW("The edge (%d,%d) is used twice. Polyhedron is not simple and closed!", x, y);
				return false; // This edge is being used twice! Cannot be simple and closed.
			}
			uniqueEdges.insert(std::make_pair(x, y));
			x = y;
		}
	}

	for(std::set<std::pair<int, int> >::iterator iter = uniqueEdges.begin();
		iter != uniqueEdges.end(); ++iter)
	{
		std::pair<int, int> reverse = std::make_pair(iter->second, iter->first);
		if (uniqueEdges.find(reverse) == uniqueEdges.end())
		{
			LOGW("The edge (%d,%d) does not exist. Polyhedron is not closed!", iter->second, iter->first);
			return false;
		}
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
		{
			float d = p.SignedDistance(Vertex(i));
			if (d > 1e-3f) // Tolerate a small epsilon error.
			{
				LOGW("Distance of vertex %d from plane %d: %f", i, f, d);
				return false;
			}
		}
	}
	return true;
}

bool Polyhedron::EulerFormulaHolds() const
{
	return NumVertices() + NumFaces() - NumEdges() == 2;
}

bool Polyhedron::FacesAreNondegeneratePlanar(float epsilon) const
{
	for(int i = 0; i < (int)f.size(); ++i)
	{
		const Face &face = f[i];
		if (face.v.size() < 3)
			return false;
		if (face.v.size() >= 4)
		{
			Plane facePlane = FacePlane(i);
			for(int j = 0; j < (int)face.v.size(); ++j)
				if (facePlane.Distance(v[face.v[j]]) > epsilon)
					return false;
		}
	}

	return true;
}

bool Polyhedron::FaceContains(int faceIndex, const float3 &worldSpacePoint, float polygonThickness) const
{
	// N.B. This implementation is a duplicate of Polygon::Contains, but adapted to avoid dynamic memory allocation
	// related to converting the face of a Polyhedron to a Polygon object.

	// Implementation based on the description from http://erich.realtimerendering.com/ptinpoly/

	const Face &face = f[faceIndex];
	const std::vector<int> &vertices = face.v;

	if (vertices.size() < 3)
		return false;

	Plane p = FacePlane(faceIndex);
	if (FacePlane(faceIndex).Distance(worldSpacePoint) > polygonThickness)
		return false;

	int numIntersections = 0;

	float3 basisU = v[vertices[1]] - v[vertices[0]];
	basisU.Normalize();
	float3 basisV = Cross(p.normal, basisU).Normalized();
	mathassert(basisU.IsNormalized());
	mathassert(basisV.IsNormalized());
	mathassert(basisU.IsPerpendicular(basisV));
	mathassert(basisU.IsPerpendicular(p.normal));
	mathassert(basisV.IsPerpendicular(p.normal));

	float2 localSpacePoint = float2(Dot(worldSpacePoint, basisU), Dot(worldSpacePoint, basisV));

	const float epsilon = 1e-4f;

	float2 p0 = float2(Dot(v[vertices.back()], basisU), Dot(v[vertices.back()], basisV)) - localSpacePoint;
	if (Abs(p0.y) < epsilon)
		p0.y = -epsilon; // Robustness check - if the ray (0,0) -> (+inf, 0) would pass through a vertex, move the vertex slightly.
	for(size_t i = 0; i < vertices.size(); ++i)
	{
		float2 p1 = float2(Dot(v[vertices[i]], basisU), Dot(v[vertices[i]], basisV)) - localSpacePoint;
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
					if (t >= 0.f && t <= 1.f && x > 1e-6f)
						++numIntersections;
				}
			}
		}
		p0 = p1;
	}

	return numIntersections % 2 == 1;
}

bool Polyhedron::Contains(const float3 &point) const
{
	int numIntersections = 0;
	for(int i = 0; i < (int)f.size(); ++i)
	{
		Plane p(v[f[i].v[0]] - point, v[f[i].v[1]] - point, v[f[i].v[2]] - point);

		// Find the intersection of the plane and the ray (0,0,0) -> (t,0,0), t >= 0.
		// <normal, point_on_ray> == d
		// n.x * t == d
		//       t == d / n.x
		if (Abs(p.normal.x) > 1e-5f)
		{
			float t = p.d / p.normal.x;
			// If t >= 0, the plane and the ray intersect, and the ray potentially also intersects the polygon.
			// Finish the test by checking whether the point of intersection is contained in the polygon, in
			// which case the ray-polygon intersection occurs.
			if (t >= 0.f && FaceContains(i, point + float3(t,0,0)))
				++numIntersections;
		}
	}

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
	float3 closestPoint = float3::nan;
	float closestDistance = FLT_MAX;
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
	float3 closestPoint = float3::nan;
	float closestDistance = FLT_MAX;
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
	float3 closestPt = float3::nan;
	float closestDistance = FLT_MAX;
	float3 closestLineSegmentPt = float3::nan;
	for(int i = 0; i < NumFaces(); ++i)
	{
		float3 lineSegPt;
		float3 pt = FacePolygon(i).ClosestPoint(lineSegment, &lineSegPt);
		float d = pt.DistanceSq(lineSegPt);
		if (d < closestDistance)
		{
			closestDistance = d;
			closestPt = pt;
			closestLineSegmentPt = lineSegPt;
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
		/* Denoting the dot product of vectors a and b with <a,b>, we have:

		   The points P on the plane p satisfy the equation <P, p.normal> == p.d.
		   The points P on the line have the parametric equation P = ptA + dir * t.
		   Solving for the distance along the line for intersection gives
		
		   t = (p.d - <p.normal, ptA>) / <p.normal, dir>.
		*/

		Plane p = FacePlane(i);
		float denom = Dot(p.normal, dir);
		float dist = p.d - Dot(p.normal, ptA);

		// Avoid division by zero. In this case the line segment runs parallel to the plane.
		if (Abs(denom) < 1e-5f)
		{
			// If <P, p.normal> < p.d, then the point lies in the negative halfspace of the plane, which is inside the polyhedron.
			// If <P, p.normal> > p.d, then the point lies in the positive halfspace of the plane, which is outside the polyhedron.
			// Therefore, if p.d - <ptA, p.normal> == dist < 0, then the whole line is outside the polyhedron.
			if (dist < 0.f)
				return false;
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
	{
		float t;
		Plane plane = FacePlane(i);
		bool intersects = Plane::IntersectLinePlane(plane.normal, plane.d, lineSegment.a, lineSegment.b - lineSegment.a, t);
		if (intersects && t >= 0.f && t <= 1.f)
			if (FaceContains(i, lineSegment.GetPoint(t)))
				return true;
	}

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
	if (polyhedron.Contains(this->Centroid()))
		return true;
	if (this->Contains(polyhedron.Centroid()))
		return true;

	// This test assumes that both this and the other polyhedron are closed.
	// This means that for each edge running through vertices i and j, there's a face
	// that contains the line segment (i,j) and another neighboring face that contains
	// the line segment (j,i). These represent the same line segment (but in opposite direction)
	// so we only have to test one of them for intersection. Take i < j as the canonical choice
	// and skip the other winding order.

	// Test for each edge of this polyhedron whether the other polyhedron intersects it.
	for(size_t i = 0; i < f.size(); ++i)
	{
		assert(!f[i].v.empty()); // Cannot have degenerate faces here, and for performance reasons, don't start checking for this condition in release mode!
		int v0 = f[i].v.back();
		float3 l0 = v[v0];
		for(size_t j = 0; j < f[i].v.size(); ++j)
		{
			int v1 = f[i].v[j];
			float3 l1 = v[v1];
			if (v0 < v1 && polyhedron.Intersects(LineSegment(l0, l1))) // If v0 < v1, then this line segment is the canonical one.
				return true;
			l0 = l1;
			v0 = v1;
		}
	}

	// Test for each edge of the other polyhedron whether this polyhedron intersects it.
	for(size_t i = 0; i < polyhedron.f.size(); ++i)
	{
		assert(!polyhedron.f[i].v.empty()); // Cannot have degenerate faces here, and for performance reasons, don't start checking for this condition in release mode!
		int v0 = polyhedron.f[i].v.back();
		float3 l0 = polyhedron.v[v0];
		for(size_t j = 0; j < polyhedron.f[i].v.size(); ++j)
		{
			int v1 = polyhedron.f[i].v[j];
			float3 l1 = polyhedron.v[v1];
			if (v0 < v1 && Intersects(LineSegment(l0, l1))) // If v0 < v1, then this line segment is the canonical one.
				return true;
			l0 = l1;
			v0 = v1;
		}
	}

	return false;
}

template<typename T>
bool PolyhedronIntersectsAABB_OBB(const Polyhedron &p, const T &obj)
{
	if (p.Contains(obj.CenterPoint()))
		return true;
	if (obj.Contains(p.Centroid()))
		return true;

	// Test for each edge of the AABB/OBB whether this polyhedron intersects it.
	for(int i = 0; i < obj.NumEdges(); ++i)
		if (p.Intersects(obj.Edge(i)))
			return true;

	// Test for each edge of this polyhedron whether the AABB/OBB intersects it.
	for(size_t i = 0; i < p.f.size(); ++i)
	{
		assert(!p.f[i].v.empty()); // Cannot have degenerate faces here, and for performance reasons, don't start checking for this condition in release mode!
		int v0 = p.f[i].v.back();
		float3 l0 = p.v[v0];
		for(size_t j = 0; j < p.f[i].v.size(); ++j)
		{
			int v1 = p.f[i].v[j];
			float3 l1 = p.v[v1];
			if (v0 < v1 && obj.Intersects(LineSegment(l0, l1))) // If v0 < v1, then this line segment is the canonical one.
				return true;
			l0 = l1;
			v0 = v1;
		}
	}

	return false;
}

bool Polyhedron::Intersects(const AABB &aabb) const
{
	return PolyhedronIntersectsAABB_OBB(*this, aabb);
}

bool Polyhedron::Intersects(const OBB &obb) const
{
	return PolyhedronIntersectsAABB_OBB(*this, obb);
}

bool Polyhedron::Intersects(const Triangle &triangle) const
{
	return PolyhedronIntersectsAABB_OBB(*this, triangle);
}

bool Polyhedron::Intersects(const Polygon &polygon) const
{
	return Intersects(polygon.ToPolyhedron());
}

bool Polyhedron::Intersects(const Frustum &frustum) const
{
	return PolyhedronIntersectsAABB_OBB(*this, frustum);
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
	return pt.DistanceSq(ptOnLineSegment) <= capsule.r * capsule.r;
}

bool Polyhedron::IntersectsConvex(const Line &line) const
{
	float tFirst = -FLT_MAX;
	float tLast = FLT_MAX;
	return ClipLineSegmentToConvexPolyhedron(line.pos, line.dir, tFirst, tLast);
}

bool Polyhedron::IntersectsConvex(const Ray &ray) const
{
	float tFirst = 0.f;
	float tLast = FLT_MAX;
	return ClipLineSegmentToConvexPolyhedron(ray.pos, ray.dir, tFirst, tLast);
}

bool Polyhedron::IntersectsConvex(const LineSegment &lineSegment) const
{
	float tFirst = 0.f;
	float tLast = 1.f;
	return ClipLineSegmentToConvexPolyhedron(lineSegment.a, lineSegment.b - lineSegment.a, tFirst, tLast);
}

void Polyhedron::MergeConvex(const float3 &point)
{
//	LOGI("mergeconvex.");
	std::set<std::pair<int, int> > deletedEdges;
	std::map<std::pair<int, int>, int> remainingEdges;

	for(size_t i = 0; i < v.size(); ++i)
		if (point.DistanceSq(v[i]) < 1e-3f)
			return;

//	bool hadDisconnectedHorizon = false;

	for(int i = 0; i < (int)f.size(); ++i)
	{
		// Delete all faces that don't contain the given point. (they have point in their positive side)
		Plane p = FacePlane(i);
		Face &face = f[i];
		if (p.SignedDistance(point) > 1e-5f)
		{
			bool isConnected = (deletedEdges.empty());

			int v0 = face.v.back();
			for(size_t j = 0; j < face.v.size() && !isConnected; ++j)
			{
				int v1 = face.v[j];
				if (deletedEdges.find(std::make_pair(v1, v0)) != deletedEdges.end())
				{
					isConnected = true;
					break;
				}
				v0 = v1;
			}

			if (isConnected)
			{
				v0 = face.v.back();
				for(size_t j = 0; j < face.v.size(); ++j)
				{
					int v1 = face.v[j];
					deletedEdges.insert(std::make_pair(v0, v1));
			//		LOGI("Edge %d,%d is to be deleted.", v0, v1);
					v0 = v1;
				}
		//		LOGI("Deleting face %d: %s. Distance to vertex %f", i, face.ToString().c_str(), p.SignedDistance(point));
				std::swap(f[i], f.back());
				f.pop_back();
				--i;
				continue;
			}
//			else
//				hadDisconnectedHorizon = true;
		}

		int v0 = face.v.back();
		for(size_t j = 0; j < face.v.size(); ++j)
		{
			int v1 = face.v[j];
			remainingEdges[std::make_pair(v0, v1)] = i;
	//		LOGI("Edge %d,%d is to be deleted.", v0, v1);
			v0 = v1;
		}

	}

	// The polyhedron contained our point, nothing to merge.
	if (deletedEdges.empty())
		return;

	// Add the new point to this polyhedron.
//	if (!v.back().Equals(point))
		v.push_back(point);

/*
	// Create a look-up index of all remaining uncapped edges of the polyhedron.
	std::map<std::pair<int,int>, int> edgesToFaces;
	for(size_t i = 0; i < f.size(); ++i)
	{
		Face &face = f[i];
		int v0 = face.v.back();
		for(size_t j = 0; j < face.v.size(); ++j)
		{
			int v1 = face.v[j];
			edgesToFaces[std::make_pair(v1, v0)] = i;
			v0 = v1;
		}
	}
*/
	// Now fix all edges by adding new triangular faces for the point.
//	for(size_t i = 0; i < deletedEdges.size(); ++i)
	for(std::set<std::pair<int, int> >::iterator iter = deletedEdges.begin(); iter != deletedEdges.end(); ++iter)
	{
		std::pair<int, int> opposite = std::make_pair(iter->second, iter->first);
		if (deletedEdges.find(opposite) != deletedEdges.end())
			continue;

//		std::map<std::pair<int,int>, int>::iterator iter = edgesToFaces.find(deletedEdges[i]);
//		std::map<std::pair<int,int>, int>::iterator iter = edgesToFaces.find(deletedEdges[i]);
//		if (iter != edgesToFaces.end())
		{
			// If the adjoining face is planar to the triangle we'd like to add, instead extend the face to enclose
			// this vertex.
			//float3 newTriangleNormal = (v[v.size()-1]-v[iter->second]).Cross(v[iter->first]-v[iter->second]).Normalized();

			std::map<std::pair<int, int>, int>::iterator existing = remainingEdges.find(opposite);
			assert(existing != remainingEdges.end());
			MARK_UNUSED(existing);

#if 0			
			int adjoiningFace = existing->second;

			if (FaceNormal(adjoiningFace).Dot(newTriangleNormal) >= 0.99999f) ///\todo float3::IsCollinear
			{
				bool added = false;
				Face &adjoining = f[adjoiningFace];
				for(size_t i = 0; i < adjoining.v.size(); ++i)
					if (adjoining.v[i] == iter->second)
					{
						adjoining.v.insert(adjoining.v.begin() + i + 1, v.size()-1);
						added = true;
						/*
						int prev2 = (i + adjoining.v.size() - 1) % adjoining.v.size();
						int prev = i;
						int cur = i + 1;
						int next = (i + 2) % adjoining.v.size();
						int next2 = (i + 3) % adjoining.v.size();

						if (float3::AreCollinear(v[prev2], v[prev], v[cur]))
							adjoining.v.erase(adjoining.v.begin() + prev);
						else if (float3::AreCollinear(v[prev], v[cur], v[next]))
							adjoining.v.erase(adjoining.v.begin() + cur);
						else if (float3::AreCollinear(v[cur], v[next], v[next2]))
							adjoining.v.erase(adjoining.v.begin() + next2);
							*/

						break;
					}
				assert(added);
				assume(added);
			}
			else
#endif
//			if (!v[deletedEdges[i].first].Equals(point) && !v[deletedEdges[i].second].Equals(point))
			{
				Face tri;
				tri.v.push_back(iter->second);
				tri.v.push_back((int)v.size()-1);
				tri.v.push_back(iter->first);
				f.push_back(tri);
	//			LOGI("Added face %d: %s.", (int)f.size()-1, tri.ToString().c_str());
			}
		}
	}

#define mathasserteq(lhs, op, rhs) do { if (!((lhs) op (rhs))) { LOGE("Condition %s %s %s (%g %s %g) failed!", #lhs, #op, #rhs, (double)(lhs), #op, (double)(rhs)); assert(false); } } while(0)

//	mathasserteq(NumVertices() + NumFaces(), ==, 2 + NumEdges());
	assert(FaceIndicesValid());
//	assert(EulerFormulaHolds());
//	assert(IsClosed());
//	assert(FacesAreNondegeneratePlanar());
//	assert(IsConvex());

//	if (hadDisconnectedHorizon)
//		MergeConvex(point);
}

void Polyhedron::Translate(const float3 &offset)
{
	for(size_t i = 0; i < v.size(); ++i)
		v[i] += offset;
}

void Polyhedron::Transform(const float3x3 &transform)
{
	if (!v.empty())
		transform.BatchTransform(&v[0], (int)v.size());
}

void Polyhedron::Transform(const float3x4 &transform)
{
	if (!v.empty())
		transform.BatchTransformPos(&v[0], (int)v.size());
}

void Polyhedron::Transform(const float4x4 &transform)
{
	for(size_t i = 0; i < v.size(); ++i)
		v[i] = transform.MulPos(v[i]); ///\todo Add float4x4::BatchTransformPos.
}

void Polyhedron::Transform(const Quat &transform)
{
	for(size_t i = 0; i < v.size(); ++i)
		v[i] = transform * v[i];
}

void Polyhedron::OrientNormalsOutsideConvex()
{
	float3 center = v[0];
	for(size_t i = 1; i < v.size(); ++i)
		center += v[i];

	center /= (float)v.size();
	for(int i = 0; i < (int)f.size(); ++i)
		if (FacePlane(i).SignedDistance(center) > 0.f)
			f[i].FlipWindingOrder();
}

/// Edge from v1->v2.
struct AdjEdge
{
//	int v1;
//	int v2;
	int f1; // The face that has v1->v2.
	int f2; // The face that has v2->v1.
};

#include <list>

struct CHullHelp
{
	std::map<std::pair<int,int>, AdjEdge> edges;
	std::list<int> livePlanes;
};

Polyhedron Polyhedron::ConvexHull(const float3 *pointArray, int numPoints)
{
	///\todo Check input ptr and size!
	std::set<int> extremes;

	const float3 dirs[] =
	{
		float3(1,0,0), float3(0,1,0), float3(0,0,1),
		float3(1,1,0), float3(1,0,1), float3(0,1,1),
		float3(1,1,1)
	};

	for(size_t i = 0; i < ARRAY_LENGTH(dirs); ++i)
	{
		int idx1, idx2;
		OBB::ExtremePointsAlongDirection(dirs[i], pointArray, numPoints, idx1, idx2);
		extremes.insert(idx1);
		extremes.insert(idx2);
	}

	Polyhedron p;
	assume(extremes.size() >= 4); ///\todo Fix this case!
	int i = 0;
	std::set<int>::iterator iter = extremes.begin();
	for(; iter != extremes.end() && i < 4; ++iter, ++i)
		p.v.push_back(pointArray[*iter]);

	Face f;
	f.v.resize(3);
	f.v[0] = 0; f.v[1] = 1; f.v[2] = 2; p.f.push_back(f);
	f.v[0] = 0; f.v[1] = 1; f.v[2] = 3; p.f.push_back(f);
	f.v[0] = 0; f.v[1] = 2; f.v[2] = 3; p.f.push_back(f);
	f.v[0] = 1; f.v[1] = 2; f.v[2] = 3; p.f.push_back(f);
	p.OrientNormalsOutsideConvex(); // Ensure that the winding order of the generated tetrahedron is correct for each face.

//	assert(p.IsClosed());
	//assert(p.IsConvex());
	assert(p.FaceIndicesValid());
	assert(p.EulerFormulaHolds());
//	assert(p.FacesAreNondegeneratePlanar());

	CHullHelp hull;
	for(int j = 0; j < (int)p.f.size(); ++j)
		hull.livePlanes.push_back(j);

	// For better performance, merge the remaining extreme points first.
	for(; iter != extremes.end(); ++iter)
	{
		p.MergeConvex(pointArray[*iter]);

		mathassert(p.FaceIndicesValid());
//		mathassert(p.IsClosed());
//		mathassert(p.FacesAreNondegeneratePlanar());
//		mathassert(p.IsConvex());
	}

	// Merge all the rest of the points.
	for(int j = 0; j < numPoints; ++j)
	{
		if (p.f.size() > 5000 && (j & 255) == 0)
			LOGI("Mergeconvex %d/%d, #vertices %d, #faces %d", j, numPoints, (int)p.v.size(), (int)p.f.size());
		p.MergeConvex(pointArray[i]);

		mathassert(p.FaceIndicesValid());
//		mathassert(p.IsClosed());
//		mathassert(p.FacesAreNondegeneratePlanar());
		//mathassert(p.IsConvex());

//		if (p.f.size() > 5000)
//			break;
	}

	return p;
}

/// See http://paulbourke.net/geometry/platonic/
Polyhedron Polyhedron::Tetrahedron(const float3 &centerPos, float scale, bool ccwIsFrontFacing)
{
	const float3 vertices[4] = { float3(1,1,1),
	                             float3(-1,1,-1),
	                             float3(1,-1,-1),
	                             float3(-1,-1,1) };
	const int faces[4][3] = { { 0, 1, 2 },
	                          { 1, 3, 2 },
	                          { 0, 2, 3 },
	                          { 0, 3, 1 } };

	scale /= 2.f;
	Polyhedron p;

	for(int i = 0; i < 4; ++i)
		p.v.push_back(vertices[i]*scale + centerPos);

	for(int i = 0; i < 4; ++i)
	{
		Face f;
		for(int j = 0; j < 3; ++j)
			f.v.push_back(faces[i][j]);
		p.f.push_back(f);
	}

	if (!ccwIsFrontFacing)
		p.FlipWindingOrder();

	return p;
}

/// See http://paulbourke.net/geometry/platonic/
Polyhedron Polyhedron::Octahedron(const float3 &centerPos, float scale, bool ccwIsFrontFacing)
{
	float a = 1.f / (2.f * Sqrt(2.f));
	float b = 0.5f;

	const float3 vertices[6] = { float3(-a, 0, a),
	                             float3(-a, 0,-a),
	                             float3( 0, b, 0),
	                             float3( a, 0,-a),
	                             float3( 0,-b, 0),
	                             float3( a, 0, a) };
	const int faces[8][3] = { { 0, 1, 2 },
	                          { 1, 3, 2 },
	                          { 3, 5, 2 },
	                          { 5, 0, 2 },
	                          { 3, 1, 4 },
	                          { 1, 0, 4 },
	                          { 5, 3, 4 },
	                          { 0, 5, 4 } };

	scale /= 2.f;
	Polyhedron p;

	for(int i = 0; i < 6; ++i)
		p.v.push_back(vertices[i]*scale + centerPos);

	for(int i = 0; i < 8; ++i)
	{
		Face f;
		for(int j = 0; j < 3; ++j)
			f.v.push_back(faces[i][j]);
		p.f.push_back(f);
	}

	if (!ccwIsFrontFacing)
		p.FlipWindingOrder();

	return p;
}

/// See http://paulbourke.net/geometry/platonic/
Polyhedron Polyhedron::Hexahedron(const float3 &centerPos, float scale, bool ccwIsFrontFacing)
{
	AABB aabb(float3(-1,-1,-1), float3(1,1,1));
	aabb.Scale(float3::zero, scale * 0.5f);
	aabb.Translate(centerPos);
	Polyhedron p = aabb.ToPolyhedron();
	if (ccwIsFrontFacing)
		p.FlipWindingOrder();
	return p;
}

/// See http://paulbourke.net/geometry/platonic/
Polyhedron Polyhedron::Icosahedron(const float3 &centerPos, float scale, bool ccwIsFrontFacing)
{
	float a = 0.5f;
	float phi = (1.f + Sqrt(5.f)) / 2.f;
	float b = 1.f / (2.f * phi);

	const float3 vertices[12] = { float3( 0,  b, -a),
	                              float3( b,  a,  0),
	                              float3(-b,  a,  0),
	                              float3( 0,  b,  a),
	                              float3( 0, -b,  a),
	                              float3(-a,  0,  b),
	                              float3( a,  0,  b),
	                              float3( 0, -b, -a),
	                              float3(-a,  0, -b),
	                              float3(-b, -a,  0),
	                              float3( b, -a,  0),
	                              float3( a,  0, -b) };
	const int faces[20][3] = { { 0,  1,  2 },
	                           { 3,  2,  1 },
	                           { 3,  4,  5 },
	                           { 3,  6,  4 },
	                           { 0,  7, 11 },
	                           { 0,  8,  7 },
	                           { 4, 10,  9 },
	                           { 7,  9, 10 },
	                           { 2,  5,  8 },
	                           { 9,  8,  5 },
	                           { 1, 11,  6 },
	                           { 10, 6, 11 },
	                           { 3,  5,  2 },
	                           { 3,  1,  6 },
	                           { 0,  2,  8 },
	                           { 0, 11,  1 },
	                           { 7,  8,  9 },
	                           { 7, 10, 11 },
	                           { 4,  9,  5 },
	                           { 4,  6, 10 } };

	Polyhedron p;

	for(int i = 0; i < 12; ++i)
		p.v.push_back(vertices[i]*scale + centerPos);

	for(int i = 0; i < 20; ++i)
	{
		Face f;
		for(int j = 0; j < 3; ++j)
			f.v.push_back(faces[i][j]);
		p.f.push_back(f);
	}

	if (!ccwIsFrontFacing)
		p.FlipWindingOrder();

	return p;
}

/// See http://paulbourke.net/geometry/platonic/
Polyhedron Polyhedron::Dodecahedron(const float3 &centerPos, float scale, bool ccwIsFrontFacing)
{
	float phi = (1.f + Sqrt(5.f)) / 2.f;
	float b = 1.f / phi;
	float c = 2.f - phi;

	const float3 vertices[20] = { float3( c,  0,  1),
	                              float3(-c,  0,  1),
	                              float3(-b,  b,  b),
	                              float3( 0,  1,  c),
	                              float3( b,  b,  b),
	                              float3( b, -b,  b),
	                              float3( 0, -1,  c),
	                              float3(-b, -b,  b),
	                              float3( 0, -1, -c),
	                              float3( b, -b, -b),
	                              float3(-c,  0, -1),
	                              float3( c,  0, -1),
	                              float3(-b, -b, -b),
	                              float3( b,  b, -b),
	                              float3( 0,  1, -c),
	                              float3(-b,  b, -b),
	                              float3( 1,  c,  0),
	                              float3(-1,  c,  0),
	                              float3(-1, -c,  0),
	                              float3( 1, -c,  0) };

	const int faces[12][5] = { {  0,  1,  2,  3,  4 },
	                           {  1,  0,  5,  6,  7 },
	                           { 11, 10, 12,  8,  9 },
	                           { 10, 11, 13, 14, 15 },
	                           { 13, 16,  4,  3, 14 }, // Note: The winding order of this face was flipped from PBourke's original representation.
	                           {  2, 17, 15, 14,  3 }, //       Winding order flipped.
	                           { 12, 18,  7,  6,  8 }, //       Winding order flipped.
	                           {  5, 19,  9,  8,  6 }, //       Winding order flipped.
	                           { 16, 19,  5,  0,  4 },
	                           { 19, 16, 13, 11,  9 },
	                           { 17, 18, 12, 10, 15 },
	                           { 18, 17,  2,  1,  7 } };

	scale /= 2.f;
	Polyhedron p;

	for(int i = 0; i < 20; ++i)
		p.v.push_back(vertices[i]*scale + centerPos);

	for(int i = 0; i < 12; ++i)
	{
		Face f;
		for(int j = 0; j < 5; ++j)
			f.v.push_back(faces[i][j]);
		p.f.push_back(f);
	}

	if (!ccwIsFrontFacing)
		p.FlipWindingOrder();

	return p;
}

int IntTriCmp(int a, int b)
{
	return a - b;
}

/** Does a binary search on the array list that is sorted in ascending order.
	@param list [in] A pointer to the array to search.
	@param numItems The number of elements in the array 'list'.
	@param value The element to search for.
	@param cmp The comparison operator to use. The comparison function is of form
		int CmpFunc(const T &a, const T &b), and it tests the mutual order of a and b.
		It should return -1 if a < b, +1 if a > b, and 0 if a == b.
	@return The index where a matching element lies, or -1 if not found. Note that if there are more than
	        one matching element, the first that is found is returned. */
template<typename T, typename CmpFunc>
int ArrayBinarySearch(const T *list, int numItems, const T &value, CmpFunc &cmp)
{
	int left = 0;
	int right = numItems-1;
	int order = cmp(list[left], value);
	if (order > 0) return -1;
	if (order == 0) return left;

	order = cmp(list[right], value);
	if (order < 0) return -1;
	if (order == 0) return right;

	int round = 0; // Counter to alternatingly round up or down.
	do
	{
		int middle = (left + right + round) / 2;
		round = (round+1)&1;
		order = cmp(list[middle], value);
		if (order == 0)
			return middle;
		if (order < 0)
			left = middle;
		else right = middle;
	} while(left < right);
	return -1;
}

void Polyhedron::RemoveRedundantVertices()
{
	std::set<int> usedVertices;

	// Gather all used vertices.
	for(size_t i = 0; i < f.size(); ++i)
		for(size_t j = 0; j < f[i].v.size(); ++j)
			usedVertices.insert(f[i].v[j]);

	// Turn the used vertices set into a vector for random access.
	std::vector<int> usedVerticesArray;
	usedVerticesArray.reserve(usedVertices.size());
	for(std::set<int>::iterator iter = usedVertices.begin(); iter != usedVertices.end(); ++iter)
		usedVerticesArray.push_back(*iter);

	// Shift all face indices to point to the new vertex array.
	for(size_t i = 0; i < f.size(); ++i)
		for(size_t j = 0; j < f[i].v.size(); ++j)
		{
			int oldIndex = f[i].v[j];
			int newIndex = ArrayBinarySearch(&usedVerticesArray[0], (int)usedVerticesArray.size(), oldIndex, IntTriCmp);
			assert(newIndex != -1);
			f[i].v[j] = newIndex;
		}

	// Delete all unused vertices from the vertex array.
	for(size_t i = 0; i < usedVerticesArray.size(); ++i)
		v[i] = v[usedVerticesArray[i]];
	v.resize(usedVerticesArray.size());
	
	assert(FaceIndicesValid());
}

void Polyhedron::MergeAdjacentPlanarFaces()
{

}

std::vector<Triangle> Polyhedron::Triangulate() const
{
	std::vector<Triangle> outTriangleList;
	for(int i = 0; i < NumFaces(); ++i)
	{
		Polygon p = FacePolygon(i);
		std::vector<Triangle> tris = p.Triangulate();
		outTriangleList.insert(outTriangleList.end(), tris.begin(), tris.end());
	}
	return outTriangleList;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void Polyhedron::Triangulate(VertexBuffer &vb, bool ccwIsFrontFacing) const
{
	for(int i = 0; i < NumFaces(); ++i)
	{
		Polygon p = FacePolygon(i);
		std::vector<Triangle> tris = p.Triangulate();
		int idx = vb.AppendVertices(3*(int)tris.size());
		for(size_t j = 0; j < tris.size(); ++j)
		{
			vb.Set(idx, VDPosition, float4(tris[j].a, 1.f));
			if (ccwIsFrontFacing)
			{
				vb.Set(idx+1, VDPosition, float4(tris[j].c, 1.f));
				vb.Set(idx+2, VDPosition, float4(tris[j].b, 1.f));
			}
			else
			{
				vb.Set(idx+1, VDPosition, float4(tris[j].b, 1.f));
				vb.Set(idx+2, VDPosition, float4(tris[j].c, 1.f));
			}
			// Generate flat normals if VB has space for normals.
			if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			{
				float3 normal = ccwIsFrontFacing ? tris[j].NormalCCW() : tris[j].NormalCW();
				vb.Set(idx, VDNormal, float4(normal, 0.f));
				vb.Set(idx+1, VDNormal, float4(normal, 0.f));
				vb.Set(idx+2, VDNormal, float4(normal, 0.f));
			}
			idx += 3;
		}
	}
}

void Polyhedron::ToLineList(VertexBuffer &vb) const
{
	std::vector<LineSegment> edges = Edges();

	int startIndex = vb.AppendVertices((int)edges.size()*2);
	for(int i = 0; i < (int)edges.size(); ++i)
	{
		vb.Set(startIndex+2*i, VDPosition, float4(edges[i].a, 1.f));
		vb.Set(startIndex+2*i+1, VDPosition, float4(edges[i].b, 1.f));
	}
}
#endif

Polyhedron operator *(const float3x3 &transform, const Polyhedron &polyhedron)
{
	Polyhedron p(polyhedron);
	p.Transform(transform);
	return p;
}

Polyhedron operator *(const float3x4 &transform, const Polyhedron &polyhedron)
{
	Polyhedron p(polyhedron);
	p.Transform(transform);
	return p;
}

Polyhedron operator *(const float4x4 &transform, const Polyhedron &polyhedron)
{
	Polyhedron p(polyhedron);
	p.Transform(transform);
	return p;
}

Polyhedron operator *(const Quat &transform, const Polyhedron &polyhedron)
{
	Polyhedron p(polyhedron);
	p.Transform(transform);
	return p;
}

MATH_END_NAMESPACE
