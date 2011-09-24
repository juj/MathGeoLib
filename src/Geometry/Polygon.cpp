/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/

#ifdef MATH_ENABLE_STL_SUPPORT
#include <cassert>
#include <utility>
#endif

#include "Geometry/Polygon.h"
#include "Geometry/Plane.h"
#include "Geometry/Line.h"
#include "Geometry/LineSegment.h"
#include "Math/MathFunc.h"

int Polygon::NumEdges() const
{
	return points.size();
}

LineSegment Polygon::Edge(int i) const
{
	if (points.size() == 0)
		return LineSegment(float3::nan, float3::nan);
	if (points.size() == 1)
		return LineSegment(points[0], points[0]);
	return LineSegment(points[i], points[(i+1)%points.size()]);
}

bool Polygon::DiagonalExists(int i, int j) const
{
	assume(false && "Not implemented!");
	return false;
}

LineSegment Polygon::Diagonal(int i, int j) const
{
	return LineSegment(points[i], points[j]);
}

bool Polygon::IsConvex() const
{
	assume(false && "Not implemented!");
	return false;
}

bool Polygon::IsPlanar(float epsilon) const
{
	if (points.size() <= 3)
		return true;
	Plane p(points[0], points[1], points[2]);
	for(size_t i = 3; i < points.size(); ++i)
		if (p.Distance(points[i]) > epsilon)
			return false;
	return true;
}

bool Polygon::IsSimple() const
{
	assume(false && "Not implemented!");
	return false;
}

float3 Polygon::NormalCCW() const
{
    ///\todo Optimize temporaries.
    return PlaneCCW().normal;
}

float3 Polygon::NormalCW() const
{
    ///\todo Optimize temporaries.
    return PlaneCW().normal;
}

Plane Polygon::PlaneCCW() const
{
	if (points.size() >= 3)
		return Plane(points[0], points[1], points[2]);
	if (points.size() == 2)
		return Plane(Line(points[0], points[1]), (points[0]-points[1]).Perpendicular());
	if (points.size() == 1)
		return Plane(points[0], float3(0,1,0));
	return Plane();
}

Plane Polygon::PlaneCW() const
{
	if (points.size() >= 3)
		return Plane(points[0], points[2], points[1]);
	if (points.size() == 2)
		return Plane(Line(points[0], points[1]), (points[0]-points[1]).Perpendicular());
	if (points.size() == 1)
		return Plane(points[0], float3(0,1,0));
	return Plane();
}

/*
/// Returns true if the edges of this polygon self-intersect.
bool IsSelfIntersecting() const;

/// Projects all vertices of this polygon to the given plane.
void ProjectToPlane(const Plane &plane);

/// Returns true if the edges of this polygon self-intersect when viewed from the given direction.
bool IsSelfIntersecting(const float3 &viewDirection) const;

bool Contains(const float3 &point, const float3 &viewDirection) const;

/// Returns the surface area of this polygon.
float Area() const;

/// Returns the total edge length of this polygon.
float Perimeter() const;

float3 Centroid() const;

/// Returns true if the given vertex is a concave vertex. Otherwise the vertex is a convex vertex.
bool IsConcaveVertex(int i) const;

/// Computes the conves hull of this polygon.
Polygon ConvexHull() const;

bool IsSupportingPoint(int i) const;

bool IsSupportingPoint(const float3 &point) const;

/// Returns true if the quadrilateral defined by the four points is convex (and not concave or bowtie).
static bool IsConvexQuad(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD);
*/