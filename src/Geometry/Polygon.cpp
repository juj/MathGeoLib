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

#include "Geometry/AABB.h"
#include "Geometry/OBB.h"
#include "Geometry/Frustum.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Plane.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Triangle.h"
#include "Math/MathFunc.h"
#include "Math/float2.h"

MATH_BEGIN_NAMESPACE

int Polygon::NumVertices() const
{
	return p.size();
}

int Polygon::NumEdges() const
{
	return p.size();
}

float3 Polygon::Vertex(int vertexIndex) const
{
    return p[vertexIndex];
}

LineSegment Polygon::Edge(int i) const
{
	if (p.size() == 0)
		return LineSegment(float3::nan, float3::nan);
	if (p.size() == 1)
		return LineSegment(p[0], p[0]);
	return LineSegment(p[i], p[(i+1)%p.size()]);
}

LineSegment Polygon::Edge2D(int i) const
{
	if (p.size() == 0)
		return LineSegment(float3::nan, float3::nan);
	if (p.size() == 1)
        return LineSegment(float3::zero, float3::zero);
    return LineSegment(float3(MapTo2D(i), 0), float3(MapTo2D((i+1)%p.size()), 0));
}

bool Polygon::DiagonalExists(int i, int j) const
{
    assume(IsPlanar());
    assume(i != j);
    if (i > j)
        Swap(i, j);
    if ((i+1)%p.size() == j)
        return true;

    Plane polygonPlane = PlaneCCW();
    LineSegment diagonal = polygonPlane.Project(LineSegment(p[i], p[j]));

    // First check that this diagonal line is not intersected by an edge of this polygon.
    for(size_t k = 0; k < p.size(); ++k)
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
    return Cross(BasisU(), PlaneCCW().normal).Normalized();
}

LineSegment Polygon::Diagonal(int i, int j) const
{
	return LineSegment(p[i], p[j]);
}

bool Polygon::IsConvex() const
{
    assume(IsPlanar());
    if (p.size() == 0)
        return false;
	if (p.size() <= 3)
		return true;
    size_t i = p.size()-2;
    size_t j = p.size()-1;
    size_t k = 0;

    while(k < p.size())
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
    return MapTo2D(p[i]);
}

float2 Polygon::MapTo2D(const float3 &point) const
{
    float3 basisU = BasisU();
    float3 basisV = BasisV();
    float3 pt = point - p[0];
    return float2(Dot(pt, basisU), Dot(pt, basisV));
}

float3 Polygon::MapFrom2D(const float2 &point) const
{
    return p[0] + point.x * BasisU() + point.y * BasisV();
}

bool Polygon::IsPlanar(float epsilon) const
{
    if (p.size() == 0)
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
    for(size_t i = 0; i < p.size(); ++i)
    {
        LineSegment si = plane.Project(Edge(i));
        for(size_t j = i+2; j < p.size(); ++j)
        {
            if (i == 0 && j == p.size() - 1)
                continue; // These two edges are consecutive and share a vertex. Don't check that pair.
            LineSegment sj = plane.Project(Edge(j));
            if (si.Intersects(sj))
                return false;
        }
    }
    return true;
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

bool Polygon::Contains(const float3 &worldSpacePoint, float polygonThickness) const
{
    if (PlaneCCW().Distance(worldSpacePoint) > polygonThickness)
        return false;
    return Contains2D(MapTo2D(worldSpacePoint));
}

bool Polygon::Contains(const Polygon &worldSpacePolygon, float polygonThickness) const
{
    for(int i = 0; i < worldSpacePolygon.NumVertices(); ++i)
        if (!Contains(worldSpacePolygon.Vertex(i), polygonThickness))
            return false;
    return true;
}

bool Polygon::Contains2D(const float2 &localSpacePoint) const
{
    if (p.size() < 3)
        return false;

    LineSegment l(float3(localSpacePoint, 0), float3(localSpacePoint,0) + float3(1,1,0).Normalized());
    int numIntersections = 0;
    for(size_t i = 0; i < p.size(); ++i)
        if (Edge2D(i).Intersects(l))
            ++numIntersections;
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

    for(size_t i = 0; i < p.size(); ++i)
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

    if (!Contains2D(localSpaceLineSegment.a.xy()) || !Contains2D(localSpaceLineSegment.b.xy()))
        return false;

    for(size_t i = 0; i < p.size(); ++i)
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
    float d;
    if (!PlaneCCW().Intersects(lineSegment, &d))
        return false;
    return Contains(lineSegment.GetPoint(d));
}

bool Polygon::Intersects(const Plane &plane) const
{
    Line intersectLine;
    bool intersects = plane.Intersects(PlaneCCW(), &intersectLine);
    if (!intersects)
        return false;
    return Intersects(intersectLine);
}
/*
bool Polygon::Intersects(const AABB &aabb) const
{
    return aabb.Intersects(*this);
}
*/
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

float3 Polygon::ClosestPointConvex(const float3 &point) const
{
    assume(IsPlanar());

    float3 ptOnPlane = PlaneCCW().Project(point);
    if (Contains(ptOnPlane))
        return ptOnPlane;

    for(int i = 0; i < NumEdges(); ++i)
    {
        Plane edgePlane = EdgePlane(i);
        if (edgePlane.SignedDistance(ptOnPlane) > 0.f)
            ptOnPlane = edgePlane.Project(ptOnPlane);
    }
    return ptOnPlane;
}

float3 Polygon::EdgeNormal(int edgeIndex) const
{
    return Cross(Edge(edgeIndex).Dir(), PlaneCCW().normal).Normalized();
}

Plane Polygon::EdgePlane(int edgeIndex) const
{
    return Plane(Edge(edgeIndex).a, EdgeNormal(edgeIndex));
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
    float3 centroid = float3::zero;
    for(int i = 0; i < NumVertices(); ++i)
        centroid += Vertex(i);
    return centroid / (float)NumVertices();
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
        poly.f[0].v.push_back(NumVertices()-1-i);
    }
    return poly;
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

MATH_END_NAMESPACE
