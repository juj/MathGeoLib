/** @file Polygon.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

//#ifdef MATH_ENABLE_STL_SUPPORT
#include <vector>
//#endif

MATH_BEGIN_NAMESPACE

/// Represents a two-dimensional closed surface in 3D space.
/** A polygon is defined by N endpoints, or corner vertices. To be a valid polygon, there must be
   at least 3 vertices (a triangle).

   Well-formed polygons are always planar, i.e. all the vertices lie on the same plane. It is possible
   to store non-planar Polygons in this structure, but their representation is ambiguous, and for all practical 
   purposes, should be avoided. */
class Polygon
{
public:
    /// @note The default ctor does not initialize any member values.
    Polygon() {}

    /// Stores the vertices of this polygon.
    std::vector<float3> p;

    /// Returns the number of edges in this polygon. Since the polygon is always closed and connected,
    /// ther number of edges is equal to the number of vertices.
    int NumEdges() const;

    /// Returns the number of vertices in this polygon. Since the polygon is always closed and connected,
    /// ther number of edges is equal to the number of vertices.
    int NumVertices() const;

    /// Returns a vertex of this polygon.
    float3 Vertex(int vertexIndex) const;

    /// Returns the given line segment between two vertices.
    /// @param i [0, NumEdges()-1].
    LineSegment Edge(int edgeIndex) const;

    /// Returns the given local space line segment between two vertices.
    /// The z-coordinates of the returned line segment are zero.
    LineSegment Edge2D(int edgeIndex) const;

    float3 EdgeNormal(int edgeIndex) const;
    Plane EdgePlane(int edgeIndex) const;

    /// Returns true if the given diagonal exists.
    /// If this function returns true, the diagonal that joins the two given vertices lies inside this polygon.
    /// This function may only be called if this Polygon is planar.
    bool DiagonalExists(int vertexIndex1, int vertexIndex2) const;

    /// Returns the diagonal that joins the two given vertices.
    /// If |i-j| == 1, then this returns an edge of this Polygon.
    /// If i==j, then a degenerate line segment is returned.
    /// Otherwise, the line segment that joins the two given vertices is returned. Note that if the polygon is not planar or convex,
    /// this line segment might not lie inside the polygon.
    LineSegment Diagonal(int vertexIndex1, int vertexIndex2) const;

    /// Tests if this polygon is convex.
    bool IsConvex() const;

    /// Tests if all the points of this polygon lie in the same plane.
    bool IsPlanar(float epsilon = 1e-3f) const;

    /// Returns true if no two nonconsecutive edges have a point in common.
    /// In other words, a planar polygon is simple if its edges do not self-intersect.
    /// This function assumes that the polygon is planar.
    bool IsSimple() const;

    /// Assumes that the first two points (p[0] and p[1]) are not degenerate and equal.
    float3 BasisU() const;
    float3 BasisV() const;

    /// Returns the given vertex of this polygon mapped to a local 2D space on this polygon.
    /// In this local space, the i=0 point lies at the coordinate (0,0).
    /// The positive x-coordinate runs in the direction specified by points[0]->points[1].
    float2 MapTo2D(int i) const;
    float2 MapTo2D(const float3 &point) const;
    /// Given a 2D point in the local space, returns the corresponding 3D point in the world space.
    float3 MapFrom2D(const float2 &point) const;

    float3 NormalCW() const;
    float3 NormalCCW() const;

    /// Returns the plane this polygon is contained in. Only call this function if this Polygon is planar.
    Plane PlaneCW() const;
    Plane PlaneCCW() const;

    /// Returns true if the edges of this polygon self-intersect.
    bool IsSelfIntersecting() const;

    /// Projects all vertices of this polygon to the given plane.
    void ProjectToPlane(const Plane &plane);

    /// Returns true if the edges of this polygon self-intersect when viewed from the given direction.
    bool IsSelfIntersecting(const float3 &viewDirection) const;

    /// Returns true if there exists edges (p_{i-1}, p_i) and (p_i, p_{i+1}) which are collinear.
    bool HasCollinearEdges() const;

    /// Returns true if the given point lies inside this polygon.
    bool Contains(const float3 &worldSpacePoint, float polygonThickness = 1e-3f) const;
    bool Contains(const LineSegment &worldSpaceLineSegment, float polygonThickness = 1e-3f) const;
    bool Contains(const Triangle &worldSpaceTriangle, float polygonThickness = 1e-3f) const;
    bool Contains(const Polygon &worldSpacePolygon, float polygonThickness = 1e-3f) const;
    ///\todo Add RTCD, p. 202. 
    //bool ContainsConvex(const float3 &worldSpacePoint, float polygonThickness = 1e-3f) const;

    /// Returns true if the given 2D point is contained inside this polygon.
    bool Contains2D(const float2 &localSpacePoint) const;
    bool Contains2D(const LineSegment &localSpaceLineSegment) const;

    bool Intersects(const Line &line) const;
    bool Intersects(const Ray &ray) const;
    bool Intersects(const LineSegment &lineSegment) const;
    bool Intersects(const Plane &plane) const;
//    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Triangle &triangle) const;
    bool Intersects(const Polygon &polygon) const;
    bool Intersects(const Frustum &frustum) const;
    bool Intersects(const Polyhedron &polyhedron) const;

    /// Returns the closest point on this polygon to the given point.
    /// This function assumes that this polygon is convex.
    float3 ClosestPointConvex(const float3 &point) const;

    /// Returns the surface area of this polygon.
    float Area() const;

    /// Returns the total edge length of this polygon.
    float Perimeter() const;

    /// Returns the center of mass of this polygon.
    float3 Centroid() const;

    /// Converts this Polygon to a Polyhedron representation.
    /// This function will create a Polyhedron with two faces, one for the front face of this Polygon,
    /// and one for the back face. These faces will be extruded along the Polygon normal so that they
    /// lie polygonThickness units apart from each other.
    Polyhedron ToPolyhedron() const;
//    Polyhedron ToPolyhedron(float polygonThickness = 0.f) const; ///\todo Add support for this form.

    // Triangulates this Polygon using the ear-clipping method.
    std::vector<Triangle> Triangulate() const;

    AABB MinimalEnclosingAABB() const;

    /// Returns true if the given vertex is a concave vertex. Otherwise the vertex is a convex vertex.
    bool IsConcaveVertex(int i) const;

    /// Computes the conves hull of this polygon.
    Polygon ConvexHull() const;

    bool IsSupportingPoint(int i) const;

    bool IsSupportingPoint(const float3 &point) const;

    /// Returns true if the quadrilateral defined by the four points is convex (and not concave or bowtie).
    static bool IsConvexQuad(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD);
};

MATH_END_NAMESPACE

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Polygon)
Q_DECLARE_METATYPE(Polygon*)
#endif
