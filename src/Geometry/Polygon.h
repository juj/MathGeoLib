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

class Polygon
{
public:
    /// @note The default ctor does not initialize any member values.
    Polygon() {}

    std::vector<float3> points;

    /// Returns the number of edges in this polygon. Since the polygon is always closed and connected,
    /// ther number of edges is equal to the number of points.
    int NumEdges() const;

    /// Returns the given line segment between two vertices.
    /// @param i [0, NumEdges()-1].
    LineSegment Edge(int i) const;

    /// Returns true if the given diagonal exists.
    bool DiagonalExists(int i, int j) const;

    /// Returns the diagonal that joins vertices i and j.
    LineSegment Diagonal(int i, int j) const;

    /// Tests if this polygon is convex.
    bool IsConvex() const;

    /// Tests if all the points of this polygon lie in the same plane.
    bool IsPlanar(float epsilon = 1e-3f) const;

    /// Returns true if no two nonconsecutive edges have a point in common.
    bool IsSimple() const;

    /// Returns the plane this polygon fits most well into.
    Plane GetPlane() const;

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
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Polygon)
Q_DECLARE_METATYPE(Polygon*)
#endif
