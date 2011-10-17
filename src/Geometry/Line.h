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

/** @file Line.h
    @author Jukka Jylänki
	@brief Implementation for the Line geometry object. */
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A line in 3D space runs through two specified points and extends to infinity in two directions.
class Line
{
public:
    /// @note The default ctor does not initialize any member values.
    Line() {}
    Line(const float3 &pos, const float3 &dir);
    explicit Line(const Ray &ray);
    explicit Line(const LineSegment &lineSegment);

    float3 pos;
    float3 dir;

    /// Returns a point on this line.
    /// GetPoint(0) returns the point 'pos'.
    /// GetPoint with a positive value returns points towards the vector 'dir'.
    float3 GetPoint(float distance) const;

    /// Applies a transformation to this line, in-place.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Returns true if this ray contains the given point, i.e. if the squared
    /// distance to this point is smaller than the given threshold epsilon.
    bool Contains(const float3 &point, float distanceThreshold = 1e-3f) const;

    /// Returns true if this Line contains the given Ray.
    bool Contains(const Ray &ray, float epsilon = 1e-3f) const;

    /// Returns true if this Line contains the given LineSegment.
    bool Contains(const LineSegment &lineSegment, float epsilon = 1e-3f) const;

    /// Returns true if this and the given line represent the same set of points.
    bool Equals(const Line &line, float epsilon = 1e-3f) const;

    /// Returns the distance of the given point to this line.
    /// @param d [out] This element will receive the distance along this line that specifies the closest point on this line to the given point.
    float Distance(const float3 &point, float *d = 0) const;

    /// Returns the distance of the given ray/line/linesegment to this line.
    /// @param d [out] Receives the distance along this line that specifies the closest point on this line to the given point.
    /// @param d2 [out] Receives the distance along the other line that specifies the closest point on that line to this line.
    float Distance(const Ray &other, float *d, float *d2 = 0) const;
    float Distance(const Ray &other) const;
    float Distance(const Line &other, float *d, float *d2 = 0) const;
    float Distance(const Line &other) const;
    float Distance(const LineSegment &other, float *d, float *d2 = 0) const;
    float Distance(const LineSegment &other) const;
    float Distance(const Sphere &other) const;
    float Distance(const Capsule &other) const;

    bool Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const;
    bool Intersects(const Plane &plane, float *d) const;
    bool Intersects(const Sphere &s, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
    bool Intersects(const AABB &aabb, float *dNear, float *dFar) const;
    bool Intersects(const OBB &obb, float *dNear, float *dFar) const;
    bool Intersects(const Capsule &capsule) const;
    bool Intersects(const Polygon &polygon) const;
    bool Intersects(const Frustum &frustum) const;
    bool Intersects(const Polyhedron &polyhedron) const;
    bool IntersectsDisc(const Circle &disc) const;

    /// Returns the closest point on <b>this</b> line to the given object.
    float3 ClosestPoint(const float3 &targetPoint, float *d = 0) const;
    float3 ClosestPoint(const Ray &other, float *d = 0, float *d2 = 0) const;
    float3 ClosestPoint(const Line &other, float *d = 0, float *d2 = 0) const;
    float3 ClosestPoint(const LineSegment &other, float *d = 0, float *d2 = 0) const;

    float3 ClosestPoint(const Triangle &triangle, float *outU, float *outV, float *outD) const;
/*
    bool Intersect(const Plane &plane) const;
    bool Intersect(const Plane &plane, float &outDistance) const;

    bool Intersect(const Sphere &sphere) const;
    bool Intersect(const Sphere &sphere, float &outDistance) const;

    bool Intersect(const Ellipsoid &ellipsoid) const;
    bool Intersect(const Ellipsoid &ellipsoid, float &outDistance) const;

    bool Intersect(const AABB &aabb) const;
    bool Intersect(const AABB &aabb, float &outDistance) const;

    bool Intersect(const OBB &aabb) const;
    bool Intersect(const OBB &aabb, float &outDistance) const;
*/
//    bool Intersect(const Triangle &triangle) const;
//    bool Intersect(const Triangle &triangle, float &outDistance) const;

//    bool Intersect(const Cylinder &cylinder) const;
//    bool Intersect(const Cylinder &cylinder, float &outDistance) const;

//    bool Intersect(const Torus &torus) const;
//    bool Intersect(const Torus &torus, float &outDistance) const;

//    bool Intersect(const Frustum &frustum) const;
//    bool Intersect(const Frustum &frustum, float &outDistance) const;

//    bool Intersect(const Polyhedron &polyhedron) const;
//    bool Intersect(const Polyhedron &polyhedron, float &outDistance) const;

    Ray ToRay() const;
    LineSegment ToLineSegment(float d) const;

    /// Returns true if the given three points are collinear (they all lie on the same line).
    static bool AreCollinear(const float3 &p1, const float3 &p2, const float3 &p3, float epsilon = 1e-3f);

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this Line. Most useful for debugging purposes.
    std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

Line operator *(const float3x3 &transform, const Line &line);
Line operator *(const float3x4 &transform, const Line &line);
Line operator *(const float4x4 &transform, const Line &line);
Line operator *(const Quat &transform, const Line &line);

// Internal helper functions.
float Dmnop(const float3 *v, int m, int n, int o, int p);
float3 LineLine(float3 start0, float3 end0, float3 start1, float3 end1, float *d, float *d2);

MATH_END_NAMESPACE

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Line)
Q_DECLARE_METATYPE(Line*)
#endif
