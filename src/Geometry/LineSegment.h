/** @file LineSegment.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

/// A line segment in 3D space is a finite line with a start and end point.
class LineSegment
{
public:
    /// @note The default ctor does not initialize any member values.
    LineSegment() {}
    LineSegment(const float3 &a, const float3 &b);
    explicit LineSegment(const Ray &ray, float d);
    explicit LineSegment(const Line &line, float d);

    /// The start point of this line segment.
    float3 a;
    /// The end point of this line segment.
    float3 b;

    /// Returns a point on the line.
    /// @param d The normalized distance along the line segment to compute. This is in the range [0, 1].
    /// @important The meaning of d here differs from Line::GetPoint and Ray::GetPoint. GetPoint(0) returns a, GetPoint(1) returns b.
    float3 GetPoint(float d) const;

    /// Returns the center point of this line segment.
    /// This function is the same as calling GetPoint(0.5f);
    float3 CenterPoint() const;

    /// Reverses this line segment in place. That is, swaps the start and end points of
    /// this line segment so that it goes from end->start now.
    void Reverse();

    /// Returns the normalized direction vector that points in the direction a->b.
    float3 Dir() const;

    /// Applies a transformation to this line.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    float Length() const;
    float LengthSq() const;

    /// Returns true if this line segment contains the given point, i.e. if the squared
    /// distance to this point is smaller than the given threshold epsilon.
    bool Contains(const float3 &point, float distanceThreshold = 1e-3f) const;

    /// Returns the closest point on this line segment to the given object.
    /// @param d [out] If specified, this parameter receives the normalized position along
    ///          this line segment which specifies the closest point on this line segment to
    ///          the specified point.
    /// @param d2 [out] If specified, this parameter receives the normalized position along
    ///          the other line object which specifies the closest point on that line to
    ///          this line segment.
    float3 ClosestPoint(const float3 &point, float *d = 0) const;
    float3 ClosestPoint(const Ray &other, float *d = 0, float *d2 = 0) const;
    float3 ClosestPoint(const Line &other, float *d = 0, float *d2 = 0) const;
    float3 ClosestPoint(const LineSegment &other, float *d = 0, float *d2 = 0) const;

    /// Computes the distance of this line segment to the given point.
    /// @param d [out] If specified, this parameter receives the normalized position along
    ///          this line segment which specifies the closes point on this line segment to
    ///          the specified point.
    float Distance(const float3 &point, float *d = 0) const;
    float Distance(const Ray &other, float *d = 0, float *d2 = 0) const;
    float Distance(const Line &other, float *d = 0, float *d2 = 0) const;
    float Distance(const LineSegment &other, float *d = 0, float *d2 = 0) const;

    bool Intersects(const Plane &plane) const;
    bool Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const;
    bool Intersects(const Sphere &s, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
    bool Intersects(const AABB &aabb, float *dNear = 0, float *dFar = 0) const;
    bool Intersects(const OBB &obb, float *dNear, float *dFar) const;

/*
    bool Intersect(const Plane &plane, float &outDistance) const;

    bool Intersect(const Sphere &sphere) const;
    bool Intersect(const Sphere &sphere, float &outDistance) const;

    bool Intersect(const AABB &aabb) const;
    bool Intersect(const AABB &aabb, float &outDistance) const;

    bool Intersect(const OBB &aabb) const;
    bool Intersect(const OBB &aabb, float &outDistance) const;

    bool Intersect(const Triangle &triangle) const;
    bool Intersect(const Triangle &triangle, float &outDistance) const;

    bool Intersect(const Frustum &frustum) const;
    bool Intersect(const Frustum &frustum, float &outDistance) const;
*/
//    bool Intersect(const Polyhedron &polyhedron) const;
//    bool Intersect(const Polyhedron &polyhedron, float &outDistance) const;

    Ray ToRay() const;
    Line ToLine() const;
};

LineSegment operator *(const float3x3 &transform, const LineSegment &line);
LineSegment operator *(const float3x4 &transform, const LineSegment &line);
LineSegment operator *(const float4x4 &transform, const LineSegment &line);
LineSegment operator *(const Quat &transform, const LineSegment &line);

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(LineSegment)
Q_DECLARE_METATYPE(LineSegment*)
#endif
