/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

#include "Math/MathFwd.h"
#include "Geometry/LineSegment.h"

MATH_BEGIN_NAMESPACE

class Capsule
{
public:
    /// Specifies the two inner points of this capsule.
    LineSegment l;

    /// Specifies the radius of this capsule.
    float r;

    /// The default ctor does not initialize the Capsule to any value.
    Capsule() {}

    /// Constructs a new Capsule given the endpoints and radius parameters.
    Capsule(const LineSegment &endPoints, float radius);
    Capsule(const float3 &bottomPoint, const float3 &topPoint, float radius);

    /// Converts the given Sphere to a Capsule.
    /** This conversion results in a Capsule which has its both endpoints at the exact same points (line length == 0). */
    void SetFrom(const Sphere &s);

    /// Returns the distance of the two inner points of this capsule.
    float LineLength() const;

    /// Returns the diameter of this capsule (2*radius).
    float Diameter() const;

    /// Returns the bottom-most point of this Capsule.
    /// @note The bottom-most point is only a naming convention, and does not correspond to the bottom-most point along any world axis. The returned
    ///  point is simply the point at the far end of this Capsule where the point l.a resides.
    float3 Bottom() const;

    /// Returns the center point of this Capsule.
    float3 Center() const;

    /// Returns the topmost point of this Capsule.
    /// @note The topmost point is only a naming convention, and does not necessarily correspond to the topmost point along any world axis. The returned
    ///  point is simply the point at the far end of this Capsule where the point l.b resides.
    float3 Top() const;

    /// Returns the normalized direction which points from the bottommost point towards the topmost point of this Capsule.
    float3 UpDirection() const;

    /// Returns the height of this capsule, i.e. LineLength() + Diameter().
    float Height() const;

    /// Returns the volume of this Capsule.
    float Volume() const;

    /// Returns the surface area of this Capsule.
    float SurfaceArea() const;

    /// Returns the cross-section circle at the given height of this Capsule.
    /// @param l A normalized parameter between [0,1]. l == 0 returns a degenerate circle of radius 0 at the bottom of this Capsule, and l == 1
    ///         will return a degenerate circle of radius 0 at the top of this Capsule.
    Circle CrossSection(float l) const;

    /// Returns a line segment that spans the far axis of this capsule from bottom to tip.
    LineSegment HeightLineSegment() const;

    /// Tests the members of this structure for NaNs and infs.
    /** This function returns true if the member variables of this Capsule are valid floats and do not contain NaNs or infs, and false otherwise. */
    bool IsFinite() const;

    /// Generates a point inside this Capsule.
    /// @param l A normalized value between [0,1]. This specifies the point position along the height line of this Capsule.
    /// @param a A normalized value between [0,1]. This specifies the normalized directed angle of the point position around the Capsule line segment.
    /// @param d A normalized value between [0,1]. This specifies the normalized distance of the point position from the Capsule line segment.
    /// @note This function does not generate points inside this Capsule uniformly, as (l,a,d) ranges uniformly over [0,1]^3.
    float3 PointInside(float l, float a, float d) const;

    /// Generates a point that perhaps lies inside this Capsule.
    /// @param l A normalized value between [0,1]. This specifies the point position along the height line of this Capsule.
    /// @param x A normalized value between [0,1]. This specifies the x coordinate on the plane of the circle cross-section specified by l.
    /// @param y A normalized value between [0,1]. This specifies the y coordinate on the plane of the circle cross-section specified by l.
    /// @note This function will generate points uniformly, but they do not necessarily lie inside the Capsule.
    float3 UniformPointPerhapsInside(float l, float x, float y) const;

    /// Returns the minimal AABB that encloses this Capsule.
    AABB EnclosingAABB() const;

    /// Returns the minimal OBB that encloses this Capsule.
    OBB EnclosingOBB() const;

    /// Generates a random point inside this Capsule.
    /** The points are distributed uniformly. */
    float3 RandomPointInside(LCG &rng) const;

    /// Generates a random point on the surface of this Capsule.
    /** \note The points are NOT distributed uniformly. */
    float3 RandomPointOnSurface(LCG &rng) const;

    /// Translates this Capsule.
    /** @param offset The amount of displacement to apply to this Capsule, in world space coordinates. */
    void Translate(const float3 &offset);

    /// Applies an uniform scale to this Capsule.
    /** This function scales this Capsule structure in-place, using the given center point as the origin 
        for the scaling operation.
        @param centerPoint Specifies the center of the scaling operation, in world space.
        @param scaleFactor The uniform scale factor to apply to each world space axis. */
    void Scale(const float3 &centerPoint, float scaleFactor);

    /// Applies a transformation to this Capsule.
    /// The passed transformation cannot contain nonuniform scale, since the Capsule cannot represent it.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Returns the closest point inside this Capsule to the given target point.
    float3 ClosestPoint(const float3 &targetPoint) const;

    /// Computes the distance of this Capsule to the given object.
    float Distance(const float3 &point) const;
    float Distance(const Plane &plane) const;
    float Distance(const Sphere &sphere) const;
    float Distance(const Ray &ray) const;
    float Distance(const Line &line) const;
    float Distance(const LineSegment &lineSegment) const;
    float Distance(const Capsule &capsule) const;

    bool Contains(const float3 &point) const;
    bool Contains(const LineSegment &lineSegment) const;
    bool Contains(const Triangle &triangle) const;
    bool Contains(const Polygon &polygon) const;
    bool Contains(const AABB &aabb) const;
    bool Contains(const OBB &obb) const;
    bool Contains(const Frustum &frustum) const;
    bool Contains(const Polyhedron &polyhedron) const;

    /// Tests if this Capsule intersects the given object.
    bool Intersects(const Ray &ray) const;
    bool Intersects(const Line &line) const;
    bool Intersects(const LineSegment &lineSegment) const;
    bool Intersects(const Plane &plane) const;
    bool Intersects(const Sphere &sphere) const;
    bool Intersects(const Capsule &capsule) const;
//    bool Intersects(const AABB &aabb) const;
//    bool Intersects(const OBB &obb) const;
//    bool Intersects(const Triangle &triangle) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this Capsule. Most useful for debugging purposes.
    /** The returned string specifies the line segment and the radius of this Capsule. */
    std::string ToString() const;
#endif
#ifdef QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

MATH_END_NAMESPACE

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Capsule)
Q_DECLARE_METATYPE(Capsule*)
#endif
