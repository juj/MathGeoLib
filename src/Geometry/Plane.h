/** @file Plane.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

#ifdef OGRE_INTEROP
#include <OgrePlane.h>
#endif

/// Specifies an plane in 3D space. This plane is an affine 2D subspace of the 3D space, meaning
/// that its sides extend to infinity, and it does not necessarily pass through the origin.
class Plane
{
public:
    /// @note The default ctor does not initialize any member values.
    Plane() {}
    /// Constructs a plane by directly specifying the normal and distance parameters.
    /// @param normal The direction the plane is facing. This vector must have been normalized in advance.
    /// @param d The signed distance of this plane from the origin.
    Plane(const float3 &normal, float d);
    /// Constructs a plane by specifying three points on the plane. The normal of the plane will point to 
    /// the halfspace from which the points are observed to be oriented in counter-clockwise order.
    /// @note The points v1, v2 and v3 must not all lie on the same line.
    Plane(const float3 &v1, const float3 &v2, const float3 &v3);
    /// Constructs a plane by specifying a single point on the plane, and the surface normal.
    /// @param normal The direction the plane is facing. This vector must have been normalized in advance.
    Plane(const float3 &point, const float3 &normal);
	/// Constructs a plane by specifying a line that lies on the plane, and the plane normal.
	Plane(const Ray &ray, const float3 &normal);
	Plane(const Line &line, const float3 &normal);
	Plane(const LineSegment &lineSegment, const float3 &normal);

    /// The direction this plane is facing at.
    float3 normal;
    /// The offset of this plane from the origin.
    /// -d gives the signed distance of this plane from origin.
    /// This class uses the convention ax+by+cz = d, which means that:
    ///  - If this variable is positive, the origin is on the negative side of this plane.
    ///  - If this variable is negative, the origin is on the on the positive side of this plane.
    /// (some sources use the convention ax+by+cz+d = 0 for the variable d)
    float d;

    /// Sets this plane by specifying three points on the plane. The normal of the plane will point to 
    /// the halfspace from which the points are observed to be oriented in counter-clockwise order.
    /// @note The points v1, v2 and v3 must not all lie on the same line.
    void Set(const float3 &v1, const float3 &v2, const float3 &v3);
    /// Sets this plane by specifying a single point on the plane, and the surface normal.
    /// @param normal The direction the plane is facing. This vector must have been normalized in advance.
    void Set(const float3 &point, const float3 &normal);

    /// Returns a point on this plane.
    /// @note This point has the property that the line passing through the origin and the returned point is
    ///       perpendicular to this plane (directed towards the normal vector of this plane).
    float3 PointOnPlane() const;

    /// Applies a transformation to this plane.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Tests if given direction vector points towards the positive side of this plane.
    bool IsInPositiveDirection(const float3 &directionVector) const;

    /// Tests if given point is on the positive side of this plane.
    bool IsOnPositiveSide(const float3 &point) const;

    /// Tests if two points are on the same side of the plane.
    bool AreOnSameSide(const float3 &p1, const float3 &p2) const;

    /// Returns the distance of this plane to the given point.
    float Distance(const float3 &point) const;

    /// Returns the signed distance of this plane to the given point.
    /// If this function returns a negative value, the given point lies in the negative halfspace of this plane.
    /// Conversely, if a positive value is returned, then the given point lies in the positive halfspace of this plane.
    float SignedDistance(const float3 &point) const;

    /// Returns the affine transformation that projects orthographically onto this plane.
    float3x4 OrthoProjection() const;

    /// Returns the affine transformation that projects onto this plane in an oblique angle.
    float3x4 ObliqueProjection(const float3 &obliqueProjectionDir) const;

    /// Returns the transformation matrix that mirrors objects along this plane.
    float3x4 ReflectionMatrix() const;

    /// Mirrors the given point along this plane.
    float3 Reflect(const float3 &point) const;

    /// Refracts the given normal vector along this plane.
    float3 Refract(const float3 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

    /// Projects the given point onto this plane orthographically (finds the closest point on this plane).
    float3 Project(const float3 &point) const;

    /// Returns the closest point on this plane to the given point. This is an alias to Plane::Project(const float3 &point).
    float3 ClosestPoint(const float3 &point) const { return Project(point); }

    /// Projects the given point onto this plane in the given oblique projection direction.
    float3 ObliqueProject(const float3 &point, const float3 &obliqueProjectionDir) const;

    /// Returns true if this plane contains the given point.
    /// @param distanceThreshold The epsilon value to use for the comparison.
    bool Contains(const float3 &point, float distanceThreshold = 1e-3f) const;

    /// Computes the intersection of two planes.
    bool Intersects(const Plane &plane, Line *outLine = 0) const;

    /// Computes the intersection of three planes.
    /// @note If two of the planes are identical, the intersection will be a line.
    ///       does not detect this case, and only returns a single point on the line.
    bool Intersects(const Plane &plane, const Plane &plane2, Line *outLine = 0, float3 *outPoint = 0) const;
    
    bool Intersects(const Ray &ray, float *d) const;
    bool Intersects(const Line &line, float *d) const;
    bool Intersects(const LineSegment &lineSegment, float *d) const;
    bool Intersects(const Sphere &sphere) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Triangle &triangle) const;
    bool Intersects(const Frustum &frustum) const;
//    bool Intersect(const Polyhedron &polyhedron) const;

    /// Clips a line segment against this plane.
    /// This function removes the part of the line segment which lies in the negative halfspace of this plane.
    /// The clipping operation is performed in-place. If the whole line segment is clipped, the input variables
    /// are not modified.
    /// @return If this function returns true, the line segment after clipping did not become degenerate.
    ///         If false is returned, the whole line segment lies in the negative halfspace, and no output line segment
    ///         was generated.
    bool Clip(LineSegment &line) const;
    bool Clip(float3 &a, float3 &b) const;

    /// Clips a line against this plane.
    /// This function removes the part of the line which lies in the negative halfspace of this plane.
    /// @return If the clipping removed the whole line, the value 0 is returned.
    ///         If the clipping resulted in a ray, the value 1 is returned.
    ///         If the clipping resulted in a line, the value 2 is returned.
    int Clip(const Line &line, Ray &outRay) const;

    /// Clips a triangle against this plane.
    /// This function removes the part of the triangle which lies in the negative halfspace of this plane.
    /// @return This function reports how many output triangles were generated.
    ///         If the whole input triangle was clipped, the value 0 is returned.
    ///         If this function returns 1, the value t1 will receive the generated output triangle.
    ///         If this function returns 2, t1 and t2 will receive the generated output triangles.
    int Clip(const Triangle &triangle, Triangle &t1, Triangle &t2) const;

    /// Tests if two planes are parallel.
    bool IsParallel(const Plane &plane, float epsilon = 1e-3f) const;

    /// Returns true if this plane contains the origin.
    /// The test is performed up to the given epsilon.
    /// @note A plane passes through the origin iff d == 0 for the plane.
    bool PassesThroughOrigin(float epsilon = 1e-3f) const;

    /// Returns the angle of intersection between two planes, in radians.
    float DihedralAngle(const Plane &plane) const;

    /// Tests if two planes are the same, up to the given epsilon.
    bool Equals(const Plane &other, float epsilon = 1e-3f) const;

    /// Returns a circle that lies on this plane, with its center as close as possible to the specified center point,
    /// and the radius as specified.
    Circle GenerateCircle(const float3 &circleCenter, float radius) const;

//    float3 RandomPointInsideCircle(const float3 &circleCenter, float radius) const;
//    float3 RandomPointOnCircleEdge(const float3 &circleCenter, float radius) const;

#ifdef OGRE_INTEROP
    Plane(const Ogre::Plane &other) { normal = other.normal; d = other.d; }
    operator Ogre::Plane() const { return Ogre::Plane(normal, d); }
#endif
};

Plane operator *(const float3x3 &transform, const Plane &plane);
Plane operator *(const float3x4 &transform, const Plane &plane);
Plane operator *(const float4x4 &transform, const Plane &plane);
Plane operator *(const Quat &transform, const Plane &plane);

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Plane)
Q_DECLARE_METATYPE(Plane*)
#endif
