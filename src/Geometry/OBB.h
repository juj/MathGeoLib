/** @file OBB.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

/// A 3D arbitrarily oriented bounding box.
/** This data structure represents a box in 3D space. The local axes of this box can be arbitrarily oriented/rotated
    with respect to the global world coordinate system. This allows OBBs to more tightly bound objects than AABBs do, 
    which always align with the world space axes. This flexibility has the drawback that the geometry tests and operations
    involving OBBs are more costly, and representing an OBB in memory takes more space (15 floats vs 6 floats). */
class OBB
{
public:
    /// The center of this OBB.
    float3 pos;

    /// Stores half-sizes to (local) x, y and z directions.
    float3 r;

    /// Specifies normalized direction vectors for the local axes (x, y and z). [noscript]
    float3 axis[3];

    /// Constructs an uninitialized OBB [opaque-qtscript].
    /** @note The default ctor does not initialize any member values. */
    OBB() {}
    OBB(const AABB &aabb);

    /// Sets this structure to a degenerate OBB that does not have any volume.
    void SetNegativeInfinity();

    /// Sets this OBB from an AABB.
    /** This conversion is exact, and does not loosen the volume.
        @param aabb The axis-aligned bounding box to convert to an OBB. */
    void SetFrom(const AABB &aabb);
    /** @param transform If a transformation matrix is supplied, this transformation is applied to the AABB before
        representing it as an oriented bounding box. The basis of this matrix is assumed to be orthogonal, which 
        means no projection or shear is allowed. */
    void SetFrom(const AABB &aabb, const float3x3 &transform);
    void SetFrom(const AABB &aabb, const float3x4 &transform);
    void SetFrom(const AABB &aabb, const float4x4 &transform);
    void SetFrom(const AABB &aabb, const Quat &transform);

    /// Sets this OBB to enclose the given sphere.
    /** This function computes the smallest possible OBB (in terms of volume) that contains the given sphere, and stores the result in this structure. */
    void SetFrom(const Sphere &sphere);

    /// Sets this OBB to enclose the given polyhedron.        
    /** This function computes the smallest OBB (in terms of volume) that contains the given polyhedron, and stores the result in this structure.
        @note An OBB cannot generally exactly represent a polyhedron. Converting a polyhedron to an OBB loses some features of the polyhedron.
        @return If the given polyhedron is closed, this function succeeds and returns true. If the polyhedron is uncapped (has infinite volume), this function
          does not modify this data structure, but returns false. */
//    bool SetFrom(const Polyhedron &polyhedron);

    /// Sets this OBB to enclose the given point cloud.
    /** This functions uses principal component analysis to generate an approximation of the smallest OBB that encloses the
        given point set. The resulting OBB will always contain all the specified points, but might not be the optimally
        smallest OBB in terms of volume. */
    void SetFromApproximate(const float3 *pointArray, int numPoints);

    /// Converts this to a polyhedron.
    /** This function returns a polyhedron representation of this OBB. This conversion is exact, meaning that the returned
        polyhedron represents the same set of points than this OBB. */
//    Polyhedron ToPolyhedron() const;

    /// Returns the smallest AABB that this OBB is inside of.
    /** This function computes the optimal minimum volume AABB that encloses this OBB. */
    AABB MinimalEnclosingAABB() const;

    /// Returns the largest AABB inside this OBB.
    /** This function computes the largest AABB that can fit inside this OBB. This AABB is unique up to the center point of the
        AABB. The returned AABB will be centered to the OBB center point. */
    AABB MaximalContainedAABB() const;

    /// Returns the side lengths of this OBB in its local x, y and z directions.
    float3 Size() const;
    /// Returns Size()/2.
    float3 HalfSize() const;

    /// Returns a diagonal vector of this OBB. This vector runs from one corner of the OBB from the opposite corner.
    float3 Diagonal() const;
    /// Returns Diagonal()/2.
    float3 HalfDiagonal() const;

    /// Returns the transformation matrix that transforms from world space to the local space of this OBB.
    float3x4 WorldToLocal() const;

    /// Returns the transformation matrix that transforms from the local space of this OBB to world space.
    float3x4 LocalToWorld() const;

    /// Returns the smallest sphere that contains this OBB.
    Sphere MinimalEnclosingSphere() const;

    /// Returns the largest sphere that can fit inside this OBB.
    /** This function computes the largest sphere that can fit inside this OBB. This sphere is unique up to the center point 
        of the sphere. The returned sphere will be positioned to the same center point as this OBB. */
    Sphere MaximalContainedSphere() const;
    
    /// Tests the members of this structure for NaNs and infs.
    /** This function returns true if the member variables of this OBB are valid floats and do not contain NaNs or infs, and false otherwise. */
    bool IsFinite() const;

    /// Tests the members of this structure for validity.
    /** This function returns true if this OBB does not span a strictly positive volume, and false if this OBB 
        represents a good non-degenerate volume.
        @note An OBB with a volume of zero is considered non-degenerate. A degenerate OBB is one with negative volume. */
    bool IsDegenerate() const;

    /// Returns the center point of this OBB.
    float3 CenterPoint() const;

    /// Generates a point inside this OBB.
    /// @param x A normalized value between [0,1]. This specifies the point position along the local x axis of the OBB.
    /// @param y A normalized value between [0,1]. This specifies the point position along the local y axis of the OBB.
    /// @param z A normalized value between [0,1]. This specifies the point position along the local z axis of the OBB.
    float3 PointInside(float x, float y, float z) const;

    /// Returns an edge of this OBB.
    /// @param edgeIndex The index of the edge line segment to get, in the range [0, 11].
    LineSegment Edge(int edgeIndex) const;

    /// Returns a corner point of this OBB.
    /** This function generates one of the eight corner points of this OBB. 
        @param cornerIndex The index of the corner point to generate, in the range [0, 7].
         The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions). */
    float3 CornerPoint(int cornerIndex) const;

    /// Returns a point on an edge of this OBB.
    /** @param edgeIndex The index of the edge to generate a point to, in the range [0, 11]. \todo Document which index generates which one.
        @param u A normalized value between [0,1]. This specifies the relative distance of the point along the edge. */
    float3 PointOnEdge(int edgeIndex, float u) const;

    /// Returns the point at the center of the given face of this OBB.
    /// @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
    float3 FaceCenterPoint(int faceIndex) const;

    /// Generates a point at the surface of the given face of this OBB.
    /// @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
    /// @param u A normalized value between [0, 1].
    /// @param v A normalized value between [0, 1].
    float3 FacePoint(int faceIndex, float u, float v) const;

    /// Computes the plane equation of the given face of this OBB.
    /// @param faceIndex The index of the OBB face. The valid range is [0, 5].
    /// @return The plane equation the specified face lies on. The normal of this plane points outwards from this OBB.
    Plane FacePlane(int faceIndex) const;

    /// Fills an array with all the eight corner points of this OBB.
    /// @param outPointArray [out] The array to write the points to. Must have space for 8 elements.
    void GetCornerPoints(float3 *outPointArray) const;

    /// Fills an array with all the six planes of this OBB.
    /// @param outPlaneArray [out] The array to write the planes to. Must have space for 6 elements.
    void GetFacePlanes(Plane *outPlaneArray) const;

    /// Finds the two extreme points along the given direction vector from the given point array.
    /// @param dir The direction vector to project the point array to. This vector does not need to be normalized.
    /// @param pointArray [in] The list of points to process.
    /// @param numPoints The number of elements in pointArray.
    /// @param idxSmallest [out] The index of the smallest point along the given direction will be received here. This may be null, if this information is of no interest.
    /// @param idxLargest [out] The index of the largest point along the given direction will be received here. This may be null, if this information is of no interest.
    static void ExtremePointsAlongDirection(const float3 &dir, const float3 *pointArray, int numPoints, int *idxSmallest, int *idxLargest);

    /// Generates an OBB that encloses the given point set.
    /** This function uses principal component analysis as the heuristics to generate the OBB. The returned OBB
        is not necessarily the optimal OBB that encloses the given point set.
        @param pointArray [in] The list of points to enclose with an OBB.
        @param numPoints The number of elements in the input array. */
    static OBB PCAEnclosingOBB(const float3 *pointArray, int numPoints);

    /// Returns the volume of this OBB.
    float Volume() const;

    /// Returns the surface area of the faces of this OBB.
    float SurfaceArea() const;

    /// Generates a random point inside this OBB.
    /** The points are distributed uniformly. */
    float3 RandomPointInside(LCG &rng) const;

    /// Generates a random point on a random face of this OBB.
    /** The points are distributed uniformly. */
    float3 RandomPointOnSurface(LCG &rng) const;

    /// Generates a random point on a random edge of this OBB.
    /** The points are distributed uniformly. */
    float3 RandomPointOnEdge(LCG &rng) const;

    /// Picks a random corner point of this OBB.
    /** The points are distributed uniformly. */
    float3 RandomCornerPoint(LCG &rng) const;

    /// Translates this OBB in the world space.
    /** @param offset The amount of displacement to apply to this OBB, in world space coordinates. */
    void Translate(const float3 &offset);

    /// Applies an uniform scale to this OBB.
    /** This function scales this OBB structure in-place, using the given center point as the origin 
        for the scaling operation.
        @param centerPoint Specifies the center of the scaling operation, in world space.
        @param scaleFactor The uniform scale factor to apply to each world space axis. */
    void Scale(const float3 &centerPoint, float scaleFactor);

    /// Applies a non-uniform scale to this OBB.
    /** This function scales this OBB structure in-place, using the given center point as the origin 
        for the scaling operation.
        @param centerPoint Specifies the center of the scaling operation, in world space.
        @param scaleFactor The non-uniform scale factors to apply to each world space axis. */
    void Scale(const float3 &centerPoint, const float3 &scaleFactor);

    /// Applies a transformation to this OBB.
    /** @param transform The transformation to apply to this OBB. This transformation must be affine, and
        must contain an orthogonal set of column vectors (may not contain shear or projection). */
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Returns the point inside this OBB that is closest to the given target point.
    float3 ClosestPoint(const float3 &targetPoint) const;

    /// Computes the distance of this OBB to the given object.
    /** The first parameter of this function specifies the object to test the distance to.
        @param outClosestPoint [out, optional] If not null, this parameter will receive the closest point on this OBB (in world space)
            to the specified object. If the actual closest point is of no importance, this parameter can be left null, which may
            speed up the query. The closest point may not be unique, in which case an arbitrary point on the surface of this OBB
            is returned.
        @return The distance between outClosestPoint and outClosestPointOther is returned. */
    float Distance(const float3 &point) const;
    /** @param outClosestDistance [out, optional] For ray, line and line segment queries, this parameter will receive the distance along
            the ray that specifies the closest point on that object to this OBB. This parameter may be left null, in which case the 
            actual distance along the ray is not computed. */
    float Distance(const Ray &ray, float3 *outClosestPoint, float *outClosestDistance) const;
    float Distance(const Line &line, float3 *outClosestPoint, float *outClosestdistance) const;
    float Distance(const LineSegment &lineSegment, float3 *outClosestPoint, float *outClosestDistance) const;
    /** @param outClosestPointOther [out, optional] If not null, this parameter will receive the closest point to this OBB on the surface
            of the other object. This parameter may be left null, if the actual point is not important. The closest point 
            may not be unique, in which case an arbitrary point on the surface of the other object is returned.*/
    float Distance(const AABB &aabb, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const OBB &obb, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Plane &plane, float3 *outClosestPoint, float3 *outClosestPointOther) const;/*
    float Distance(const Sphere &sphere, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Ellipsoid &ellipsoid, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Triangle &triangle, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Cylinder &cylinder, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Capsule &capsule, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Torus &torus, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Frustum &frustum, float3 *outClosestPoint, float3 *outClosestPointOther) const;
    float Distance(const Polygon &polygon, float3 *outClosestPoint, float3 *outClosestPointOther) const; */
//    float Distance(const Polyhedron &polyhedron, float3 *outClosestPoint, float3 *outClosestPointOther) const;

    /// Tests if this OBB fully contains the given object.
    /** This function returns true if the given object lies inside this OBB, and false otherwise.
        @note The comparison is performed using less-or-equal, so the faces of this OBB count as being inside, but
        due to float inaccuracies, this cannot generally be relied upon. */
    bool Contains(const float3 &point) const;
    bool Contains(const LineSegment &lineSegment) const;
    bool Contains(const AABB &aabb) const;
    bool Contains(const OBB &obb) const;
    bool Contains(const Triangle &triangle) const;

    /// Tests if this OBB intersects the given object.
    /** The first parameter of this function specifies the object to test against.
        @param outDistance [out] For rays, lines and line segments, this parameter receives the distance along the ray
            that specifies the hit point.        
        @return The HitInfo structure that describes the details of the intersection that occurred. */
//    HitInfo Intersect(const Ray &ray, float *outDistance) const;
    /** @param maxDistance If specified, limits the maximum distance along the ray to which the intersection
        is checked. This effectively utilizes the ray as if it was a line segment. */
//    HitInfo Intersect(const Ray &ray, float maxDistance, float *outDistance) const;
//    HitInfo Intersect(const Line &line, float *outDistance) const;
//    HitInfo Intersect(const LineSegment &lineSegment, float *outDistance) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb, float epsilon = 1e-3f) const;
    bool Intersects(const Plane &plane) const;

    bool Intersects(const Ray &ray, float *dNear, float *dFar) const;
    bool Intersects(const Line &line, float *dNear, float *dFar) const;
    bool Intersects(const LineSegment &lineSegment, float *dNear, float *dFar) const;
    bool Intersects(const Sphere &sphere, float3 *closestPointOnOBB) const;
    bool Intersects(const Triangle &triangle) const;

/*  HitInfo Intersect(const Plane &plane) const; 
    HitInfo Intersect(const Sphere &sphere) const;
    HitInfo Intersect(const Ellipsoid &ellipsoid) const;
    HitInfo Intersect(const Triangle &triangle) const;
    HitInfo Intersect(const Cylinder &cylinder) const;
    HitInfo Intersect(const Capsule &capsule) const;
    HitInfo Intersect(const Torus &torus) const;
    HitInfo Intersect(const Frustum &frustum) const;
    HitInfo Intersect(const Polygon &polygon) const; */
//    HitInfo Intersect(const Polyhedron &polyhedron) const;

    /// Expands this OBB to enclose the given object.
    /** This function computes the OBB that encloses both this OBB and the specified object, and stores the resulting
        OBB into this.
        @return For the polyhedron case, this function returns true if the polyhedron is closed and computing the enclosure
            succeeded. If the polyhedron is not closed (it has infinite volume), this function returns false, and this OBB
            is not modified. For other object types, this function always succeeds, and does not return a value. */
/*    void Enclose(const float3 &point);
    void Enclose(const LineSegment &lineSegment);
    void Enclose(const AABB &aabb);
    void Enclose(const OBB &obb);
    void Enclose(const Sphere &sphere);
    void Enclose(const Ellipsoid &ellipsoid);
    void Enclose(const Triangle &triangle);
    void Enclose(const Cylinder &cylinder);
    void Enclose(const Capsule &capsule);
    void Enclose(const Torus &torus);
    void Enclose(const Frustum &frustum);
    void Enclose(const Polygon &polygon);
    bool Enclose(const Polyhedron &polyhedron);
    void Enclose(const float3 *pointArray, int numPoints);*/

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this OBB. Most useful for debugging purposes.
    /** The returned string specifies the center point and the half-axes of this OBB. */
    std::string ToString() const;
#endif
#ifdef QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif

    /// Finds the set intersection of this and the given OBB.
    /** @return This function returns the Polyhedron that is contained in both this and the given OBB. */
//    Polyhedron Intersection(const AABB &aabb) const;

    /// Finds the set intersection of this and the given OBB.
    /** @return This function returns the Polyhedron that is contained in both this and the given OBB. */
//    Polyhedron Intersection(const OBB &obb) const;

    /// Finds the set intersection of this OBB and the given Polyhedron.
    /** @return This function returns a Polyhedron that represents the set of points that are contained in this OBB
        and the given Polyhedron. */
//    Polyhedron Intersection(const Polyhedron &polyhedron) const;
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(OBB)
Q_DECLARE_METATYPE(OBB*)
#endif
