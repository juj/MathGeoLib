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

/** @file AABB.h
    @author Jukka Jylänki
	@brief The Axis-Aligned Bounding Box (AABB) geometry object. */
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

#ifdef OGRE_INTEROP
#include <OgreAxisAlignedBox.h>
#endif

MATH_BEGIN_NAMESPACE

/// A 3D axis-aligned bounding box.
/** This data structure can be used to represent coarse bounds of objects, in situations where detailed triangle-level
    computations can be avoided. In physics systems, bounding boxes are used as an efficient early-out test for geometry
    intersection queries. 
    
    The 'axis-aligned' part in the name means that the local axes of this bounding box are restricted to align with the 
    axes of the world space coordinate system. This makes computations involving AABB's very fast, since AABB's cannot
    be arbitrarily oriented in the space with respect to each other.

    If you need to represent a box in 3D space with arbitrary orientation, see the class OBB. */
class AABB
{
public:
    /// The default ctor does not initialize the AABB to any value.
    AABB() {}

    /// Constructs this AABB by specifying the minimum and maximum extending corners of the box.
    AABB(const float3 &minPoint, const float3 &maxPoint);

    /// Constructs this AABB to enclose the given OBB.
    explicit AABB(const OBB &obb);

    /// Constructs this AABB to enclose the given Sphere.
    explicit AABB(const Sphere &s);

    /// Specifies the minimum extent of this AABB in the world space x, y and z axes.
    float3 minPoint;

    /// Specifies the maximum extent of this AABB in the world space x, y and z axes.
    float3 maxPoint;

	float MinX() const { return minPoint.x; }
	float MinY() const { return minPoint.y; }
	float MinZ() const { return minPoint.z; }
	float MaxX() const { return maxPoint.x; }
	float MaxY() const { return maxPoint.y; }
	float MaxZ() const { return maxPoint.z; }

    /// Sets this structure to a degenerate AABB that does not have any volume.
    /// This function is useful for initializing the AABB to "null" before a loop of calls to Enclose(),
    /// which incrementally expand the contents of this AABB to enclose the given objects.
    void SetNegativeInfinity();

    /// Sets this AABB by specifying the center and half-diagonal vector.
    void SetCenter(const float3 &center, const float3 &halfSize);

    /// Sets this AABB to enclose the given OBB.
    /** This function computes the minimal axis-aligned bounding box for the given oriented bounding box. If the orientation
        of the OBB is not aligned with the world axes, this conversion loosens the volume of the bounding box. */
    void SetFrom(const OBB &obb);

    /// Computes the minimal enclosing AABB of the given polyhedron.        
    /** This function computes the smallest AABB (in terms of volume) that contains the given polyhedron, and stores 
        the result in this structure.
        @note An AABB cannot generally exactly represent a polyhedron. Converting a polyhedron to an AABB loses some 
        features of the polyhedron.
        @return If the given polyhedron is closed, this function succeeds and returns true. If the polyhedron is uncapped 
            (has infinite volume), this function does not modify this data structure, but returns false. */
//    bool SetFrom(const Polyhedron &polyhedron);

    /// Sets this AABB to enclose the given sphere.
    /** This function computes the smallest possible AABB (in terms of volume) that contains the given sphere, and stores the result in this structure. */
    void SetFrom(const Sphere &s);

    /// Sets this AABB to enclose the given set of points. [noscript]
    void SetFrom(const float3 *pointArray, int numPoints);

    /// Converts this to a polyhedron.
    /** This function returns a polyhedron representation of this AABB. This conversion is exact, meaning that the returned
        polyhedron represents the same set of points than this AABB. */
    Polyhedron ToPolyhedron() const;

    /// Converts this to an oriented bounding box.
    /** This function returns an OBB representation of this AABB. This conversion is exact, meaning that the returned
        OBB represents the same set of points than this AABB. */
    OBB ToOBB() const;

    /// Returns the smallest sphere that contains this AABB.
    Sphere MinimalEnclosingSphere() const;

    /// Returns the largest sphere that can fit inside this AABB.
    /** This function computes the largest sphere that can fit inside this AABB. This sphere is unique up to the center point 
        of the sphere. The returned sphere will be positioned to the same center point as this AABB. */
    Sphere MaximalContainedSphere() const;

    /// Tests the members of this structure for NaNs and infs.
    /** This function returns true if the member variables of this AABB are valid floats and do not contain NaNs or infs, and false otherwise. */
    bool IsFinite() const;

    /// Tests the members of this structure for validity.
    /** This function returns true if this AABB does not span a positive volume, and false if this AABB represents a good non-degenerate volume. */
    bool IsDegenerate() const;

    /// Returns the center point of this AABB.
    float3 CenterPoint() const;
    
    /// Returns the center of mass of this AABB.
    float3 Centroid() const { return CenterPoint(); }

    /// Generates a point inside this AABB.
    /// @param x A normalized value between [0,1]. This specifies the point position along the world x axis.
    /// @param y A normalized value between [0,1]. This specifies the point position along the world y axis.
    /// @param z A normalized value between [0,1]. This specifies the point position along the world z axis.
    float3 PointInside(float x, float y, float z) const;

    /// Returns an edge of this AABB.
    /// @param edgeIndex The index of the edge line segment to get, in the range [0, 11].
    LineSegment Edge(int edgeIndex) const;

    /// Returns a corner point of this AABB.
    /** This function generates one of the eight corner points of this AABB. 
        @param cornerIndex The index of the corner point to generate, in the range [0, 7].
         The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions). */
    float3 CornerPoint(int cornerIndex) const;

    /// Returns a point on an edge of this AABB.
    /** @param edgeIndex The index of the edge to generate a point to, in the range [0, 11]. \todo Document which index generates which one.
        @param u A normalized value between [0,1]. This specifies the relative distance of the point along the edge. */
    float3 PointOnEdge(int edgeIndex, float u) const;

    /// Returns the point at the center of the given face of this AABB.
    /// @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
    ///                  This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
    float3 FaceCenterPoint(int faceIndex) const;

    /// Generates a point at the surface of the given face of this AABB.
    /// @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
    ///                  This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
    /// @param u A normalized value between [0, 1].
    /// @param v A normalized value between [0, 1].
    float3 FacePoint(int faceIndex, float u, float v) const;

    /// Returns the surface normal direction vector the given face points towards.
    /// @param faceIndex The index of the AABB face to generate the point at. The valid range is [0, 5].
    ///                  This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
    float3 FaceNormal(int faceIndex) const;

    /// Computes the plane equation of the given face of this AABB.
    /// @param faceIndex The index of the AABB face. The valid range is [0, 5].
    ///                  This index corresponds to the planes in the order (-X, +X, -Y, +Y, -Z, +Z).
    /// @return The plane equation the specified face lies on. The normal of this plane points outwards from this AABB.
    Plane FacePlane(int faceIndex) const;

    /// Fills an array with all the eight corner points of this AABB.
    /// @param outPointArray [out] The array to write the points to. Must have space for 8 elements.
    void GetCornerPoints(float3 *outPointArray) const;

    /// Fills an array with all the six planes of this AABB.
    /// @param outPlaneArray [out] The array to write the planes to. Must have space for 6 elements.
    void GetFacePlanes(Plane *outPlaneArray) const;

    /// Generates an AABB that encloses the given point set.
    /** This function finds the smallest AABB that contains the given set of points. [noscript] */
    static AABB MinimalEnclosingAABB(const float3 *pointArray, int numPoints);

    /// Finds the most extremal points along the three world axes simultaneously.
    /// @param minx [out] Receives the point that has the smallest x coordinate.
    /// @param maxx [out] Receives the point that has the largest x coordinate.
    static void ExtremePointsAlongAABB(const float3 *pointArray, int numPoints, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz);

    /// Creates a new AABB given is center position and size along the X, Y and Z axes.
    static AABB FromCenterAndSize(const float3 &aabbCenterPos, const float3 &aabbSize);

    /// Returns the side lengths of this AABB in x, y and z directions.
    /// The returned vector is equal to the diagonal vector of this AABB, i.e. it spans from the
    /// minimum corner of the AABB to the maximum corner of the AABB.
    float3 Size() const;
    /// Returns Size()/2.
    float3 HalfSize() const;

    /// Returns the diameter vector of this AABB. 
    /// @note For AABB, Diagonal() and Size() are the same concept. These functions are provided for symmetry
    /// with the OBB class.
    float3 Diagonal() const { return Size(); }
    /// Returns Diagonal()/2.
    float3 HalfDiagonal() const { return HalfSize(); }

    /// Returns the volume of this AABB.
    float Volume() const;

    /// Returns the surface area of the faces of this AABB.
    float SurfaceArea() const;

    /// Generates a random point inside this AABB.
    /** The points are distributed uniformly. */
    float3 RandomPointInside(LCG &rng) const;

    /// Generates a random point on a random face of this AABB.
    /** The points are distributed uniformly. */
    float3 RandomPointOnSurface(LCG &rng) const;

    /// Generates a random point on a random edge of this AABB.
    /** The points are distributed uniformly. */
    float3 RandomPointOnEdge(LCG &rng) const;

    /// Picks a random corner point of this AABB.
    /** The points are distributed uniformly. */
    float3 RandomCornerPoint(LCG &rng) const;

    /// Translates this AABB in the world space.
    /** @param offset The amount of displacement to apply to this AABB, in world space coordinates. */
    void Translate(const float3 &offset);

    /// Applies an uniform scale to this AABB.
    /** This function scales this AABB structure in-place, using the given center point as the origin 
        for the scaling operation.
        @param centerPoint Specifies the center of the scaling operation, in world space.
        @param scaleFactor The uniform scale factor to apply to each world space axis. */
    void Scale(const float3 &centerPoint, float scaleFactor);

    /// Applies a non-uniform scale to this AABB.
    /** This function scales this AABB structure in-place, using the given center point as the origin 
        for the scaling operation.
        @param centerPoint Specifies the center of the scaling operation, in world space.
        @param scaleFactor The non-uniform scale factors to apply to each world space axis. */
    void Scale(const float3 &centerPoint, const float3 &scaleFactor);

    /// Applies a transformation to this AABB.
    /** This function transforms this AABB with the given transformation, and then recomputes this AABB
        to enclose the resulting oriented bounding box. This transformation is not exact and in general, calling 
        this function results in the loosening of the AABB bounds. 
        @param transform The transformation to apply to this AABB. This function assumes that this
        transformation does not contain shear, nonuniform scaling or perspective properties, i.e. that the fourth 
        row of the float4x4 is [0 0 0 1]. */
    void TransformAsAABB(const float3x3 &transform);
    void TransformAsAABB(const float3x4 &transform);
    void TransformAsAABB(const float4x4 &transform);
    void TransformAsAABB(const Quat &transform);

    /// Applies a transformation to this AABB and returns the resulting OBB.
    /** Transforming an AABB produces an oriented bounding box. This set of functions does not apply the transformation
        to this object itself, but instead returns the OBB that results in the transformation.
        @param transform The transformation to apply to this AABB. This function assumes that this
        transformation does not contain shear, nonuniform scaling or perspective properties, i.e. that the fourth 
        row of the float4x4 is [0 0 0 1]. */
    OBB Transform(const float3x3 &transform) const;
    OBB Transform(const float3x4 &transform) const;
    OBB Transform(const float4x4 &transform) const;
    OBB Transform(const Quat &transform) const;

    /// Returns the closest point inside this AABB to the given target point.
    /// If the target point is inside the AABB, then it is the closest point, otherwise
    /// a point on a face of this AABB is returned.
    float3 ClosestPoint(const float3 &targetPoint) const;

    /// Computes the distance of this AABB to the given object.
    /** The first parameter of this function specifies the object to test the distance to.
        @param outClosestPoint [out, optional] If not null, this parameter will receive the closest point on this AABB (in world space)
            to the specified object. If the actual closest point is of no importance, this parameter can be left null, which may
            speed up the query. The closest point may not be unique, in which case an arbitrary point on the surface of this AABB
            is returned.
        @return The distance between outClosestPoint and outClosestPointOther is returned. */
    float Distance(const float3 &point) const;
    float Distance(const Sphere &sphere) const;
    /** @param outClosestDistance [out, optional] For ray, line and line segment queries, this parameter will receive the distance along
            the ray that specifies the closest point on that object to this AABB. This parameter may be left null, in which case the 
            actual distance along the ray is not computed. */
  //  float Distance(const Ray &ray, float3 *outClosestPoint, float *outClosestDistance) const;
  //  float Distance(const Line &line, float3 *outClosestPoint, float *outClosestdistance) const;
  //  float Distance(const LineSegment &lineSegment, float3 *outClosestPoint, float *outClosestDistance) const;
    /** @param outClosestPointOther [out, optional] If not null, this parameter will receive the closest point to this AABB on the surface
            of the other object. This parameter may be left null, if the actual point is not important. The closest point 
            may not be unique, in which case an arbitrary point on the surface of the other object is returned.*/
 /*   float Distance(const AABB &aabb, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const OBB &obb, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Plane &plane, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Sphere &sphere, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Ellipsoid &ellipsoid, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Triangle &triangle, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Cylinder &cylinder, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Capsule &capsule, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Torus &torus, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
    float Distance(const Frustum &frustum, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Polygon &polygon, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Polyhedron &polyhedron, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
*/
    /// Tests if this AABB fully contains the given object.
    /** This function returns true if the given object lies inside this AABB, and false otherwise.
        @note The comparison is performed using less-or-equal, so the faces of this AABB count as being inside, but
        due to float inaccuracies, this cannot generally be relied upon. */
    bool Contains(const float3 &point) const;
    bool Contains(const LineSegment &lineSegment) const;
    bool Contains(const AABB &aabb) const;
    bool Contains(const OBB &obb) const;
    bool Contains(const Sphere &sphere) const;
    bool Contains(const Triangle &triangle) const;
//    bool Contains(const Polygon &polygon) const;
    bool Contains(const Frustum &frustum) const;
    bool Contains(const Polyhedron &polyhedron) const;

    /// Tests if this AABB intersects the given object.
    bool Intersects(const Ray &ray, float *dNear, float *dFar) const;
    bool Intersects(const Line &line, float *dNear, float *dFar) const;
    bool Intersects(const LineSegment &lineSegment, float *dNear, float *dFar) const;
    bool Intersects(const Plane &plane) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb) const;
    /// @param closestPointOnAABB [out] Returns the closest point on this AABB to the given sphere.
    bool Intersects(const Sphere &sphere, float3 *closestPointOnAABB) const;
    bool Intersects(const Capsule &capsule) const;
    bool Intersects(const Triangle &triangle) const;
    bool Intersects(const Polygon &polygon) const;
    bool Intersects(const Frustum &frustum) const;
    bool Intersects(const Polyhedron &polyhedron) const;

    /// Projects this AABB onto the given axis.
    /// @param axis The axis to project onto. This vector can be unnormalized.
    /// @param dMin [out] Returns the minimum extent of this AABB on the given axis.
    /// @param dMax [out] Returns the maximum extent of this AABB on the given axis.
    void ProjectToAxis(const float3 &axis, float &dMin, float &dMax) const;

    /** The first parameter of this function specifies the object to test against.
        @param outDistance [out] For rays, lines and line segments, this parameter receives the distance along the ray
            that specifies the hit point.        
        @return The HitInfo structure that describes the details of the intersection that occurred. */
//    HitInfo Intersect(const Ray &ray, float *outDistance) const; ///< [noscript]
    /** @param maxDistance If specified, limits the maximum distance along the ray to which the intersection
        is checked. This effectively utilizes the ray as if it was a line segment. */
/*    HitInfo Intersect(const Ray &ray, float maxDistance, float *outDistance) const; ///< [noscript]
    HitInfo Intersect(const Line &line, float *outDistance) const; ///< [noscript]
    HitInfo Intersect(const LineSegment &lineSegment, float *outDistance) const; ///< [noscript]
    HitInfo Intersect(const AABB &aabb) const; ///< [noscript]
    HitInfo Intersect(const OBB &obb) const; ///< [noscript]
    HitInfo Intersect(const Plane &plane) const; ///< [noscript]
    HitInfo Intersect(const Sphere &sphere) const; ///< [noscript]
    HitInfo Intersect(const Ellipsoid &ellipsoid) const; ///< [noscript]
    HitInfo Intersect(const Triangle &triangle) const; ///< [noscript]
    HitInfo Intersect(const Cylinder &cylinder) const; ///< [noscript]
//    HitInfo Intersect(const Capsule &capsule) const; ///< [noscript]
//    HitInfo Intersect(const Torus &torus) const; ///< [noscript]
    HitInfo Intersect(const Frustum &frustum) const; ///< [noscript]
//    HitInfo Intersect(const Polygon &polygon) const; ///< [noscript]
//    HitInfo Intersect(const Polyhedron &polyhedron) const; ///< [noscript]
*/
    /// Expands this AABB to enclose the given object.
    /** This function computes the AABB that encloses both this AOBB and the specified object, and stores the resulting
        AABB into this. */
    void Enclose(const float3 &point);
    void Enclose(const LineSegment &lineSegment);
    void Enclose(const AABB &aabb);
    void Enclose(const OBB &obb);
    void Enclose(const Sphere &sphere);
    void Enclose(const Triangle &triangle);
    void Enclose(const Capsule &capsule);
    void Enclose(const Frustum &frustum);
    void Enclose(const Polygon &polygon);
    void Enclose(const Polyhedron &polyhedron);
    /// [noscript]
    void Enclose(const float3 *pointArray, int numPoints);

	/// Generates an unindexed triangle mesh representation of this AABB.
    /// @param x The number of faces to generate along the X axis. This value must be >= 1.
    /// @param y The number of faces to generate along the Y axis. This value must be >= 1.
    /// @param z The number of faces to generate along the Z axis. This value must be >= 1.
	/// @param outPos [out] An array of size numVertices which will receive a triangle list 
    ///                     of vertex positions. Cannot be null.
	/// @param outNormal [out] An array of size numVertices which will receive vertex normals. 
    ///                        If this parameter is null, vertex normals are not returned.
	/// @param outUV [out] An array of size numVertices which will receive vertex UV coordinates. 
    ///                        If this parameter is null, a UV mapping is not generated.
    /// The number of vertices that outPos, outNormal and outUV must be able to contain is
    /// (x*y + x*z + y*z)*2*6. If x==y==z==1, then a total of 36 vertices are required. Call
    /// NumVerticesInTriangulation to obtain this value.
    void Triangulate(int x, int y, int z, float3 *outPos, float3 *outNormal, float2 *outUV) const;

    static int NumVerticesInTriangulation(int numFacesX, int numFacesY, int numFacesZ)
    {
        return (numFacesX*numFacesY + numFacesX*numFacesZ + numFacesY*numFacesZ)*2*6;
    }
    
    /// Generates an edge list representation of the edges of this AABB.
    /// @param outPos [out] An array that contains space for at least 24 vertices (NumVerticesInEdgeList()).
    void ToEdgeList(float3 *outPos) const;

    static int NumVerticesInEdgeList()
    {
        return 4*3*2;
    }

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this AABB. Most useful for debugging purposes.
    /** The returned string specifies the center point and the half-axes of this AABB. */
    std::string ToString() const;
#endif
#ifdef QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif

    /// Finds the set intersection of this and the given AABB.
    /** @return This function returns the AABB that is contained in both this and the given AABB. */
    AABB Intersection(const AABB &aabb) const;

    /// Finds the set intersection of this AABB and the given OBB.
    /** @return This function returns a Polyhedron that represents the set of points that are contained in this AABB
        and the given OBB. */
//    Polyhedron Intersection(const OBB &obb) const;

    /// Finds the set intersection of this AABB and the given Polyhedron.
    /** @return This function returns a Polyhedron that represents the set of points that are contained in this AABB
        and the given Polyhedron. */
//    Polyhedron Intersection(const Polyhedron &polyhedron) const;

#ifdef OGRE_INTEROP
    AABB(const Ogre::AxisAlignedBox &other) { minPoint = other.getMinimum(); maxPoint = other.getMaximum(); }
    operator Ogre::AxisAlignedBox() const { return Ogre::AxisAlignedBox(minPoint, maxPoint); }
#endif
};

MATH_END_NAMESPACE

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(AABB)
Q_DECLARE_METATYPE(AABB*)
#endif

