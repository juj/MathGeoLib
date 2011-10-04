/** @file Sphere.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A 3D sphere.
class Sphere
{
public:
    /// The center point of this sphere.
    float3 pos;
    /// The radius of this sphere.
    float r;

    /// @note The default ctor does not initialize any member values.
    Sphere() {}

    Sphere(const float3 &center, float radius);

    /// Constructs the minimal sphere enclosing the given two points.
    Sphere(const float3 &pointA, const float3 &pointB);

    /// Constructs the sphere to pass through the given three points.
    /// @note The resulting sphere may not be the minimal enclosing sphere for the three points!
    Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC);

    /// Constructs the sphere to pass through the given four points.
    /// @note The resulting sphere may not be the minimal enclosing sphere for the three points!
    Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD);

    AABB MinimalEnclosingAABB() const;
    AABB MaximalContainedAABB() const;

    /// Sets pos = (0,0,0) and r = -inf.
    void SetNegativeInfinity();

    float Volume() const;

    float SurfaceArea() const;

    /// Returns the center of mass of this Sphere.
    float3 Centroid() const { return pos; }

    bool IsFinite() const;

    bool IsDegenerate() const;

    bool Contains(const float3 &point) const;
    bool Contains(const LineSegment &lineSegment) const;
    bool Contains(const Triangle &triangle) const;
    bool Contains(const Polygon &polygon) const;
    bool Contains(const AABB &aabb) const;
    bool Contains(const OBB &obb) const;
    bool Contains(const Frustum &frustum) const;
    bool Contains(const Polyhedron &polyhedron) const;

    /// Returns a Sphere that bounds the given point array.
    /// This functions implements a fast approximate (though rather crude) algorithm of Jack Ritter.
    /// See "An Efficient Bounding Sphere", in Graphics Gems 1, pp. 301-303,
    /// or Christer Ericson's Real-time Collision Detection, pp. 89-91.
    /// This algorithm performs two linear passes over the data set, i.e. it is O(n).
    static Sphere FastEnclosingSphere(const float3 *pointArray, int numPoints);

    /// Returns a Sphere that bounds the given point array.
    /// This function implements Emo Welzl's optimal enclosing sphere algorithm.
    /// See "Smallest enclosing disks (balls and ellipsoids)", Lecture Notes in Computer Science 555 (1991) pp. 359-370.
    /// The running time is expected linear time, but compared to Ritter's algorithm (the FastEnclosingSphere() function),
    /// this algorithm is considerably slower.   
//    static Sphere OptimalEnclosingSphere(const float3 *pointArray, int numPoints);

/*
    static Sphere ApproximateEnclosingSphere(const float3 *pointArray, int numPoints);

    float3 RandomPointInside(LCG &rng) const;
    float3 RandomPointOnSurface(LCG &rng) const;
*/
    /// Returns the distance of this sphere to the given point.
    float Distance(const float3 &point) const;
    float Distance(const Sphere &sphere) const;
    float Distance(const Capsule &capsule) const;
    float Distance(const AABB &aabb) const;
    float Distance(const OBB &obb) const;
    float Distance(const Plane &plane) const;
    float Distance(const Triangle &triangle) const;
    float Distance(const Ray &ray) const;
    float Distance(const Line &line) const;
    float Distance(const LineSegment &lineSegment) const;

    /// Returns the closest point inside this sphere to the given point.
    float3 ClosestPoint(const float3 &point) const;

    /// Computes the intersection of a ray and a sphere.
    /** @param r The ray to test. The ray direction vector must be normalized.
	    @param center The center position of the sphere.
	    @param radius The sphere radius.
	    @param intersectionPoint [out] The intersection position will be returned here.
	    @param intersectionNormal [out] The normal vector of the sphere at intersection position will be returned here.
	    @param d [out] The distance from ray origin to the intersection point along the ray line will be returned here.
	    @return True if an intersection occurs, false otherwise. */
    bool Intersects(const LineSegment &l, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
    bool Intersects(const Line &l, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
    bool Intersects(const Ray &r, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
    bool Intersects(const Plane &plane) const;
    bool Intersects(const AABB &aabb, float3 *closestPointOnAABB) const;
    bool Intersects(const OBB &obb, float3 *closestPointOnOBB) const;
    bool Intersects(const Triangle &triangle, float3 *closestPointOnTriangle) const;
    bool Intersects(const Capsule &capsule) const;
    bool Intersects(const Polygon &polygon) const;

    /*
    float Distance(const float3 &point, float3 &outClosestPointOnSphere) const;

    bool Intersects(const Line &line, float &outDistance) const;
    bool Intersects(const LineSegment &lineSegment, float &outDistance) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Plane &plane) const;
    bool Intersects(const Ellipsoid &ellipsoid) const;
    bool Intersects(const Triangle &triangle) const;
    bool Intersects(const Cylinder &cylinder) const;
    bool Intersects(const Torus &torus) const;
    bool Intersects(const Frustum &frustum) const;
    bool Intersects(const Polygon &polygon) const;
    bool Intersects(const Polyhedron &polyhedron) const;
    */
    bool Intersects(const Sphere &sphere) const;

    void Enclose(const AABB &aabb);
    void Enclose(const OBB &obb);
    void Enclose(const Sphere &sphere);
//    void Enclose(const Triangle &triangle);
//    void Enclose(const Polygon &polygon);
//    bool Enclose(const Polyhedron &polyhedron);
    void Enclose(const LineSegment &lineSegment);
    void Enclose(const float3 &point);
    void Enclose(const float3 *pointArray, int numPoints);

	float3 RandomPointInside(LCG &lcg);
	float3 RandomPointOnSurface(LCG &lcg);
	static float3 RandomPointInside(LCG &lcg, const float3 &center, float radius);
	static float3 RandomPointOnSurface(LCG &lcg, const float3 &center, float radius);

	static float3 RandomUnitaryFloat3(LCG &lcg) { return Sphere(float3(0,0,0), 1.f).RandomPointOnSurface(lcg); }

	/// Produces a geosphere-triangulation of this Sphere.
	/// @param outPos [out] An array of size numVertices which will receive a triangle list of vertex positions. Cannot be null.
	/// @param outNormal [out] An array of size numVertices which will receive vertex normals. If this parameter is null, vertex normals are not returned.
	/// @param numVertices The size of the input arrays outPos and outNormal. This value should be of form 12 + 6*n for some n >= 0.
	///                   To generate a perfect geosphere, pass in a number of form 3 * 4 * 3^k for some k >= 0.
	/// @return The actual number of vertices generated (== the number of elements written to outPos and outNormal).
	int Triangulate(float3 *outPos, float3 *outNormal, float2 *outUV, int numVertices);

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this Sphere. Most useful for debugging purposes.
    std::string ToString() const;
#endif
#ifdef QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

MATH_END_NAMESPACE

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Sphere)
Q_DECLARE_METATYPE(Sphere*)
#endif
