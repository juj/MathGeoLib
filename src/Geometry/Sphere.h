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

/** @file Sphere.h
	@author Jukka Jylänki
	@brief The Sphere geometry object. */
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
	/** [similarOverload: pos] */
	float r;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos and r are undefined after creating a new Sphere using this
		default constructor. Remember to assign to them before use.
		@see pos, r. */
	Sphere() {}

	/// Constructs a sphere with a given position and radius.
	/** @param radius A value > 0 constructs a sphere with positive volume. A value of <= 0 is valid, and constructs a degenerate sphere.
		@see pos, r, IsFinite(), IsDegenerate() */
	Sphere(const float3 &center, float radius);

	/// Constructs a sphere that passes through the given two points.
	/** The constructed sphere will be the minimal sphere that encloses the given two points. The center point of this 
		sphere will lie midway between pointA and pointB, and the radius will be half the distance between pointA and 
		pointB. Both input points are assumed to be finite. */
	Sphere(const float3 &pointA, const float3 &pointB);

	/// Constructs a sphere that passes through the given three points.
	/** @note The resulting sphere may not be the minimal enclosing sphere for the three points! */
	Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC);

	/// Constructs a sphere that passes through the given four points.
	/** @note The resulting sphere may not be the minimal enclosing sphere for the four points! */
	Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD);

	/// Returns the smallest AABB that encloses this sphere.
	/** The returned AABB is a cube, with a center position coincident with this sphere, and a side length of 2*r.
		@see MaximalContainedAABB(). */
	AABB MinimalEnclosingAABB() const;

	/// Returns the largest AABB that fits inside this sphere.
	/** The returned AABB is a cube, with a center position coincident with this sphere, and a side length of 2*r/sqrt(3).
		@see MinimalEnclosingAABB(). */
	AABB MaximalContainedAABB() const;

	/// Sets pos = (0,0,0) and r = -inf.
	/** After a call to this function, both IsFinite() and IsDegenerate() will return true.
		@see IsFinite(), IsDegenerate(). */
	void SetNegativeInfinity();

	/// Computes the volume of this sphere.
	/// @return 4*pi*r^3/3.
	/// @see SurfaceArea().
	float Volume() const;

	/// Computes the surface area of this sphere.
	/// @return 4*pi*r^2.
	/// @see Volume().
	float SurfaceArea() const;

	/// Returns the center of mass of this sphere.
	/** @return pos.
		@see pos */
	float3 Centroid() const { return pos; }

	/// Computes the extreme point of this Sphere in the given direction.
	/** An extreme point is a farthest point of this Sphere in the given direction. For 
		a Sphere, this point is unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return The extreme point of this Sphere in the given direction. */
	float3 ExtremePoint(const float3 &direction) const;

	/// Tests if this Sphere is finite.
	/** A sphere is <b><i>finite</i></b> if its members pos and r do not contain floating-point NaNs or +/-infs
		in them.
		@return True if the members pos and r both have finite floating-point values.
		@see pos, r, IsDegenerate(), ::IsFinite(), IsInf(), IsNan(), isfinite(), inf, negInf, nan, float3::nan, float3::inf, SetNegativeInfinity(). */
	bool IsFinite() const;

	/// Returns true if this Sphere is degenerate.
	/** A sphere is <b><i>degenerate</i></b> if it is not finite, or if the radius of the sphere is less or equal to 0.
		@see pos, r, IsFinite(), SetNegativeInfinity(). */
	bool IsDegenerate() const;

	/// Tests if the given object is fully contained inside this sphere.
	/** @see Distance(), Intersects(), ClosestPoint().
		@todo Add Sphere::Contains(Circle/Disc). */
	bool Contains(const float3 &point) const;
	bool Contains(const LineSegment &lineSegment) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;
	bool Contains(const AABB &aabb) const;
	bool Contains(const OBB &obb) const;
	bool Contains(const Frustum &frustum) const;
	bool Contains(const Polyhedron &polyhedron) const;
	bool Contains(const Sphere &sphere) const;
	bool Contains(const Capsule &capsule) const;

	/// Computes a Sphere that bounds the given point array.
	/** This functions implements a fast approximate (though rather crude) algorithm of Jack Ritter.
		See "An Efficient Bounding Sphere", in Graphics Gems 1, pp. 301-303,
		or Christer Ericson's Real-time Collision Detection, pp. 89-91.
		This algorithm performs two linear passes over the data set, i.e. it has a time complexity of O(n).
		@param pointArray An array of points to compute an enclosing sphere for. This pointer must not be null.
		@param numPoints The number of elements in the input array pointArray.
		@see OptimalEnclosingSphere(). */
	static Sphere FastEnclosingSphere(const float3 *pointArray, int numPoints);

	/// Returns a Sphere that bounds the given point array.
	/** This function implements Emo Welzl's optimal enclosing sphere algorithm.
		See "Smallest enclosing disks (balls and ellipsoids)", Lecture Notes in Computer Science 555 (1991) pp. 359-370.
		The running time is expected linear time, but compared to Ritter's algorithm (the FastEnclosingSphere() function),
		this algorithm is considerably slower.
		@param pointArray An array of points to compute an enclosing sphere for. This pointer must not be null.
		@param numPoints The number of elements in the input array pointArray.
		@see FastEnclosingSphere(). */
	static Sphere OptimalEnclosingSphere(const float3 *pointArray, int numPoints);

/*
	static Sphere ApproximateEnclosingSphere(const float3 *pointArray, int numPoints);

*/
	/// Returns the distance between this sphere and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@see Contains(), Intersects(), ClosestPoint().
		@todo Add Sphere::Distance(Polygon/Circle/Disc/Frustum/Polyhedron). */
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

	/// Computes the closest point on this sphere to the given object.
	/** If the other object intersects this sphere, this function will return an arbitrary point inside
		the region of intersection.
		@see Contains(), Distance(), Intersects(). 
		@todo Add Sphere::ClosestPoint(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	float3 ClosestPoint(const float3 &point) const;

	/// Tests whether this sphere and the given object intersect.
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside 
		another, this function still returns true. (e.g. in case a line segment is contained inside this sphere, 
		or this sphere is contained inside a polyhedron, etc.)
		@param intersectionPoint [out] If specified, receives the actual point of intersection. This pointer may be null.
		@param intersectionNormal [out] If specified, receives the sphere normal at the point of intersection. This pointer may be null.
		@param d [out] If specified, receives the distance along the Line/LineSegment/Ray to the intersection. This pointer may be null.
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@see Contains(), Distance(), ClosestPoint(), LineSegment::GetPoint(). */
	bool Intersects(const LineSegment &lineSegment, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
	bool Intersects(const Line &line, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
	bool Intersects(const Ray &ray, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const AABB &aabb, float3 *closestPointOnAABB) const;
	bool Intersects(const OBB &obb, float3 *closestPointOnOBB) const;
	bool Intersects(const Triangle &triangle, float3 *closestPointOnTriangle) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	bool Intersects(const Sphere &sphere) const;

	/// Expands this sphere to enclose both the original sphere and the given object.
	/** This function may adjust both the position and the radius of this sphere to produce a new sphere that encloses
		both objects. The sphere that results may not be the optimal enclosing sphere.
		This function operates in-place.
		@see FastEnclosingSphere().
		@todo Add Sphere::Enclose(Triangle/Polygon/Polyhedron). */
	void Enclose(const float3 &point);
	void Enclose(const float3 *pointArray, int numPoints);
	void Enclose(const LineSegment &lineSegment);
	void Enclose(const AABB &aabb);
	void Enclose(const OBB &obb);
	void Enclose(const Sphere &sphere);

	/// Generates a random point inside this sphere.
	/** The points are distributed uniformly.
		This function uses the rejection method to generate a uniform distribution of points inside a sphere. Therefore
		it is assumed that this sphere is not degenerate, i.e. it has a positive radius.
		A fixed number of 1000 tries is performed, after which the sphere center position is returned as a fallback.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointOnSurface(), IsDegenerate().
		@todo Add Sphere::Point(polarYaw, polarPitch, radius). */
	float3 RandomPointInside(LCG &lcg);
	static float3 RandomPointInside(LCG &lcg, const float3 &center, float radius);
	/// Generates a random point on the surface of this sphere.
	/** The points are distributed uniformly.
		This function uses the rejection method to generate a uniform distribution of points on the surface. Therefore
		it is assumed that this sphere is not degenerate, i.e. it has a positive radius.
		A fixed number of 1000 tries is performed, after which a fixed point on the surface is returned as a fallback.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), IsDegenerate().
		@todo Add Sphere::PointOnSurface(polarYaw, polarPitch). */
	float3 RandomPointOnSurface(LCG &lcg);
	static float3 RandomPointOnSurface(LCG &lcg, const float3 &center, float radius);

	/// Returns a random normalized float3 vector.
	/** @param rng A pre-seeded random number generator object that is to be used by this function to generate random values. */
	static float3 RandomUnitaryFloat3(LCG &lcg) { return Sphere(float3(0,0,0), 1.f).RandomPointOnSurface(lcg); }

	/// Produces a geosphere-triangulation of this sphere.
	/** @param outPos [out] An array of size numVertices which will receive a triangle list of vertex positions. Cannot be null.
		@param outNormal [out] An array of size numVertices which will receive vertex normals. If this parameter is null, vertex normals are not generated.
		@param outUV [out] An array of size numVertices which will receive UV coordinates. If this parameter is null, UV coordinates are not generated.
		@param numVertices The size of the input arrays outPos and outNormal. This value should be of form 12 + 6*n for some n >= 0.
						  To generate a perfect geosphere, pass in a number of form 3 * 4 * 3^k for some k >= 0.
		@return The actual number of vertices generated (== the number of elements written to outPos and outNormal). */
	int Triangulate(float3 *outPos, float3 *outNormal, float2 *outUV, int numVertices);

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this Sphere. Most useful for debugging purposes.
	std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Sphere)
Q_DECLARE_METATYPE(Sphere*)
#endif

MATH_END_NAMESPACE
