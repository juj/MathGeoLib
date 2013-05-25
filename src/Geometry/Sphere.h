/* Copyright Jukka Jylänki

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

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

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

	/// Translates this Sphere in world space.
	/** @param offset The amount of displacement to apply to this Sphere, in world space coordinates.
		@see Transform(). */
	void Translate(const float3 &offset);

	/// Applies a transformation to this Sphere, in-place.
	/** See Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Returns the smallest AABB that encloses this sphere.
	/** The returned AABB is a cube, with a center position coincident with this sphere, and a side length of 2*r.
		@see MaximalContainedAABB(). */
	AABB MinimalEnclosingAABB() const;

	/// Returns the largest AABB that fits inside this sphere.
	/** The returned AABB is a cube, with a center position coincident with this sphere, and a side length of 2*r/Sqrt(3).
		@see MinimalEnclosingAABB(). */
	AABB MaximalContainedAABB() const;

	/// Sets pos = (0,0,0) and r = -inf.
	/** After a call to this function, both IsFinite() and IsDegenerate() will return true.
		@see IsFinite(), IsDegenerate(). */
	void SetNegativeInfinity();

	/// Computes the volume of this sphere.
	/// @return 4*pi*r^3/3.
	/// @see SurfaceArea(), Diameter().
	float Volume() const;

	/// Computes the surface area of this sphere.
	/// @return 4*pi*r^2.
	/// @see Volume(), Diameter().
	float SurfaceArea() const;

	/// Computes the diameter of this sphere.
	/// @return 2*r.
	/// @see r, SurfaceArea(), Volume().
	float Diameter() const { return 2.f * r; }
			
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

	/// Projects this Sphere onto the given 1D axis direction vector.
	/** This function collapses this Sphere onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const;

	/// Tests if this Sphere is finite.
	/** A sphere is <b><i>finite</i></b> if its members pos and r do not contain floating-point NaNs or +/-infs
		in them.
		@return True if the members pos and r both have finite floating-point values.
		@see pos, r, IsDegenerate(), ::IsFinite(), IsInf(), IsNan(), IsFinite(), inf, negInf, nan, float3::nan, float3::inf, SetNegativeInfinity(). */
	bool IsFinite() const;

	/// Returns true if this Sphere is degenerate.
	/** A sphere is <b><i>degenerate</i></b> if it is not finite, or if the radius of the sphere is less or equal to 0.
		@see pos, r, IsFinite(), SetNegativeInfinity(). */
	bool IsDegenerate() const;

	/// Resets the members of this Sphere to NaN values.
	/** After calling this function, pos = NaN and r = NaN for this sphere. */
	void SetDegenerate();

	/// Tests if the given object is fully contained inside this sphere.
	/** @see Distance(), Intersects(), ClosestPoint().
		@todo Add Sphere::Contains(Circle/Disc). */
	bool Contains(const float3 &point) const;
	bool Contains(const float3 &point, float epsilon) const;
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

	/// Computes the minimal bounding sphere for the given point array.
	/** This function implements Emo Welzl's optimal enclosing sphere algorithm.
		See "Smallest enclosing disks (balls and ellipsoids)", Lecture Notes in Computer Science 555 (1991) pp. 359-370.
		The running time is expected linear time, but compared to Ritter's algorithm (the FastEnclosingSphere() function),
		this algorithm is considerably slower.
		The implementation of this function is based on the book Geometric Tools for Computer Graphics, pp. 807-813, by
		Schneider and Eberly.
		@param pointArray An array of points to compute an enclosing sphere for. This pointer must not be null.
		@param numPoints The number of elements in the input array pointArray.
		@see FastEnclosingSphere(). */
	static Sphere OptimalEnclosingSphere(const float3 *pointArray, int numPoints);

	/// Computes the minimal bounding sphere for two points.
	/** This function computes the smallest volume sphere that contains the given two points. For two points,
		the OptimalEnclosingSphere(a, b) is the same as calling FitThroughPoints(a, b).
		@see FitThroughPoints(). */
	static Sphere OptimalEnclosingSphere(const float3 &a, const float3 &b);

	/// Computes the minimal bounding sphere for three points.
	/** This function computes the smallest volume sphere that contains the given three points. The smallest
		enclosing sphere may not pass through all the three points that are specified. */
	static Sphere OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c);

	/// Computes the minimal bounding sphere for four points.
	/** This function computes the smallest volume sphere that contains the given four points. The smallest
		enclosing sphere may not pass through all the four points that are specified. */
	static Sphere OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c, const float3 &d);

	/// Computes the minimal bounding sphere for four points.
	/** This function computes the smallest volume sphere that contains the given five points.
		@param redundantPoint [out] Since a sphere is characterized by four points, one of the given points
			will necessarily be redundant and not part of the support set of the minimal enclosing sphere. This
			parameter will receive the index [0, 4] of the five input points denoting which point became redundant.
			(Note that there may be more than one point not lying on the support set of the sphere, but this function
			only returns one)
		@return The smallest volume sphere that encloses the given five points. */
	static Sphere OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c, const float3 &d, const float3 &e,
	                                     int &redundantPoint);

	/// Fits a sphere through the given two points.
	/** Two points do not uniquely define a sphere in 3D space. This function computes the minimal enclosing
		sphere for the given two points, which is uniquely defined. This function is identical to
		OptimalEnclosingSphere(a, b) and is simply an alias for that function.
		@see OptimalEnclosingSphere(). */
	static Sphere FitThroughPoints(const float3 &a, const float3 &b) { return OptimalEnclosingSphere(a, b); }

	/// Fits a sphere through the given three points.
	/** Three points do not uniquely define a sphere in 3D space. This function computes the sphere that goes
		through these three points and has the minimal volume. Note that this is not the same than the smallest
		sphere that encloses three given points, since the smallest enclosing sphere does not necessarily pass
		through all the three points.
		@note The three points that are passed in must not be collinear, because in that case a sphere cannot
			be fitted through these points.
		@see OptimalEnclosingSphere(). */
	static Sphere FitThroughPoints(const float3 &a, const float3 &b, const float3 &c);

	/// Fits a sphere through the given four points.
	/** Four points uniquely define a sphere in 3D space. This function computes the sphere that passes through
		these four points. Note that this is not the same than the smallest enclosing sphere for the four points,
		since the smallest enclosing sphere does not necessarily need to pass through all of these four points.
		@note The four points that are passed in must not be coplanar, because in that case a sphere cannot
			be fitted through these points.
		@see OptimalEnclosingSphere(). */
	static Sphere FitThroughPoints(const float3 &a, const float3 &b, const float3 &c, const float3 &d);

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

	/// Returns the maximal distance of this sphere to the given point.
	/** The maximal distance is the distance of the farthest point inside this sphere to the target point. */
	float MaxDistance(const float3 &point) const;

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
		@return In the case of Ray/Line/LineSegment intersection tests, the number of intersections is returned (0, 1 or 2).
			For other functions, true is returned if an intersection occurs or one of the objects is contained inside the other, and false otherwise.
		@see Contains(), Distance(), ClosestPoint(), LineSegment::GetPoint(). */
	int Intersects(const LineSegment &lineSegment, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0, float *d2 = 0) const;
	int Intersects(const Line &line, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0, float *d2 = 0) const;
	int Intersects(const Ray &ray, float3 *intersectionPoint = 0, float3 *intersectionNormal = 0, float *d = 0, float *d2 = 0) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const AABB &aabb, float3 *closestPointOnAABB = 0) const;
	bool Intersects(const OBB &obb, float3 *closestPointOnOBB = 0) const;
	bool Intersects(const Triangle &triangle, float3 *closestPointOnTriangle = 0) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	bool Intersects(const Sphere &sphere) const;

	/// Expands this sphere to enclose both the original sphere and the given object.
	/** This function may adjust both the position and the radius of this sphere to produce a new sphere that encloses
		both objects. The sphere that results may not be the optimal enclosing sphere.
		This function operates in-place.
		@param epsilon An extra distance-squared parameter for the amount to overgrow the Sphere to encompass the given point.
			This can be set to zero, but it may cause that Sphere::Contains(point) may return false.
		@note This function will not produce the optimal enclosure.
		@see FastEnclosingSphere(), OptimalEnclosingSphere(). */
	void Enclose(const float3 &point, float epsilon = 1e-2f);
	void Enclose(const float3 *pointArray, int numPoints);
	void Enclose(const LineSegment &lineSegment);
	void Enclose(const AABB &aabb);
	void Enclose(const OBB &obb);
	void Enclose(const Sphere &sphere);
	void Enclose(const Triangle &triangle);
	void Enclose(const Polygon &polygon);
	void Enclose(const Polyhedron &polyhedron);
	void Enclose(const Frustum &frustum);

	/// Generates a random point inside this sphere.
	/** The points are distributed uniformly.
		This function uses the rejection method to generate a uniform distribution of points inside a sphere. Therefore
		it is assumed that this sphere is not degenerate, i.e. it has a positive radius.
		A fixed number of 1000 tries is performed, after which the sphere center position is returned as a fallback.
		@param lcg A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointOnSurface(), IsDegenerate().
		@todo Add Sphere::Point(polarYaw, polarPitch, radius). */
	float3 RandomPointInside(LCG &lcg);
	static float3 RandomPointInside(LCG &lcg, const float3 &center, float radius);
	/// Generates a random point on the surface of this sphere.
	/** The points are distributed uniformly.
		This function uses the rejection method to generate a uniform distribution of points on the surface. Therefore
		it is assumed that this sphere is not degenerate, i.e. it has a positive radius.
		A fixed number of 1000 tries is performed, after which a fixed point on the surface is returned as a fallback.
		@param lcg A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), IsDegenerate().
		@todo Add Sphere::PointOnSurface(polarYaw, polarPitch). */
	float3 RandomPointOnSurface(LCG &lcg);
	static float3 RandomPointOnSurface(LCG &lcg, const float3 &center, float radius);

	/// Returns a random normalized float3 vector.
	/** @param lcg A pre-seeded random number generator object that is to be used by this function to generate random values. */
	static float3 RandomUnitaryFloat3(LCG &lcg) { return Sphere(float3(0,0,0), 1.f).RandomPointOnSurface(lcg); }

	/// Produces a geosphere-triangulation of this sphere.
	/** @param outPos [out] An array of size numVertices which will receive a triangle list of vertex positions. Cannot be null.
		@param outNormal [out] An array of size numVertices which will receive vertex normals. If this parameter is null, vertex normals are not generated.
		@param outUV [out] An array of size numVertices which will receive UV coordinates. If this parameter is null, UV coordinates are not generated.
		@param numVertices The size of the input arrays outPos and outNormal. This value should be of form 24*4^n for some n >= 0. (24, 96, 384, 1536, 6144, 24576, ...)
		@return The actual number of vertices generated (== the number of elements written to outPos and outNormal). */
	int Triangulate(float3 *outPos, float3 *outNormal, float2 *outUV, int numVertices, bool ccwIsFrontFacing) const;

	/// Computes the intersection of a line and a sphere.
	/** This function solves the points of intersection between a line and a sphere.
		A line intersects sphere at 0, 1 or 2 points. When only one point of intersection is reported,
		the given line is tangent to the sphere.
		@param linePos The source position of the line.
		@param lineDir The direction of the line. This vector must be normalized in advance.
		@param sphereCenter The center position of the sphere to test.
		@param sphereRadius The radius of the sphere, which must be >= 0.
		@param t1 [out] This parameter receives the parametric distance along the line to the first point of intersection.
			If the sphere and the line do not intersect, this variable is not written to. To receive the actual
			world space point corresponding to this point of intersection, compute the vector 'linePos + t1 * lineDir'.
		@param t2 [out] This parameter receives the parametric distance along the line to the second point of intersection.
			If the sphere and the line do not intersect, this variable is not written to. If the line is tangent to this
			sphere (one point of intersection), this variable will be set equal to t1, so that the line segment
			[t1, t2] always forms a line segment completely enclosed inside the sphere. To receive the actual world space
			point corresponding to this point of intersection, compute the vector 'linePos + t2 * lineDir'.
		@return The number of intersection points: 0, 1 or 2. In case of 0, the line and sphere do not intersect. In
			case of 1, the line is tangent to the sphere. If the value of 2 is returned, the line properly intersects the
			sphere.
		@note The outputted variables t1 and t2 always satisfy t1 < t2. This allows distinguishing between the "enter"
			and "exit" positions of the line, if the line is interpreted more like a ray starting at linePos, and extending
			towards lineDir. */
	static int IntersectLine(const float3 &linePos, const float3 &lineDir, const float3 &sphereCenter,
	                         float sphereRadius, float &t1, float &t2);

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this Sphere. Most useful for debugging purposes.
	std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
	void Triangulate(VertexBuffer &vb, int numVertices, bool ccwIsFrontFacing) const;
#endif
};

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Sphere)
Q_DECLARE_METATYPE(Sphere*)
#endif

Sphere operator *(const float3x3 &m, const Sphere &s);
Sphere operator *(const float3x4 &m, const Sphere &s);
Sphere operator *(const float4x4 &m, const Sphere &s);
Sphere operator *(const Quat &q, const Sphere &s);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Sphere &sphere);
#endif

MATH_END_NAMESPACE
