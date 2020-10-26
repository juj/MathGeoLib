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

/** @file Plane.h
	@author Jukka Jylänki
	@brief The Plane geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// Specifies a plane in 3D space. This plane is an affine 2D subspace of the 3D space, meaning
/// that its sides extend to infinity, and it does not necessarily pass through the origin.
class Plane
{
public:
	/// The direction this plane is facing at.
	/** This direction vector is always normalized. If you assign to this directly, please remember to only
		assign normalized direction vectors. */
	vec normal;
	/// The offset of this plane from the origin. [similarOverload: normal]
	/** The value -d gives the signed distance of this plane from origin.
		Denoting normal:=(a,b,c), this class uses the convention ax+by+cz = d, which means that:
		 - If this variable is positive, the vector space origin (0,0,0) is on the negative side of this plane.
		 - If this variable is negative, the vector space origin (0,0,0) is on the on the positive side of this plane.
		@note Some sources use the opposite convention ax+by+cz+d = 0 to define the variable d. When comparing equations
			and algorithm regarding planes, always make sure you know which convention is being used, since it affects the
			sign of d. */
	float d;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members normal and d are undefined after creating a new Plane using this
		default constructor. Remember to assign to them before use.
		@see normal, d. */
	Plane() {}
	/// Constructs a plane by directly specifying the normal and distance parameters.
	/** @param normal The direction the plane is facing. This vector must have been normalized in advance.
		@param d The offset of this plane from the origin. The value -d gives the signed distance of this plane from the origin.
		@see normal, d. */
	Plane(const vec &normal, float d);
	/// Constructs a plane by specifying three points on the plane.
	/** The normal of the plane will point to
		the halfspace from which the points are observed to be oriented in counter-clockwise order.
		@note The points v1, v2 and v3 must not all lie on the same line.
		@see Set(). */
	Plane(const vec &v1, const vec &v2, const vec &v3);
	/// Constructs a plane by specifying a single point on the plane, and the surface normal.
	/** @param normal The direction the plane is facing. This vector must have been normalized in advance.
		@see Set(). */
	Plane(const vec &point, const vec &normal);
	/// Constructs a plane by specifying a line that lies on the plane, and the plane normal.
	/** @param line The line object that is to be contained in the newly constructed plane.
		@param normal The direction the plane if facing. This vector must have been normalized in advance. The normal
			of the line must not be collinear with the direction of this normal. If a line segment is specified, the line
			segment must not be degenerate. */
	Plane(const Ray &line, const vec &normal);
	Plane(const Line &line, const vec &normal);
	Plane(const LineSegment &line, const vec &normal);

	bool IsDegenerate() const;

	/// Sets this plane by specifying three points on the plane.
	/** The normal of the plane will point to the halfspace from which the points are observed to be oriented in
		counter-clockwise order.
		@note The points v1, v2 and v3 must not all lie on the same line. */
	void Set(const vec &v1, const vec &v2, const vec &v3);
	/// Sets this plane by specifying a single point on the plane, and the surface normal.
	/** @param normal The direction the plane is facing. This vector must have been normalized in advance. */
	void Set(const vec &point, const vec &normal);

	/// Reverses the direction of the plane normal, while still representing the same set of points.
	/** This function sets normal = -normal and d = -d for this plane.
		@see normal, d. */
	void ReverseNormal();

	/// Returns a point on this plane.
	/** @note This point has the special property that the line passing through the vector space origin (0,0,0)
			and the returned point is perpendicular to this plane (directed towards the normal vector of this plane).
		@see Point(). */
	vec PointOnPlane() const;

	/// Returns a point on this plane, parameterized at the given coordinates.
	/** The basis directions for U and V are arbitrarily (but consistently) defined.
		Calling Point(0,0) is the same as calling PointOnPlane().
		@see PointOnPlane(). */
	vec Point(float u, float v) const;

	/// Returns a point on this plane, parameterized at the given coordinates.
	/** The basis directions for U and V are arbitrarily (but consistently) defined.
		@param referenceOrigin A point that defines an origin for the returned points. This point does not have to lie
			on this plane.
		Calling Point(0, 0, referenceOrigin) returns the point referenceOrigin projected onto this plane.
		Calling Point(u, v) is the same as calling Point(u, v, PointOnPlane()).
		@see PointOnPlane(). */
	vec Point(float u, float v, const vec &referenceOrigin) const;

	/// Translates this Plane in world space.
	/** @param offset The amount of displacement to apply to this Plane, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this plane.
	/** This function operates in-place.
		@see Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Tests if the given direction vector points towards the positive side of this plane.
	/** @param directionVector The direction vector to compare with the normal of this plane. This vector
		may be unnormalized.
		@see IsOnPositiveSide. */
	bool IsInPositiveDirection(const vec &directionVector) const;

	/// Tests if the given point lies on the positive side of this plane.
	/** A plane divides the space in three sets: the negative halfspace, the plane itself, and the positive halfspace.
		The normal vector of the plane points towards the positive halfspace.
		@return This function returns true if the given point lies either on this plane itself, or in the positive
			halfspace of this plane.
		@see IsInPositiveDirection, AreOnSameSide(), Distance(), SignedDistance(). */
	bool IsOnPositiveSide(const vec &point) const;

	/// Performs a Triangle-Plane intersection test.
	/** @return This function returns the value 1 if the whole triangle is on the positive side of this plane, the
			value -1 if the whole triangle lies in the negative side of this plane, and 0 if the triangle intersects this plane.
		@see Intersects(), AreOnSameSide(), Distance(), SignedDistance(), Contains(). */
	int ExamineSide(const Triangle &triangle) const;

	/// Tests if two points are on the same side of this plane.
	/** @return This function returns true if both p1 and p2 are on the positive side or this plane, or if both p1 and p2
			are on the negative side of this plane.
		@see IsOnPositiveSide(), Distance(), SignedDistance(). */
	bool AreOnSameSide(const vec &p1, const vec &p2) const;

	/// Returns the distance of this plane to the given object.
	/** If the given object intersects or lies in this plane, then the returned distance is zero.
		@note This function always returns a positive distance value, even when the given object lies on the negative side
			of this plane. See the SignedDistance() function to produce a distance value that differentiates between the
			front and back sides of this plane.
		@see SignedDistance(), Intersects(), Contains(). */
	float Distance(const vec &point) const;
	float Distance(const LineSegment &lineSegment) const;
	float Distance(const Sphere &sphere) const;
	float Distance(const Capsule &capsule) const;

	/// Returns the signed distance of this plane to the given point.
	/** If this function returns a negative value, the given point lies in the negative halfspace of this plane.
		Conversely, if a positive value is returned, then the given point lies in the positive halfspace of this plane.
		@see Distance(), IsOnPositiveSide(), AreOnSameSide(). */
	float SignedDistance(const vec &point) const;

	float SignedDistance(const AABB &aabb) const;
	float SignedDistance(const OBB &obb) const;
	float SignedDistance(const Capsule &capsule) const;
//	float SignedDistance(const Circle &circle) const;
	float SignedDistance(const Frustum &frustum) const;
	float SignedDistance(const Line &line) const;
	float SignedDistance(const LineSegment &lineSegment) const;
	float SignedDistance(const Ray &ray) const;
//	float SignedDistance(const Plane &plane) const;
	float SignedDistance(const Polygon &polygon) const;
	float SignedDistance(const Polyhedron &polyhedron) const;
	float SignedDistance(const Sphere &sphere) const;
	float SignedDistance(const Triangle &triangle) const;

	/// Computes the affine transformation matrix that projects orthographically onto this plane.
	/** @see ObliqueProjection(), MirrorMatrix(), Project(). */
	float3x4 OrthoProjection() const;

	/// Projects the given object onto this plane orthographically.
	/** @note This mapping can be expressed as a float3x4 matrix operation. See the OrthoProjection() function.
		@see OrthoProjection(), ProjectToPositiveHalf(), ProjectToNegativeHalf(). */
	vec Project(const vec &point) const;
	LineSegment Project(const LineSegment &lineSegment) const;
	/** @param nonDegenerate [out] If the line or ray is perpendicular to the plane, the projection is
		a single point. In that case, the .pos parameter of the returned object will specify the point
		location, the .dir parameter of the object will be undefined and the nonDegenerate pointer will be
		set to false. This pointer may be null. */
	Line Project(const Line &line, bool *nonDegenerate) const;
	Ray Project(const Ray &ray, bool *nonDegenerate) const;

	Triangle Project(const Triangle &triangle) const;
	Polygon Project(const Polygon &polygon) const;

	/// Projects the given point to the negative half-space of this plane.
	/** This means that if the point lies on the plane, or in the negative half-space, the same point is 
		returned unchanged. If the point lies on the positive half-space, it is projected orthographically onto the plane.
		@see ProjectToPositiveHalf(), Project() */
	vec ProjectToNegativeHalf(const vec &point) const;

	/// Projects the given point to the positivehalf-space of this plane.
	/** @see ProjectToNegativeHalf(), Project() */
	vec ProjectToPositiveHalf(const vec &point) const;

#if 0
	/// Computes the affine transformation matrix that projects onto this plane in an oblique (slanted) angle.
	/** @param obliqueProjectionDir The projection direction. This vector must be normalized. If a vector collinear to the
			normal of this plane is specified, this function returns the same matrix as calling OrthoProjection() would.
		@see OrthoProjection(), MirrorMatrix(), ObliqueProject(). */
	float3x4 ObliqueProjection(const float3 &obliqueProjectionDir) const;

	/// Projects the given point onto this plane in the given oblique projection direction.
	/** @param obliqueProjectionDir The projection direction. This vector must be normalized. If a vector collinear to the
			normal of this plane is specified, this function returns the same matrix as calling OrthoProjection() would.
		@note This mapping can be expressed as a float3x4 operation. See the ObliqueProjection() function.
		@see ObliqueProjection(), Project(). */
	float3 ObliqueProject(const float3 &point, const float3 &obliqueProjectionDir) const;
#endif

	/// Returns the transformation matrix that mirrors objects along this plane.
	/** This matrix maps each point to its mirror point on the opposite side of this plane.
		@see Mirror(). */
	float3x4 MirrorMatrix() const;

	/// Mirrors the given point with respect to this plane.
	/** This function maps the given point to its mirror point on the opposite side of this plane.
		@note This operation can be expressed as a float3x4 matrix operation. To compute the mirror matrix for this
			plane, use the MirrorMatrix() function.
		@see MirrorMatrix(). */
	vec Mirror(const vec &point) const;

	/// Refracts the given incident vector along this plane.
	/** By convention, the input vector should point towards the plane, and the returned vector will point away from the plane.
		When the ray is going from a denser material to a lighter one, total internal reflection can occur.
		In this case, this function will just return a reflected vector from a call to Reflect().
		@param vec Specifies the incident ray direction.
		@param negativeSideRefractionIndex The refraction index of the material we are exiting.
		@param positiveSideRefractionIndex The refraction index of the material we are entering.
		@todo Add Plane::Reflect. */
	vec Refract(const vec &vec, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

	/// Computes the closest point on this plane to the given object.
	/** If the other object intersects this plane, this function will return an arbitrary point inside
		the region of intersection.
		@see Contains(), Distance(), Intersects(). */
	vec ClosestPoint(const vec &point) const { return Project(point); }
	vec ClosestPoint(const Ray &ray) const;
	vec ClosestPoint(const LineSegment &lineSegment) const;

	/// Tests if this plane contains the given object.
	/** @param epsilon Since a plane is a 2D object in a 3D space, an distance threshold is used for the test.
		This value gives a "thickness" to this plane for the purposes of the test.
		@return True if the given object is contained in this plane, up to the given epsilon distance. */
	bool Contains(const vec &point, float epsilon = 1e-3f) const;
	bool Contains(const Line &line, float epsilon = 1e-3f) const;
	bool Contains(const Ray &ray, float epsilon = 1e-3f) const;
	bool Contains(const LineSegment &lineSegment, float epsilon = 1e-3f) const;
	bool Contains(const Triangle &triangle, float epsilon = 1e-3f) const;
	bool Contains(const Circle &circle, float epsilon = 1e-3f) const;
	bool Contains(const Polygon &polygon, float epsilon = 1e-3f) const;

	/// Returns true if this plane represents the same set of points than the other plane.
	/** For this test, the surface normals of the two planes may point in opposite directions, as long as
		the set of points is the same.
		@see Equals(), IsParallel(), DihedralAngle(). */
	bool SetEquals(const Plane &plane, float epsilon = 1e-3f) const;

	/// Returns true if the two planes are equal, and their normals are oriented to the same direction.
	/** @see SetEquals(), IsParallel(), DihedralAngle(). */
	bool Equals(const Plane &other, float epsilon = 1e-3f) const;

	/// Compares whether this Plane and the given Plane are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Plane &other) const { return normal.BitEquals(other.normal) && ReinterpretAsU32(d) == ReinterpretAsU32(other.d); }

	/// Tests if two planes are parallel.
	/** @see SetEquals(), Equals(), DihedralAngle(). */
	bool IsParallel(const Plane &plane, float epsilon = 1e-3f) const;

	/// Returns the angle of intersection between two planes, in radians.
	/** @see SetEquals(), Equals(), IsParallel(). */
	float DihedralAngle(const Plane &plane) const;

	/// Computes the intersection of three planes.
	/** This function computes the intersection of this plane, and the given two planes.
		@param outLine [out] If the three planes are configured in such a manner that intersection is a line,
			this parameter receives the line of intersection. This pointer may be null.
		@param outPoint [out] If the three planes are configured in such a manner that the interesction is a point,
			this parameter receives the point of intersection. This pointer may be null.
		@bug This function never outputs outLine.
		@return True if the intersection was a point, in which case outPoint is written to.
		@see Contains(), Distance(), ClosestPoint(). */
	bool Intersects(const Plane &plane, const Plane &plane2, Line *outLine = 0, vec *outPoint = 0) const;

	/// Tests whether this plane and the given object intersect.
	/** @param outLine [out] The intersection of two planes forms a line. If an intersection occurs, this parameter will receive
		the line of intersection. This pointer may be null.
		@return True if the given object intersects with this plane. */
	bool Intersects(const Plane &plane, Line *outLine = 0) const;
	/** @param dist [out] If specified, this parameter will receive the parametric distance of
			the intersection point along the line object. Use the GetPoint(d) function of the line class
			to get the actual point of intersection. This pointer may be null. */
	bool Intersects(const Ray &ray, float *dist = 0) const;
	bool Intersects(const Line &line, float *dist = 0) const;
	bool Intersects(const LineSegment &lineSegment, float *dist = 0) const;
	bool Intersects(const Sphere &sphere) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	/// @todo Add a version of Plane-Triangle intersection which returns the line segment of intersection.
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Capsule &capsule) const;
	/// Tests if this plane intersects with the given circle.
	/** @param pt1 [out] If specified, receives the first point of intersection. This pointer may be null.
		@param pt2 [out] If specified, receives the second point of intersection. This pointer may be null.
		@return The number of intersections that occurred: 0, 1 or 2. */
	int Intersects(const Circle &circle, vec *pt1, vec *pt2) const;
	int Intersects(const Circle &circle) const;

	/// Clips a line segment against this plane.
	/** This function removes the part of the line segment which lies in the negative halfspace of this plane.
		The clipping operation is performed in-place. If the whole line segment is clipped, the input variables
		are not modified.
		@return If this function returns true, the line segment after clipping did not become degenerate.
				If false is returned, the whole line segment lies in the negative halfspace, and no output line segment
				was generated. */
	bool Clip(LineSegment &line) const;
	bool Clip(vec &a, vec &b) const;

	/// Clips a line against this plane.
	/** This function removes the part of the line which lies in the negative halfspace of this plane.
		@param line The line to clip. If this function returns 2, then the input line should be preserved.
		@param outRay [out] If this function returns 1, then this parameter will receive the ray object that was formed.
		@return If the clipping removed the whole line, the value 0 is returned.
				If the clipping resulted in a ray, the value 1 is returned.
				If the clipping resulted in a line, the value 2 is returned. */
	int Clip(const Line &line, Ray &outRay) const;

	/// Clips a triangle against this plane.
	/** This function removes the part of the triangle which lies in the negative halfspace of this plane.
		@return This function reports how many output triangles were generated.
				If the whole input triangle was clipped, the value 0 is returned.
				If this function returns 1, the value t1 will receive the generated output triangle.
				If this function returns 2, t1 and t2 will receive the generated output triangles. */
	int Clip(const Triangle &triangle, Triangle &t1, Triangle &t2) const;

	/// Returns true if this plane contains the origin.
	/** The test is performed up to the given epsilon.
		@note A plane passes through the origin if and only if d == 0 for the plane.
		@see d. */
	bool PassesThroughOrigin(float epsilon = 1e-3f) const;

	// Returns true if this plane is a separating plane for the given two objects.
//	bool IsSeparatingPlane(const Polyhedron &obj1, const Polyhedron &obj2) const;

	/// Returns a circle that lies on this plane.
	/** @return The generated circle has its center as close as possible to the specified center point,
		and the radius is as specified. */
	Circle GenerateCircle(const vec &circleCenter, float radius) const;

//	float3 RandomPointInsideCircle(const float3 &circleCenter, float radius) const;
//	float3 RandomPointOnCircleEdge(const float3 &circleCenter, float radius) const;

	/// Computes the intersection of a line and a plane.
	/** @param planeNormal The plane normal direction vector. This vector can be unnormalized.
		@param planeD The distance parameter of the plane equation.
		@param linePos The starting point of the line.
		@param lineDir The line direction vector. This vector does not need to be normalized.
		@param t [out] If this function returns true, this parameter will receive the distance along the line where intersection occurs.
					That is, the point lineStart + t * lineDir will be the intersection point. Note that if |lineDir| != 1,
					then t will not contain the real distance, but one scaled to the units of lineDir.
		@return If an intersection occurs, this function returns true. */
	static bool IntersectLinePlane(const vec &planeNormal, float planeD, const vec &linePos, const vec &lineDir, float &t);

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this Plane. Most useful for debugging purposes.
	StringT ToString() const;
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static Plane FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	static Plane FromString(const char *str, const char **outEndStr = 0);

#ifdef MATH_GRAPHICSENGINE_INTEROP
	void Triangulate(VertexBuffer &vb, float uWidth, float vHeight, const vec &centerPoint, int numFacesU, int numFacesV, bool ccwIsFrontFacing) const;
	void ToLineList(VertexBuffer &vb, float uWidth, float vHeight, const vec &centerPoint, int numLinesU, int numLinesV) const;
#endif
};

struct Plane_storage
{
	float4_storage normal;
	float d;
#ifdef MATH_VEC_IS_FLOAT4
	float padding[3];
#endif
	Plane_storage(){}
	Plane_storage(const Plane &rhs)
	{
		*reinterpret_cast<Plane*>(this) = rhs;
	}
	operator Plane() const { return *reinterpret_cast<const Plane*>(this); }
};

Plane operator *(const float3x3 &transform, const Plane &plane);
Plane operator *(const float3x4 &transform, const Plane &plane);
Plane operator *(const float4x4 &transform, const Plane &plane);
Plane operator *(const Quat &transform, const Plane &plane);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Plane &plane);
#endif

MATH_END_NAMESPACE
