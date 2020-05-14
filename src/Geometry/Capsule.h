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

/** @file Capsule.h
	@author Jukka Jylänki
	@brief The Capsule geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "LineSegment.h"

MATH_BEGIN_NAMESPACE

/// A 3D cylinder with spherical ends.
class Capsule
{
public:
	/// Specifies the two inner points of this capsule.
	LineSegment l;

	/// Specifies the radius of this capsule. [similarOverload: l]
	float r;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members l and r are both undefined after creating a new capsule using
		this default constructor. Remember to assign to them before use.
		@see l, r. */
	Capsule() {}

	/// Constructs a new capsule by explicitly specifying the member variables.
	/** @param endPoints Specifies the line segment of the capsule.
		@param radius Specifies the size of this capsule.
		@see l, r. */
	Capsule(const LineSegment &endPoints, float radius);

	/// Constructs a new capsule by explicitly specifying the member variables.
	/** This constructor is equivalent to calling Capsule(LineSegment(bottomPoint, topPoint), radius), but provided
		here for conveniency.
		@see l, r. */
	Capsule(const vec &bottomPoint, const vec &topPoint, float radius);

	/// Constructs a new capsule from a sphere.
	/** This conversion results in a capsule which has its both endpoints at the exact same coordinates, and hence the
		length of the inner line segment is set to 0. */
	void SetFrom(const Sphere &s);

	/// Sets this Capsule to a degenerate negative-volume state.
	void SetDegenerate();

	/// Tests if this Capsule is degenerate.
	/** @return True if this Capsule does not span a strictly positive volume. */
	bool IsDegenerate() const;

	/// Computes the distance of the two inner points of this capsule.
	/** <img src="CapsuleFunctions.png" />
		@see Height(). */
	float LineLength() const;

	/// Computes the total height of this capsule, i.e. LineLength() + Diameter().
	/** <img src="CapsuleFunctions.png" />
		@see LineLength(). */
	float Height() const;

	/// Computes the diameter of this capsule.
	/** <img src="CapsuleFunctions.png" />
		@return 2*r.
		@see r. */
	float Diameter() const;

	/// Returns the bottom-most point of this Capsule.
	/** <img src="CapsuleFunctions.png" />
		@note The bottom-most point is only a naming convention, and does not correspond to the bottom-most point along any world axis. The returned
			point is simply the point at the far end of this Capsule where the point l.a resides.
		@note The bottom-most point of the capsule is different than the point l.a. The returned point is the point at the very far
			edge of this capsule, and does not lie on the internal line. See the attached diagram.
		@see Top(), l. */
	vec Bottom() const;

	/// Returns the center point of this Capsule.
	/** <img src="CapsuleFunctions.png" />
		@return The point (l.a + l.b) / 2. This point is the center of mass for this capsule.
		@see l, Bottom(), Top(). */
	vec Center() const;
	vec Centroid() const { return l.CenterPoint(); } ///< [similarOverload: Center]

	/// Quickly returns an arbitrary point inside this Capsule. Used in GJK intersection test.
	inline vec AnyPointFast() const { return l.a; }

	/// Computes the extreme point of this Capsule in the given direction.
	/** An extreme point is a farthest point of this Capsule in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return The extreme point of this Capsule in the given direction. */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Projects this Capsule onto the given 1D axis direction vector.
	/** This function collapses this Capsule onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	/// Returns the topmost point of this Capsule.
	/** <img src="CapsuleFunctions.png" />
		@note The topmost point is only a naming convention, and does not correspond to the topmost point along any world axis. The returned
			point is simply the point at the far end of this Capsule where the point l.b resides.
		@note The topmost point of the capsule is different than the point l.b. The returned point is the point at the very far
			edge of this capsule, and does not lie on the internal line. See the attached diagram.
		@see Bottom(), l. */
	vec Top() const;

	/// Returns the direction from the bottommost point towards the topmost point of this Capsule.
	/** <img src="CapsuleFunctions.png" />
		@return The normalized direction vector from l.a to l.b.
		@see l. */
	vec UpDirection() const;

	/// Computes the volume of this Capsule.
	/** @return pi * r^2 * |b-a| + 4 * pi * r^2 / 3.
		@see SurfaceArea(). */
	float Volume() const;

	/// Computes the surface area of this Capsule.
	/** @return 2 * pi * r * |b-a| + 4 * pi * r^2.
		@see Volume(). */
	float SurfaceArea() const;

	/// Returns the cross-section circle at the given height of this Capsule.
	/** <img src="CapsuleFunctions.png" />
		@param l A normalized parameter between [0,1]. l == 0 returns a degenerate circle of radius 0 at the bottom of this Capsule, and l == 1
				will return a degenerate circle of radius 0 at the top of this Capsule. */
	Circle CrossSection(float l) const;

	/// Returns a line segment that spans the far axis of this capsule from bottom to tip.
	/** <img src="CapsuleFunctions.png" />
		@see Height(), LineLength(), l. */
	LineSegment HeightLineSegment() const;

	/// Tests if this Capsule is finite.
	/** A Capsule is <b><i>finite</i></b> if none of its member variables contain floating-point NaNs or +/-infs
		in them.
		@return True if each member variable has a finite floating-point value.
		@see l, r.
		@todo Implement IsDegenerate(). */
	bool IsFinite() const;

	/// Generates a point inside this capsule.
	/** @param height A normalized value between [0,1]. This specifies the point position along the height line of this capsule.
		@param angle A normalized value between [0,1]. This specifies the normalized directed angle of the point position around the capsule line segment.
		@param dist A normalized value between [0,1]. This specifies the normalized distance of the point position from the capsule line segment.
		@note This function does not generate points inside this capsule uniformly, as (l,a,d) ranges uniformly over [0,1]^3.
		@see UniformPointPerhapsInside(), RandomPointInside(). */
	vec PointInside(float height, float angle, float dist) const;

	/// Generates a point that perhaps lies inside this capsule.
	/** @param height A normalized value between [0,1]. This specifies the point position along the height line of this capsule.
		@param x A normalized value between [0,1]. This specifies the x coordinate on the plane of the circle cross-section specified by l.
		@param y A normalized value between [0,1]. This specifies the y coordinate on the plane of the circle cross-section specified by l.
		@note This function will generate points uniformly, but they do not necessarily lie inside the capsule.
		@see PointInside(). */
	vec UniformPointPerhapsInside(float height, float x, float y) const;

	/// Returns the Sphere defining the 'bottom' section of this Capsule (corresponding to the endpoint l.a)
	Sphere SphereA() const;

	/// Returns the Sphere defining the 'top' section of this Capsule (corresponding to the endpoint l.b)
	Sphere SphereB() const;

	/// Returns the smallest AABB that encloses this capsule.
	/** @see MinimalEnclosingOBB(). */
	AABB MinimalEnclosingAABB() const;

	/// Returns the smallest OBB that encloses this capsule.
	/** @see MinimalEnclosingAABB(). */
	OBB MinimalEnclosingOBB() const;

	/// Generates a random point inside this capsule.
	/** The points are distributed uniformly.
		@see RandomPointOnSurface(). */
	vec RandomPointInside(LCG &rng) const;

	/// Generates a random point on the surface of this Capsule.
	/** @todo The points are NOT distributed uniformly. Convert this to using the rejection method and RandomPointInside()
			to produce a uniform distribution.
		@see RandomPointInside(). */
	vec RandomPointOnSurface(LCG &rng) const;

	/// Moves this capsule by the given offset vector.
	/** @note This function operates in-place.
		@param offset The world space offset to apply to the position of this capsule.
		@see Transform(), Scale(). */
	void Translate(const vec &offset);
	Capsule Translated(const vec &offset) const;

	/// Applies a uniform scale to this Capsule.
	/** This function scales this capsule structure in-place, using the given center point as the origin
		for the scaling operation.
		@param centerPoint Specifies the center of the scaling operation, in world space.
		@param scaleFactor The uniform scale factor to apply to each world space axis.
		@see Translate(), Transform(). */
	void Scale(const vec &centerPoint, float scaleFactor);

	/// Applies a transformation to this capsule.
	/** @param transform The transformation to apply to this capsule. This transformation must be
		affine, and must contain an orthogonal set of column vectors (may not contain shear or projection).
		The transformation can only contain uniform scale, and may not contain mirroring.
		@see Translate(), Scale(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Computes the closest point inside this capsule to the given point.
	/** If the target point lies inside this capsule, then that point is returned.
		@see Distance(), Contains(), Intersects().
		@todo Add ClosestPoint(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	vec ClosestPoint(const vec &targetPoint) const;

	/// Computes the distance between this capsule and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@todo Add Distance(Triangle/Polygon/Circle/Disc/Capsule).
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const vec &point) const;
	float Distance(const Plane &plane) const;
	float Distance(const Sphere &sphere) const;
	float Distance(const Ray &ray) const;
	float Distance(const Line &line) const;
	float Distance(const LineSegment &lineSegment) const;
	float Distance(const Capsule &capsule) const;

	/// Tests if the given object is fully contained inside this capsule.
	/** This function returns true if the given object lies inside this capsule, and false otherwise.
		@note The comparison is performed using less-or-equal, so the surface of this capsule count as being inside, but
			due to float inaccuracies, this cannot generally be relied upon.
		@todo Add Contains(Circle/Disc/Sphere/Capsule).
		@see Distance(), Intersects(), ClosestPoint(). */
	bool Contains(const vec &point) const;
	bool Contains(const LineSegment &lineSegment) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;
	bool Contains(const AABB &aabb) const;
	bool Contains(const OBB &obb) const;
	bool Contains(const Frustum &frustum) const;
	bool Contains(const Polyhedron &polyhedron) const;

	/// Tests whether this capsule and the given object intersect.	
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this capsule,
		or this capsule is contained inside a Sphere, etc.)
		The first parameter of this function specifies the other object to test against.
		@see Contains(), Distance(), ClosestPoint().
		@todo Add Intersects(Circle/Disc). */
	bool Intersects(const Ray &ray) const;
	bool Intersects(const Line &line) const;
	bool Intersects(const LineSegment &lineSegment) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const Sphere &sphere) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this Capsule. Most useful for debugging purposes.
	/** The returned string specifies the line segment and the radius of this Capsule. */
	StringT ToString() const;
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;

	static Capsule FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	static Capsule FromString(const char *str, const char **outEndStr = 0);

	bool Equals(const Capsule &rhs, float epsilon = 1e-3f) const { return l.Equals(rhs.l, epsilon) && EqualAbs(r, rhs.r, epsilon); }

	/// Compares whether this Capsule and the given Capsule are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Capsule &other) const;
};

Capsule operator *(const float3x3 &transform, const Capsule &capsule);
Capsule operator *(const float3x4 &transform, const Capsule &capsule);
Capsule operator *(const float4x4 &transform, const Capsule &capsule);
Capsule operator *(const Quat &transform, const Capsule &capsule);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Capsule &capsule);
#endif

MATH_END_NAMESPACE
