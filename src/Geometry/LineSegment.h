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

/** @file LineSegment.h
	@author Jukka Jylänki
	@brief The LineSegment geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A line segment in 3D space is a finite line with a start and end point.
class LineSegment
{
public:
	/// The starting point of this line segment.
	vec a;
	/// The end point of this line segment. [similarOverload: a]
	vec b;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members a and b are undefined after creating a new LineSegment using this
		default constructor. Remember to assign to them before use.
		@see a, b. */
	LineSegment() {}

	/// Constructs a line segment through the given end points.
	/** @see a, b. */
	LineSegment(const vec &a, const vec &b);

	/// Constructs a line segment from a ray or a line.
	/** This constructor takes the ray/line origin position as the starting point of this line segment, and defines the end point
		of the line segment using the given distance parameter.
		@param d The distance along the ray/line for the end point of this line segment. This will set b = ray.pos + d * ray.dir
			as the end point. When converting a ray to a line segment, it is possible to pass in a d value < 0, but in that case
			the resulting line segment will not lie on the ray.
		@see a, b, classes Ray, Line, Line::GetPoint(), Ray::GetPoint(). */
	explicit LineSegment(const Ray &ray, float d);
	explicit LineSegment(const Line &line, float d);

	/// Returns a point on the line.
	/** @param d The normalized distance along the line segment to compute. If a value in the range [0, 1] is passed, then the
			returned point lies along this line segment. If some other value is specified, the returned point lies on the
			line defined by this line segment, but not inside the interval from a to b.
		@note The meaning of d here differs from Line::GetPoint and Ray::GetPoint. For the class LineSegment,
			GetPoint(0) returns a, and GetPoint(1) returns b. This means that GetPoint(1) will not generally be exactly one unit
			away from the starting point of this line segment, as is the case with Line and Ray.
		@return (1-d)*a + d*b.
		@see a, b, Line::GetPoint(), Ray::GetPoint(). */
	vec GetPoint(float d) const;

	/// Returns the center point of this line segment.
	/** This function is the same as calling GetPoint(0.5f), but provided here as conveniency.
		@see GetPoint(). */
	vec CenterPoint() const;

	/// Reverses the direction of this line segment.
	/** This function swaps the start and end points of this line segment so that it runs from b to a.
		This does not have an effect on the set of points represented by this line segment, but it reverses
		the direction of the vector returned by Dir().
		@note This function operates in-place.
		@see a, b, Dir(). */
	void Reverse();

	/// Returns the normalized direction vector that points in the direction a->b.
	/** @note The returned vector is normalized, meaning that its length is 1, not |b-a|.
		@see a, b. */
	vec Dir() const;

	/// Quickly returns an arbitrary point inside this LineSegment. Used in GJK intersection test.
	inline vec AnyPointFast() const { return a; }

	/// Computes an extreme point of this LineSegment in the given direction.
	/** An extreme point is a farthest point along this LineSegment in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this LineSegment in the given direction. The returned point is always
			either a or b.
		@see a, b.*/
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Translates this LineSegment in world space.
	/** @param offset The amount of displacement to apply to this LineSegment, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this line.
	/** This function operates in-place.
		@see Translate(), classes float3x3, float3x4, float4x4, Quat, Transform(). */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Computes the length of this line segment.
	/** @return |b-a|.
		@see a, b. */
	float Length() const;
	/// Computes the squared length of this line segment.
	/** Calling this function is faster than calling Length(), since this function avoids computing a square root.
		If you only need to compare lengths to each other and are not interested in the actual length values,
		you can compare by using LengthSq(), instead of Length(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function. [similarOverload: Length] */
	float LengthSq() const;

	/// Tests if this line segment is finite.
	/** A line segment is <b><i>finite</i></b> if its endpoints a and b do not contain floating-point NaNs or +/-infs
		in them.
		@return True if both a and b have finite floating-point values. */
	bool IsFinite() const;

	/// Tests if this line segment represents the same set of points than the given line segment.
	/** @param distanceThreshold Specifies how much distance threshold to allow in the comparison.
		@return True if a == rhs.a && b == rhs.b, or, a == rhs.b && b = rhs.a, within the given epsilon. */
	bool Equals(const LineSegment &rhs, float distanceThreshold = 1e-3f) const;

	/// Compares whether this LineSegment and the given LineSegment are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const LineSegment &other) const { return a.BitEquals(other.a) && b.BitEquals(other.b); }

	/// Tests if the given point or line segment is contained on this line segment.
	/** @param distanceThreshold Because a line segment is an one-dimensional object in 3D space, an epsilon value
			is used as a threshold for this test. This effectively transforms this line segment to a capsule with
			the radius indicated by this value.
		@return True if this line segment contains the given point or line segment.
		@see Intersects, ClosestPoint(), Distance(). */
	bool Contains(const vec &point, float distanceThreshold = 1e-3f) const;
	bool Contains(const LineSegment &lineSegment, float distanceThreshold = 1e-3f) const;

	/// Computes the closest point on this line segment to the given object.
	/** @param d [out] If specified, this parameter receives the normalized distance along
			this line segment which specifies the closest point on this line segment to
			the specified point.
		@return The closest point on this line segment to the given object.
		@see Contains(), Distance(), Intersects(). */
	vec ClosestPoint(const vec &point) const { float d; return ClosestPoint(point, d); }
	vec ClosestPoint(const vec &point, float &d) const;
	vec ClosestPointD(const vec &point, double &d) const;
	/** @param d2 [out] If specified, this parameter receives the (normalized, in case of line segment)
			distance along the other line object which specifies the closest point on that line to
			this line segment. */
	vec ClosestPoint(const Ray &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d, float &d2) const;
	vec ClosestPoint(const Line &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d, float &d2) const;
	vec ClosestPoint(const LineSegment &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d, float &d2) const;

	/// Computes the distance between this line segment and the given object.
	/** @param d [out] If specified, this parameter receives the normalized distance along
			this line segment which specifies the closest point on this line segment to
			the specified point.
		@return The distance between this line segment and the given object.
		@see Constains(), ClosestPoint(), Intersects(). */
	float Distance(const vec &point) const { float d; return Distance(point, d); }
	float Distance(const vec &point, float &d) const;
	/** @param d2 [out] If specified, this parameter receives the (normalized, in case of line segment)
			distance along the other line object which specifies the closest point on that line to
			this line segment. */
	float Distance(const Ray &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d, float &d2) const;
	float Distance(const Line &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d, float &d2) const;
	float Distance(const LineSegment &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d, float &d2) const;
	float Distance(const Plane &other) const;
	float Distance(const Sphere &other) const;
	float Distance(const Capsule &other) const;

	float DistanceSq(const vec &point) const;
	float DistanceSq(const LineSegment &other) const;
	double DistanceSqD(const vec &point) const;

	/// Tests whether this line segment and the given object intersect.	
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (for example, if this line segment is contained inside a sphere)
		@todo Output intersection point. */
	bool Intersects(const Plane &plane) const;
	/** @param d [out] If specified, this parameter receives the normalized distance along
			this line segment which specifies the closest point on this line segment to
			the specified point. This pointer may be null. */
	bool Intersects(const Plane &plane, float *d) const;
	/** @param intersectionPoint [out] If specified, receives the point of intersection. This pointer may be null. */
	bool Intersects(const Triangle &triangle, float *d, vec *intersectionPoint) const;
	/** @param intersectionNormal [out] If specified, receives the normal vector of the other object at the point of intersection.
			This pointer may be null. */
	bool Intersects(const Sphere &s, vec *intersectionPoint = 0, vec *intersectionNormal = 0, float *d = 0) const;
	/** @param dNear [out] If specified, receives the parametric distance along this line segment denoting where the line entered the
			bounding box object.
		@param dFar [out] If specified, receives the parametric distance along this line segment denoting where the line exited the
			bounding box object. */
	bool Intersects(const AABB &aabb, float &dNear, float &dFar) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb, float &dNear, float &dFar) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	/** @param epsilon If testing intersection between two line segments, a distance threshold value is used to account
			for floating-point inaccuracies. */
	bool Intersects(const LineSegment &lineSegment, float epsilon = 1e-3f) const;
	/// Tests if this line segment intersects the given disc.
	/// @todo This signature will be moved to bool Intersects(const Disc &disc) const;
	bool IntersectsDisc(const Circle &disc) const;

	/// Converts this LineSegment to a Ray.
	/** The pos member of the returned Ray will be equal to a, and the dir member equal to Dir().
		@see class Ray, ToLine(). */
	Ray ToRay() const;
	/// Converts this LineSegment to a Line.
	/** The pos member of the returned Line will be equal to a, and the dir member equal to Dir().
		@see class Line, ToRay(). */
	Line ToLine() const;

	/// Projects this LineSegment onto the given 1D axis direction vector.
	/** This function collapses this LineSegment onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this LineSegment. Most useful for debugging purposes.
	StringT ToString() const;
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static LineSegment FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	static LineSegment FromString(const char *str, const char **outEndStr = 0);

#ifdef MATH_GRAPHICSENGINE_INTEROP
	void ToLineList(VertexBuffer &vb) const;
#endif
};

struct LineSegment_storage
{
	vec_storage a,b;
	LineSegment_storage(){}
	LineSegment_storage(const LineSegment &rhs)
	{
		*reinterpret_cast<LineSegment*>(this) = rhs;
	}
	operator LineSegment() const { return *reinterpret_cast<const LineSegment*>(this); }
};

LineSegment operator *(const float3x3 &transform, const LineSegment &line);
LineSegment operator *(const float3x4 &transform, const LineSegment &line);
LineSegment operator *(const float4x4 &transform, const LineSegment &line);
LineSegment operator *(const Quat &transform, const LineSegment &line);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const LineSegment &lineSegment);
#endif

MATH_END_NAMESPACE
