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

/** @file Line.h
	@author Jukka Jylänki
	@brief Implementation for the Line geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A line in 3D space is defined by an origin point and a direction, and extends to infinity in two directions.
class Line
{
public:
	/// Specifies the origin of this line.
	vec pos;

	/// The normalized direction vector of this ray. [similarOverload: pos]
	/** @note For proper functionality, this direction vector needs to always be normalized. If you set to this
		member manually, remember to make sure you only assign normalized direction vectors. */
	vec dir;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos and dir are undefined after creating a new Line using this
		default constructor. Remember to assign to them before use.
		@see pos, dir. */
	Line() {}

	/// Constructs a new line by explicitly specifying the member variables.
	/** @param pos The origin position of the line.
		@param dir The direction of the line. This vector must be normalized, this function will not normalize
			the vector for you (for performance reasons).
		@see pos, dir. */
	Line(const vec &pos, const vec &dir);

	/// Converts a Ray to a Line.
	/** This conversion simply copies the members pos and dir over from the given Ray to this Line.
		This means that the new Line starts at the same position, but extends to two directions in space,
		instead of one.
		@see class Ray, ToRay(). */
	explicit Line(const Ray &ray);

	/// Converts a LineSegment to a Line.
	/** This constructor sets pos = lineSegment.a, and dir = (lineSegment.b - lineSegment.a).Normalized().
		@see class LineSegment, ToLineSegment(). */
	explicit Line(const LineSegment &lineSegment);

	bool IsFinite() const;

	/// Gets a point along the line at the given distance.
	/** Use this function to convert a 1D parametric point along the Line to a 3D point in the linear space.
		@param distance The point to compute. GetPoint(0) will return pos. GetPoint(t) will return a point
			at distance |t| from pos, towards the direction specified by dir. If a negative value is specified,
			a point towards the direction -dir is returned.
		@return pos + distance * dir.
		@see pos, dir. */
	vec GetPoint(float distance) const;

	/// Translates this Line in world space.
	/** @param offset The amount of displacement to apply to this Line, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this line, in-place.
	/** @see Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Tests if the given object is fully contained on this line.
	/** @param distanceThreshold The magnitude of the epsilon test threshold to use. Since a Line
		is a 1D object in a 3D space, an epsilon threshold is used to allow errors caused by floating-point
		inaccuracies.
		@return True if this line contains the given object, up to the given distance threshold.
		@see class LineSegment, class Ray, Distance(), ClosestPoint(), Intersects(). */
	bool Contains(const vec &point, float distanceThreshold = 1e-3f) const;
	bool Contains(const Ray &ray, float distanceThreshold = 1e-3f) const;
	bool Contains(const LineSegment &lineSegment, float distanceThreshold = 1e-3f) const;

	/// Tests if two lines are equal.
	/** This function tests for set equality (not just member value equality). This means that the pos and dir parameters
		of either line can be completely different, as long as the set of points on the both lines are equal.
		@return True if this and the given Line represent the same set of points, up to the given epsilon. */
	bool Equals(const Line &line, float epsilon = 1e-3f) const;

	/// Compares whether this Line and the given Line are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Line &other) const { return pos.BitEquals(other.pos) && dir.BitEquals(other.dir); }

	/// Computes the distance between this line and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@param d [out] If specified, receives the parametric distance along this line that
			specifies the closest point on this line to the given object. The value returned here can be negative.
		@see Contains(), Intersects(), ClosestPoint(), GetPoint(). */
	float Distance(const vec &point) const { float d; return Distance(point, d); }
	float Distance(const vec &point, float &d) const;
	/** @param d2 [out] If specified, receives the parametric distance along the other line that specifies the
		closest point on that line to this line. The value returned here can be negative. */
	float Distance(const Ray &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d, float &d2) const;
	float Distance(const Line &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d, float &d2) const;
	float Distance(const LineSegment &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d, float &d2) const;
	float Distance(const Sphere &other) const;
	float Distance(const Capsule &other) const;

	/// Computes the closest point on this line to the given object.
	/** If the other object intersects this line, this function will return an arbitrary point inside
		the region of intersection.
		@param d [out] If specified, receives the parametric distance along this line that
			specifies the closest point on this line to the given object. The value returned here can be negative.
		@see Contains(), Distance(), Intersects(), GetPoint(). */
	vec ClosestPoint(const vec &targetPoint) const { float d; return ClosestPoint(targetPoint, d); }
	vec ClosestPoint(const vec &targetPoint, float &d) const;
	/** @param d2 [out] If specified, receives the parametric distance along the other line that specifies the
		closest point on that line to this line. The value returned here can be negative. */
	vec ClosestPoint(const Ray &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d, float &d2) const;
	vec ClosestPoint(const Line &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d, float &d2) const;
	vec ClosestPoint(const LineSegment &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d, float &d2) const;
	/** @param outBarycentricUV [out] If specified, receives the barycentric UV coordinates (in two-coordinate barycentric UV convention)
			representing the closest point on the triangle to this line.
		@see Contains(), Distance(), Intersects(), GetPoint(), Triangle::Point(float u, float v). */
	vec ClosestPoint(const Triangle &triangle) const { float d; return ClosestPoint(triangle, d); }
	vec ClosestPoint(const Triangle &triangle, float &d) const;
	vec ClosestPoint(const Triangle &triangle, float &d, float2 &outBarycentricUV) const;

	/// Tests whether this line and the given object intersect.	
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true.
		@param d [out] If specified, this parameter will receive the parametric distance of
			the intersection point along this object. Use the GetPoint(d) function
			to get the actual point of intersection. This pointer may be null.
		@param intersectionPoint [out] If specified, receives the actual point of intersection. This pointer
			may be null.
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@see Contains(), Distance(), ClosestPoint(), GetPoint(). */
	bool Intersects(const Triangle &triangle, float *d, vec *intersectionPoint) const;
	bool Intersects(const Plane &plane, float *d) const;
	/** @param intersectionNormal [out] If specified, receives the surface normal of the other object at
		the point of intersection. This pointer may be null. */
	bool Intersects(const Sphere &s, vec *intersectionPoint = 0, vec *intersectionNormal = 0, float *d = 0) const;
	/** @param dNear [out] If specified, receives the distance along this line to where the line enters
		the bounding box.
		@param dFar [out] If specified, receives the distance along this line to where the line exits
		the bounding box. */
	bool Intersects(const AABB &aabb, float &dNear, float &dFar) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb, float &dNear, float &dFar) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	/// Tests if this ray intersects the given disc.
	/// @todo This signature will be moved to bool Intersects(const Disc &disc) const;
	bool IntersectsDisc(const Circle &disc) const;

	/// Converts this Line to a Ray.
	/** The pos and dir members of the returned Ray will be equal to this Line. The only difference is
		that a Line extends to infinity in two directions, whereas the returned Ray spans only in
		the positive direction.
		@see dir, Line::Line, class Ray, ToLineSegment(). */
	Ray ToRay() const;

	/// Converts this Line to a LineSegment.
	/** @param d Specifies the position of the other endpoint along this Line. This parameter may be negative.
		@return A LineSegment with point a at pos, and point b at pos + d * dir.
		@see pos, dir, Line::Line, class LineSegment, ToRay(). */
	LineSegment ToLineSegment(float d) const;

	/// Converts this Line to a LineSegment.
	/** @param dStart Specifies the position of the first endpoint along this Line. This parameter may be negative,
		in which case the starting point lies to the opposite direction of the Line.
		@param dEnd Specifies the position of the second endpoint along this Line. This parameter may also be negative.
		@return A LineSegment with point a at pos + dStart * dir, and point b at pos + dEnd * dir.
		@see pos, dir, Line::Line, class LineSegment, ToLine(). */
	LineSegment ToLineSegment(float dStart, float dEnd) const;

	/// Projects this Line onto the given 1D axis direction vector.
	/** This function collapses this Line onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	/// Tests if the given three points are collinear.
	/** This function tests whether the given three functions all lie on the same line.
		@param epsilon The comparison threshold to use to account for floating-point inaccuracies. */
	static bool AreCollinear(const vec &p1, const vec &p2, const vec &p3, float epsilon = 1e-3f);

	static void ClosestPointLineLine(const vec &start0, const vec &dir0, const vec &start1, const vec &dir1, float &d, float &d2);

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this Line.
	/** The returned string specifies the position and direction of this Line. */
	StringT ToString() const;
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static Line FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	static Line FromString(const char *str, const char **outEndStr = 0);
};

Line operator *(const float3x3 &transform, const Line &line);
Line operator *(const float3x4 &transform, const Line &line);
Line operator *(const float4x4 &transform, const Line &line);
Line operator *(const Quat &transform, const Line &line);

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Line &line);
#endif

MATH_END_NAMESPACE
