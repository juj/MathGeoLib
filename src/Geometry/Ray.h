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

/** @file Ray.h
	@author Jukka Jylänki
	@brief The Ray geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

#ifdef MATH_OGRE_INTEROP
#include <OgreRay.h>
#endif
#ifdef MATH_URHO3D_INTEROP
#include <Urho3D/Math/Ray.h>
#endif

MATH_BEGIN_NAMESPACE

/// A ray in 3D space is a line that starts from an origin point and extends to infinity in one direction.
class Ray
{
public:
	/// Specifies the origin of this ray.
	vec pos;

	/// The normalized direction vector of this ray. [similarOverload: pos]
	/** @note For proper functionality, this direction vector needs to always be normalized. If you set to this
		member manually, remember to make sure you only assign normalized direction vectors. */
	vec dir;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos and dir are undefined after creating a new Ray using this
		default constructor. Remember to assign to them before use.
		@see pos, dir. */
	Ray() {}

	/// Constructs a new ray by explicitly specifying the member variables.
	/** @param pos The origin position of the ray.
		@param dir The direction of the ray. This vector must be normalized, this function will not normalize
			the vector for you (for performance reasons).
		@see pos, dir. */
	Ray(const vec &pos, const vec &dir);

	/// Converts a Line to a Ray.
	/** This conversion simply copies the members pos and dir over from the given Line to this Ray.
		This means that the new Ray starts at the same position, but only extends to one direction in space,
		instead of two.
		@see class Line, ToLine(). */
	explicit Ray(const Line &line);

	/// Converts a LineSegment to a Ray.
	/** This constructor sets pos = lineSegment.a, and dir = (lineSegment.b - lineSegment.a).Normalized().
		@see class LineSegment, ToLineSegment(). */
	explicit Ray(const LineSegment &lineSegment);

	bool IsFinite() const;

	/// Gets a point along the ray at the given distance.
	/** Use this function to convert a 1D parametric point along the Ray to a 3D point in the linear space.
		@param distance The point to compute. GetPoint(0) will return pos. GetPoint(t) will return a point
			at distance |t| from pos. Passing in negative values is allowed, but in that case, the
			returned point does not actually lie on this Ray.
		@return pos + distance * dir.
		@see pos, dir. */
	vec GetPoint(float distance) const;

	/// Translates this Ray in world space.
	/** @param offset The amount of displacement to apply to this Ray, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this Ray, in-place.
	/** See Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Tests if the given object is fully contained on this ray.
	/** @param distanceThreshold The magnitude of the epsilon test threshold to use. Since a Ray
		is a 1D object in a 3D space, an epsilon threshold is used to allow errors caused by floating-point
		inaccuracies.
		@return True if this ray contains the given object, up to the given distance threshold.
		@see class LineSegment, Distance(), ClosestPoint(), Intersects(). */
	bool Contains(const vec &point, float distanceThreshold = 1e-3f) const;
	bool Contains(const LineSegment &lineSegment, float distanceThreshold = 1e-3f) const;

	/// Tests if two rays are equal.
	/** @return True if this and the given Ray represent the same set of points, up to the given epsilon. */
	bool Equals(const Ray &otherRay, float epsilon = 1e-3f) const;

	/// Compares whether this Ray and the given Ray are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Ray &other) const { return pos.BitEquals(other.pos) && dir.BitEquals(other.dir); }

	/// Computes the distance between this ray and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@param d [out] If specified, receives the parametric distance along this ray that
			specifies the closest point on this ray to the given object. The value returned here can be negative.
		@see Contains(), Intersects(), ClosestPoint(), GetPoint(). */
	float Distance(const vec &point) const { float d; return Distance(point, d); }
	float Distance(const vec &point, float &d) const;

	/** @param d2 [out] If specified, receives the parametric distance along the other line that specifies the
		closest point on that line to this ray. The value returned here can be negative. */
	float Distance(const Ray &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Ray &other, float &d, float &d2) const;
	float Distance(const Line &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const Line &other, float &d, float &d2) const;
	float Distance(const LineSegment &other) const { float d, d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d) const { float d2; return Distance(other, d, d2); }
	float Distance(const LineSegment &other, float &d, float &d2) const;
	float Distance(const Sphere &sphere) const;
	float Distance(const Capsule &capsule) const;

	/// Computes the closest point on this ray to the given object.
	/** If the other object intersects this ray, this function will return an arbitrary point inside
		the region of intersection.
		@param d [out] If specified, receives the parametric distance along this ray that
			specifies the closest point on this ray to the given object. The value returned here can be negative.
		@see Contains(), Distance(), Intersects(), GetPoint(). */
	vec ClosestPoint(const vec &targetPoint) const { float d; return ClosestPoint(targetPoint, d); }
	vec ClosestPoint(const vec &targetPoint, float &d) const;
	/** @param d2 [out] If specified, receives the parametric distance along the other line that specifies the
		closest point on that line to this ray. The value returned here can be negative. */
	vec ClosestPoint(const Ray &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Ray &other, float &d, float &d2) const;
	vec ClosestPoint(const Line &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const Line &other, float &d, float &d2) const;
	vec ClosestPoint(const LineSegment &other) const { float d, d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d) const { float d2; return ClosestPoint(other, d, d2); }
	vec ClosestPoint(const LineSegment &other, float &d, float &d2) const;

	/// Tests whether this ray and the given object intersect.	
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
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Plane &plane, float *d) const;
	bool Intersects(const Plane &plane) const;
	/** @param intersectionNormal [out] If specified, receives the surface normal of the other object at
		the point of intersection. This pointer may be null. */
	bool Intersects(const Sphere &s, vec *intersectionPoint, vec *intersectionNormal, float *d) const;
	bool Intersects(const Sphere &s) const;
	/** @param dNear [out] If specified, receives the distance along this ray to where the ray enters
		the bounding box.
		@param dFar [out] If specified, receives the distance along this ray to where the ray exits
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

	/// Converts this Ray to a Line.
	/** The pos and dir members of the returned Line will be equal to this Ray. The only difference is
		that a Line extends to infinity in two directions, whereas the Ray spans only in the positive
		direction.
		@see dir, Ray::Ray, class Line, ToLineSegment(). */
	Line ToLine() const;
	/// Converts this Ray to a LineSegment.
	/** @param d Specifies the position of the other endpoint along this Ray. This parameter may be negative,
		in which case the returned LineSegment does not lie inside this Ray.
		@return A LineSegment with point a at pos, and point b at pos + d * dir.
		@see pos, dir, Ray::Ray, class LineSegment, ToLine(). */
	LineSegment ToLineSegment(float d) const;

	/// Projects this Ray onto the given 1D axis direction vector.
	/** This function collapses this Ray onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	/// Converts this Ray to a LineSegment.
	/** @param dStart Specifies the position of the first endpoint along this Ray. This parameter may be negative,
		in which case the starting point lies outside this Ray to the opposite direction of the Ray.
		@param dEnd Specifies the position of the second endpoint along this Ray. This parameter may also be negative.
		@return A LineSegment with point a at pos + dStart * dir, and point b at pos + dEnd * dir.
		@see pos, dir, Ray::Ray, class LineSegment, ToLine(). */
	LineSegment ToLineSegment(float dStart, float dEnd) const;

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this Ray.
	/** The returned string specifies the position and direction of this Ray. */
	std::string ToString() const;
	std::string SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	std::string SerializeToCodeString() const;
#endif

	static Ray FromString(const char *str, const char **outEndStr = 0);
#ifdef MATH_ENABLE_STL_SUPPORT
	static Ray FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif
#ifdef MATH_OGRE_INTEROP
	Ray(const Ogre::Ray &other):pos(other.getOrigin()), dir(other.getDirection()) {}
	operator Ogre::Ray() const { return Ogre::Ray(pos, dir); }
#endif
#ifdef MATH_URHO3D_INTEROP
	Ray(const Urho3D::Ray &other) : pos(other.origin_), dir(other.direction_) {}
	operator Urho3D::Ray() const { return Urho3D::Ray(pos, dir); }
#endif
};

/// @note Assumes that transform may contain scaling, and re-normalizes the ray direction
///		after the transform.
Ray operator *(const float3x3 &transform, const Ray &ray);
/// @note Assumes that transform may contain scaling, and re-normalizes the ray direction
///		after the transform.
Ray operator *(const float3x4 &transform, const Ray &ray);
/// @note Assumes that transform may contain scaling, and re-normalizes the ray direction
///		after the transform.
Ray operator *(const float4x4 &transform, const Ray &ray);
Ray operator *(const Quat &transform, const Ray &ray);

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Ray)
Q_DECLARE_METATYPE(Ray*)
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Ray &ray);
#endif

MATH_END_NAMESPACE
