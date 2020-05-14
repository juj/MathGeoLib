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

/** @file Triangle2D.h
	@author Jukka Jylänki
	@brief The Triangle2D geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/vec2d.h"

MATH_BEGIN_NAMESPACE

/// Specifies a triangle through three points in 3D space.
/** This class stores three member vertices a, b and c to specify the triangle. */
class Triangle2D
{
public:
	/// The first triangle endpoint.
	vec2d a;
	/// The second triangle endpoint.
	/** [similarOverload: a] */
	vec2d b;
	/// The third triangle endpoint.
	/** [similarOverload: a]
		@note The order in which the vertices are stored in this data structure is important. The triangles
			(a,b,c) and (a,c,b) are otherwise equivalent, but their plane normals point to the opposite directions.
		@see PlaneCCW(), PlaneCW(). */
	vec2d c;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members a, b and c are undefined after creating a new Triangle2D using this
		default constructor. Remember to assign to them before use.
		@see a, b, c. */
	Triangle2D() {}

	/// Constructs a triangle from three given endpoints.
	/** The normal of the plane will be constructed to point towards the halfspace where
		the vertices a, b and c wind in counter-clockwise order.
		@see a, b, c. */
	Triangle2D(const vec2d &a, const vec2d &b, const vec2d &c);

	FORCE_INLINE static int NumFaces() { return 1; }
	FORCE_INLINE static int NumEdges() { return 3; }
	FORCE_INLINE static int NumVertices() { return 3; }

	/// Translates this Triangle2D in world space.
	/** @param offset The amount of displacement to apply to this Triangle2D, in world space coordinates.
		@see Transform(). */
	void Translate(const vec2d &offset);

	/// Applies a transformation to this Triangle2D, in-place.
	/** See Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);

	/// Expresses the given point in barycentric (u,v,w) coordinates.
	/** @note There are two different conventions for representing barycentric coordinates. One uses
			a (u,v,w) triplet with the equation pt == u*a + v*b + w*c, and the other uses a (u,v) pair
			with the equation pt == a + u*(b-a) + v*(c-a). These two are equivalent. Use the mappings
			(u,v) -> (1-u-v, u, v) and (u,v,w)->(v,w) to convert between these two representations.
		@param point The point of the vector space to express in barycentric coordinates. This point should
			lie in the plane formed by this triangle.
		@return The factors (u,v,w) that satisfy the weighted sum equation point == u*a + v*b + w*c.
		@see BarycentricUV(), BarycentricInsideTriangle(), Point(), http://mathworld.wolfram.com/BarycentricCoordinates.html */
	float3 BarycentricUVW(const vec2d &point) const;

	/// Expresses the given point in barycentric (u,v) coordinates.
	/** @note There are two different conventions for representing barycentric coordinates. One uses
			a (u,v,w) triplet with the equation pt == u*a + v*b + w*c, and the other uses a (u,v) pair
			with the equation pt == a + u*(b-a) + v*(c-a). These two are equivalent. Use the mappings
			(u,v) -> (1-u-v, u, v) and (u,v,w)->(v,w) to convert between these two representations.
		@param point The point to express in barycentric coordinates. This point should lie in the plane
			formed by this triangle.
		@return The factors (u,v) that satisfy the weighted sum equation point == a + u*(b-a) + v*(c-a).
		@see BarycentricUVW(), BarycentricInsideTriangle(), Point(). */
	float2 BarycentricUV(const vec2d &point) const;

	/// Tests if the given barycentric UVW coordinates lie inside a triangle.
	/** A barycentric UVW coordinate represents a point inside a triangle if
		  a) 0 <= u,v,w <= 1 and
		  b) u+v+w == 1.
		@param uvw The barycentric vector containing the barycentric (u,v,w) coordinates.
		@return True if the given coordinates lie inside a triangle.
		@see BarycentricUV(), BarycentricUVW(), Point(). */
	static bool BarycentricInsideTriangle(const float3 &uvw);

	/// Returns the point at the given barycentric coordinates.
	/** This function computes the vector space point at the given barycentric coordinates.
		@param uvw The barycentric UVW coordinate triplet. The condition u+v+w == 1 should hold for the input coordinate.
			If 0 <= u,v,w <= 1, the returned point lies inside this triangle.
		@return u*a + v*b + w*c. */
	vec2d Point(const float3 &uvw) const;
	vec2d Point(float u, float v, float w) const;
	/** These functions are an alternate form of Point(u,v,w) for the case when the barycentric coordinates are
		represented as a (u,v) pair and not as a (u,v,w) triplet. This function is provided for convenience
		and effectively just computes Point(1-u-v, u, v).
		@param uv The barycentric UV coordinates. If 0 <= u,v <= 1 and u+v <= 1, then the returned point lies inside
			this triangle.
		@return a + (b-a)*u + (c-a)*v.
		@see BarycentricUV(), BarycentricUVW(), BarycentricInsideTriangle(). */
 	vec2d Point(const float2 &uv) const;
	vec2d Point(float u, float v) const;

	/// Computes the center of mass of this triangle.
	/** @return The point (a+b+c)/3. */
	vec2d Centroid() const;
	/// Identical to CenterPoint(), but provided to enable common signature with Triangle2D, AABB2D and OBB2D to allow them to be used
	/// in template classes.
	vec2d CenterPoint() const { return Centroid(); }

	/// Computes the surface area of this triangle.
	/** @return The surface area of this triangle.
		@see Perimeter(), Area2D(), SignedArea(). */
	float Area() const;

	/// Computes the total edge length of this triangle.
	/** @return |a-b| + |b-c| + |c-a|.
		@see Area(), Edge(). */
	float Perimeter() const;

	/// Returns a pointer to the vertices of this triangle. The array contains three elements.
	vec2d *VertexArrayPtr() { return &a; }
	const vec2d *VertexArrayPtr() const { return &a; }

	/// Returns a vertex of this triangle.
	/** @param i The vertex of this triangle to get: 0, 1 or 2.
		@return Vertex(0) returns the point a, Vertex(1) returns the point b, and Vertex(2) returns the point c.
		@note If an index outside [0, 2] is passed, an assume() failure occurs and a NaN vector is returned.
		@see Edge(). */
	vec2d Vertex(int i) const;
	vec2d CornerPoint(int i) const { return Vertex(i); } // An alias for Vertex() to be able to mash Triangle2D into templatized algorithms.

	/// Returns an edge of this triangle.
	/** @param i The index of the edge to generate: 0, 1 or 2.
		@return A LineSegment2D representing the given edge of this triangle. Edge(0) returns LineSegment2D(a,b), Edge(1)
			returns LineSegment2D(b,c) and Edge(2) returns LineSegment2D(c,a).
		@note If an index outside [0, 2] is passed, an assume() failure occurs and LineSegment2D(NaN, NaN) is returned.
		@see Vertex(). */
	LineSegment2D Edge(int i) const;

	/// Quickly returns an arbitrary point inside this Triangle2D. Used in GJK intersection test.
	inline vec2d AnyPointFast() const { return a; }

	/// Computes an extreme point of this Triangle2D in the given direction.
	/** An extreme point is a farthest point of this Triangle2D in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Triangle2D in the given direction. The returned point is always a
			vertex of this Triangle2D.
		@see Vertex(). */
	vec2d ExtremePoint(const vec2d &direction) const;
	vec2d ExtremePoint(const vec2d &direction, float &projectionDistance) const;

#if 0
	/// Returns a Polygon2D representation of this Triangle2D.
	/** The returned polygon is identical to this Triangle2D. It has three vertices a, b and c which wind in the same
		direction than in this triangle.
	@see class Polygon2D, ToPolyhedron(). */
	Polygon2D ToPolygon() const;
#endif
	/// Returns the tight AABB2D that encloses this Triangle2D.
	AABB2D BoundingAABB() const;

	/// Computes the surface area of the given 2D triangle.
	/** This math library does not have a separate class for 2D triangles. To compute the area of a 2D triangle,
		use this Triangle2D class and set z=0 for each coordinate, or use this helper function.
		@see Area(), SignedArea(). */
	static float Area2D(const float2 &p1, const float2 &p2, const float2 &p3);

	/// Relates the barycentric coordinate of the given point to the surface area of triangle abc.
	/** This function computes the ratio of the signed area of the triangle (point, b, c) to the signed area of
		the triangle (a, b, c). This is the same as computing the barycentric u-coordinate of the given point
		on the triangle (a, b, c).
		@see Area(), Area2D(), BarycentricUVW(). */
	static float SignedArea(const vec2d &point, const vec2d &a, const vec2d &b, const vec2d &c);

	/// Tests if this Triangle2D is finite.
	/** A triangle is <b><i>finite</i></b> if its vertices a, b and c do not contain floating-point NaNs or +/-infs
		in them.
		@return True if each coordinate of each vertex of this triangle has a finite floating-point value.
		@see a, b, c, IsDegenerate(), ::IsFinite(), IsInf(), IsNan(), IsFinite(), inf, negInf, nan, float3::nan, float3::inf. */
	bool IsFinite() const;

	/// Returns true if this triangle is degenerate.
	/** A triangle is <b><i>degenerate</i></b> if it is not finite, or if the surface area of the triangle is
		close to zero.
		@param epsilon The threshold to test against. If two vertices of this triangle are closer than this, the
		triangle is considered degenerate.
		@see a, b, c, IsFinite(). */
	bool IsDegenerate(float epsilon = 1e-3f) const;

	/// Returns true if the triangle defined by the three given points is degenerate.
	static bool IsDegenerate(const vec2d &p1, const vec2d &p2, const vec2d &p3, float epsilon = 1e-3f);

	/// In some templated algorithms, the input can either be a Triangle2D or a Polygon2D. Provide trivial Polygon2D-specific API
	/// for compatibility in those template functions.
	bool IsConvex() const { return true; }
	bool IsPlanar() const { return true; }
	bool IsSimple() const { return true; }

	/// Tests if the given object is fully contained inside this triangle.
	/** @param triangleThickness An epsilon threshold value to use for this test. triangleThicknessSq is the squared version of this parameter.
			This specifies the maximum distance the given object can lie from the plane defined by this triangle.
		@see Distance(), Intersects(), ClosestPoint().
		@todo Add Triangle2D::Contains(Circle) and Triangle2D::Contains(Disc). */
	bool Contains(const vec2d &point, float triangleThicknessSq = 1e-5f) const;
#if 0
	bool Contains(const LineSegment2D &lineSegment, float triangleThickness = 1e-3f) const;
	bool Contains(const Triangle2D &triangle, float triangleThickness = 1e-3f) const;
#endif

	/// Computes the distance between this triangle and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@todo Add Triangle2D::Distance(Line2D/Ray2D/LineSegment2D/Plane/Triangle2D/Polygon2D/Circle/Disc/AABB2D/OBB2D/Capsule2D/Frustum/Polyhedron).
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const vec2d &point) const;
#if 0
	float Distance(const Sphere2D &sphere) const;
	float Distance(const Capsule2D &capsule) const;
#endif

	float DistanceSq(const vec2d &point) const;

#if 0
	/// Tests whether this triangle and the given object intersect.	
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this triangle,
		or this triangle is contained inside a sphere, etc.)
		@param d [out] If specified, this parameter will receive the parametric distance of
			the intersection point along the line object. Use the GetPoint(d) function of the line class
			to get the actual point of intersection. This pointer may be null.
		@param intersectionPoint [out] If specified, receives the actual point of intersection. This pointer
			may be null.
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@see Contains(), Distance(), ClosestPoint(), LineSegment2D::GetPoint(). */
	bool Intersects(const LineSegment2D &lineSegment, float *d = 0, vec2d *intersectionPoint = 0) const;
	bool Intersects(const Line2D &line, float *d = 0, vec2d *intersectionPoint = 0) const;
	bool Intersects(const Ray2D &ray, float *d = 0, vec2d *intersectionPoint = 0) const;
	/** @param closestPointOnTriangle [out] If specified, receives the point of intersection between the Sphere2D
			and this Triangle2D. Even if no intersection occurred, this parameter will receive the closest point on
			the Triangle2D to the Sphere2D. This pointer may be null. */
	bool Intersects(const Sphere2D &sphere, vec2d *closestPointOnTriangle) const;
	bool Intersects(const Sphere2D &sphere) const;
	/** @param outLine [out] If specified, receives the line segment of the common points shared by the two
			intersecting triangles. If the two triangles do not intersect, this pointer is not written to.
			This pointer may be null. */
	bool Intersects(const Triangle2D &triangle, LineSegment2D *outLine = 0) const;
	bool Intersects(const AABB2D &aabb) const;
	bool Intersects(const OBB2D &obb) const;
	bool Intersects(const Polygon2D &polygon) const;
	bool Intersects(const Capsule2D &capsule) const;
#endif

	/// A helper function used in line-triangle tests.
	static float IntersectLineTri(const vec2d &linePos, const vec2d &lineDir,
		const vec2d &v0, const vec2d &v1, const vec2d &v2,
		float &u, float &v);

	/// Projects this Triangle2D onto the given axis.
	/** This function is used in SAT tests (separate axis theorem) to check the interval this triangle
		lies in on an 1D line.
		@param axis The axis to project onto. This vector can be unnormalized.
		@param dMin [out] Returns the minimum extent of this triangle on the given axis.
		@param dMax [out] Returns the maximum extent of this triangle on the given axis. */
	void ProjectToAxis(const vec2d &axis, float &dMin, float &dMax) const;

	int UniqueFaceNormals(vec2d *out) const;
	int UniqueEdgeDirections(vec2d *out) const;

	/// Computes the closest point on this triangle to the given object.
	/** If the other object intersects this triangle, this function will return an arbitrary point inside
		the region of intersection.
		@see Contains(), Distance(), Intersects(), ClosestPointToTriangleEdge(). */
	vec2d ClosestPoint(const vec2d &point) const;
	/** @param otherPt [out] If specified, receives the closest point on the other object to this triangle.
		This pointer may be null. */
	vec2d ClosestPoint(const LineSegment2D &lineSegment, vec2d *otherPt = 0) const;
	/** @param outU [out] If specified, receives the barycentric U coordinate of the returned point (in the UV convention).
			This pointer may be null. TODO Add this parameter back.
		@param outV [out] If specified, receives the barycentric V coordinate of the returned point (in the UV convention).
			This pointer may be null. TODO Add this parameter back.
		@param outD [out] If specified, receives the distance along the line of the closest point on the line to this triangle. TODO Add this parameter back.
		@return The closest point on this triangle to the given object.
		@todo Add ClosestPoint(Ray2D/Plane/Polygon2D/Circle/Disk/AABB2D/OBB2D/Sphere2D/Capsule2D/Frustum/Polyhedron).
		@see Distance(), Contains(), Intersects(), ClosestPointToTriangleEdge(), Line2D::GetPoint. */
#if 0
	vec2d ClosestPoint(const Line2D &line, vec2d *otherPt = 0) const;
	vec2d ClosestPoint(const Triangle2D &triangle, vec2d *otherPt = 0) const;
#endif

	/// Computes the closest point on the edge of this triangle to the given object.
	/** @param outU [out] If specified, receives the barycentric U coordinate of the returned point (in the UV convention).
			This pointer may be null.
		@param outV [out] If specified, receives the barycentric V coordinate of the returned point (in the UV convention).
			This pointer may be null.
		@param outD [out] If specified, receives the distance along the line of the closest point on the line to the edge of this triangle.
		@return The closest point on the edge of this triangle to the given object.
		@todo Add ClosestPointToTriangleEdge(Point/Ray2D/Triangle2D/Plane/Polygon2D/Circle/Disk/AABB2D/OBB2D/Sphere2D/Capsule2D/Frustum/Polyhedron).
		@see Distance(), Contains(), Intersects(), ClosestPointToTriangleEdge(), Line2D::GetPoint. */
#if 0
	vec2d ClosestPointToTriangleEdge(const Line2D &line, float *outU, float *outV, float *outD) const;
	vec2d ClosestPointToTriangleEdge(const LineSegment2D &lineSegment, float *outU, float *outV, float *outD) const;
#endif

	/// Generates a random point inside this Triangle2D.
	/** The points are distributed uniformly.
		The implementation of this function is based on Graphics Gems 1, p. 25:
		"1.5 Generating random points in triangles. Method 2." The Method 1 presented in the book
		uses a Sqrt() instead of the if().
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointOnEdge(), RandomVertex(), Point(). */
	vec2d RandomPointInside(LCG &rng) const;

	/// Chooses a corner vertex of this Triangle2D at random.
	/** This function returns one of the vertices {a, b, c} at uniformly random.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), RandomPointOnEdge(), Vertex(). */
	vec2d RandomVertex(LCG &rng) const;

	/// Generates a random point on the edge of this Triangle2D.
	/** The points are distributed uniformly.
		This function requires that this triangle is not degenerate. If it is, an assume() error will be printed,
		and the return value will be undefined.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), RandomVertex(), Edge(), class LineSegment2D, IsDegenerate(). */
	vec2d RandomPointOnEdge(LCG &rng) const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this Line2D. Most useful for debugging purposes.
	StringT ToString() const;
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static Triangle2D FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	static Triangle2D FromString(const char *str, const char **outEndStr = 0);

	bool Equals(const Triangle2D &rhs, float epsilon = 1e-3f) const { return a.Equals(rhs.a, epsilon) && b.Equals(rhs.b, epsilon) && c.Equals(rhs.c, epsilon); }

	/// Compares whether this Triangle2D and the given Triangle2D are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Triangle2D &other) const { return a.BitEquals(other.a) && b.BitEquals(other.b) && c.BitEquals(other.c); }
};

Triangle2D operator *(const float3x3 &transform, const Triangle2D &t);
Triangle2D operator *(const float3x4 &transform, const Triangle2D &t);
Triangle2D operator *(const float4x4 &transform, const Triangle2D &t);

struct Triangle2D_storage
{
	vec2d_storage v0, v1, v2;
	Triangle2D_storage(const Triangle2D &rhs)
	{
		*reinterpret_cast<Triangle2D*>(this) = rhs;
	}
	operator Triangle2D() const { return *reinterpret_cast<const Triangle2D*>(this); }
};
#define TRIANGLE(x) (*(Triangle2D*)&x)

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Triangle2D &triangle);
#endif

bool LineSegment2DLineSegment2DIntersect(const float2 &p0, const float2 &dir0, const float2 &p1, const float2 &dir1, float &s, float &t);

MATH_END_NAMESPACE
