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

/** @file Triangle.h
	@author Jukka Jylänki
	@brief The Triangle geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// Specifies a triangle through three points in 3D space.
/** This class stores three member vertices a, b and c to specify the triangle. */
class Triangle
{
public:
	/// The first triangle endpoint.
	vec a;
	/// The second triangle endpoint.
	/** [similarOverload: a] */
	vec b;
	/// The third triangle endpoint.
	/** [similarOverload: a]
		@note The order in which the vertices are stored in this data structure is important. The triangles
			(a,b,c) and (a,c,b) are otherwise equivalent, but their plane normals point to the opposite directions.
		@see PlaneCCW(), PlaneCW(). */
	vec c;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members a, b and c are undefined after creating a new Triangle using this
		default constructor. Remember to assign to them before use.
		@see a, b, c. */
	Triangle() {}

	/// Constructs a triangle from three given endpoints.
	/** The normal of the plane will be constructed to point towards the halfspace where
		the vertices a, b and c wind in counter-clockwise order.
		@see a, b, c. */
	Triangle(const vec &a, const vec &b, const vec &c);

	FORCE_INLINE static int NumFaces() { return 1; }
	FORCE_INLINE static int NumEdges() { return 3; }
	FORCE_INLINE static int NumVertices() { return 3; }

	/// Translates this Triangle in world space.
	/** @param offset The amount of displacement to apply to this Triangle, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this Triangle, in-place.
	/** See Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Expresses the given point in barycentric (u,v,w) coordinates.
	/** @note There are two different conventions for representing barycentric coordinates. One uses
			a (u,v,w) triplet with the equation pt == u*a + v*b + w*c, and the other uses a (u,v) pair
			with the equation pt == a + u*(b-a) + v*(c-a). These two are equivalent. Use the mappings
			(u,v) -> (1-u-v, u, v) and (u,v,w)->(v,w) to convert between these two representations.
		@param point The point of the vector space to express in barycentric coordinates. This point should
			lie in the plane formed by this triangle.
		@return The factors (u,v,w) that satisfy the weighted sum equation point == u*a + v*b + w*c.
		@see BarycentricUV(), BarycentricInsideTriangle(), Point(), http://mathworld.wolfram.com/BarycentricCoordinates.html */
	float3 BarycentricUVW(const vec &point) const;

	/// Expresses the given point in barycentric (u,v) coordinates.
	/** @note There are two different conventions for representing barycentric coordinates. One uses
			a (u,v,w) triplet with the equation pt == u*a + v*b + w*c, and the other uses a (u,v) pair
			with the equation pt == a + u*(b-a) + v*(c-a). These two are equivalent. Use the mappings
			(u,v) -> (1-u-v, u, v) and (u,v,w)->(v,w) to convert between these two representations.
		@param point The point to express in barycentric coordinates. This point should lie in the plane
			formed by this triangle.
		@return The factors (u,v) that satisfy the weighted sum equation point == a + u*(b-a) + v*(c-a).
		@see BarycentricUVW(), BarycentricInsideTriangle(), Point(). */
	float2 BarycentricUV(const vec &point) const;

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
	vec Point(const float3 &uvw) const;
	vec Point(float u, float v, float w) const;
	/** These functions are an alternate form of Point(u,v,w) for the case when the barycentric coordinates are
		represented as a (u,v) pair and not as a (u,v,w) triplet. This function is provided for convenience
		and effectively just computes Point(1-u-v, u, v).
		@param uv The barycentric UV coordinates. If 0 <= u,v <= 1 and u+v <= 1, then the returned point lies inside
			this triangle.
		@return a + (b-a)*u + (c-a)*v.
		@see BarycentricUV(), BarycentricUVW(), BarycentricInsideTriangle(). */
 	vec Point(const float2 &uv) const;
	vec Point(float u, float v) const;

	/// Computes the center of mass of this triangle.
	/** @return The point (a+b+c)/3. */
	vec Centroid() const;
	/// Identical to CenterPoint(), but provided to enable common signature with Triangle, AABB and OBB to allow them to be used
	/// in template classes.
	vec CenterPoint() const { return Centroid(); }

	/// Computes the surface area of this triangle.
	/** @return The surface area of this triangle.
		@see Perimeter(), Area2D(), SignedArea(). */
	float Area() const;

	/// Computes the total edge length of this triangle.
	/** @return |a-b| + |b-c| + |c-a|.
		@see Area(), Edge(). */
	float Perimeter() const;

	/// Returns a pointer to the vertices of this triangle. The array contains three elements.
	vec *VertexArrayPtr() { return &a; }
	const vec *VertexArrayPtr() const { return &a; }

	/// Returns a vertex of this triangle.
	/** @param i The vertex of this triangle to get: 0, 1 or 2.
		@return Vertex(0) returns the point a, Vertex(1) returns the point b, and Vertex(2) returns the point c.
		@note If an index outside [0, 2] is passed, an assume() failure occurs and a NaN vector is returned.
		@see Edge(). */
	vec Vertex(int i) const;
	vec CornerPoint(int i) const { return Vertex(i); } // An alias for Vertex() to be able to mash Triangle into templatized algorithms.

	/// Returns an edge of this triangle.
	/** @param i The index of the edge to generate: 0, 1 or 2.
		@return A LineSegment representing the given edge of this triangle. Edge(0) returns LineSegment(a,b), Edge(1)
			returns LineSegment(b,c) and Edge(2) returns LineSegment(c,a).
		@note If an index outside [0, 2] is passed, an assume() failure occurs and LineSegment(NaN, NaN) is returned.
		@see Vertex(). */
	LineSegment Edge(int i) const;

	/// Returns the counterclockwise-oriented plane this triangle lies on.
	/** The normal of the returned plane points towards the halfspace in which the vertices of this triangle are winded
		in <b>counter-clockwise</b> direction. */
	Plane PlaneCCW() const;

	/// Returns the clockwise-oriented plane this triangle lies on.
	/** The normal of the returned plane points towards the halfspace in which the vertices of this triangle are winded
		in <b>clockwise</b> direction. [similarOverload: PlaneCCW]
		@see NormalCCW(), NormalCW(), UnnormalizedNormalCCW(), UnnormalizedNormalCW(). */
	Plane PlaneCW() const;

	/// Returns the normalized triangle normal pointing to the counter-clockwise direction of this triangle.
	/** This function computes the normalized triangle normal vector that points towards the halfspace,
		from which observed, the vertices of this triangle wind in <b>counter-clockwise</b> (CCW) order.
		@see PlaneCCW(), PlaneCW(), UnnormalizedNormalCCW(), UnnormalizedNormalCW(). */
	vec NormalCCW() const;

	/// Returns the normalized triangle normal pointing to the clockwise direction of this triangle.
	/** This function computes the normalized triangle normal vector that points towards the halfspace,
		from which observed, the vertices of this triangle wind in <b>clockwise</b> (CW) order. [similarOverload: NormalCCW]
		@see PlaneCCW(), PlaneCW(), UnnormalizedNormalCCW(), UnnormalizedNormalCW(). */
	vec NormalCW() const;

	/// Computes an unnormalized counter-clockwise oriented triangle normal vector.
	vec UnnormalizedNormalCCW() const;
	/// Computes an unnormalized clockwise-oriented triangle normal vector.
	/** These functions are equivalent to NormalCCW() and NormalCW(), except these functions do not produce
		a unit-length triangle normal vectors. Use these functions instead of NormalCCW/CW() to obtain a small
		speed benefit in cases where the normalization step is not required. [similarOverload: UnnormalizedNormalCCW]
		@see PlaneCCW(), PlaneCW(), NormalCCW(), NormalCW(). */
	vec UnnormalizedNormalCW() const;

	/// Quickly returns an arbitrary point inside this Triangle. Used in GJK intersection test.
	inline vec AnyPointFast() const { return a; }

	/// Computes an extreme point of this Triangle in the given direction.
	/** An extreme point is a farthest point of this Triangle in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Triangle in the given direction. The returned point is always a
			vertex of this Triangle.
		@see Vertex(). */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Returns a Polygon representation of this Triangle.
	/** The returned polygon is identical to this Triangle. It has three vertices a, b and c which wind in the same
		direction than in this triangle.
	@see class Polygon, ToPolyhedron(). */
	Polygon ToPolygon() const;

	/// Returns a Polyhedron representation of this Triangle.
	/** The generated polyhedron will be closed and has two triangular faces, and three vertices (a, b and c).
		The two faces share the same vertices, but in opposite winding orders. This creates a polyhedron with zero
		volume and the surface area twice of this Triangle.
		@see class Polyhedron, ToPolygon(). */
	Polyhedron ToPolyhedron() const;

	/// Returns the tight AABB that encloses this Triangle.
	AABB BoundingAABB() const;

	/// Computes the surface area of the given 2D triangle.
	/** This math library does not have a separate class for 2D triangles. To compute the area of a 2D triangle,
		use this Triangle class and set z=0 for each coordinate, or use this helper function.
		@see Area(), SignedArea(). */
	static float Area2D(const float2 &p1, const float2 &p2, const float2 &p3);

	/// Relates the barycentric coordinate of the given point to the surface area of triangle abc.
	/** This function computes the ratio of the signed area of the triangle (point, b, c) to the signed area of
		the triangle (a, b, c). This is the same as computing the barycentric u-coordinate of the given point
		on the triangle (a, b, c).
		@see Area(), Area2D(), BarycentricUVW(). */
	static float SignedArea(const vec &point, const vec &a, const vec &b, const vec &c);

	/// Tests if this Triangle is finite.
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
	static bool IsDegenerate(const vec &p1, const vec &p2, const vec &p3, float epsilon = 1e-3f);

	/// In some templated algorithms, the input can either be a Triangle or a Polygon. Provide trivial Polygon-specific API
	/// for compatibility in those template functions.
	bool IsConvex() const { return true; }
	bool IsPlanar() const { return true; }
	bool IsSimple() const { return true; }

	/// Tests if the given object is fully contained inside this triangle.
	/** @param triangleThickness An epsilon threshold value to use for this test. triangleThicknessSq is the squared version of this parameter.
			This specifies the maximum distance the given object can lie from the plane defined by this triangle.
		@see Distance(), Intersects(), ClosestPoint().
		@todo Add Triangle::Contains(Circle) and Triangle::Contains(Disc). */
	bool Contains(const vec &point, float triangleThicknessSq = 1e-5f) const;
	bool Contains(const LineSegment &lineSegment, float triangleThickness = 1e-3f) const;
	bool Contains(const Triangle &triangle, float triangleThickness = 1e-3f) const;

	/// Computes the distance between this triangle and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@todo Add Triangle::Distance(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Capsule/Frustum/Polyhedron).
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const vec &point) const;
	float Distance(const Sphere &sphere) const;
	float Distance(const Capsule &capsule) const;

	float DistanceSq(const vec &point) const;

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
		@see Contains(), Distance(), ClosestPoint(), LineSegment::GetPoint(). */
	bool Intersects(const LineSegment &lineSegment, float *d = 0, vec *intersectionPoint = 0) const;
	bool Intersects(const Line &line, float *d = 0, vec *intersectionPoint = 0) const;
	bool Intersects(const Ray &ray, float *d = 0, vec *intersectionPoint = 0) const;
	bool Intersects(const Plane &plane) const;
	/** @param closestPointOnTriangle [out] If specified, receives the point of intersection between the Sphere
			and this Triangle. Even if no intersection occurred, this parameter will receive the closest point on
			the Triangle to the Sphere. This pointer may be null. */
	bool Intersects(const Sphere &sphere, vec *closestPointOnTriangle) const;
	bool Intersects(const Sphere &sphere) const;
	/** @param outLine [out] If specified, receives the line segment of the common points shared by the two
			intersecting triangles. If the two triangles do not intersect, this pointer is not written to.
			This pointer may be null. */
	bool Intersects(const Triangle &triangle, LineSegment *outLine = 0) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	bool Intersects(const Capsule &capsule) const;

	/// A helper function used in line-triangle tests.
	static float IntersectLineTri(const vec &linePos, const vec &lineDir,
		const vec &v0, const vec &v1, const vec &v2,
		float &u, float &v);

	/// Projects this Triangle onto the given axis.
	/** This function is used in SAT tests (separate axis theorem) to check the interval this triangle
		lies in on an 1D line.
		@param axis The axis to project onto. This vector can be unnormalized.
		@param dMin [out] Returns the minimum extent of this triangle on the given axis.
		@param dMax [out] Returns the maximum extent of this triangle on the given axis. */
	void ProjectToAxis(const vec &axis, float &dMin, float &dMax) const;

	int UniqueFaceNormals(vec *out) const;
	int UniqueEdgeDirections(vec *out) const;

	/// Computes the closest point on this triangle to the given object.
	/** If the other object intersects this triangle, this function will return an arbitrary point inside
		the region of intersection.
		@see Contains(), Distance(), Intersects(), ClosestPointToTriangleEdge(). */
	vec ClosestPoint(const vec &point) const;
	/** @param otherPt [out] If specified, receives the closest point on the other object to this triangle.
		This pointer may be null. */
	vec ClosestPoint(const LineSegment &lineSegment, vec *otherPt = 0) const;
	/** @param outU [out] If specified, receives the barycentric U coordinate of the returned point (in the UV convention).
			This pointer may be null. TODO Add this parameter back.
		@param outV [out] If specified, receives the barycentric V coordinate of the returned point (in the UV convention).
			This pointer may be null. TODO Add this parameter back.
		@param outD [out] If specified, receives the distance along the line of the closest point on the line to this triangle. TODO Add this parameter back.
		@return The closest point on this triangle to the given object.
		@todo Add ClosestPoint(Ray/Plane/Polygon/Circle/Disk/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron).
		@see Distance(), Contains(), Intersects(), ClosestPointToTriangleEdge(), Line::GetPoint. */
	vec ClosestPoint(const Line &line, vec *otherPt = 0) const;
	vec ClosestPoint(const Triangle &triangle, vec *otherPt = 0) const;

	/// Computes the closest point on the edge of this triangle to the given object.
	/** @param outU [out] If specified, receives the barycentric U coordinate of the returned point (in the UV convention).
			This pointer may be null.
		@param outV [out] If specified, receives the barycentric V coordinate of the returned point (in the UV convention).
			This pointer may be null.
		@param outD [out] If specified, receives the distance along the line of the closest point on the line to the edge of this triangle.
		@return The closest point on the edge of this triangle to the given object.
		@todo Add ClosestPointToTriangleEdge(Point/Ray/Triangle/Plane/Polygon/Circle/Disk/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron).
		@see Distance(), Contains(), Intersects(), ClosestPointToTriangleEdge(), Line::GetPoint. */
	vec ClosestPointToTriangleEdge(const Line &line, float *outU, float *outV, float *outD) const;
	vec ClosestPointToTriangleEdge(const LineSegment &lineSegment, float *outU, float *outV, float *outD) const;

	/// Generates a random point inside this Triangle.
	/** The points are distributed uniformly.
		The implementation of this function is based on Graphics Gems 1, p. 25:
		"1.5 Generating random points in triangles. Method 2." The Method 1 presented in the book
		uses a Sqrt() instead of the if().
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointOnEdge(), RandomVertex(), Point(). */
	vec RandomPointInside(LCG &rng) const;

	/// Chooses a corner vertex of this Triangle at random.
	/** This function returns one of the vertices {a, b, c} at uniformly random.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), RandomPointOnEdge(), Vertex(). */
	vec RandomVertex(LCG &rng) const;

	/// Generates a random point on the edge of this Triangle.
	/** The points are distributed uniformly.
		This function requires that this triangle is not degenerate. If it is, an assume() error will be printed,
		and the return value will be undefined.
		@param rng A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointInside(), RandomVertex(), Edge(), class LineSegment, IsDegenerate(). */
	vec RandomPointOnEdge(LCG &rng) const;

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this Line. Most useful for debugging purposes.
	std::string ToString() const;
	std::string SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	std::string SerializeToCodeString() const;
#endif

	static Triangle FromString(const char *str, const char **outEndStr = 0);
#ifdef MATH_ENABLE_STL_SUPPORT
	static Triangle FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif

	bool Equals(const Triangle &rhs, float epsilon = 1e-3f) const { return a.Equals(rhs.a, epsilon) && b.Equals(rhs.b, epsilon) && c.Equals(rhs.c, epsilon); }

	/// Compares whether this Triangle and the given Triangle are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Triangle &other) const { return a.BitEquals(other.a) && b.BitEquals(other.b) && c.BitEquals(other.c); }
};

Triangle operator *(const float3x3 &transform, const Triangle &t);
Triangle operator *(const float3x4 &transform, const Triangle &t);
Triangle operator *(const float4x4 &transform, const Triangle &t);
Triangle operator *(const Quat &transform, const Triangle &t);

struct Triangle_storage
{
	vec_storage v0, v1, v2;
	Triangle_storage(const Triangle &rhs)
	{
		*reinterpret_cast<Triangle*>(this) = rhs;
	}
	operator Triangle() const { return *reinterpret_cast<const Triangle*>(this); }
};
#define TRIANGLE(x) (*(Triangle*)&x)

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Triangle)
Q_DECLARE_METATYPE(Triangle*)
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Triangle &triangle);
#endif

bool LineSegment2DLineSegment2DIntersect(const float2 &p0, const float2 &dir0, const float2 &p1, const float2 &dir1, float &s, float &t);

MATH_END_NAMESPACE
