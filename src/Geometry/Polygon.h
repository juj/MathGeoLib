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

/** @file Polygon.h
	@author Jukka Jylänki
	@brief The Polygon geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

//#ifdef MATH_ENABLE_STL_SUPPORT
#include <vector>
//#endif

MATH_BEGIN_NAMESPACE

/// Represents a two-dimensional closed surface in 3D space.
/** A polygon is defined by N endpoints, or corner vertices. To be a valid polygon, there must be
   at least 3 vertices (a triangle).

   Well-formed polygons are always planar, i.e. all the vertices lie on the same plane. It is possible
   to store non-planar Polygons in this structure, but their representation is ambiguous, and for all practical
   purposes, should be avoided. */
class Polygon
{
public:
	/// The default constructor creates a null polygon.
	/** A null polygon has 0 vertices.
		@see IsNull(). */
	Polygon() {}

	/// Stores the vertices of this polygon.
	VecArray p;

	FORCE_INLINE static int NumFaces() { return 1; }

	/// Returns the number of edges in this polygon.
	/** Since the polygon is always closed and connected, the number of edges is equal to the number of vertices.
		@see Edge(), NumVertices(). */
	int NumEdges() const;

	/// Returns the number of vertices in this polygon.
	/** Since the polygon is always closed and connected, the number of edges is equal to the number of vertices.
		@see p, Vertex(), NumVertices(). */
	int NumVertices() const;

	/// Returns a pointer to an array of vertices of this polygon. The array contains NumVertices() elements.
	/// @note Do NOT hold on to this pointer, since it is an alias to the underlying std::vector owned by this polygon. Calling any non-const Polygon member function may invalidate the pointer!
	vec *VertexArrayPtr() { return !p.empty() ? (vec*)&p[0] : 0; }
	const vec *VertexArrayPtr() const { return !p.empty() ? (vec*)&p[0] : 0; }

	/// Quickly returns an arbitrary point inside this AABB. Used in GJK intersection test.
	vec AnyPointFast() const { return !p.empty() ? p[0] : vec::nan; }

	/// Returns a vertex of this polygon.
	/** @param vertexIndex The index of the vertex to get, in the range [0, NumVertices()-1].
		@see p, NumVertices(), Edge(). */
	vec Vertex(int vertexIndex) const;

	/// Returns a line segment between two adjacent vertices of this polygon.
	/** @param edgeIndex The index of the edge line segment to construct, in the range [0, NumEdges()-1].
		@return LineSegment(Vertex(edgeIndex), Vertex((edgeIndex+1)%NumVertices()).
		@see NumEdges(), Vertex(), Edge2D(), EdgeNormal(), EdgePlane(). */
	LineSegment Edge(int edgeIndex) const;

	/// Returns a line segment between two adjacent vertices of this polygon, in the local space of this polygon.
	/** In the local space of the polygon, the z-coordinate is always zero, and the polygon lies in the XY-plane, with
		the first vertex of the polygon being in the origin, and the x-axis running in the direction given by BasisU() and
		the y-axis running in the direction given by BasisV().
		@param edgeIndex The index of the edge line segment to construct, in the range [0, NumEdges()-1].
		@see NumEdges(), Vertex(), Edge2D(), EdgeNormal(), EdgePlane(), BasisU(), BasisV(). */
	LineSegment Edge2D(int edgeIndex) const;

	/// Returns the normal vector of the given edge.
	/** The normal vector is perpendicular to the normal of the plane the polygon lies in, and the direction the given edge
		is pointing towards. The vector points outwards from the polygon.
		@param edgeIndex The index of the edge line segment to construct, in the range [0, NumEdges()-1].
		@return A normalized direction vector perpendicular to the normal of the polygon, and the given edge.
		@see NumEdges(), Edge(), Edge2D(), EdgePlane(). */
	vec EdgeNormal(int edgeIndex) const;

	/// Returns the normal plane of the given edge.
	/** The normal vector of the returned plane points towards the direction specified by EdgeNormal(), and the given edge
		lies inside the returned plane.
		@param edgeIndex The index of the edge line segment to construct, in the range [0, NumEdges()-1]. */
	Plane EdgePlane(int edgeIndex) const;

	/// Computes an extreme point of this Polygon in the given direction.
	/** An extreme point is a farthest point of this Polygon in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Polygon in the given direction. The returned point is always a
			vertex of this Polygon.
		@see Vertex(). */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Projects this Polygon onto the given 1D axis direction vector.
	/** This function collapses this Polygon onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	/// Tests if the given diagonal exists.
	/** This function tests whether the diagonal that joins the two given vertices lies inside this polygon and is not intersected
		by the edges of this polygon.
		This function may only be called if this Polygon is planar.
		@param i Index of the first endpoint of the diagonal LineSegment, in the range [0, NumVertices-1].
		@param j Index of the second endpoint of the diagonal LineSegment, in the range [0, NumVertices-1]. Do not pass in i==j
			or |i-j| == 1.
		@note If |i-j| <= 1, then assume() failure is generated and false is returned.
		@return True if Diagonal(vertexIndex1, vertexIndex2) exists and does not intersect the edges of this polygon.
		@see Diagonal(), Vertex(), Edge(). */
	bool DiagonalExists(int i, int j) const;

	/// Returns the diagonal that joins the two given vertices.
	/** If |i-j| == 1, then this returns an edge of this Polygon.
		If i==j, then a degenerate line segment of zero length is returned.
		Otherwise, the line segment that joins the two given vertices is returned. Note that if the polygon is not planar or convex,
		this line segment might not lie inside the polygon. Use the DiagonalExists() function to test whether the returned
		LineSegment actually is a diagonal of this polygon.
		@param i Index of the first endpoint of the diagonal LineSegment, in the range [0, NumVertices-1].
		@param j Index of the second endpoint of the diagonal LineSegment, in the range [0, NumVertices-1].
		@note Whereas it is invalid to call DiagonalExists() with values |i-j|<=1, it is acceptable for this function. This is to
			simplify generation of code that iterates over diagonal vertex pairs.	
		@return LineSegment(Vertex(i), Vertex(j)) without checking if this actually is a valid diagonal of this polygon. If
			indices outside the valid range are passed, LineSegment(nan, nan) is returned.
		@see Vertex(), NumVertices(), DiagonalExists(). */
	LineSegment Diagonal(int i, int j) const;

	/// Tests if this polygon is convex.
	/** A polygon is convex, if for each pair of points inside the polygon, also the line segment joining those points is
		inside the polygon.
		@note This data structure can be used with both convex and non-convex polygons. In general, using convex polygons
			allows more efficient algorithms to be used with some operations. These more efficient variants are of form
			xxxConvex() in this class. Do not call those functions unless you know that the polygon is convex.
		@see IsPlanar(), IsSimple(), IsNull(), IsFinite(), IsDegenerate(). */
	bool IsConvex() const;

	/// Tests if this polygon is planar.
	/** A polygon is planar if all its vertices lie on the same plane.
		@note Almost all functions in this class require that the polygon is planar. While you can store vertices of
			non-planar polygons in this class, they are better avoided. Read the member function documentation carefully
			to avoid calling for non-planar polygons any functions which assume planarity.
		@see IsConvex(), IsSimple(), IsNull(), IsFinite(), IsDegenerate(). */
	bool IsPlanar(float epsilonSq = 1e-4f) const;

	/** Tests if this polygon is simple.
		A polygon is simple if no two nonconsecutive edges have a point in common.
		In other words, a planar polygon is simple if its edges do not self-intersect, and if each vertex is joined by
		exactly two edges.
		@note This function assumes that the polygon is planar.
		@see IsConvex(), IsPlanar(), IsNull(), IsFinite(), IsDegenerate(). */
	bool IsSimple() const;

	/// Tests if this polygon is null.
	/** A polygon is null if it has zero vertices.
		@note The null polygon is degenerate and finite.
		@see p, IsConvex(), IsPlanar(), IsSimple(), IsFinite(), IsDegenerate(). */
	bool IsNull() const;

	/// Tests if this polygon is finite.
	/** A polygon is finite if each of its vertices have finite floating point coordinates (no nans or infs).
		@note The null polygon is finite.
		@see p, IsConvex(), IsPlanar(), IsSimple(), IsNull(), IsDegenerate(), ::IsFinite(), IsInf(), IsNan(), IsFinite(), inf, negInf, nan, float3::nan, float3::inf. */
	bool IsFinite() const;

	/// Tests if this polygon is degenerate.
	/** A polygon is degenerate if it has two or less vertices, or if its surface area is less or equal than the given epsilon.
		@note The null polygon is degenerate and finite.
		@see p, IsConvex(), IsPlanar(), IsSimple(), IsNull(), IsDegenerate(), Area(). */
	bool IsDegenerate(float epsilon = 1e-4f) const;

	/// Generates the U vector of the local space of this polygon.
	/** This vector specifies in global (world) space the direction of the local X axis of this polygon.
		@note This function assumes that the first two points (p[0] and p[1]) of this polygon are finite and inequal. */
	vec BasisU() const;
	/// Generates the V vector of the local space of this polygon. [similarOverload: BasisU]
	/** This vector specifies in global (world) space the direction of the local Y axis of this polygon.
		@note This function assumes that the first two points (p[0] and p[1]) of this polygon are finite and inequal.
		@see MapTo2D(), MapFrom2D(), Edge2D(), BasisU(), BasisV(). */
	vec BasisV() const;

	/// Returns the given vertex of this polygon mapped to a local 2D space on this polygon.
	/** In the local space of the polygon, the z-coordinate is always zero, and the polygon lies in the XY-plane, with
		the first vertex of the polygon being in the origin, and the x-axis running in the direction given by BasisU() and
		the y-axis running in the direction given by BasisV().
		@param i The index of the vertices of this polygon to generate, in the range [0, NumVertices()-1].
		@see NumVertices(), MapFrom2D(), Edge2D(), BasisU(), BasisV(). */
	float2 MapTo2D(int i) const;

	/// Maps the given global (world) space point to the local 2D space of this polygon.
	/// @todo Return a float3 to be able to read the distance of the point from the plane of the polygon? (or add an overload for that)
	/// @todo Add MapTo2D(Line/LineSegment/Ray/Triangle/Polygon).
	float2 MapTo2D(const vec &point) const;

	/// Given a 2D point in the local space, returns the corresponding 3D point in the global (world) space.
	/** @see MapTo2D(), BasisU(), BasisV(). */
	vec MapFrom2D(const float2 &point) const;

	/// Computes the normal of this polygon.
	/** @return The normal of this polygon. This vector is normalized and points to the direction from which observed the
		vertices of this polygon wind in counter-clockwise order.
		@note Only call this function if this Polygon is planar. */
	vec NormalCCW() const;
	/// Computes the normal of this polygon in clockwise direction. [similarOverload: NormalCCW]
	/** @return The normal of this polygon in clockwise direction. This vector is normalized and points to the direction
		from which observed the vertices of this polygon wind in clockwise order.
		@note Only call this function if this Polygon is planar.
		@note These two functions follow the relation NormalCCW() == -NormalCW().
		@see PlaneCW(), PlaneCCW(). */
	vec NormalCW() const;

	/// Computes the plane this polygon is contained in.
	/** @note Only call this function if this Polygon is planar.
		@return The plane equation of this polygon. This normal vector of the plane points to the direction from which observed the
		vertices of this polygon wind in counter-clockwise order. */
	Plane PlaneCCW() const;
	/// Computes the plane this polygon is contained in, with a normal vector that points in the clockwise direction. [similarOverload: PlaneCCW]
	/** @note Only call this function if this Polygon is planar.
		@note The functions PlaneCCW() and PlaneCW() return the same plane, except the normals of the planes point in opposite directions.
		@see NormalCCW(), NormalCW(). */
	Plane PlaneCW() const;

	/// Translates this Polygon in world space.
	/** @param offset The amount of displacement to apply to this Polygon, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this Polygon.
	/** This function operates in-place.
		@see Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	// Returns true if the edges of this polygon self-intersect.
//	bool IsSelfIntersecting() const;

	// Projects all vertices of this polygon to the given plane.
//	void ProjectToPlane(const Plane &plane);

	// Returns true if the edges of this polygon self-intersect when viewed from the given direction.
//	bool IsSelfIntersecting(const vec &viewDirection) const;

	// Returns true if there exists edges (p_{i-1}, p_i) and (p_i, p_{i+1}) which are collinear.
//	bool HasCollinearEdges() const;

	/// Tests if the given object, expressed in global (world) space, is fully contained inside this polygon.
	/** This test is performed in global space of this polygon, i.e. by specifying the other object in global (world)
		space coordinates.
		@note For the containment tests, this Polygon can either be simple or non-simple (self-intersecting), but
			it must always be planar.
		@param point The point to test for containment.
		@param polygonThickness Since a polygon is a 2D object in a 3D space, a threshold value is used to
			allow floating-point inaccuracies. This parameter defines how much "thickness" to give to the polygon
			for the purposes of the test. polygonThicknessSq is this parameter squared.
		@return True if the given object is fully contained inside this polygon (and the plane of this polygon).
		@todo Add ContainsConvex(vec/etc.). See RTCD p. 202.
		@todo Add Contains(Circle/Disc). */
	bool Contains(const vec &point, float polygonThicknessSq = 1e-5f) const;
	bool Contains(const LineSegment &lineSegment, float polygonThickness = 1e-3f) const;
	bool Contains(const Triangle &triangle, float polygonThickness = 1e-3f) const;
	bool Contains(const Polygon &polygon, float polygonThickness = 1e-3f) const;
	//todo Add RTCD, p. 202.
	//bool ContainsConvex(const vec &worldSpacePoint, float polygonThickness = 1e-3f) const;

	/// Tests if the given object, expressed in the local space of this polygon, is fully contained inside this polygon.
	/** This test is exactly like in Contains(), except it is performed in 2D in the local space of this polygon.
		@see Contains(), MapTo2D().
		@todo Add Contains2D(Circle/Disc/Triangle/Polygon). */
	bool Contains2D(const LineSegment &localSpaceLineSegment) const;

	/// Tests if the given object, expressed in the local space of this polygon, is fully contained inside this polygon.
	/** This test is exactly like in Intersects(), except it is performed in 2D in the local space of this polygon.
		@see Contains(), MapTo2D().
		@todo Add Intersects2D(Circle/Disc/Triangle/Polygon). */
	bool Intersects2D(const LineSegment &localSpaceLineSegment) const;

	/// Tests whether this polygon and the given object intersect.
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true.
		This test is performed in the global (world) space of this polygon.
		@note These functions assume that this polygon might be concave, which requires a significantly slower test. If
			you know ahead of time that this polygon is convex, you can instead use one of the faster ConvexIntersects()
			functions.
		@note These tests (as well as any other functions of the Polygon class that relate to the specific set of points
			defined by the polygon) assume that this Polygon is planar, because non-planar Polygons do not have a well-defined area.
			The Polygon can however be non-simple (self-intersecting).
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@see ConvexIntersects(), Contains(), ClosestPoint(), Distance().
		@todo Add Intersects(Circle/Disc). */
	bool Intersects(const Line &line) const;
	bool Intersects(const Ray &ray) const;
	bool Intersects(const LineSegment &lineSegment) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Triangle &triangle, float polygonThickness = 1e-3f) const;
	bool Intersects(const Polygon &polygon, float polygonThickness = 1e-3f) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	bool Intersects(const Sphere &sphere) const;
	bool Intersects(const Capsule &capsule) const;

	/// Tests whether this convex polygon and the given object intersect.
	/** @note These functions make an implicit assumption that this polygon is convex. If you call these functions on
		a convex polygon, the intersection test is effectively performed on the convex hull of this polygon, meaning
		that it can result in false positives, so these functions can still be useful as an approximate or an early-out 
		test for concave polygons. */
	bool ConvexIntersects(const AABB &aabb) const;
	bool ConvexIntersects(const OBB &obb) const;
	bool ConvexIntersects(const Frustum &frustum) const;

	/// Computes the closest point on this polygon to the given object.
	/** If the other object intersects this polygon, this function will return an arbitrary point inside
		the region of intersection.
		@param lineSegment The line segment to find the closest point to.
		@param lineSegmentPt [out] If specified, receives the closest point on the line segment to this polygon. This
			pointer may be null.
		@see Contains(), Distance(), Intersects().
		@todo Add ClosestPoint(Line/Ray/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	vec ClosestPoint(const LineSegment &lineSegment, vec *lineSegmentPt) const;
	vec ClosestPoint(const LineSegment &lineSegment) const;
	vec ClosestPoint(const vec &point) const;

	/// Returns the distance between this polygon and the given point.
	/** @see Contains(), ClosestPoint(), Intersects(). */
	float Distance(const vec &point) const;

	/// Returns the surface area of this polygon.
	/** @see Perimeter(), Centroid(). */
	float Area() const;

	/// Returns the total edge length of this polygon.
	/** @see Area(), Centroid(). */
	float Perimeter() const;

	/// Returns the center of mass of this polygon.
	/** @see Area(), Perimeter(). */
	vec Centroid() const;
	/// Identical to CenterPoint(), but provided to enable common signature with Triangle, AABB and OBB to allow them to be used
	/// in template classes.
	vec CenterPoint() const { return Centroid(); }

	/// Computes a point on the perimeter of this polygon.
	/** @param normalizedDistance A value in the range [0,1[ specifying the distance along the polygon edge to travel.
		The polygon perimeter forms a closed loop, so PointOnEdge(0.f) == PointOnEdge(1.f) and is equal to the point p[0] of this
		polygon. As another example, PointOnEdge(0.5f) returns the point half-way around the polygon edge (but not necessarily the farthest
		point from p[0]).
		@see p, RandomPointOnEdge(). */
	vec PointOnEdge(float normalizedDistance) const;

	/// Computes a random point on the perimeter of this polygon.
	/** This function generates points with uniform distribution.
		@see PointOnEdge(). */
	vec RandomPointOnEdge(LCG &rng) const;

	vec FastRandomPointInside(LCG &rng) const;

	/// Converts this Polygon to a Polyhedron representation.
	/** This function will create a Polyhedron with two faces, one for the front face of this Polygon,
		and one for the back face.
		@todo Add ToPolyhedron(float polygonThickness)
		@see Triangulate(), MinimalEnclosingAABB(). */
	Polyhedron ToPolyhedron() const;

	// These faces will be extruded along the Polygon normal so that they lie polygonThickness units apart from each other.
//	Polyhedron ToPolyhedron(float polygonThickness = 0.f) const; ///@todo Add support for this form.

	/// Triangulates this Polygon using the ear-clipping method.
	/** @see ToPolyhedron(), MinimalEnclosingAABB(). */
	TriangleArray Triangulate() const;

	/// Returns the smallest AABB that encloses this polygon.
	/** @todo Add MinimalEnclosingSphere() and MinimalEnclosingOBB().
		@see ToPolyhedron(), Triangulate(). */
	AABB MinimalEnclosingAABB() const;

#if defined(MATH_ENABLE_STL_SUPPORT)
	std::string ToString() const;

	std::string SerializeToString() const;

	static Polygon FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

	static Polygon FromString(const char *str, const char **outEndStr = 0);

	bool Equals(const Polygon &other) const;
	bool BitEquals(const Polygon &other) const;

	// Returns true if the given vertex is a concave vertex. Otherwise the vertex is a convex vertex.
//	bool IsConcaveVertex(int i) const;

	// Computes the conves hull of this polygon.
//	Polygon ConvexHull() const;

//	bool IsSupportingPoint(int i) const;

//	bool IsSupportingPoint(const vec &point) const;

	// Returns true if the quadrilateral defined by the four points is convex (and not concave or bowtie).
//	static bool IsConvexQuad(const vec &pointA, const vec &pointB, const vec &pointC, const vec &pointD);
};

Polygon operator *(const float3x3 &transform, const Polygon &polygon);
Polygon operator *(const float3x4 &transform, const Polygon &polygon);
Polygon operator *(const float4x4 &transform, const Polygon &polygon);
Polygon operator *(const Quat &transform, const Polygon &polygon);

// @todo Add this.
//#ifdef MATH_ENABLE_STL_SUPPORT
//std::ostream &operator <<(std::ostream &o, const Polygon &polygon);
//#endif

MATH_END_NAMESPACE
