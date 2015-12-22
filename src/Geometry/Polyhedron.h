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

/** @file Polyhedron.h
	@author Jukka Jylänki
	@brief The Polyhedron geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

//#ifdef MATH_ENABLE_STL_SUPPORT
#include <vector>
#include <string>
//#endif

MATH_BEGIN_NAMESPACE

/// Represents a three-dimensional closed geometric solid defined by flat polygonal faces.
class Polyhedron
{
public:
	/// Stores a list of indices of a single face of a Polyhedron.
	struct Face
	{
		/// Specifies the indices of the corner vertices of this polyhedron.
		/// Indices point to the polyhedron vertex array.
		/// The face vertices should all lie on the same plane.
		/// The positive direction of the plane (the direction the face outwards normal points to)
		/// is the one where the vertices are wound in counter-clockwise order.
		std::vector<int> v;

		/// Reverses the winding order of this face. This has the effect of reversing the direction
		/// the normal of this face points to.
		void FlipWindingOrder();

		/// Returns a string of form "0,1,2,3,4" that refers to the indices of the vertices that this face uses.
		std::string ToString() const;

		static Face FromString(const char *str);
	};

	/// Specifies the vertices of this polyhedron.
	VecArray v;

	/// Specifies the individual faces of this polyhedron.  [similarOverload: v]
	/** Each face is described by a list of indices to the vertex array. The indices define a
		simple polygon in counter-clockwise winding order. */
	std::vector<Face> f;

	/// The default constructor creates a null polyhedron.
	/** A null polyhedron has 0 vertices and 0 faces.
		@see IsNull(). */
	Polyhedron() {}

	/// Returns the number of vertices in this polyhedron.
	/** The running time of this function is O(1).
		@see NumFaces(), NumEdges(), EulerFormulaHolds(). */
	int NumVertices() const { return (int)v.size(); }

	/// Returns the number of faces in this polyhedron.
	/** The running time of this function is O(1).
		@see NumVertices(), NumEdges(), EulerFormulaHolds(), FacePolygon(), FacePlane(). */
	int NumFaces() const { return (int)f.size(); }

	/// Returns the number of (unique) edges in this polyhedron.
	/** This function will enumerate through all faces of this polyhedron to compute the number of unique edges.
		The running time is linear to the number of faces and vertices in this Polyhedron.
		@see NumVertices(), NumFaces(), EulerFormulaHolds(), Edge(), Edges(), EdgeIndices(). */
	int NumEdges() const;

	/// Returns a pointer to an array of vertices of this polyhedron. The array contains NumVertices() elements.
	/// @note Do NOT hold on to this pointer, since it is an alias to the underlying std::vector owned by this polyhedron. Calling any non-const Polyhedron member function may invalidate the pointer!
	vec *VertexArrayPtr() { return !v.empty() ? (vec*)&v[0] : 0; }
	const vec *VertexArrayPtr() const { return !v.empty() ? (vec*)&v[0] : 0; }

	/// Returns the <i>i</i>th vertex of this polyhedron.
	/** @param vertexIndex The vertex to get, in the range [0, NumVertices()-1].
		@see NumVertices(). */
	vec Vertex(int vertexIndex) const;

	/// Returns the <i>i</i>th edge of this polyhedron.
	/** Performance warning: Use this function only if you are interested in a single edge of this Polyhedron.
		This function calls the Edges() member function to receive a list of all the edges, so has
		a complexity of O(|V|log|V|), where |V| is the number of vertices in the polyhedron.
		@param edgeIndex The index of the edge to get, in the range [0, NumEdges()-1].
		@see NumEdges(), Edges(), EdgeIndices(). */
	LineSegment Edge(int edgeIndex) const;

	/// Returns all the (unique) edges of this polyhedron.
	/** Has complexity of O(|V|log|V|), where |V| is the number of vertices in the polyhedron.
		@todo Support this in linear time.
		@see NumEdges(), Edge(), EdgeIndices(). */
	LineSegmentArray Edges() const;

	std::vector<Polygon> Faces() const;

	/// Returns all the (unique) edges of this polyhedron, as indices to the polyhedron vertex array.
	/** Has complexity of O(|V|log|V|), where |V| is the number of vertices in the polyhedron.
		@todo Support this in linear time.
		@see NumEdges(), Edge(), Edges(). */
	std::vector<std::pair<int, int> > EdgeIndices() const;

	/// Returns a polygon representing the given face.
	/** The winding order of the polygon will be the same as in the input. The normal of the polygon
		points outwards from this polyhedron, i.e. towards the space that is not part of the polyhedron.
		This function constructs a new Polygon object, so it has a time and space complexity of
		O(|V|), where |V| is the number of vertices in this polyhedron.
		@param faceIndex The index of the face to get, in the range [0, NumFaces()-1].
		@see NumFaces(), FacePlane(). */
	Polygon FacePolygon(int faceIndex) const;

	/// Returns the plane of the given polyhedron face.
	/** The normal of the plane points outwards from this polyhedron, i.e. towards the space that
		is not part of the polyhedron.
		This function assumes that the given face of the polyhedron is planar, as should be for all
		well-formed polyhedron.
		@param faceIndex The index of the face to get, in the range [0, NumFaces()-1].
		@see NumFaces(), FacePolygon(). */
	Plane FacePlane(int faceIndex) const;

	/// Returns the normalized normal vector of the given face.
	vec FaceNormal(int faceIndex) const;

	/// Returns the index of the vertex of this polyhedron that reaches farthest in the given direction.
	/** @param direction The direction vector to query for. This vector can be unnormalized.
		@return The supporting point of this polyhedron that reaches farthest in the given direction.
			The supporting point for a given direction is not necessarily unique, but this function
			will always return one of the vertices of this polyhedron.
		@see v, NumVertices(), Vertex(). */
	int ExtremeVertex(const vec &direction) const;
	// vec SupportingPoint(const vec &dir) const;
	// bool IsSupportingPlane(const Plane &plane) const;

	/// Computes an extreme point of this Polyhedron in the given direction.
	/** An extreme point is a farthest point of this Polyhedron in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Polyhedron in the given direction. The returned point is always a
			corner point of this Polyhedron.
		@see CornerPoint(). */
	vec ExtremePoint(const vec &direction) const;
	vec ExtremePoint(const vec &direction, float &projectionDistance) const;

	/// Quickly returns an arbitrary point inside this AABB. Used in GJK intersection test.
	vec AnyPointFast() const { return v[0]; }

	// Computes the most extreme point of this convex Polyhedron into the given direction.
	/** @param adjacencyData A precomputed data structure that specifies the adjacency information between the vertices of this Polyhedron.
			Call GenerateVertexAdjacencyData() to compute this structure.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@param floodFillVisited A temporary structure to an array of size |V| where each element specifies whether the extreme
			vertex search has visited that vertex or not. If floodFillVisited[i] == floodFillVisitColor, then the vertex i
			has been visited, otherwise not.
		@param mostExtremeDistance [out] Receives the 1D projection distance of the most extreme vertex onto the direction vector.
		@param startingVertex [optional] Specifies a hint vertex from where to start the search. Specifying a know vertex that is close
			to being the most extreme vertex in the given direction may speed up the search.
		@return The index of the most extreme vertex into the specified direction. */

#define MATH_NUMSTEPS_STATS

#ifdef MATH_NUMSTEPS_STATS
	mutable int numSearchStepsDone, numImprovementsMade;
#endif
	int ExtremeVertexConvex(const std::vector<std::vector<int> > &adjacencyData, const vec &direction,
		std::vector<unsigned int> &floodFillVisited, unsigned int floodFillVisitColor, float &mostExtremeDistance, int startingVertex = 0) const;

	/// Projects this Polyhedron onto the given 1D axis direction vector.
	/** This function collapses this Polyhedron onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const vec &direction, float &outMin, float &outMax) const;

	/// Returns the exact center of mass of the convex hull of this polyhedron.
	/** @see SurfaceArea(), Volume(), ApproximateConvexCentroid(). */
	vec ConvexCentroid() const;

	// Computes the average of all vertices of this Polyhedron.
	/** If this Polyhedron is a tetrahedron, this is the center of mass for the Polyhedron. Otherwise it
		is a kind of an approximate enter of mass, biased towards the direction where there are lots of
		vertices in the Polyhedron.
		@note This function is considerably faster than ConvexCentroid().
		@see SurfaceArea(), Volume(), ConvexCentroid(). */
	vec ApproximateConvexCentroid() const;

	/// Computes the total surface area of the faces of this polyhedron.
	/** @note When you call this function, none of the faces of this polyhedron may be self-intersecting.
		@see ConvexCentroid(), Volume(). */
	float SurfaceArea() const;

	/// Computes the internal volume of this polyhedron.
	/** @see ConvexCentroid(), SurfaceArea(). */
	float Volume() const;

	/// Returns the smallest AABB that encloses this polyhedron.
	/// @todo Add MinimalEnclosingSphere() and MinimalEnclosingOBB().
	AABB MinimalEnclosingAABB() const;

#ifdef MATH_CONTAINERLIB_SUPPORT
	OBB MinimalEnclosingOBB() const;
#endif

	/// Computes a data structure that specifies adjacent vertices for each vertex.
	/** In the returned vector of vectors V, the vector V[i] specifies all the vertex indices that vertex i
		is connected to. */
	std::vector<std::vector<int> > GenerateVertexAdjacencyData() const;

	/// Tests if the faces in this polyhedron refer to valid existing vertices.
	/** This function performs sanity checks on the face indices array.
		1) Each vertex index for each face must be in the range [0, NumVertices()-1], i.e. refer to a vertex
		   that exists in the array.
		2) Each face must contain at least three vertices. If a face contains two or one vertex, it is
		   degenerate.
		3) Each face may refer to a vertex at most once.
		@return True if the abovementioned conditions hold. Note that this does not guarantee that the
			polyhedron is completely well-formed, but if false is returned, the polyhedron is definitely
			ill-formed.
		@see IsNull(), IsClosed(), IsConvex(). */
	bool FaceIndicesValid() const;

	/// Flips the winding order of all faces in this polyhedron.
	void FlipWindingOrder();

	/// Assuming that this polyhedron is convex, reorients all faces of this polyhedron
	/// so that each face plane has its normal pointing outwards. That is, the "negative" side of the
	/// polyhedron lies inside the polyhedron, and the positive side of the polyhedron is outside the convex
	/// shape.
	void OrientNormalsOutsideConvex();

	/// Removes from the vertex array all vertices that are not referred to by any of the faces of this polyhedron.
	void RemoveRedundantVertices();

	/// Removes all faces from this polyhedron which have two or less vertices in them.
	void RemoveDegenerateFaces();

	/// Finds all neighboring faces that have identical face normals and merges them together.
	/// Warning: this introduces T-junctions to the polyhedron, as well as increases the vertex count on the merged faces.
	/// Use this only as preprocessing when needed.
	/// @return The number of faces that were removed by merging.
	int MergeAdjacentPlanarFaces(bool snapVerticesToMergedPlanes, bool conservativeEnclose = true, float angleEpsilon = 1e-16f, float distanceEpsilon = 1e-8f);

	/// Returns true if this polyhedron has 0 vertices and 0 faces.
	/** @see FaceIndicesValid(), IsClosed(), IsConvex(). */
	bool IsNull() const { return v.empty() && f.empty(); }

	/// Returns true if this polyhedron is closed and does not have any gaps.
	/** \note This function performs a quick check, which might not be complete.
		The running time is O(FlogE) ~ O(VlogV).
		@see FaceIndicesValid(), IsClosed(), IsConvex(). */
	bool IsClosed() const;

	// Returns true if this polyhedron forms a single connected solid volume.
//	bool IsConnected() const;

	/// Returns true if this polyhedron is convex.
	/** The running time is O(F*V) ~ O(V^2).
		@see FaceIndicesValid(), IsClosed(), IsConvex().*/
	bool IsConvex() const;

	/// Returns true if the Euler formula (V + F - E == 2) holds for this Polyhedron.
	/** The running time is O(E) ~ O(V).
		@see NumVertices(), NumEdges(), NumFaces(). */
	bool EulerFormulaHolds() const;

	/// Tests whether all the faces of this polyhedron are non-degenerate (have at least 3 vertices)
	/// and in case they have more than 3 vertices, tests that the faces are planar.
	/** The running time is O(F) ~ O(V). */
	bool FacesAreNondegeneratePlanar(float epsilon = 1e-2f) const;

	/// Clips the line/ray/line segment specified by L(t) = ptA + t * dir, tFirst <= t <= tLast,
	/// inside this <b>convex</b> polyhedron.
	/** The implementation of this function is adapted from Christer Ericson's Real-time Collision Detection, p. 199.
		@param ptA The first endpoint of the line segment.
		@param dir The direction of the line segment. This member can be unnormalized.
		@param tFirst [in, out] As input, takes the parametric distance along the line segment which
			specifies the starting point of the line segment. As output, the starting point of the line segment
			after the clipping operation is returned here.
		@param tLast [in, out] As input, takes the parametric distance along the line segment which
			specifies the ending point of the line segment. As output, the endingpoint of the line segment
			after the clipping operation is returned here.
		@note To clip a line, pass in tFirst=-FLOAT_INF, tLast=FLOAT_INF. To clip a ray, pass in tFirst=0 and tLast = FLOAT_INF.
			To clip a line segment, pass in tFirst=0, tLast=1, and an unnormalized dir = lineSegment.b-lineSegment.a.
		@return True if the outputted range [tFirst, tLast] did not become degenerate, and these two variables contain
			valid data. If false, the whole line segment was clipped away (it was completely outside this polyhedron).
		@see FLOAT_INF. */
	bool ClipLineSegmentToConvexPolyhedron(const vec &ptA, const vec &dir, float &tFirst, float &tLast) const;

	/// Returns the index of the nearest vertex to the given point.
	/** @note In the degenerate case when this Polyhedron is null and does not contain any vertices, this function returns -1. */
	int NearestVertex(const vec &point) const;

	/// Tests if the given object is fully contained inside this closed polyhedron.
	/** This function treats this polyhedron as a non-convex object. If you know this polyhedron
		to be convex, you can use the faster ContainsConvex() function.
		@note This function assumes that this polyhedron is closed and its edges are not self-intersecting.
		@see ContainsConvex(), ClosestPoint(), ClosestPointConvex(), Distance(), Intersects(), IntersectsConvex().
		@todo Add Contains(Circle/Disc/Sphere/Capsule). */
	bool Contains(const vec &point) const;
	bool Contains(const LineSegment &lineSegment) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;
	bool Contains(const AABB &aabb) const;
	bool Contains(const OBB &obb) const;
	bool Contains(const Frustum &frustum) const;
	bool Contains(const Polyhedron &polyhedron) const;

	/// Tests if the given face of this Polyhedron contains the given point.
	bool FaceContains(int faceIndex, const vec &worldSpacePoint, float polygonThickness = 1e-3f) const;

	/// A helper for Contains() and FaceContains() tests: Returns a positive value if the given point is contained in the given face,
	/// and a negative value if the given point is outside the face. The magnitude of the return value reports a pseudo-distance
	/// from the point to the nearest edge of the face polygon. This is used as a robustness/stability criterion to estimate how
	/// numerically believable the result is.
	float FaceContainmentDistance2D(int faceIndex, const vec &worldSpacePoint, float polygonThickness = 1e-3f) const;

	/// Tests if the given object is fully contained inside this <b>convex</b> polyhedron.
	/** This function behaves exactly like Contains(), except this version of the containment test
		assumes this polyhedron is convex, and uses a faster method of testing containment.
		@note This function assumes that this polyhedron is closed and its edges are not self-intersecting.
		@see Contains(), ClosestPoint(), ClosestPointConvex(), Distance(), Intersects(), IntersectsConvex().
		@todo Add ContainsConvex(Polygon/AABB/OBB/Frustum/Polyhedron/Circle/Disc/Sphere/Capsule). */
	bool ContainsConvex(const vec &point, float epsilon = 1e-4f) const;
	bool ContainsConvex(const LineSegment &lineSegment) const;
	bool ContainsConvex(const Triangle &triangle) const;

	/// Computes the closest point on this polyhedron to the given object.
	/** If the other object intersects this polyhedron, this function will return an arbitrary point inside
		the region of intersection.
		@param lineSegment The line segment to find the closest point to.
		@param lineSegmentPt [out] If specified, returns the closest point on the line segment to this
		polyhedron. This pointer may be null.
		@todo Make lineSegmentPt an out-reference instead of an out-pointer.
		@note This function assumes that this polyhedron is closed and the edges are not self-intersecting.
		@see Contains(), ContainsConvex(), ClosestPointConvex(), Distance(), Intersects(), IntersectsConvex().
		@todo Add ClosestPoint(Line/Ray/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	vec ClosestPoint(const LineSegment &lineSegment, vec *lineSegmentPt) const;
	vec ClosestPoint(const LineSegment &lineSegment) const;
	/** @param point The point to find the closest point to. */
	vec ClosestPoint(const vec &point) const;

	/// Returns the closest point on this <b>convex</b> polyhedron to the given point.
	/** This function behaves exactly like ClosestPoint(), except this version of the test assumes
		this polyhedron is convex, and uses a faster method of finding the closest point.
		@note This function assumes that this polyhedron is closed and the edges are not self-intersecting.
		@see Contains(), ContainsConvex(), ClosestPoint(), Distance(), Intersects(), IntersectsConvex().
		@todo Add ClosestPointConvex(Line/LineSegment/Ray/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	vec ClosestPointConvex(const vec &point) const;

	/// Returns the distance between this polyhedron and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@note This function assumes that this polyhedron is closed and the edges are not self-intersecting.
		@see Contains(), ContainsConvex(), ClosestPoint(), ClosestPointConvex(), Intersects(), IntersectsConvex().
		@todo Add Distance(Line/LineSegment/Ray/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	float Distance(const vec &point) const;

	/// Tests whether this polyhedron and the given object intersect.
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this polyhedron,
		or this polyhedron is contained inside a sphere, etc.)
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@note This function assumes that this polyhedron is closed and the edges are not self-intersecting.
		@see Contains(), ContainsConvex(), ClosestPoint(), ClosestPointConvex(), Distance(), IntersectsConvex().
		@todo Add Intersects(Circle/Disc). */
	bool Intersects(const LineSegment &lineSegment) const;
	bool Intersects(const Line &line) const;
	bool Intersects(const Ray &ray) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const Polyhedron &polyhedron) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Sphere &sphere) const;
	bool Intersects(const Capsule &capsule) const;

	/// Tests whether this <b>convex</b> polyhedron and the given object intersect.
	/** This function is exactly like Intersects(), but this version assumes that this polyhedron is convex,
		and uses a faster method of testing the intersection.
		@return True if an intersection occurs or one of the objects is contained inside the other, false otherwise.
		@note This function assumes that this polyhedron is closed and the edges are not self-intersecting.
		@see Contains(), ContainsConvex(), ClosestPoint(), ClosestPointConvex(), Distance(), Intersects().
		@todo Add Intersects(Circle/Disc). */
	bool IntersectsConvex(const Line &line) const;
	bool IntersectsConvex(const Ray &ray) const;
	bool IntersectsConvex(const LineSegment &lineSegment) const;

	void MergeConvex(const vec &point);

	/// Translates this Polyhedron in world space.
	/** @param offset The amount of displacement to apply to this Polyhedron, in world space coordinates.
		@see Transform(). */
	void Translate(const vec &offset);

	/// Applies a transformation to this Polyhedron.
	/** This function operates in-place.
		@see Translate(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Creates a Polyhedron object that represents the convex hull of the given point array.
	/// \todo This function is strongly WIP!
	static Polyhedron ConvexHull(const VecArray &points) { return !points.empty() ? ConvexHull((const vec*)&points[0], (int)points.size()) : Polyhedron(); }
	static Polyhedron ConvexHull(const VecArray &points, LCG &rng) { return !points.empty() ? ConvexHull((const vec*)&points[0], (int)points.size(), rng) : Polyhedron(); }
	static Polyhedron ConvexHull(const vec *pointArray, int numPoints);
	static Polyhedron ConvexHull(const vec *pointArray, int numPoints, LCG &rng);

	static Polyhedron Tetrahedron(const vec &centerPos = POINT_VEC_SCALAR(0.f), float scale = 1.f, bool ccwIsFrontFacing = true);
	static Polyhedron Octahedron(const vec &centerPos = POINT_VEC_SCALAR(0.f), float scale = 1.f, bool ccwIsFrontFacing = true);
	static Polyhedron Hexahedron(const vec &centerPos = POINT_VEC_SCALAR(0.f), float scale = 1.f, bool ccwIsFrontFacing = true);
	static Polyhedron Icosahedron(const vec &centerPos = POINT_VEC_SCALAR(0.f), float scale = 1.f, bool ccwIsFrontFacing = true);
	static Polyhedron Dodecahedron(const vec &centerPos = POINT_VEC_SCALAR(0.f), float scale = 1.f, bool ccwIsFrontFacing = true);

	static Polyhedron CreateCapsule(const vec &a, const vec &b, float r, int verticesPerCap, bool ccwIsFrontFacing = true);
	static Polyhedron CreateSharpCapsule(const vec &a, const vec &b, float r, float capPointDistance, int verticesPerCap, bool ccwIsFrontFacing = true);

	/// Tests if these two polyhedrons represent the same set of points.
	/// @note This function is very slow, and should be used only for debugging purposes.
	/// @note This function mutates this and the given polyhedron in order to make the test more feasible. The set of space represented
	///       by the polyhedrons will not change.
	bool SetEquals(Polyhedron &p2);

	/// Swaps two vertices in the vertex array and updates all faces of this polyhedron so that the volume represented by this polyhedron
	/// stays the same.
	void SwapVertices(int i, int j);

	/// Converts the list of faces into a canonical order for easier comparison.
	void CanonicalizeFaceArray();

	/// Returns true if one of the faces of this Polyhedron has the same ordered of indices as the given Face.
	bool ContainsFace(const Face &face) const;

	/// Searches each vertex of this polyhedron to find the closest vertex to the given target point.
	/// @param outDistanceSq [out] Outputs the distance between the target point and the closest vertex, or FLOAT_INF if no such point was found.
	/// @return An index to the vertex array of this polyhedron denoting the closest vertex to the target point.
	///         Returns -1 if no such point is found. (no vertices in polyhedron, or all of them contained NaNs/Infs)
	int FindClosestVertex(const vec &pt, float &outDistanceSq) const;

	TriangleArray Triangulate() const;

	std::string ToString() const;
	void DumpStructure() const;

#ifdef MATH_GRAPHICSENGINE_INTEROP
	void Triangulate(VertexBuffer &vb, bool ccwIsFrontFacing, int faceStart = 0, int faceEnd = 0x7FFFFFFF) const;
	void ToLineList(VertexBuffer &vb) const;
#endif
};

Polyhedron operator *(const float3x3 &transform, const Polyhedron &polyhedron);
Polyhedron operator *(const float3x4 &transform, const Polyhedron &polyhedron);
Polyhedron operator *(const float4x4 &transform, const Polyhedron &polyhedron);
Polyhedron operator *(const Quat &transform, const Polyhedron &polyhedron);

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Polyhedron)
Q_DECLARE_METATYPE(Polyhedron*)
#endif
/*
Polyhedron operator *(const float3x3 &m, const Polyhedron &s);
Polyhedron operator *(const float3x4 &m, const Polyhedron &s);
Polyhedron operator *(const float4x4 &m, const Polyhedron &s);
Polyhedron operator *(const Quat &q, const Polyhedron &s);
*/
// @todo Add this
//#ifdef MATH_ENABLE_STL_SUPPORT
//std::ostream &operator <<(std::ostream &o, const Polyhedron &polyhedron);
//#endif

MATH_END_NAMESPACE
