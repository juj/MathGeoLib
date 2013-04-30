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

/** @file OBB.h
	@author Jukka Jylänki
	@brief The Oriented Bounding Box (OBB) geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A 3D arbitrarily oriented bounding box.
/** This data structure represents a box in 3D space. The local axes of this box can be arbitrarily oriented/rotated
	with respect to the global world coordinate system. This allows OBBs to more tightly bound objects than AABBs do,
	which always align with the world space axes. This flexibility has the drawback that the geometry tests and operations
	involving OBBs are more costly, and representing an OBB in memory takes more space (15 floats vs 6 floats). */
class OBB
{
public:
	/// The center position of this OBB.
	/** In the local space of the OBB, the center of this OBB is at (r.x,r.y,r.z), and the OBB is an AABB with
		size 2*r. */
	float3 pos;

	/// Stores half-sizes to x, y and z directions in the local space of this OBB. [similarOverload: pos]
	/** These members should be positive to represent a non-degenerate OBB. */
	float3 r;

	/// Specifies normalized direction vectors for the local axes. [noscript] [similarOverload: pos]
	/** axis[0] specifies the +X direction in the local space of this OBB, axis[1] the +Y direction and axis[2]
		the +Z direction.
		The scale of these vectors is always normalized. The half-length of the OBB along its local axes is
		specified by the vector r.
		The axis vectors must always be orthonormal. Be sure to guarantee that condition holds if you
		directly set to this member variable. */
	float3 axis[3];

	/// The default constructor does not initialize any members of this class. [opaque-qtscript]
	/** This means that the values of the members pos, r and axis are undefined after creating a new OBB using this
		default constructor. Remember to assign to them before use.
		@see pos, r, axis. */
	OBB() {}

	/// Constructs an OBB from an AABB.
	/** Since the OBB is an AABB with arbirary rotations allowed, this conversion is exact, i.e. it does not loosen
		the set of points represented by the AABB. Therefore this constructor is implicit, meaning that you
		can directly assign an AABB to an OBB.
		@note Converting an OBB back to an AABB is not exact. See the MinimalEnclosingAABB() function for
			converting to the opposite direction.
		@see class AABB, SetFrom(), MinimalEnclosingAABB(). */
	OBB(const AABB &aabb);

	FORCE_INLINE static int NumFaces() { return 6; }
	FORCE_INLINE static int NumEdges() { return 12; }
	FORCE_INLINE static int NumVertices() { return 8; }

	/// Sets this structure to a degenerate OBB that does not have any volume.
	/** This function sets pos=(0,0,0), r = (-inf,-inf,-inf) and axis=float3x3::identity for this OBB.
		@note This function operates in-place. After calling this function, this OBB is degenerate.
		@see pos, r, axis, IsDegenerate(). */
	void SetNegativeInfinity();

	/// Sets this OBB from an AABB.
	/** This conversion is exact, and does not loosen the volume.
		@note Converting an OBB back to an AABB is not exact. See the MinimalEnclosingAABB() function for
			converting to the opposite direction.
		@param aabb The axis-aligned bounding box to convert to an OBB.
		@see classes AABB, float3x3, float3x4, float4x4, Quat, MinimalEnclosingAABB(). */
	void SetFrom(const AABB &aabb);
	/** @param transform If a transformation matrix is supplied, this transformation is applied to the AABB before
		representing it as an oriented bounding box. The basis of this matrix is assumed to be orthogonal, which
		means no projection or shear is allowed. Additionally, the matrix must contain only uniform scaling. */
	void SetFrom(const AABB &aabb, const float3x3 &transform);
	void SetFrom(const AABB &aabb, const float3x4 &transform);
	void SetFrom(const AABB &aabb, const float4x4 &transform);
	void SetFrom(const AABB &aabb, const Quat &transform);

	/// Sets this OBB to enclose the given sphere.
	/** This function computes the tightest possible OBB (in terms of volume) that contains the given sphere, and stores the result in this structure.
		@note Since an OBB is a box, and Sphere is, well, a sphere, this conversion is not exact, but loosens the set of points in the representation.
		@see class Sphere, MinimalEnclosingSphere(). */
	void SetFrom(const Sphere &sphere);

#ifdef MATH_CONTAINERLIB_SUPPORT
	/// Sets this OBB to enclose the given polyhedron.		
	/** This function computes the smallest OBB (in terms of volume) that contains the given polyhedron, and stores the result in this structure.
		@note An OBB cannot generally exactly represent a polyhedron. Converting a polyhedron to an OBB loses some features of the polyhedron.
		@return True if the resulting OBB is not degenerate, false otherwise. In either case, the old values of this OBB are destroyed.
		@see SetFromApproximate(), ToPolyhedron(). */
	bool SetFrom(const Polyhedron &polyhedron);
#endif

#if 0
	/// Sets this OBB to enclose the given point cloud.
	/** This functions uses principal component analysis to generate an approximation of the smallest OBB that encloses the
		given point set. The resulting OBB will always contain all the specified points, but might not be the optimally
		smallest OBB in terms of volume.
		@see SetFrom(), ToPolyhedron(), PCAEnclosingOBB(). */
	void SetFromApproximate(const float3 *pointArray, int numPoints);
#endif

	/// Converts this OBB to a polyhedron.
	/** This function returns a polyhedron representation of this OBB. This conversion is exact, meaning that the returned
		polyhedron represents the same set of points than this OBB. */
	Polyhedron ToPolyhedron() const;

	/// Returns the tightest AABB that contains this OBB.
	/** This function computes the optimal minimum volume AABB that encloses this OBB.
		@note Since an AABB cannot generally represent an OBB, this conversion is not exact, but the returned AABB
			specifies a larger volume.			
		@see SetFrom(), MaximalContainedAABB(), MinimalEnclosingSphere(), MaximalContainedSphere(). */
	AABB MinimalEnclosingAABB() const;

#if 0
	/// Returns the largest AABB that can fit inside this OBB.
	/** This function computes the largest AABB that can fit inside this OBB. This AABB is unique up to the center point of the
		AABB. The returned AABB will be centered to the center point of this OBB.
		@see MinimalEnclosingAABB(), MinimalEnclosingSphere(), MaximalContainedSphere(). */
	AABB MaximalContainedAABB() const;
#endif

	/// Returns the smallest sphere that contains this OBB.
	/** This function computes the optimal minimum volume sphere that encloses this OBB.
		@see MinimalEnclosingAABB(), MaximalContainedAABB(), MaximalContainedSphere(). */
	Sphere MinimalEnclosingSphere() const;

	/// Returns the largest sphere that can fit inside this OBB. [similarOverload: MinimalEnclosingSphere]
	/** This function computes the largest sphere that can fit inside this OBB. This sphere is unique up to the center point
		of the sphere. The returned sphere will be positioned to the same center point as this OBB.
		@see MinimalEnclosingSphere(), MaximalContainedAABB(), MaximalContainedSphere(). */
	Sphere MaximalContainedSphere() const;

	/// Returns the side lengths of this OBB in its local x, y and z directions.
	/** @return 2*r. */
	float3 Size() const;

	/// Returns the half-side lengths of this OBB in its local x, y and z directions. [similarOverload: Size]
	/** @return r.
		@see r, Size(), HalfSize(). */
	float3 HalfSize() const;

	/// Returns a diagonal vector of this OBB.
	/** This vector runs from one corner of the OBB from the opposite corner.
		@note A box has four diagonals. This function returns the direction vector from the -X-Y-Z corner to
			the +X+Y+Z corner of the OBB, in the global space of this OBB. */
	float3 Diagonal() const;
	/// Returns Diagonal()/2. [similarOverload: Diagonal].
	/** @return A direction vector from the center of this OBB to the +X+Y+Z corner of this OBB, in global space.
		@see Size(), HalfSize(). */
	float3 HalfDiagonal() const;

	/// Computes the transformation matrix that maps from the global (world) space of this OBB to the local space of this OBB.
	/** In local space, the center of the OBB lies at (r.x,r.y,r.z), and the OBB is aligned along the cardinal axes, i.e. is an AABB.
		The local +X vector runs in the direction specified by axis[0], the +Y direction is specified by axis[1], and +Z by axis[2].
		The size of the OBB is 2*r.
		In global (world) space, the center of the OBB lies at the point given by the pos member variable.
		@return This global (world) to local space transform can be represented using a float3x4 matrix. This function computes
			and returns the matrix that maps from the world space of this OBB to the local space of this OBB.
		@see pos, r, axis. */
	float3x4 WorldToLocal() const;

	/// Computes the transformation matrix that maps from the local space of this OBB to the global (world) space of this OBB. [similarOverload: WorldToLocal]
	/** This mapping is the inverse of the transform computed by WorldToLocal().
		@return A matrix that transforms from the local space of this OBB to the global (world) space of this OBB.
		@see pos, r, axis. */
	float3x4 LocalToWorld() const;
	
	/// Tests if this OBB is finite.
	/** @return True if the member variables of this OBB are valid floats and do not contain NaNs or infs, and false otherwise.
		@see IsDegenerate(). */
	bool IsFinite() const;

	/// Tests if this OBB is degenerate.
	/** @return True if this OBB does not span a strictly positive volume.
		@see r, Volume(). */
	bool IsDegenerate() const;

	/// Returns the center point of this OBB in global (world) space of this OBB.
	/** @note The center point of this OBB in local space is always (r.x, r.y, r.z).
		@see pos.  */
	float3 CenterPoint() const;

	/// Returns the center of mass of this OBB. [similarOverload: CenterPoint]
	/** @note This function is identical to CenterPoint(), and is provided to ease template function implementations.
		@see Volume(), SurfaceArea(). */
	float3 Centroid() const { return CenterPoint(); }

	/// Computes the volume of this OBB.
	/** @see CenterPoint(), SurfaceArea(). */
	float Volume() const;

	/// Computes the total surface area of the faces of this OBB.
	/** @see CenterPoint(), Volume(). */
	float SurfaceArea() const;

	/// Generates a point inside this OBB.
	/** @param x A normalized value between [0,1]. This specifies the point position along the local x axis of the OBB.
		@param y A normalized value between [0,1]. This specifies the point position along the local y axis of the OBB.
		@param z A normalized value between [0,1]. This specifies the point position along the local z axis of the OBB.
		@return A point in the global space of this OBB corresponding to the parametric coordinate (x,y,z).
		@see Edge(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	float3 PointInside(float x, float y, float z) const;

	/// Returns an edge of this OBB.
	/** @param edgeIndex The index of the edge line segment to get, in the range [0, 11].
		@todo Draw a diagram that shows which index generates which edge.
		@see PointInside(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	LineSegment Edge(int edgeIndex) const;

	/// Returns a corner point of this OBB.
	/** This function generates one of the eight corner points of this OBB.
		@param cornerIndex The index of the corner point to generate, in the range [0, 7].
		 The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions).
		@todo Draw a diagram that shows which index generates which edge.
		@see PointInside(), Edge(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	float3 CornerPoint(int cornerIndex) const;

	/// Computes an extreme point of this OBB in the given direction.
	/** An extreme point is a farthest point of this OBB in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this OBB in the given direction. The returned point is always a
			corner point of this OBB.
		@see CornerPoint(). */
	float3 ExtremePoint(const float3 &direction) const;

	/// Projects this OBB onto the given 1D axis direction vector.
	/** This function collapses this OBB onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const;

	/// Returns a point on an edge of this OBB.
	/** @param edgeIndex The index of the edge to generate a point to, in the range [0, 11]. @todo Document which index generates which one.
		@param u A normalized value between [0,1]. This specifies the relative distance of the point along the edge.
		@see PointInside(), Edge(), CornerPoint(), FaceCenterPoint(), FacePoint(). */
	float3 PointOnEdge(int edgeIndex, float u) const;

	/// Returns the point at the center of the given face of this OBB.
	/** @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
		@todo Document which index generates which face.
		@see PointInside(), Edge(), CornerPoint(), PointOnEdge(), FacePoint(). */
	float3 FaceCenterPoint(int faceIndex) const;

	/// Generates a point at the surface of the given face of this OBB.
	/** @param faceIndex The index of the OBB face to generate the point at. The valid range is [0, 5].
		@param u A normalized value between [0, 1].
		@param v A normalized value between [0, 1].
		@todo Document which index generates which face.
		@see PointInside(), Edge(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePlane(). */
	float3 FacePoint(int faceIndex, float u, float v) const;

	/// Returns the plane of the given face of this OBB.
	/** The normal of the plane points outwards from this OBB, i.e. towards the space that
		is not part of the OBB.
		@param faceIndex The index of the face to get, in the range [0, 5].
		@see PointInside(), Edge(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	Plane FacePlane(int faceIndex) const;

	/// Fills an array with all the eight corner points of this OBB.
	/** @param outPointArray [out] The array to write the points to. Must have space for 8 elements.
		@see CornerPoint(), GetFacePlanes(). */
	void GetCornerPoints(float3 *outPointArray) const;

	/// Fills an array with all the six planes of this OBB.
	/** @param outPlaneArray [out] The array to write the planes to. Must have space for 6 elements.
		@see FacePlane(), GetCornerPoints(). */
	void GetFacePlanes(Plane *outPlaneArray) const;

	/// Finds the two extreme points along the given direction vector from the given point array.
	/** @param dir The direction vector to project the point array to. This vector does not need to be normalized.
		@param pointArray [in] The list of points to process.
		@param numPoints The number of elements in pointArray.
		@param idxSmallest [out] The index of the smallest point along the given direction will be received here.
			This pointer may be left null, if this information is of no interest.
		@param idxLargest [out] The index of the largest point along the given direction will be received here.
			This pointer may be left null, if this information is of no interest. */
	static void ExtremePointsAlongDirection(const float3 &dir, const float3 *pointArray, int numPoints, int &idxSmallest, int &idxLargest);

#if 0
	/// Generates an OBB that encloses the given point set.
	/** This function uses principal component analysis as the heuristics to generate the OBB. The returned OBB
		is not necessarily the optimal OBB that encloses the given point set.
		@param pointArray [in] The list of points to enclose with an OBB.
		@param numPoints The number of elements in the input array.
		@see SetFromApproximate(). */
	static OBB PCAEnclosingOBB(const float3 *pointArray, int numPoints);
#endif

#ifdef MATH_CONTAINERLIB_SUPPORT
	///\todo This function is strongly WIP! (Works, but is very very slow!)
	static OBB OptimalEnclosingOBB(const float3 *pointArray, int numPoints);
#endif

	/// Generates a random point inside this OBB.
	/** The points are distributed uniformly.
		@see class LCG, PointInside(), RandomPointOnSurface(), RandomPointOnEdge(), RandomCornerPoint(). */
	float3 RandomPointInside(LCG &rng) const;

	/// Generates a random point on a random face of this OBB.
	/** The points are distributed uniformly.
		@see class LCG, FacePoint(), RandomPointInside(), RandomPointOnEdge(), RandomCornerPoint(). */
	float3 RandomPointOnSurface(LCG &rng) const;

	/// Generates a random point on a random edge of this OBB.
	/** The points are distributed uniformly.
		@see class LCG, PointOnEdge(), RandomPointInside(), RandomPointOnSurface(), RandomCornerPoint(). */
	float3 RandomPointOnEdge(LCG &rng) const;

	/// Picks a random corner point of this OBB.
	/** The points are distributed uniformly.
		@see class LCG, CornerPoint(), RandomPointInside(), RandomPointOnSurface(), RandomPointOnEdge(). */
	float3 RandomCornerPoint(LCG &rng) const;

	/// Translates this OBB in world space.
	/** @param offset The amount of displacement to apply to this OBB, in world space coordinates.
		@see Scale(), Transform(). */
	void Translate(const float3 &offset);

	/// Applies a uniform scale to this OBB.
	/** This function scales this OBB structure in-place, using the given center point as the origin
		for the scaling operation.
		@param centerPoint Specifies the center of the scaling operation, in global (world) space.
		@param scaleFactor The uniform scale factor to apply to each global (world) space axis.
		@see Translate(), Transform(). */
	void Scale(const float3 &centerPoint, float scaleFactor);

	/// Applies a non-uniform scale to the local axes of this OBB.
	/** This function scales this OBB structure in-place, using the given global space center point as
		the origin for the scaling operation.
		@param centerPoint Specifies the center of the scaling operation, in global (world) space.
		@param scaleFactor The non-uniform scale factors to apply to each local axis of this OBB.
		@see Translate(), Transform(). */
	void Scale(const float3 &centerPoint, const float3 &scaleFactor);

	/// Applies a transformation to this OBB.
	/** @param transform The transformation to apply to this OBB. This transformation must be affine, and
			must contain an orthogonal set of column vectors (may not contain shear or projection).
			The transformation can only contain uniform scale, and may not contain mirroring.
		@see Translate(), Scale(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Computes the closest point inside this OBB to the given point.
	/** If the target point lies inside this OBB, then that point is returned.
		@see Distance(), Contains(), Intersects().
		@todo Add ClosestPoint(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	float3 ClosestPoint(const float3 &point) const;

	/// Computes the distance between this OBB and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@todo Add OBB::Distance(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Capsule/Frustum/Polyhedron).
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const float3 &point) const;
	float Distance(const Sphere &sphere) const;

	/// Tests if the given object is fully contained inside this OBB.
	/** This function returns true if the given object lies inside this OBB, and false otherwise.
		@note The comparison is performed using less-or-equal, so the faces of this OBB count as being inside, but
			due to float inaccuracies, this cannot generally be relied upon.
		@todo Add Contains(Circle/Disc/Sphere/Capsule).
		@see Distance(), Intersects(), ClosestPoint(). */
	bool Contains(const float3 &point) const;
	bool Contains(const LineSegment &lineSegment) const;
	bool Contains(const AABB &aabb) const;
	bool Contains(const OBB &obb) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;
	bool Contains(const Frustum &frustum) const;
	bool Contains(const Polyhedron &polyhedron) const;

	/// Tests whether this OBB and the given object intersect.
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this OBB,
		or this OBB is contained inside a Sphere, etc.)
		The first parameter of this function specifies the other object to test against.
		The OBB-OBB intersection test is from Christer Ericson's book Real-Time Collision Detection, p. 101-106.
		See http://realtimecollisiondetection.net/ [groupSyntax]
		@param obb The other oriented bounding box to test intersection with.
		@param epsilon The OBB-OBB test utilizes a SAT test to detect the intersection. A robust implementation requires
			an epsilon threshold to test that the used axes are not degenerate.
		@see Contains(), Distance(), ClosestPoint().
		@todo Add Intersects(Circle/Disc). */
	bool Intersects(const OBB &obb, float epsilon = 1e-3f) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const Plane &plane) const;
	/** @param dNear [out] If specified, receives the parametric distance along the line denoting where the
			line entered this OBB.
		@param dFar [out] If specified, receives the parametric distance along the line denoting where the
			line exited this OBB. */
	bool Intersects(const Ray &ray, float &dNear, float &dFar) const;
	bool Intersects(const Ray &ray) const;
	bool Intersects(const Line &line, float &dNear, float &dFar) const;
	bool Intersects(const Line &line) const;
	bool Intersects(const LineSegment &lineSegment, float &dNear, float &dFar) const;
	bool Intersects(const LineSegment &lineSegment) const;
	/** @param closestPointOnOBB [out] If specified, receives the closest point on this OBB To the given sphere. This
			pointer may be null. */
	bool Intersects(const Sphere &sphere, float3 *closestPointOnOBB = 0) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;

	/// Expands this OBB to enclose the given object. The axis directions of this OBB remain intact.
	/** This function operates in-place. This function does not necessarily result in an OBB that is an
		optimal fit for the previous OBB and the given point. */
	void Enclose(const float3 &point);

	/// Generates an unindexed triangle mesh representation of this OBB.
	/** @param numFacesX The number of faces to generate along the X axis. This value must be >= 1.
		@param numFacesY The number of faces to generate along the Y axis. This value must be >= 1.
		@param numFacesZ The number of faces to generate along the Z axis. This value must be >= 1.
		@param outPos [out] An array of size numVertices which will receive a triangle list
							of vertex positions. Cannot be null.
		@param outNormal [out] An array of size numVertices which will receive vertex normals.
							   If this parameter is null, vertex normals are not returned.
		@param outUV [out] An array of size numVertices which will receive vertex UV coordinates.
							   If this parameter is null, a UV mapping is not generated.
		The number of vertices that outPos, outNormal and outUV must be able to contain is
		(x*y + x*z + y*z)*2*6. If x==y==z==1, then a total of 36 vertices are required. Call
		NumVerticesInTriangulation to obtain this value.
		@see ToPolyhedron(), ToEdgeList(), NumVerticesInTriangulation(). */
	void Triangulate(int numFacesX, int numFacesY, int numFacesZ, float3 *outPos, float3 *outNormal, float2 *outUV, bool ccwIsFrontFacing) const;

	/// Returns the number of vertices that the Triangulate() function will output with the given subdivision parameters.
	/** @see Triangulate(). */
	static int NumVerticesInTriangulation(int numFacesX, int numFacesY, int numFacesZ)
	{
		return (numFacesX*numFacesY + numFacesX*numFacesZ + numFacesY*numFacesZ)*2*6;
	}

	/// Generates an edge list representation of the edges of this OBB.
	/** @param outPos [out] An array that contains space for at least 24 vertices (NumVerticesInEdgeList()).
		@see Triangulate(), Edge(), NumVerticesInEdgeList(). */
	void ToEdgeList(float3 *outPos) const;

	/// Returns the number of vertices that the ToEdgeList() function will output.
	/** @see ToEdgeList(). */
	static int NumVerticesInEdgeList()
	{
		return 4*3*2;
	}

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this OBB. Most useful for debugging purposes.
	/** The returned string specifies the center point and the half-axes of this OBB. */
	std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif

	// Finds the set intersection of this and the given OBB.
	/* @return This function returns the Polyhedron that is contained in both this and the given OBB. */
//	Polyhedron Intersection(const AABB &aabb) const;

	// Finds the set intersection of this and the given OBB.
	/* @return This function returns the Polyhedron that is contained in both this and the given OBB. */
//	Polyhedron Intersection(const OBB &obb) const;

	// Finds the set intersection of this OBB and the given Polyhedron.
	/* @return This function returns a Polyhedron that represents the set of points that are contained in this OBB
		and the given Polyhedron. */
//	Polyhedron Intersection(const Polyhedron &polyhedron) const;

#ifdef MATH_GRAPHICSENGINE_INTEROP
	void Triangulate(VertexBuffer &vb, int numFacesX, int numFacesY, int numFacesZ, bool ccwIsFrontFacing) const;
	void ToLineList(VertexBuffer &vb) const;
#endif
};

OBB operator *(const float3x3 &transform, const OBB &obb);
OBB operator *(const float3x4 &transform, const OBB &obb);
OBB operator *(const float4x4 &transform, const OBB &obb);
OBB operator *(const Quat &transform, const OBB &obb);

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(OBB)
Q_DECLARE_METATYPE(OBB*)
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const OBB &obb);
#endif

MATH_END_NAMESPACE
