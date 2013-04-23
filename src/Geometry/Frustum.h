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

/** @file Frustum.h
	@author Jukka Jylänki
	@brief The Frustum geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float2.h"
#include "../Math/float3.h"
#include "Ray.h"

#ifdef MATH_TINYXML_INTEROP
#include "Config/tinyxml/tinyxml.h"
#endif

MATH_BEGIN_NAMESPACE

enum FrustumType
{
	InvalidFrustum,
	OrthographicFrustum,
	PerspectiveFrustum
};

/// Represents either an orthographic or a perspective viewing frustum.
class Frustum
{
public:
	/// Specifies whether this frustum is a perspective or an orthographic frustum.
	/** [noscript] @todo Remove the noscript attribute. */
	FrustumType type;
	/// The eye point of this frustum.
	/** Specifies the position of the camera (the eye point) for this frustum in world (global) space. [similarOverload: type] */
	float3 pos;
	/// The normalized direction this frustum is watching towards. [similarOverload: type]
	/** This vector is specified in world (global) space. This vector is always normalized.
		If you assign to this member directly, be sure to only assign normalized vectors. */
	float3 front;
	/// The normalized up direction for this frustum. [similarOverload: type]
	/** This vector is specified in world (global) space. This vector is always normalized.
		If you assign to this member directly, be sure to only assign normalized vectors.
		@note The vectors front and up must always be orthogonal to each other. */
	float3 up;
	/// Distance from the eye point to the front plane. [similarOverload: type]
	/** This parameter must be positive. If perspective projection is used, this parameter must be strictly positive
		(0 is not allowed). If orthographic projection is used, 0 is possible (but uncommon, and not recommended).
		When using the Frustum class to derive perspective projection matrices for a GPU, it should be noted that too
		small values cause poor resolution of Z values near the back plane in post-perspective space, if non-linear
		depth is used (which is common). The larger this value is, the more resolution there is for the Z values across the
		depth range. Too large values cause clipping of geometry when they come very near the camera. */
	float nearPlaneDistance;
	/// Distance from the eye point to the back plane of the projection matrix. [similarOverload: type]
	/** This parameter must be strictly greater than nearPlaneDistance. The range [nearPlaneDistance, farPlaneDistance]
		specifies the visible range of objects inside the Frustum. When using the Frustum class for deriving perspective
		projection matrix for GPU rendering, it should be remembered that any geometry farther from the camera (in Z value)
		than this distance will be clipped from the view, and not rendered. */
	float farPlaneDistance;
	union
	{
		/// Horizontal field-of-view, in radians. This field is only valid if type == PerspectiveFrustum.
		/** @see type. */
		float horizontalFov;
		/// The width of the orthographic frustum. This field is only valid if type == OrthographicFrustum.
		/** @see type. */
		float orthographicWidth;
	};
	union
	{
		/// Vertical field-of-view, in radians. This field is only valid if type == PerspectiveFrustum.
		/** @see type. */
		float verticalFov;
		/// The height of the orthographic frustum. This field is only valid if type == OrthographicFrustum.
		/** @see type. */
		float orthographicHeight;
	};

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members type, pos, front, up, nearPlaneDistance, farPlaneDistance, horizontalFov/orthographicWidth and
		verticalFov/orthographicHeight are all undefined after creating a new Frustum using this
		default constructor. Remember to assign to them before use. [opaque-qtscript] @todo remove the opaque-qtscript attribute.
		@see type, pos, front, up, nearPlaneDistance, farPlaneDistance, horizontalFov, verticalFov, orthographicWidth, orthographicHeight. */
	Frustum() {}

	int NumEdges() const { return 12; }

	/// Returns the aspect ratio of the view rectangle on the near plane.
	/** The aspect ratio is the ratio of the width of the viewing rectangle to its height. This can also be computed by
		the expression horizontalFov / verticalFov. To produce a proper non-stretched image when rendering, this
		aspect ratio should match the aspect ratio of the actual render target (e.g. 4:3, 16:9 or 16:10 in full screen mode).
		@see horizontalFov, verticalFov. */
	float AspectRatio() const;

	/// Computes the direction vector that points logically to the right-hand side of the Frustum.
	/** This vector together with the member variables 'front' and 'up' form the orthonormal basis of the view frustum.
		@see pos, front. */
	float3 WorldRight() const
	{
		return Cross(front, up);
	}

	/// Computes the plane equation of the near plane of this Frustum.
	/** The normal vector of the returned plane points outwards from the volume inside the frustum, i.e. towards the eye point
		(towards -front).
		@see front, FarPlane(), LeftPlane(), RightPlane(), TopPlane(), BottomPlane(), GetPlane(), GetPlanes(). */
	Plane NearPlane() const;

	/// Computes the width of the near plane quad in world space units.
	/** @see NearPlaneHeight(). */
	float NearPlaneWidth() const;

	/// Computes the height of the near plane quad in world space units.
	/** @see NearPlaneHeight(). */
	float NearPlaneHeight() const;

	/// Computes the plane equation of the far plane of this Frustum. [similarOverload: NearPlane]
	/** The normal vector of the returned plane points outwards from the volume inside the frustum, i.e. away from the eye point.
		(towards front).
		@see front, FarPlane(), LeftPlane(), RightPlane(), TopPlane(), BottomPlane(), GetPlane(), GetPlanes(). */
	Plane FarPlane() const;

	/// Returns the plane equation of the specified side of this Frustum.
	/** The normal vector of the returned plane points outwards from the volume inside the frustum.
		[indexTitle: Left/Right/Top/BottomPlane]
		@see NearPlane(), FarPlane(), GetPlane(), GetPlanes(). */
	Plane LeftPlane() const;
	Plane RightPlane() const; ///< [similarOverload: LeftPlane] [hideIndex]
	Plane TopPlane() const; ///< [similarOverload: LeftPlane] [hideIndex]
	Plane BottomPlane() const; ///< [similarOverload: LeftPlane] [hideIndex]

	/// Returns the specified plane of this frustum.
	/** The normal vector of the returned plane points outwards from the volume inside the frustum.
		@param faceIndex A number in the range [0,5], which returns the plane at the selected index from
			the array { near, far, left, right, top, bottom }.
		@see GetPlanes(), NearPlane(), FarPlane(), LeftPlane(), RightPlane(), TopPlane(), BottomPlane(). */
	Plane GetPlane(int faceIndex) const;

	/// Returns all six planes of this Frustum.
	/** The planes will be output in the order { near, far, left, right, top, bottom }.
		@param outArray [out] A pointer to an array of at least 6 elements. This pointer will receive the planes of this Frustum.
			This pointer may not be null.
		@see GetPlane(), NearPlane(), FarPlane(), LeftPlane(), RightPlane(), TopPlane(), BottomPlane(). */
	void GetPlanes(Plane *outArray) const;

	float3 CenterPoint() const;

	/// Returns an edge of this Frustum.
	/** @param edgeIndex The index of the edge line segment to get, in the range [0, 11].
		@todo Draw a diagram that shows which index generates which edge.
		@see PointInside(), CornerPoint(), PointOnEdge(), FaceCenterPoint(), FacePoint(). */
	LineSegment Edge(int edgeIndex) const;

	/// Generates one of the eight corner points of this Frustum.
	/** @param cornerIndex The index of the corner point to generate, in the range [0, 7].
		 The points are returned in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
		 (corresponding the XYZ axis directions). */
	float3 CornerPoint(int cornerIndex) const;

	/// Returns all eight corner points of this array.
	/** @param outPointArray [out] A pointer to an array of at least 8 elements. This pointer will receive the corner vertices
			of this Frustum. This pointer may not be null. */
	void GetCornerPoints(float3 *outPointArray) const;

	/// Computes an extreme point of this Frustum in the given direction.
	/** An extreme point is a farthest point of this Frustum in the given direction. Given a direction,
		this point is not necessarily unique.
		@param direction The direction vector of the direction to find the extreme point. This vector may
			be unnormalized, but may not be null.
		@return An extreme point of this Frustum in the given direction. The returned point is always a
			corner point of this Frustum.
		@see CornerPoint(). */
	float3 ExtremePoint(const float3 &direction) const;

	/// Projects this Frustum onto the given 1D axis direction vector.
	/** This function collapses this Frustum onto an 1D axis for the purposes of e.g. separate axis test computations.
		The function returns a 1D range [outMin, outMax] denoting the interval of the projection.
		@param direction The 1D axis to project to. This vector may be unnormalized, in which case the output
			of this function gets scaled by the length of this vector.
		@param outMin [out] Returns the minimum extent of this object along the projection axis.
		@param outMax [out] Returns the maximum extent of this object along the projection axis. */
	void ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const;

	/// Sets the pos, front and up members of this frustum from the given world transform.
	/** This function sets the 'front' parameter of this Frustum to look towards the -Z axis of the given matrix,
		and the 'up' parameter of this Frustum to point towards the +Y axis of the given matrix.
		@param worldTransform An orthonormalized matrix with determinant of +1 (no mirroring). */
	void SetWorldMatrix(const float3x4 &worldTransform);

	/// Computes the matrix that transforms from the view space to the world (global) space of this Frustum.
	/** @note The returned matrix is the inverse of the matrix returned by ViewMatrix().
		@return An orthonormal affine matrix that performs the view->world transformation. The returned
			matrix is built to use the convention Matrix * vector to map a point between these spaces.
			(as opposed to the convention v*M).
		@see ViewMatrix(), ProjectionMatrix(), ViewProjMatrix(). */
	float3x4 WorldMatrix() const;

	/// Computes the matrix that transforms from the world (global) space to the view space of this Frustum.
	/** @note The returned matrix is the inverse of the matrix returned by WorldMatrix().
		@return An orthonormal affine matrix that performs the world->view transformation. The returned
			matrix is built to use the convention Matrix * vector to map a point between these spaces.
			(as opposed to the convention v*M).
		@see WorldMatrix(), ProjectionMatrix(), ViewProjMatrix(). */
	float3x4 ViewMatrix() const;

	/// Computes the matrix that projects from the view space to the projection space of this Frustum.
	/** @return A projection matrix that performs the view->proj transformation. This matrix is neither
			invertible or orthonormal. The returned matrix is built to use the convention Matrix * vector
			to map a point between these spaces. (as opposed to the convention v*M).
		@see WorldMatrix(), ViewMatrix(), ViewProjMatrix(). */
	float4x4 ProjectionMatrix() const;

	/// Computes the matrix that transforms from the world (global) space to the projection space of this Frustum.
	/** The matrix computed by this function is simply the concatenation ProjectionMatrix()*ViewMatrix(). This order
		of concatenation follows the M*v convention of transforming vectors (as opposed to the v*M convention). This
		multiplication order is used, since the matrices ProjectionMatrix() and ViewMatrix() also follow the M*v convention.
		@return A matrix that performs the world->view->proj transformation. This matrix is neither invertible or
			orthonormal. The returned matrix is built to use the convention Matrix * vector
			to map a point between these spaces. (as opposed to the convention v*M).
		@see WorldMatrix(), ViewMatrix(), ProjectionMatrix(). */
	float4x4 ViewProjMatrix() const;

	/// Finds a ray in world space that originates at the eye point and looks in the given direction inside the frustum.
	/** The (x,y) coordinate specifies the normalized viewport coordinate through which the ray passes.
		Both x and y must be in the range [-1,1].
		Specifying (-1, -1) returns the bottom-left corner of the near plane.
		The point (1, 1) corresponds to the top-right corner of the near plane. */
	Ray UnProject(float x, float y) const;
	Ray UnProject(const float2 &xy) const { return UnProject(xy.x, xy.y); }

	///\todo Add float3 UnProject(const float3 &point) const;
	/** Like UnProject, but if the frustum type is PerspectiveFrustum, the ray originates at the near plane,
		and not at the camera eye point. For orthographic frustum, LookAt and LookAtFromNearPlane are identical
		(always originates at near plane). */
	Ray UnProjectFromNearPlane(float x, float y) const;

	/// Returns the world-space line segment of the points that project to the given normalized viewport coordinate (x,y).
	/** The (x,y) coordinate specifies the normalized viewport coordinate through which the line segment passes.
		Both x and y must be in the range [-1,1]. */
	LineSegment UnProjectLineSegment(float x, float y) const;

	/// Returns a point inside this frustum parameterized by three scalar coordinates.
	/** @param x The horizontal normalized viewport coordinate in the range [-1, 1].
		@param y The vertical normalized viewport coordinate in the range [-1, 1].
		@param z The linear depth coordinate in the range [0, 1].
		@note This function is slightly different than multiplying by inv(view*proj), since depth is handled linearly.
		@see FastRandomPointInside(), UniformRandomPointInside(). */
	float3 PointInside(float x, float y, float z) const;
	float3 PointInside(const float3 &xyz) const { return PointInside(xyz.x, xyz.y, xyz.z); }

	/// Projects the given point onto the near plane of this frustum.
	/** The (x,y) component of the returned float3 gives the normalized viewport coordinates of the point on the
		near plane. The z component gives the normalized depth of the point.
		If the point is inside the frustum, x and y are in the range [-1, 1] and z is in the range [0, 1]. If the point
		was behind the near plane, z will return a negative value. If the point lies exactly on the near plane, z==0
		will be returned. If the point lies exactly on the far plane, z==1 will be returned, and if a z>1 is returned,
		the given point was outside the far plane of this Frustum.
		@param point A point in world space to project onto the near plane of this frustum.
		@return The normalized 2D (x,y) coordinate of the given point projected onto the near plane of this Frustum.
			The z coordinate specifies the normalized (linear) depth of the projected point. */
	float3 Project(const float3 &point) const;

	/// Returns a point on the near plane.
	/** @param x A value in the range [-1, 1].
		@param y A value in the range [-1, 1].
		Specifying (-1, -1) returns the bottom-left corner of the near plane.
		The point (1, 1) corresponds to the top-right corner of the near plane.
		@note This coordinate space is called the normalized viewport coordinate space.
		@see FarPlanePos(). */
	float3 NearPlanePos(float x, float y) const;
	float3 NearPlanePos(const float2 &point) const;

	/// Returns a point on the far plane.
	/** @param x A value in the range [-1, 1].
		@param y A value in the range [-1, 1].
		Specifying (-1, -1) returns the bottom-left corner of the far plane.
		The point (1, 1) corresponds to the top-right corner of the far plane.
		@note This coordinate space is called the normalized viewport coordinate space.
		@see NearPlanePos(). */
	float3 FarPlanePos(float x, float y) const;
	float3 FarPlanePos(const float2 &point) const;

	/// Maps a point from the normalized viewport space to the screen space.
	/** In normalized viewport space, top-left: (-1, 1), top-right: (1, 1), bottom-left: (-1, -1), bottom-right: (-1, 1).
		In screen space, top-left: (0, 0), top-right: (0, screenWidth-1), bottom-left: (0, screenHeight-1), bottom-right: (screenWidth-1, screenHeight-1).
		This mapping is affine.
		@see ScreenToViewportSpace(). */
	static float2 ViewportToScreenSpace(float x, float y, int screenWidth, int screenHeight);
	static float2 ViewportToScreenSpace(const float2 &point, int screenWidth, int screenHeight);

	/// Maps a point from screen space to normalized viewport space.
	/** This function computes the inverse function of ViewportToScreenSpace(). This mapping is affine.
		@see ViewportToScreenSpace(). */
	static float2 ScreenToViewportSpace(float x, float y, int screenWidth, int screenHeight);
	static float2 ScreenToViewportSpace(const float2 &point, int screenWidth, int screenHeight);

	/// Tests if this Frustum is finite.
	/** A Frustum is <b><i>finite</i></b> if none of its member variables contain floating-point NaNs or +/-infs
		in them.
		@return True if each member variable has a finite floating-point value.
		@see type, pos, front, up, nearPlaneDistance, farPlaneDistance, horizontalFov, verticalFov, orthographicWidth, orthographicHeight.
		@todo Implement IsDegenerate(). */
	bool IsFinite() const;

	/// Computes the volume of this Frustum.
	float Volume() const;

	/// Quickly generates a random point inside this Frustum.
	/** If the frustum type is orthographic, then the points are uniformly distributed. If the frustum type is perspective, then not.
		@see class LCG, UniformRandomPointInside(), PointInside(). */
	float3 FastRandomPointInside(LCG &rng) const;

	/// Generates a uniformly random point inside this Frustum.
	/** For orthographic frustum type, this function is identical to FastRandomPointInside.
		@see class LCG, FastRandomPointInside(), PointInside(). */
	float3 UniformRandomPointInside(LCG &rng) const;

	/// Moves this Frustum by the given offset vector.
	/** @note This function operates in-place.
		@param offset The world space offset to apply to the position of this Frustum.
		@see Transform(). */
	void Translate(const float3 &offset);

	/// Applies a transformation to this Frustum.
	/** @param transform The transformation to apply to this Frustum. This transformation must be
		affine, and must contain an orthogonal set of column vectors (may not contain shear or projection).
		The transformation can only contain uniform scale, and may not contain mirroring.
		@see Translate(), Scale(), classes float3x3, float3x4, float4x4, Quat. */
	void Transform(const float3x3 &transform);
	void Transform(const float3x4 &transform);
	void Transform(const float4x4 &transform);
	void Transform(const Quat &transform);

	/// Returns the tightest AABB that contains this Frustum.
	/** This function computes the optimal minimum volume AABB that encloses this Frustum.
		@note Since an AABB cannot generally represent a Frustum, this conversion is not exact, but the returned AABB
			specifies a larger volume.			
		@see MinimalEnclosingOBB(), ToPolyhedron(). */
	AABB MinimalEnclosingAABB() const;

	/// Returns the tightest OBB that encloses this Frustum.
	/** This function computes the optimal minimum volume OBB that encloses this Frustum.
		@note If the type of this frustum is Perspective, this conversion is not exact, but the returned OBB specifies
			a larger volume. If the type of this Frustum is orthographic, this conversion is exact, since the shape of an
			orthographic Frustum is an OBB.
		@see MinimalEnclosingAABB(), ToPolyhedron(). */
	OBB MinimalEnclosingOBB() const;

	/// Converts this Frustum to a Polyhedron.
	/** This function returns a Polyhedron representation of this Frustum. This conversion is exact, meaning that the returned
		Polyhedron represents exactly the same set of points than this Frustum.
		@see MinimalEnclosingAABB(), MinimalEnclosingOBB(). */
	Polyhedron ToPolyhedron() const;

	/// Tests if the given object is fully contained inside this Frustum.
	/** This function returns true if the given object lies inside this Frustum, and false otherwise.
		@note The comparison is performed using less-or-equal, so the faces of this Frustum count as being inside, but
			due to float inaccuracies, this cannot generally be relied upon.
		@todo Add Contains(Circle/Disc/Sphere/Capsule).
		@see Distance(), Intersects(), ClosestPoint(). */
	bool Contains(const float3 &point) const;
	bool Contains(const LineSegment &lineSegment) const;
	bool Contains(const Triangle &triangle) const;
	bool Contains(const Polygon &polygon) const;
	bool Contains(const AABB &aabb) const;
	bool Contains(const OBB &obb) const;
	bool Contains(const Frustum &frustum) const;
	bool Contains(const Polyhedron &polyhedron) const;

	/// Computes the closest point inside this Frustum to the given point.
	/** If the target point lies inside this Frustum, then that point is returned.
		@see Distance(), Contains(), Intersects().
		@todo Add ClosestPoint(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Sphere/Capsule/Frustum/Polyhedron). */
	float3 ClosestPoint(const float3 &point) const;

	/// Computes the distance between this Frustum and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@todo Add Frustum::Distance(Line/Ray/LineSegment/Plane/Triangle/Polygon/Circle/Disc/AABB/OBB/Capsule/Frustum/Polyhedron).
		@see Contains(), Intersects(), ClosestPoint(). */
	float Distance(const float3 &point) const;

	/// Tests whether this Frustum and the given object intersect.	
	/** Both objects are treated as "solid", meaning that if one of the objects is fully contained inside
		another, this function still returns true. (e.g. in case a line segment is contained inside this Frustum,
		or this Frustum is contained inside a Sphere, etc.)
		The first parameter of this function specifies the other object to test against.
		@see Contains(), Distance(), ClosestPoint().
		@todo Add Intersects(Circle/Disc). */
	bool Intersects(const Ray &ray) const;
	bool Intersects(const Line &line) const;
	bool Intersects(const LineSegment &lineSegment) const;
	bool Intersects(const AABB &aabb) const;
	bool Intersects(const OBB &obb) const;
	bool Intersects(const Plane &plane) const;
	bool Intersects(const Triangle &triangle) const;
	bool Intersects(const Polygon &polygon) const;
	bool Intersects(const Sphere &sphere) const;
	bool Intersects(const Capsule &capsule) const;
	bool Intersects(const Frustum &frustum) const;
	bool Intersects(const Polyhedron &polyhedron) const;

#if defined(MATH_TINYXML_INTEROP) && defined(MATH_CONTAINERLIB_SUPPORT)
	void DeserializeFromXml(TiXmlElement *e);
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns a human-readable representation of this Frustum. Most useful for debugging purposes.
	std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
	operator QString() const { return toString(); }
	QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

Frustum operator *(const float3x3 &transform, const Frustum &frustum);
Frustum operator *(const float3x4 &transform, const Frustum &frustum);
Frustum operator *(const float4x4 &transform, const Frustum &frustum);
Frustum operator *(const Quat &transform, const Frustum &frustum);

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Frustum)
Q_DECLARE_METATYPE(Frustum*)
#endif

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &o, const Frustum &frustum);
#endif

MATH_END_NAMESPACE
