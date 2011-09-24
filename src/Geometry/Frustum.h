/** @file Frustum.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

enum FrustumType
{
	InvalidFrustum,
	OrthographicFrustum,
	PerspectiveFrustum
};

/// Represents either an orthographic or perspective view frustum.
class Frustum
{
public:
    Frustum() {}

	/// Specifies whether this frustum is a perspective or an orthographic frustum.
	FrustumType type;
    /// The eye point of this frustum.
    float3 pos;
    /// The normalized look-at direction this frustum is watching towards.
    float3 front;
    /// The normalized up direction for this frustum.
    float3 up;
    /// Distance from the eye point to the front plane.
    float nearPlaneDistance;
    /// Distance from the eye point to the back plane.
    float farPlaneDistance;
	union
	{
	    /// Horizontal field-of-view, in radians. This field is only valid if type == PerspectiveFrustum.
		float horizontalFov;
		/// The width of the orthographic frustum. This field is only valid if type == OrthographicFrustum.
		float orthographicWidth;
	};
	union
	{
	    /// Vertical field-of-view, in radians. This field is only valid if type == PerspectiveFrustum.
		float verticalFov;
		/// The height of the orthographic frustum. This field is only valid if type == OrthographicFrustum.
		float orthographicHeight;
	};

    /// Returns the aspect ratio of the view rectangle on the near plane.
    float AspectRatio() const;
    Plane NearPlane() const;
    Plane FarPlane() const;
    Plane LeftPlane() const;
    Plane RightPlane() const;
    Plane TopPlane() const;
    Plane BottomPlane() const;

    /// Returns the view->world matrix of this Frustum.
    float3x4 WorldMatrix() const;

    /// Returns the world->view matrix of this Frustum.
    float3x4 ViewMatrix() const;

    /// Returns the view->proj matrix of this Frustum.
    float4x4 ProjectionMatrix() const;

    /// Returns the world->view->proj matrix of this Frustum.
    /// The matrix returned by this function is simply the concatenation ProjectionMatrix()*ViewMatrix().
    float4x4 ViewProjMatrix() const;

    /// Finds a ray in world space that originates at the eye point and looks in the given direction inside the frustum.
    /// The (x,y) coordinate specifies the normalized viewport coordinate through which the ray passes.
    /// Both x and y must be in the range [-1,1].
	/// Specifying (-1, -1) returns the bottom-left corner of the near plane.
	/// The point (1, 1) corresponds to the top-right corner of the near plane.
    Ray LookAt(float x, float y) const;

    /// Like LookAt, but if the frustum type is PerspectiveFrustum, the ray originates at the near plane, 
    /// and not at the camera eye point. For orthographic frustum, LookAt and LookAtFromNearPlane are identical
    /// (always originates at near plane).
    Ray LookAtFromNearPlane(float x, float y) const;

    /// Projects the given point onto the near plane of this frustum.
    /// The (x,y) component of the returned float3 gives the normalized viewport coordinates of the point on the
    /// near plane. The z component gives the normalized depth of the point.
    /// If the point is inside the frustum, x and y are in the range [-1, 1] and z is in the range [0, 1]. If the point
    /// was behind the near plane, z will return a negative value. If the point lies exactly on the near plane, z==0
    /// will be returned. If the point lies exactly on the far plane, z==1 will be returned, and if a z>1 is returned,
    /// the given point was outside the far plane of this Frustum.
    float3 Project(const float3 &point) const;

	/// Returns a point on the near plane.
	/// @param x A value in the range [-1, 1].
	/// @param y A value in the range [-1, 1].
	/// Specifying (-1, -1) returns the bottom-left corner of the near plane.
	/// The point (1, 1) corresponds to the top-right corner of the near plane.
	/// @note This coordinate space is called the normalized viewport coordinate space.
    float3 NearPlanePos(float x, float y) const;
    float3 NearPlanePos(const float2 &point) const;

	/// Returns a point on the far plane.
	/// @param x A value in the range [-1, 1].
	/// @param y A value in the range [-1, 1].
	/// Specifying (-1, -1) returns the bottom-left corner of the far plane.
	/// The point (1, 1) corresponds to the top-right corner of the far plane.
	/// @note This coordinate space is called the normalized viewport coordinate space.
    float3 FarPlanePos(float x, float y) const;
    float3 FarPlanePos(const float2 &point) const;

	/// A helper function that maps a point from the normalized viewport space to the screen space.
	/// In normalized viewport space, top-left: (-1, 1), top-right: (1, 1), bottom-left: (-1, -1), bottom-right: (-1, 1).
	/// In screen space, top-left: (0, 0), top-right: (0, screenWidth-1), bottom-left: (0, screenHeight-1), bottom-right: (screenWidth-1, screenHeight-1).
	/// (This mapping is affine).
	static float2 ViewportToScreenSpace(float x, float y, int screenWidth, int screenHeight);
	static float2 ViewportToScreenSpace(const float2 &point, int screenWidth, int screenHeight);

	/// This function computes the inverse mapping of ViewportToScreenSpace.
	/// (This mapping is affine).
	static float2 ScreenToViewportSpace(float x, float y, int screenWidth, int screenHeight);
	static float2 ScreenToViewportSpace(const float2 &point, int screenWidth, int screenHeight);

    /// Returns true if the elements in this data structure contain valid finite float values.
    bool IsFinite() const;

    bool IsDegenerate() const;

    /// Returns the specified plane of this frustum.
    /// @param faceIndex A number in the range [0,5], which returns the plane at the selected index from 
    ///                  the array { near, far, left, right, top, bottom }.
    Plane GetPlane(int faceIndex) const;

    float Volume() const;

    float3 RandomPointInside(LCG &rng) const;

    /// Translates this Frustum by the given amount.
    void Translate(const float3 &offset);

    /// Scales this Frustum by with respect to the given center point.
    void Scale(const float3 &centerPoint, float uniformScaleFactor);
    void Scale(const float3 &centerPoint, const float3 &nonuniformScaleFactors);

    /// Transforms this Frustum.
    void Transform(const float3x3 &transform);
    void Transform(const float3x4 &transform);
    void Transform(const float4x4 &transform);
    void Transform(const Quat &transform);

    /// Returns all the frustum planes in the order { near, far, left, right, top, bottom }.
    void GetPlanes(Plane *outArray) const;

    void GetCornerPoints(float3 *outPointArray) const;

    float3 CornerPoint(int cornerIndex) const;

    /// Returns the minimal enclosing AABB of this frustum.
    AABB ToAABB() const;

    /// Returns an OBB that encloses this frustum.
    OBB ToOBB() const;

    /// Returns true if the given point is contained inside this frustum.
    bool Contains(const float3 &point) const;
    bool Contains(const LineSegment &lineSegment) const;
    bool Contains(const Triangle &triangle) const;

    /// Returns an exact polyhedron representation of this frustum.
//    Polyhedron ToPolyhedron() const;

    /// \todo Instead of returning a bool, return a value that specifies if the object lies completely
    ///       on the negative or positive halfspace.
    bool Intersects(const Ray &ray, float &outDistance) const;
    bool Intersects(const Line &line, float &outDistance) const;
    bool Intersects(const LineSegment &lineSegment, float &outDistance) const;
    bool Intersects(const AABB &aabb) const;
    bool Intersects(const OBB &obb) const;
    bool Intersects(const Plane &plane) const;
    bool Intersects(const Sphere &sphere) const;
    bool Intersects(const Ellipsoid &ellipsoid) const;
    bool Intersects(const Triangle &triangle) const;
    bool Intersects(const Cylinder &cylinder) const;
    bool Intersects(const Torus &torus) const;
    bool Intersects(const Frustum &frustum) const;
    bool Intersects(const Polyhedron &polyhedron) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this Frustum. Most useful for debugging purposes.
    std::string ToString() const;
#endif
#ifdef QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Frustum)
Q_DECLARE_METATYPE(Frustum*)
#endif
