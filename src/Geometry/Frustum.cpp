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

/** @file Frustum.cpp
	@author Jukka Jylänki
	@brief Implementation for the Frustum geometry object. */
#include "Frustum.h"
#include "AABB.h"
#include "Circle.h"
#include "../Math/MathFunc.h"
#include "Plane.h"
#include "Line.h"
#include "OBB.h"
#include "Polyhedron.h"
#include "Polygon.h"
#include "Ray.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Triangle.h"
#include "LineSegment.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4x4.h"
#include "../Math/float4.h"
#include "../Math/Quat.h"
#include "../Algorithm/Random/LCG.h"
#include "../Algorithm/GJK.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#if defined(MATH_TINYXML_INTEROP) && defined(MATH_CONTAINERLIB_SUPPORT)
#include "Container/UString.h"
#endif

MATH_BEGIN_NAMESPACE

Frustum::Frustum()
{
	// For conveniency, allow automatic initialization of the graphics API and handedness in use.
	// If neither of the #defines are set, user must specify per-instance.

#ifdef MATH_USE_DIRECT3D
	projectiveSpace = FrustumSpaceD3D;
#elif defined(MATH_USE_OPENGL)
	projectiveSpace = FrustumSpaceGL;
#endif

#ifdef MATH_LEFTHANDED_CAMERA
	handedness = FrustumLeftHanded;
#elif defined(MATH_RIGHTHANDED_CAMERA)
	handedness = FrustumRightHanded;
#endif
}

float Frustum::AspectRatio() const
{
	return Tan(horizontalFov*0.5f) / Tan(verticalFov*0.5f);
}

void Frustum::SetHorizontalFovAndAspectRatio(float hFov, float aspectRatio)
{
	horizontalFov = hFov;
	verticalFov = 2.f * Atan(Tan(hFov*0.5f)/aspectRatio);
}

void Frustum::SetVerticalFovAndAspectRatio(float vFov, float aspectRatio)
{
	verticalFov = vFov;
	horizontalFov = 2.f * Atan(Tan(vFov*0.5f)*aspectRatio);
}

vec Frustum::WorldRight() const
{
	if (handedness == FrustumRightHanded)
		return Cross(front, up);
	else
		return Cross(up, front);
}

float Frustum::NearPlaneWidth() const
{
	if (type == PerspectiveFrustum)
		return Tan(horizontalFov*0.5f)*2.f * nearPlaneDistance;
	else
		return orthographicWidth;
}

float Frustum::NearPlaneHeight() const
{
	if (type == PerspectiveFrustum)
		return Tan(verticalFov*0.5f)*2.f * nearPlaneDistance;
	else
		return orthographicHeight;
}

Plane Frustum::NearPlane() const
{
	return Plane(pos + front * nearPlaneDistance, -front);
}

Plane Frustum::FarPlane() const
{
	return Plane(pos + front * farPlaneDistance, front);
}

Plane Frustum::LeftPlane() const
{
	if (type == PerspectiveFrustum)
	{
		vec left = -WorldRight();
		left.ScaleToLength(Tan(horizontalFov*0.5f));
		vec leftSide = front + left;
		vec leftSideNormal = ((handedness == FrustumRightHanded) ? Cross(up, leftSide) : Cross(leftSide, up)).Normalized();
		return Plane(pos, leftSideNormal);
	}
	else
	{
		vec left = -WorldRight();
		return Plane(NearPlanePos(-1.f, 0.f), left.Normalized());
	}
}

Plane Frustum::RightPlane() const
{
	if (type == PerspectiveFrustum)
	{
		vec right = WorldRight();
		right.ScaleToLength(Tan(horizontalFov*0.5f));
		vec rightSide = front + right;
		vec rightSideNormal = ((handedness == FrustumRightHanded) ? Cross(rightSide, up) : Cross(up, rightSide)).Normalized();
		return Plane(pos, rightSideNormal);
	}
	else
	{
		vec right = WorldRight();
		return Plane(NearPlanePos(1.f, 0.f), right.Normalized());
	}
}

Plane Frustum::TopPlane() const
{
	if (type == PerspectiveFrustum)
	{
		vec topSide = front + Tan(verticalFov * 0.5f) * up;
		vec right = WorldRight();
		vec topSideNormal = ((handedness == FrustumRightHanded) ? Cross(right, topSide) : Cross(topSide, right)).Normalized();
		return Plane(pos, topSideNormal);
	}
	else
	{
		return Plane(NearPlanePos(0.f, 1.f), up);
	}
}

Plane Frustum::BottomPlane() const
{
	if (type == PerspectiveFrustum)
	{
		vec bottomSide = front - Tan(verticalFov * 0.5f) * up;
		vec left = -WorldRight();
		vec bottomSideNormal = ((handedness == FrustumRightHanded) ? Cross(left, bottomSide) : Cross(bottomSide, left)).Normalized();
		return Plane(pos, bottomSideNormal);
	}
	else
	{
		return Plane(NearPlanePos(0.f, -1.f), -up);
	}
}

void Frustum::SetWorldMatrix(const float3x4 &worldTransform)
{
	pos = POINT_VEC(worldTransform.TranslatePart());
	if (handedness == FrustumRightHanded)
		front = -DIR_VEC(worldTransform.Col(2)); // The camera looks towards -Z axis of the given transform.
	else
		front = DIR_VEC(worldTransform.Col(2)); // The camera looks towards +Z axis of the given transform.
	up = DIR_VEC(worldTransform.Col(1)); // The camera up points towards +Y of the given transform.
	assume(pos.IsFinite());
	assume(front.IsNormalized());
	assume(up.IsNormalized());
	assume(worldTransform.IsColOrthogonal3()); // Front and up must be orthogonal to each other.
	assume(EqualAbs(worldTransform.Determinant(), 1.f)); // The matrix cannot contain mirroring.
}

float3x4 Frustum::WorldMatrix() const
{
	assume(up.IsNormalized());
	assume(front.IsNormalized());
	float3x4 m;
	m.SetCol(0, DIR_TO_FLOAT3(WorldRight().Normalized()));
	m.SetCol(1, DIR_TO_FLOAT3(up));
	if (handedness == FrustumRightHanded)
		m.SetCol(2, DIR_TO_FLOAT3(-front)); // In right-handed convention, the -Z axis must map towards the front vector. (so +Z maps to -front)
	else
		m.SetCol(2, DIR_TO_FLOAT3(front)); // In left-handed convention, the +Z axis must map towards the front vector.
	m.SetCol(3, POINT_TO_FLOAT3(pos));
	assume(!m.HasNegativeScale());
	return m;
}

float3x4 Frustum::ViewMatrix() const
{
	float3x4 world = WorldMatrix();
	world.InverseOrthonormal();
	return world;
}

float4x4 Frustum::ViewProjMatrix() const
{
	return ProjectionMatrix() * ViewMatrix();
}

float4x4 Frustum::ProjectionMatrix() const
{
	assume(type == PerspectiveFrustum || type == OrthographicFrustum);
	assume(projectiveSpace == FrustumSpaceGL || projectiveSpace == FrustumSpaceD3D);
	assume(handedness == FrustumLeftHanded || handedness == FrustumRightHanded);
	if (type == PerspectiveFrustum)
	{
		if (projectiveSpace == FrustumSpaceGL)
		{
			if (handedness == FrustumRightHanded)
				return float4x4::OpenGLPerspProjRH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
			else if (handedness == FrustumLeftHanded)
				return float4x4::OpenGLPerspProjLH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
		}
		else if (projectiveSpace == FrustumSpaceD3D)
		{
			if (handedness == FrustumRightHanded)
				return float4x4::D3DPerspProjRH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
			else if (handedness == FrustumLeftHanded)
				return float4x4::D3DPerspProjLH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
		}
	}
	else if (type == OrthographicFrustum)
	{
		if (projectiveSpace == FrustumSpaceGL)
		{
			if (handedness == FrustumRightHanded)
				return float4x4::OpenGLOrthoProjRH(nearPlaneDistance, farPlaneDistance, orthographicWidth, orthographicHeight);
			else if (handedness == FrustumLeftHanded)
				return float4x4::OpenGLOrthoProjLH(nearPlaneDistance, farPlaneDistance, orthographicWidth, orthographicHeight);
		}
		else if (projectiveSpace == FrustumSpaceD3D)
		{
			if (handedness == FrustumRightHanded)
				return float4x4::D3DOrthoProjRH(nearPlaneDistance, farPlaneDistance, orthographicWidth, orthographicHeight);
			else if (handedness == FrustumLeftHanded)
				return float4x4::D3DOrthoProjLH(nearPlaneDistance, farPlaneDistance, orthographicWidth, orthographicHeight);
		}
	}
#ifndef OPTIMIZED_RELEASE
	LOGE("Not all values of Frustum were initialized properly! Please initialize correctly before calling Frustum::ProjectionMatrix()!");
#endif
	return float4x4::nan;
}

vec Frustum::NearPlanePos(float x, float y) const
{
	assume(type == PerspectiveFrustum || type == OrthographicFrustum);

	if (type == PerspectiveFrustum)
	{
		float frontPlaneHalfWidth = Tan(horizontalFov*0.5f)*nearPlaneDistance;
		float frontPlaneHalfHeight = Tan(verticalFov*0.5f)*nearPlaneDistance;
		x = x * frontPlaneHalfWidth; // Map [-1,1] to [-width/2, width/2].
		y = y * frontPlaneHalfHeight;  // Map [-1,1] to [-height/2, height/2].
		vec right = WorldRight();
		return pos + front * nearPlaneDistance + x * right + y * up;
	}
	else
	{
		vec right = WorldRight();
		return pos + front * nearPlaneDistance
				   + x * orthographicWidth * 0.5f * right
				   + y * orthographicHeight * 0.5f * up;
	}
}

vec Frustum::NearPlanePos(const float2 &point) const
{
	return NearPlanePos(point.x, point.y);
}

vec Frustum::FarPlanePos(float x, float y) const
{
	assume(type == PerspectiveFrustum || type == OrthographicFrustum);

	if (type == PerspectiveFrustum)
	{
		float farPlaneHalfWidth = Tan(horizontalFov*0.5f)*farPlaneDistance;
		float farPlaneHalfHeight = Tan(verticalFov*0.5f)*farPlaneDistance;
		x = x * farPlaneHalfWidth;
		y = y * farPlaneHalfHeight;
		vec right = WorldRight();
		return pos + front * farPlaneDistance + x * right + y * up;
	}
	else
	{
		vec right = WorldRight();
		return pos + front * farPlaneDistance
				   + x * orthographicWidth * 0.5f * right
				   + y * orthographicHeight * 0.5f * up;
	}
}

vec Frustum::FarPlanePos(const float2 &point) const
{
	return FarPlanePos(point.x, point.y);
}

float2 Frustum::ViewportToScreenSpace(float x, float y, int screenWidth, int screenHeight)
{
	return float2((x + 1.f) * 0.5f * (screenWidth-1.f), (1.f - y) * 0.5f * (screenHeight-1.f));
}

float2 Frustum::ViewportToScreenSpace(const float2 &point, int screenWidth, int screenHeight)
{
	return ViewportToScreenSpace(point.x, point.y, screenWidth, screenHeight);
}

float2 Frustum::ScreenToViewportSpace(float x, float y, int screenWidth, int screenHeight)
{
	return float2(x * 2.f / (screenWidth-1.f) - 1.f, 1.f - y * 2.f / (screenHeight - 1.f));
}

float2 Frustum::ScreenToViewportSpace(const float2 &point, int screenWidth, int screenHeight)
{
	return ScreenToViewportSpace(point.x, point.y, screenWidth, screenHeight);
}

Ray Frustum::UnProject(float x, float y) const
{
	assume1(x >= -1.f, x);
	assume1(x <= 1.f, x);
	assume1(y >= -1.f, y);
	assume1(y <= 1.f, y);
	if (type == PerspectiveFrustum)
	{
		vec nearPlanePos = NearPlanePos(x, y);
		return Ray(pos, (nearPlanePos - pos).Normalized());
	}
	else
		return UnProjectFromNearPlane(x, y);
}

LineSegment Frustum::UnProjectLineSegment(float x, float y) const
{
	vec nearPlanePos = NearPlanePos(x, y);
	vec farPlanePos = FarPlanePos(x, y);
	return LineSegment(nearPlanePos, farPlanePos);
}

Ray Frustum::UnProjectFromNearPlane(float x, float y) const
{
	return UnProjectLineSegment(x, y).ToRay();
}

vec Frustum::PointInside(float x, float y, float z) const
{
	assume(z >= 0.f);
	assume(z <= 1.f);
	return UnProjectLineSegment(x, y).GetPoint(z);
}

vec Frustum::Project(const vec &point) const
{
	float4 projectedPoint = ViewProjMatrix().Mul(POINT_TO_FLOAT4(point));
	projectedPoint /= projectedPoint.w; // Post-projective perspective divide.
	return FLOAT4_TO_POINT(projectedPoint);
}

bool Frustum::Contains(const vec &point) const
{
	const float eps = 1e-3f;
	const float pos = 1.f + eps;
	const float neg = -pos;
	vec projected = Project(point);
	if (projectiveSpace == FrustumSpaceD3D)
	{
		return projected.x >= neg && projected.x <= pos &&
			projected.y >= neg && projected.y <= pos &&
			projected.z >= -eps && projected.z <= pos;
	}
	else if (projectiveSpace == FrustumSpaceGL)
	{
		return projected.x >= neg && projected.x <= pos &&
			projected.y >= neg && projected.y <= pos &&
			projected.z >= neg && projected.z <= pos;
	}
	else
	{
#ifndef OPTIMIZED_RELEASE
		///\todo Make Frustum::Contains agnostic of the projection settings.
		LOGE("Not all values of Frustum were initialized properly! Please initialize correctly before calling Frustum::Contains()!");
#endif
		return false;
	}
}

bool Frustum::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool Frustum::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool Frustum::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool Frustum::Contains(const AABB &aabb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool Frustum::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool Frustum::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool Frustum::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

vec Frustum::ClosestPoint(const vec &point) const
{
	return ToPolyhedron().ClosestPoint(point);

///\todo Improve numerical stability enough to do effectively this - but do so without temporary memory allocations.
//	return ToPolyhedron().ClosestPointConvex(point);
}

float Frustum::Distance(const vec &point) const
{
	vec pt = ClosestPoint(point);
	return pt.Distance(point);
}

bool Frustum::IsFinite() const
{
	return pos.IsFinite() && front.IsFinite() && up.IsFinite() && MATH_NS::IsFinite(nearPlaneDistance)
		&& MATH_NS::IsFinite(farPlaneDistance) && MATH_NS::IsFinite(horizontalFov) && MATH_NS::IsFinite(verticalFov);
}

Plane Frustum::GetPlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
		default: // For release builds where assume() is disabled, always return the first option if out-of-bounds.
		case 0: return NearPlane();
		case 1: return FarPlane();
		case 2: return LeftPlane();
		case 3: return RightPlane();
		case 4: return TopPlane();
		case 5: return BottomPlane();
	}
}

float Frustum::Volume() const
{
	return ToPolyhedron().Volume();
}

vec Frustum::FastRandomPointInside(LCG &rng) const
{
	float f1 = rng.Float(-1.f, 1.f);
	float f2 = rng.Float(-1.f, 1.f);
	float f3 = rng.Float(0.f, 1.f);
	return PointInside(f1, f2, f3);
}

vec Frustum::UniformRandomPointInside(LCG &rng) const
{
	if (type == OrthographicFrustum)
		return FastRandomPointInside(rng);
	else
	{
		OBB o = MinimalEnclosingOBB();
		for(int numTries = 0; numTries < 1000; ++numTries)
		{
			vec pt = o.RandomPointInside(rng);
			if (Contains(pt))
				return pt;
		}
		LOGW("Rejection sampling failed in Frustum::UniformRandomPointInside! Producing a non-uniformly distributed point inside the frustum!");
		return FastRandomPointInside(rng);
	}
}

void Frustum::Translate(const vec &offset)
{
	pos += offset;
}

void Frustum::Transform(const float3x3 &transform)
{
	assume(transform.HasUniformScale());
	pos = transform * pos;
	front = transform * front;
	float scaleFactor = front.Normalize();
	up = (transform * up).Normalized();
	nearPlaneDistance *= scaleFactor;
	farPlaneDistance *= scaleFactor;
	if (type == OrthographicFrustum)
	{
		orthographicWidth *= scaleFactor;
		orthographicHeight *= scaleFactor;
	}
}

void Frustum::Transform(const float3x4 &transform)
{
	assume(transform.HasUniformScale());
	pos = transform.MulPos(pos);
	front = transform.MulDir(front);
	float scaleFactor = front.Normalize();
	up = transform.MulDir(up).Normalized();
	nearPlaneDistance *= scaleFactor;
	farPlaneDistance *= scaleFactor;
	if (type == OrthographicFrustum)
	{
		orthographicWidth *= scaleFactor;
		orthographicHeight *= scaleFactor;
	}
}

void Frustum::Transform(const float4x4 &transform)
{
	assume(transform.Row(3).Equals(0,0,0,1));
	Transform(transform.Float3x4Part());
}

void Frustum::Transform(const Quat &transform)
{
	Transform(transform.ToFloat3x3());
}

void Frustum::GetPlanes(Plane *outArray) const
{
	assume(outArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outArray)
		return;
#endif
	for(int i = 0; i < 6; ++i)
		outArray[i] = GetPlane(i);
}

vec Frustum::CenterPoint() const
{
	return pos + (nearPlaneDistance + farPlaneDistance) * 0.5f * front;
}

void Frustum::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif

	if (type == PerspectiveFrustum)
	{
		float tanhfov = Tan(horizontalFov*0.5f);
		float tanvfov = Tan(verticalFov*0.5f);
		float frontPlaneHalfWidth = tanhfov*nearPlaneDistance;
		float frontPlaneHalfHeight = tanvfov*nearPlaneDistance;
		float farPlaneHalfWidth = tanhfov*farPlaneDistance;
		float farPlaneHalfHeight = tanvfov*farPlaneDistance;

		vec right = WorldRight();

		vec nearCenter = pos + front * nearPlaneDistance;
		vec nearHalfWidth = frontPlaneHalfWidth*right;
		vec nearHalfHeight = frontPlaneHalfHeight*up;
		outPointArray[0] = nearCenter - nearHalfWidth - nearHalfHeight;
		outPointArray[1] = nearCenter + nearHalfWidth - nearHalfHeight;
		outPointArray[2] = nearCenter - nearHalfWidth + nearHalfHeight;
		outPointArray[3] = nearCenter + nearHalfWidth + nearHalfHeight;

		vec farCenter = pos + front * farPlaneDistance;
		vec farHalfWidth = farPlaneHalfWidth*right;
		vec farHalfHeight = farPlaneHalfHeight*up;
		outPointArray[4] = farCenter - farHalfWidth - farHalfHeight;
		outPointArray[5] = farCenter + farHalfWidth - farHalfHeight;
		outPointArray[6] = farCenter - farHalfWidth + farHalfHeight;
		outPointArray[7] = farCenter + farHalfWidth + farHalfHeight;
	}
	else
	{
		vec right = WorldRight();
		vec nearCenter = pos + front * nearPlaneDistance;
		vec farCenter = pos + front * farPlaneDistance;
		vec halfWidth = orthographicWidth * 0.5f * right;
		vec halfHeight = orthographicHeight * 0.5f * up;

		outPointArray[0] = nearCenter - halfWidth - halfHeight;
		outPointArray[1] = nearCenter + halfWidth - halfHeight;
		outPointArray[2] = nearCenter - halfWidth + halfHeight;
		outPointArray[3] = nearCenter + halfWidth + halfHeight;
		outPointArray[4] = farCenter - halfWidth - halfHeight;
		outPointArray[5] = farCenter + halfWidth - halfHeight;
		outPointArray[6] = farCenter - halfWidth + halfHeight;
		outPointArray[7] = farCenter + halfWidth + halfHeight;
	}
}

LineSegment Frustum::Edge(int edgeIndex) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	switch(edgeIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return LineSegment(CornerPoint(0), CornerPoint(1));
		case 1: return LineSegment(CornerPoint(0), CornerPoint(2));
		case 2: return LineSegment(CornerPoint(0), CornerPoint(4));
		case 3: return LineSegment(CornerPoint(1), CornerPoint(3));
		case 4: return LineSegment(CornerPoint(1), CornerPoint(5));
		case 5: return LineSegment(CornerPoint(2), CornerPoint(3));
		case 6: return LineSegment(CornerPoint(2), CornerPoint(6));
		case 7: return LineSegment(CornerPoint(3), CornerPoint(7));
		case 8: return LineSegment(CornerPoint(4), CornerPoint(5));
		case 9: return LineSegment(CornerPoint(4), CornerPoint(6));
		case 10: return LineSegment(CornerPoint(5), CornerPoint(7));
		case 11: return LineSegment(CornerPoint(6), CornerPoint(7));
	}
}

vec Frustum::CornerPoint(int cornerIndex) const
{
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return NearPlanePos(-1, -1);
		case 1: return FarPlanePos(-1, -1);
		case 2: return NearPlanePos(-1, 1);
		case 3: return FarPlanePos(-1, 1);
		case 4: return NearPlanePos(1, -1);
		case 5: return FarPlanePos(1, -1);
		case 6: return NearPlanePos(1, 1);
		case 7: return FarPlanePos(1, 1);
	}
}

vec Frustum::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec corners[8];
	GetCornerPoints(corners);
	vec mostExtreme = vec::nan;
	projectionDistance = -FLOAT_INF;
	for(int i = 0; i < 8; ++i)
	{
		float d = Dot(direction, corners[i]);
		if (d > projectionDistance)
		{
			projectionDistance = d;
			mostExtreme = corners[i];
		}
	}
	return mostExtreme;
}

void Frustum::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	vec corners[8];
	GetCornerPoints(corners);
	outMax = -FLOAT_INF;
	outMin = FLOAT_INF;
	for(int i = 0; i < 8; ++i)
	{
		float d = Dot(direction, corners[i]);
		outMax = Max(outMax, d);
		outMin = Min(outMin, d);
	}
}

int Frustum::UniqueFaceNormals(vec *out) const
{
	if (type == PerspectiveFrustum)
	{
		out[0] = front;
		out[1] = LeftPlane().normal;
		out[2] = RightPlane().normal;
		out[3] = TopPlane().normal;
		out[4] = BottomPlane().normal;
		return 5;
	}
	else
	{
		out[0] = front;
		out[1] = up;
		out[2] = Cross(front, up);
		return 3;
	}
}

int Frustum::UniqueEdgeDirections(vec *out) const
{
	if (type == PerspectiveFrustum)
	{
		out[0] = NearPlanePos(-1, -1) - NearPlanePos(1, -1);
		out[1] = NearPlanePos(-1, -1) - NearPlanePos(-1, 1);
		out[2] = FarPlanePos(-1, -1) - NearPlanePos(-1, -1);
		out[3] = FarPlanePos( 1, -1) - NearPlanePos( 1, -1);
		out[4] = FarPlanePos(-1,  1) - NearPlanePos(-1,  1);
		out[5] = FarPlanePos( 1,  1) - NearPlanePos( 1,  1);
		return 6;
	}
	else
	{
		out[0] = front;
		out[1] = up;
		out[2] = Cross(front, up);
		return 3;
	}
}

AABB Frustum::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetNegativeInfinity();
	for(int i = 0; i < 8; ++i)
		aabb.Enclose(CornerPoint(i));
	return aabb;
}

OBB Frustum::MinimalEnclosingOBB(float expandGuardband) const
{
	assume(IsFinite());
	assume(front.IsNormalized());
	assume(up.IsNormalized());

	OBB obb;
	obb.pos = pos + (nearPlaneDistance + farPlaneDistance) * 0.5f * front;
	obb.axis[1] = up;
	obb.axis[2] = -front;
	obb.axis[0] = Cross(obb.axis[1], obb.axis[2]);
	obb.r = vec::zero;
	for(int i = 0; i < 8; ++i)
		obb.Enclose(CornerPoint(i));

	// Expand the generated OBB very slightly to avoid numerical issues when
	// testing whether this Frustum actually is contained inside the generated OBB.
	obb.r.x += expandGuardband;
	obb.r.y += expandGuardband;
	obb.r.z += expandGuardband;
	return obb;
}

Polyhedron Frustum::ToPolyhedron() const
{
	// Note to maintainer: This function is an exact copy of AABB:ToPolyhedron() and OBB::ToPolyhedron().

	Polyhedron p;
	// Populate the corners of this Frustum.
	// The will be in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
	for(int i = 0; i < 8; ++i)
		p.v.push_back(CornerPoint(i));

	// Generate the 6 faces of this Frustum. The function Frustum::GetPlane() has a convention of returning
	// the planes in order near,far,left,right,top,bottom, so follow the same convention here.
	const int faces[6][4] =
	{
		{ 0, 4, 6, 2 }, // Z-: near plane
		{ 1, 3, 7, 5 }, // Z+: far plane
		{ 0, 2, 3, 1 }, // X-: left plane
		{ 4, 5, 7, 6 }, // X+: right plane
		{ 7, 3, 2, 6 }, // Y+: top plane
		{ 0, 1, 5, 4 }, // Y-: bottom plane
	};

	for(int f = 0; f < 6; ++f)
	{
		Polyhedron::Face face;
		if (this->handedness == FrustumLeftHanded)
		{
			for(int v = 0; v < 4; ++v)
				face.v.push_back(faces[f][3-v]);
		}
		else
		{
			for(int v = 0; v < 4; ++v)
				face.v.push_back(faces[f][v]);
		}
		p.f.push_back(face);
	}

	return p;
}

bool Frustum::Intersects(const Ray &ray) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(ray);
}

bool Frustum::Intersects(const Line &line) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(line);
}

bool Frustum::Intersects(const LineSegment &lineSegment) const
{
	return GJKIntersect(*this, lineSegment);
}

bool Frustum::Intersects(const AABB &aabb) const
{
	return GJKIntersect(*this, aabb);
}

bool Frustum::Intersects(const OBB &obb) const
{
	return GJKIntersect(*this, obb);
}

bool Frustum::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool Frustum::Intersects(const Triangle &triangle) const
{
	return GJKIntersect(*this, triangle);
}

bool Frustum::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Frustum::Intersects(const Sphere &sphere) const
{
	return GJKIntersect(*this, sphere);
}

bool Frustum::Intersects(const Capsule &capsule) const
{
	return GJKIntersect(*this, capsule);
}

bool Frustum::Intersects(const Frustum &frustum) const
{
	return GJKIntersect(*this, frustum);
}

bool Frustum::Intersects(const Polyhedron &polyhedron) const
{
	return this->ToPolyhedron().Intersects(polyhedron);
}

#if defined(MATH_TINYXML_INTEROP) && defined(MATH_CONTAINERLIB_SUPPORT)

void Frustum::DeserializeFromXml(TiXmlElement *e)
{
	type = StrCaseEq(e->Attribute("orthographic"), "true") ? OrthographicFrustum : PerspectiveFrustum;
	pos = POINT_VEC(float3::FromString(e->Attribute("pos")));
	front = DIR_VEC(float3::FromString(e->Attribute("front")));
	up = DIR_VEC(float3::FromString(e->Attribute("up")));
	e->QueryFloatAttribute("nearPlaneDistance", &nearPlaneDistance);
	e->QueryFloatAttribute("farPlaneDistance", &farPlaneDistance);
	e->QueryFloatAttribute("horizontalFov", &horizontalFov);
	e->QueryFloatAttribute("verticalFov", &verticalFov);
}

#endif

#ifdef MATH_ENABLE_STL_SUPPORT

std::string FrustumTypeToString(FrustumType t)
{
	if (t == InvalidFrustum) return "InvalidFrustum";
	if (t == OrthographicFrustum) return "OrthographicFrustum";
	if (t == PerspectiveFrustum) return "PerspectiveFrustum";
	return "(invalid frustum type)";
}

std::string Frustum::ToString() const
{
	char str[256];
	sprintf(str, "Frustum(%s pos:(%.2f, %.2f, %.2f) front:(%.2f, %.2f, %.2f), up:(%.2f, %.2f, %.2f), near: %.2f, far: %.2f, %s: %.2f, %s: %.2f)",
		FrustumTypeToString(type).c_str(), pos.x, pos.y, pos.z, front.x, front.y, front.z,
		up.x, up.y, up.z, nearPlaneDistance, farPlaneDistance,
		type == OrthographicFrustum ? "ortho width:" : "hFov",
		horizontalFov,
		type == OrthographicFrustum ? "ortho height:" : "vFov",
		verticalFov);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Frustum &frustum)
{
	o << frustum.ToString();
	return o;
}

#endif

Frustum operator *(const float3x3 &transform, const Frustum &frustum)
{
	Frustum f(frustum);
	f.Transform(transform);
	return f;
}

Frustum operator *(const float3x4 &transform, const Frustum &frustum)
{
	Frustum f(frustum);
	f.Transform(transform);
	return f;
}

Frustum operator *(const float4x4 &transform, const Frustum &frustum)
{
	Frustum f(frustum);
	f.Transform(transform);
	return f;
}

Frustum operator *(const Quat &transform, const Frustum &frustum)
{
	Frustum f(frustum);
	f.Transform(transform);
	return f;
}

MATH_END_NAMESPACE
