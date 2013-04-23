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
#include "Triangle.h"
#include "LineSegment.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/Quat.h"
#include "../Algorithm/Random/LCG.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#if defined(MATH_TINYXML_INTEROP) && defined(MATH_CONTAINERLIB_SUPPORT)
#include "Container/UString.h"
#endif

MATH_BEGIN_NAMESPACE

float Frustum::AspectRatio() const
{
	return horizontalFov / verticalFov;
}

float Frustum::NearPlaneWidth() const
{
	///\todo Optimize!
	return NearPlanePos(-1.f, 0.f).Distance(NearPlanePos(1.f, 0.f));
}

float Frustum::NearPlaneHeight() const
{
	///\todo Optimize!
	return NearPlanePos(0.f, -1.f).Distance(NearPlanePos(0.f, 1.f));
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
	float3 left = Cross(up, front);
	left.ScaleToLength(Tan(horizontalFov*0.5f));
	float3 leftSide = front + left;
	float3 leftSideNormal = Cross(up, leftSide).Normalized();
	return Plane(pos, leftSideNormal);
}

Plane Frustum::RightPlane() const
{
	float3 right = WorldRight();
	right.ScaleToLength(Tan(horizontalFov*0.5f));
	float3 rightSide = front + right;
	float3 rightSideNormal = Cross(rightSide, up).Normalized();
	return Plane(pos, rightSideNormal);
}

Plane Frustum::TopPlane() const
{
	float3 topSide = front + Tan(verticalFov * 0.5f) * up;
	float3 right = WorldRight();
	float3 topSideNormal = Cross(right, topSide).Normalized();
	return Plane(pos, topSideNormal);
}

Plane Frustum::BottomPlane() const
{
	float3 bottomSide = front - Tan(verticalFov * 0.5f) * up;
	float3 left = Cross(up, front);
	float3 bottomSideNormal = Cross(left, bottomSide).Normalized();
	return Plane(pos, bottomSideNormal);
}

void Frustum::SetWorldMatrix(const float3x4 &worldTransform)
{
	pos = worldTransform.TranslatePart();
	front = -worldTransform.Col(2); // The camera looks towards -Z axis of the given transform.
	up = worldTransform.Col(1); // The camera up points towards +Y of the given transform.
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
	m.SetCol(0, front.Cross(up).Normalized());
	m.SetCol(1, up);
	m.SetCol(2, -front);
	m.SetCol(3, pos);
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
	if (type == PerspectiveFrustum)
	{
#if USE_D3D11
		return float4x4::D3DPerspProjRH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
#else
		return float4x4::OpenGLPerspProjRH(nearPlaneDistance, farPlaneDistance, NearPlaneWidth(), NearPlaneHeight());
#endif
	}
	else
	{
		return float4x4::D3DOrthoProjRH(nearPlaneDistance, farPlaneDistance, orthographicWidth, orthographicHeight);
	}
}

float3 Frustum::NearPlanePos(float x, float y) const
{
	assume(type == PerspectiveFrustum || type == OrthographicFrustum);

	if (type == PerspectiveFrustum)
	{
		float frontPlaneHalfWidth = Tan(horizontalFov*0.5f)*nearPlaneDistance;
		float frontPlaneHalfHeight = Tan(verticalFov*0.5f)*nearPlaneDistance;
		x = x * frontPlaneHalfWidth; // Map [-1,1] to [-width/2, width/2].
		y = y * frontPlaneHalfHeight;  // Map [-1,1] to [-height/2, height/2].
		float3 right = WorldRight();
		return pos + front * nearPlaneDistance + x * right + y * up;
	}
	else
	{
		float3 right = WorldRight();
		return pos + front * nearPlaneDistance
				   + x * orthographicWidth * 0.5f * right
				   + y * orthographicHeight * 0.5f * up;
	}
}

float3 Frustum::NearPlanePos(const float2 &point) const
{
	return NearPlanePos(point.x, point.y);
}

float3 Frustum::FarPlanePos(float x, float y) const
{
	assume(type == PerspectiveFrustum || type == OrthographicFrustum);

	if (type == PerspectiveFrustum)
	{
		float farPlaneHalfWidth = Tan(horizontalFov*0.5f)*farPlaneDistance;
		float farPlaneHalfHeight = Tan(verticalFov*0.5f)*farPlaneDistance;
		x = x * farPlaneHalfWidth;
		y = y * farPlaneHalfHeight;
		float3 right = WorldRight();
		return pos + front * farPlaneDistance + x * right + y * up;
	}
	else
	{
		float3 right = WorldRight();
		return pos + front * farPlaneDistance
				   + x * orthographicWidth * 0.5f * right
				   + y * orthographicHeight * 0.5f * up;
	}
}

float3 Frustum::FarPlanePos(const float2 &point) const
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
	assume(x >= -1.f);
	assume(x <= 1.f);
	assume(y >= -1.f);
	assume(y <= 1.f);
	if (type == PerspectiveFrustum)
	{
		float3 nearPlanePos = NearPlanePos(x, y);
		return Ray(pos, (nearPlanePos - pos).Normalized());
	}
	else
		return UnProjectFromNearPlane(x, y);
}

LineSegment Frustum::UnProjectLineSegment(float x, float y) const
{
	float3 nearPlanePos = NearPlanePos(x, y);
	float3 farPlanePos = FarPlanePos(x, y);
	return LineSegment(nearPlanePos, farPlanePos);
}

Ray Frustum::UnProjectFromNearPlane(float x, float y) const
{
	return UnProjectLineSegment(x, y).ToRay();
}

float3 Frustum::PointInside(float x, float y, float z) const
{
	assume(z >= 0.f);
	assume(z <= 1.f);
	return UnProjectLineSegment(x, y).GetPoint(z);
}

float3 Frustum::Project(const float3 &point) const
{
	float4 projectedPoint = ViewProjMatrix().Mul(float4(point, 1.f));
	projectedPoint /= projectedPoint.w; // Post-projective perspective divide.
	return projectedPoint.xyz();
}

bool Frustum::Contains(const float3 &point) const
{
	float3 projected = Project(point);
#ifdef USE_D3D11
	return projected.x >= -1.f && projected.x <= 1.f &&
		projected.y >= -1.f && projected.y <= 1.f &&
		projected.z >= 0.f && projected.z <= 1.f;
#else
	return projected.x >= -1.f && projected.x <= 1.f &&
		projected.y >= -1.f && projected.y <= 1.f &&
		projected.z >= -1.f && projected.z <= 1.f;
#endif
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

float3 Frustum::ClosestPoint(const float3 &point) const
{
	return ToPolyhedron().ClosestPointConvex(point);
}

float Frustum::Distance(const float3 &point) const
{
	float3 pt = ClosestPoint(point);
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

float3 Frustum::FastRandomPointInside(LCG &rng) const
{
	return PointInside(rng.Float(-1.f, 1.f), rng.Float(-1.f, 1.f), rng.Float(0.f, 1.f));
}

float3 Frustum::UniformRandomPointInside(LCG &rng) const
{
	if (type == OrthographicFrustum)
		return FastRandomPointInside(rng);
	else
	{
		OBB o = MinimalEnclosingOBB();
		for(int numTries = 0; numTries < 1000; ++numTries)
		{
			float3 pt = o.RandomPointInside(rng);
			if (Contains(pt))
				return pt;
		}
		LOGW("Rejection sampling failed in Frustum::UniformRandomPointInside! Producing a non-uniformly distributed point inside the frustum!");
		return FastRandomPointInside(rng);
	}
}

void Frustum::Translate(const float3 &offset)
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

float3 Frustum::CenterPoint() const
{
	return pos + (nearPlaneDistance + farPlaneDistance) * 0.5f * front;
}

void Frustum::GetCornerPoints(float3 *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
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

float3 Frustum::CornerPoint(int cornerIndex) const
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

float3 Frustum::ExtremePoint(const float3 &direction) const
{
	float3 mostExtreme = float3::nan;
	float mostExtremeDist = -FLT_MAX;
	for(int i = 0; i < 8; ++i)
	{
		float3 pt = CornerPoint(i);
		float d = Dot(direction, pt);
		if (d > mostExtremeDist)
		{
			mostExtremeDist = d;
			mostExtreme = pt;
		}
	}
	return mostExtreme;
}

void Frustum::ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const
{
	///\todo Optimize!
	float3 minPt = ExtremePoint(-direction);
	float3 maxPt = ExtremePoint(direction);
	outMin = Dot(minPt, direction);
	outMax = Dot(maxPt, direction);
}

AABB Frustum::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetNegativeInfinity();
	for(int i = 0; i < 8; ++i)
		aabb.Enclose(CornerPoint(i));
	return aabb;
}

OBB Frustum::MinimalEnclosingOBB() const
{
	assume(IsFinite());
	assume(front.IsNormalized());
	assume(up.IsNormalized());

	OBB obb;
	obb.pos = pos + (nearPlaneDistance + farPlaneDistance) * 0.5f * front;
	obb.axis[1] = up;
	obb.axis[2] = -front;
	obb.axis[0] = Cross(obb.axis[1], obb.axis[2]);
	obb.r = float3::zero;
	for(int i = 0; i < 8; ++i)
		obb.Enclose(CornerPoint(i));
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

	// Generate the 6 faces of this Frustum.
	const int faces[6][4] =
	{
		{ 0, 2, 3, 1 }, // X-
		{ 4, 5, 7, 6 }, // X+
		{ 0, 1, 5, 4 }, // Y-
		{ 7, 3, 2, 6 }, // Y+
		{ 0, 4, 6, 2 }, // Z-
		{ 1, 3, 7, 5 }, // Z+
	};

	for(int f = 0; f < 6; ++f)
	{
		Polyhedron::Face face;
		for(int v = 0; v < 4; ++v)
			face.v.push_back(faces[f][v]);
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
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(lineSegment);
}

bool Frustum::Intersects(const AABB &aabb) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(aabb);
}

bool Frustum::Intersects(const OBB &obb) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(obb);
}

bool Frustum::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool Frustum::Intersects(const Triangle &triangle) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(triangle);
}

bool Frustum::Intersects(const Polygon &polygon) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(polygon);
}

bool Frustum::Intersects(const Sphere &sphere) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(sphere);
}

bool Frustum::Intersects(const Capsule &capsule) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(capsule);
}

bool Frustum::Intersects(const Frustum &frustum) const
{
	///@todo This is a naive test. Implement a faster version.
	return this->ToPolyhedron().Intersects(frustum);
}

bool Frustum::Intersects(const Polyhedron &polyhedron) const
{
	return this->ToPolyhedron().Intersects(polyhedron);
}

#if defined(MATH_TINYXML_INTEROP) && defined(MATH_CONTAINERLIB_SUPPORT)

void Frustum::DeserializeFromXml(TiXmlElement *e)
{
	type = StrCaseEq(e->Attribute("orthographic"), "true") ? OrthographicFrustum : PerspectiveFrustum;
	pos = float3::FromString(e->Attribute("pos"));
	front = float3::FromString(e->Attribute("front"));
	up = float3::FromString(e->Attribute("up"));
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
