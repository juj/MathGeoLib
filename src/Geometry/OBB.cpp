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

/** @file OBB.cpp
	@author Jukka Jylänki
	@brief Implementation for the Oriented Bounding Box (OBB) geometry object. */
#include "OBB.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#include <utility>
#endif
#include "../Math/MathFunc.h"
#include "AABB.h"
#include "Frustum.h"
#include "../Algorithm/Random/LCG.h"
#include "LineSegment.h"
#include "Line.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Sphere.h"
#include "Capsule.h"
#include "../Math/float2.inl"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "PBVolume.h"
#include "Ray.h"
#include "Triangle.h"
#include <stdlib.h>
#include "../Time/Clock.h"

#include <set>
#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Algorithm/Sort/Sort.h"
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

#if defined(MATH_SIMD) && defined(MATH_AUTOMATIC_SSE)
#include "../Math/float4_neon.h"
#include "../Math/float4_sse.h"
#include "../Math/float4x4_sse.h"
#endif

#define MATH_ENCLOSINGOBB_DOUBLE_PRECISION

#ifdef MATH_ENCLOSINGOBB_DOUBLE_PRECISION
#include "../Math/float4d.h"
#endif

MATH_BEGIN_NAMESPACE

#ifdef MATH_ENCLOSINGOBB_DOUBLE_PRECISION
typedef float4d cv;
typedef double cs;
typedef std::vector<float4d> VecdArray;
#else
typedef vec cv;
typedef float cs;
typedef VecArray VecdArray;
#endif

OBB::OBB(const vec &pos, const vec &r, const vec &axis0, const vec &axis1, const vec &axis2)
:pos(pos), r(r)
{
	axis[0] = axis0;
	axis[1] = axis1;
	axis[2] = axis2;
}

OBB::OBB(const AABB &aabb)
{
	SetFrom(aabb);
}

void OBB::SetNegativeInfinity()
{
	pos = POINT_VEC_SCALAR(0.f);
	r.SetFromScalar(-FLOAT_INF);
	axis[0] = DIR_VEC(1,0,0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

void OBB::SetFrom(const AABB &aabb)
{
	pos = aabb.CenterPoint();
	r = aabb.HalfSize();
	axis[0] = DIR_VEC(1, 0, 0);
	axis[1] = DIR_VEC(0, 1, 0);
	axis[2] = DIR_VEC(0, 0, 1);
}

template<typename Matrix>
void OBBSetFrom(OBB &obb, const AABB &aabb, const Matrix &m)
{
	assume(m.IsColOrthogonal()); // We cannot convert transform an AABB to OBB if it gets sheared in the process.
	assume(m.HasUniformScale()); // Nonuniform scale will produce shear as well.
	obb.pos = m.MulPos(aabb.CenterPoint());
	obb.r = aabb.HalfSize();
	obb.axis[0] = DIR_VEC(m.Col(0));
	obb.axis[1] = DIR_VEC(m.Col(1));
	obb.axis[2] = DIR_VEC(m.Col(2));
	// If the matrix m contains scaling, propagate the scaling from the axis vectors to the half-length vectors,
	// since we want to keep the axis vectors always normalized in our representation.
	float matrixScale = obb.axis[0].LengthSq();
	matrixScale = Sqrt(matrixScale);
	obb.r *= matrixScale;
	matrixScale = 1.f / matrixScale;
	obb.axis[0] *= matrixScale;
	obb.axis[1] *= matrixScale;
	obb.axis[2] *= matrixScale;

//	mathassert(vec::AreOrthogonal(obb.axis[0], obb.axis[1], obb.axis[2]));
//	mathassert(vec::AreOrthonormal(obb.axis[0], obb.axis[1], obb.axis[2]));
	///@todo Would like to simply do the above, but instead numerical stability requires to do the following:
	vec::Orthonormalize(obb.axis[0], obb.axis[1], obb.axis[2]);
}

void OBB::SetFrom(const AABB &aabb, const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float3x4 &transform)
{
	OBBSetFrom(*this, aabb, transform);
}

void OBB::SetFrom(const AABB &aabb, const float4x4 &transform)
{
	assume(transform.Row(3).Equals(0,0,0,1));
	OBBSetFrom(*this, aabb, transform.Float3x4Part());
}

void OBB::SetFrom(const AABB &aabb, const Quat &transform)
{
	OBBSetFrom(*this, aabb, float3x3(transform));
}

void OBB::SetFrom(const Sphere &sphere)
{
	pos = sphere.pos;
	r.SetFromScalar(sphere.r);
	axis[0] = DIR_VEC(1,0,0);
	axis[1] = DIR_VEC(0,1,0);
	axis[2] = DIR_VEC(0,0,1);
}

#ifdef MATH_CONTAINERLIB_SUPPORT
bool OBB::SetFrom(const Polyhedron &polyhedron)
{
	if (!polyhedron.v.empty())
	{
		*this = OBB::OptimalEnclosingOBB((vec*)&polyhedron.v[0], (int)polyhedron.v.size());
		return true;
	}
	else
	{
		SetNegativeInfinity();
		return false;
	}
}
#endif

Polyhedron OBB::ToPolyhedron() const
{
	// Note to maintainer: This function is an exact copy of AABB:ToPolyhedron() and Frustum::ToPolyhedron().

	Polyhedron p;
	// Populate the corners of this OBB.
	// The will be in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
	for(int i = 0; i < 8; ++i)
		p.v.push_back(CornerPoint(i));

	// Generate the 6 faces of this OBB.
	const int faces[6][4] =
	{
		{ 0, 1, 3, 2 }, // X-
		{ 4, 6, 7, 5 }, // X+
		{ 0, 4, 5, 1 }, // Y-
		{ 7, 6, 2, 3 }, // Y+
		{ 0, 2, 6, 4 }, // Z-
		{ 1, 5, 7, 3 }, // Z+
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

PBVolume<6> OBB::ToPBVolume() const
{
	PBVolume<6> pbVolume;
	for(int i = 0; i < 6; ++i)
		pbVolume.p[i] = FacePlane(i);

	return pbVolume;
}

AABB OBB::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetFrom(*this);
	return aabb;
}

#if 0

AABB OBB::MaximalContainedAABB() const
{
#ifdef _MSC_VER
#pragma warning(OBB::MaximalContainedAABB not implemented!)
#else
#warning OBB::MaximalContainedAABB not implemented!
#endif
	assume(false && "OBB::MaximalContainedAABB not implemented!"); /// @todo Implement.
	return AABB();
}
#endif

Sphere OBB::MinimalEnclosingSphere() const
{
	Sphere s;
	s.pos = pos;
	s.r = HalfDiagonal().Length();
	return s;
}

Sphere OBB::MaximalContainedSphere() const
{
	Sphere s;
	s.pos = pos;
	s.r = r.MinElement();
	return s;
}

bool OBB::IsFinite() const
{
	return pos.IsFinite() && r.IsFinite() && axis[0].IsFinite() && axis[1].IsFinite() && axis[2].IsFinite();
}

bool OBB::IsDegenerate() const
{
	return !(r.x > 0.f && r.y > 0.f && r.z > 0.f);
}

vec OBB::CenterPoint() const
{
	return pos;
}

vec OBB::PointInside(float x, float y, float z) const
{
	assume(0.f <= x && x <= 1.f);
	assume(0.f <= y && y <= 1.f);
	assume(0.f <= z && z <= 1.f);

	return pos + axis[0] * (2.f * r.x * x - r.x)
			   + axis[1] * (2.f * r.y * y - r.y)
			   + axis[2] * (2.f * r.z * z - r.z);
}

LineSegment OBB::Edge(int edgeIndex) const
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

vec OBB::CornerPoint(int cornerIndex) const
{	
	assume(0 <= cornerIndex && cornerIndex <= 7);
	switch(cornerIndex)
	{
		default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
		case 0: return pos - r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 1: return pos - r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 2: return pos - r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 3: return pos - r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
		case 4: return pos + r.x * axis[0] - r.y * axis[1] - r.z * axis[2];
		case 5: return pos + r.x * axis[0] - r.y * axis[1] + r.z * axis[2];
		case 6: return pos + r.x * axis[0] + r.y * axis[1] - r.z * axis[2];
		case 7: return pos + r.x * axis[0] + r.y * axis[1] + r.z * axis[2];
	}
}

vec OBB::ExtremePoint(const vec &direction) const
{
	vec pt = pos;
	pt += axis[0] * (Dot(direction, axis[0]) >= 0.f ? r.x : -r.x);
	pt += axis[1] * (Dot(direction, axis[1]) >= 0.f ? r.y : -r.y);
	pt += axis[2] * (Dot(direction, axis[2]) >= 0.f ? r.z : -r.z);
	return pt;
}

vec OBB::ExtremePoint(const vec &direction, float &projectionDistance) const
{
	vec extremePoint = ExtremePoint(direction);
	projectionDistance = extremePoint.Dot(direction);
	return extremePoint;
}

void OBB::ProjectToAxis(const vec &direction, float &outMin, float &outMax) const
{
	float x = Abs(Dot(direction, axis[0]) * r.x);
	float y = Abs(Dot(direction, axis[1]) * r.y);
	float z = Abs(Dot(direction, axis[2]) * r.z);
	float pt = Dot(direction, pos);
	outMin = pt - x - y - z;
	outMax = pt + x + y + z;
}

int OBB::UniqueFaceNormals(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

int OBB::UniqueEdgeDirections(vec *out) const
{
	out[0] = axis[0];
	out[1] = axis[1];
	out[2] = axis[2];
	return 3;
}

vec OBB::PointOnEdge(int edgeIndex, float u) const
{
	assume(0 <= edgeIndex && edgeIndex <= 11);
	assume(0 <= u && u <= 1.f);

	edgeIndex = Clamp(edgeIndex, 0, 11);
	vec d = axis[edgeIndex/4] * (2.f * u - 1.f) * r[edgeIndex/4];
	switch(edgeIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.y * axis[1] - r.z * axis[2] + d;
	case 1: return pos - r.y * axis[1] + r.z * axis[2] + d;
	case 2: return pos + r.y * axis[1] - r.z * axis[2] + d;
	case 3: return pos + r.y * axis[1] + r.z * axis[2] + d;

	case 4: return pos - r.x * axis[0] - r.z * axis[2] + d;
	case 5: return pos - r.x * axis[0] + r.z * axis[2] + d;
	case 6: return pos + r.x * axis[0] - r.z * axis[2] + d;
	case 7: return pos + r.x * axis[0] + r.z * axis[2] + d;

	case 8: return pos - r.x * axis[0] - r.y * axis[1] + d;
	case 9: return pos - r.x * axis[0] + r.y * axis[1] + d;
	case 10: return pos + r.x * axis[0] - r.y * axis[1] + d;
	case 11: return pos + r.x * axis[0] + r.y * axis[1] + d;
	}
}

vec OBB::FaceCenterPoint(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);

	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.x * axis[0];
	case 1: return pos + r.x * axis[0];
	case 2: return pos - r.y * axis[1];
	case 3: return pos + r.y * axis[1];
	case 4: return pos - r.z * axis[2];
	case 5: return pos + r.z * axis[2];
	}
}

vec OBB::FacePoint(int faceIndex, float u, float v) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	assume(0 <= u && u <= 1.f);
	assume(0 <= v && v <= 1.f);

	int uIdx = faceIndex/2;
	int vIdx = (faceIndex/2 + 1) % 3;
	vec U = axis[uIdx] * (2.f * u - 1.f) * r[uIdx];
	vec V = axis[vIdx] * (2.f * v - 1.f) * r[vIdx];
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return pos - r.z * axis[2] + U + V;
	case 1: return pos + r.z * axis[2] + U + V;
	case 2: return pos - r.x * axis[0] + U + V;
	case 3: return pos + r.x * axis[0] + U + V;
	case 4: return pos - r.y * axis[1] + U + V;
	case 5: return pos + r.y * axis[1] + U + V;
	}
}

Plane OBB::FacePlane(int faceIndex) const
{
	assume(0 <= faceIndex && faceIndex <= 5);
	switch(faceIndex)
	{
	default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
	case 0: return Plane(FaceCenterPoint(0), -axis[0]);
	case 1: return Plane(FaceCenterPoint(1), axis[0]);
	case 2: return Plane(FaceCenterPoint(2), -axis[1]);
	case 3: return Plane(FaceCenterPoint(3), axis[1]);
	case 4: return Plane(FaceCenterPoint(4), -axis[2]);
	case 5: return Plane(FaceCenterPoint(5), axis[2]);
	}
}

void OBB::GetCornerPoints(vec *outPointArray) const
{
	assume(outPointArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPointArray)
		return;
#endif
	for(int i = 0; i < 8; ++i)
		outPointArray[i] = CornerPoint(i);
}

void OBB::GetFacePlanes(Plane *outPlaneArray) const
{
	assume(outPlaneArray);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPlaneArray)
		return;
#endif
	for(int i = 0; i < 6; ++i)
		outPlaneArray[i] = FacePlane(i);
}

/// See Christer Ericson's book Real-Time Collision Detection, page 83.
void OBB::ExtremePointsAlongDirection(const vec &dir, const vec *pointArray, int numPoints, int &idxSmallest, int &idxLargest, float &smallestD, float &largestD)
{
	assume(pointArray || numPoints == 0);

	idxSmallest = idxLargest = 0;

#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pointArray)
		return;
#endif

	smallestD = FLOAT_INF;
	largestD = -FLOAT_INF;
	for(int i = 0; i < numPoints; ++i)
	{
		float d = Dot(pointArray[i], dir);
		if (d < smallestD)
		{
			smallestD = d;
			idxSmallest = i;
		}
		if (d > largestD)
		{
			largestD = d;
			idxLargest = i;
		}
	}
}

float SmallestOBBVolumeJiggle(const vec &edge_, const Polyhedron &convexHull, std::vector<float2> &pts,
	vec &outEdgeA, vec &outEdgeB)
{
	vec edge = edge_;
	int numTimesNotImproved = 0;
	float bestVolume = FLOAT_INF;
	float2 c10, c20;
	vec u, v;
	vec prevSecondChoice = vec::nan;
	int numJiggles = 2;
	while(numTimesNotImproved < 2)
	{
		int e1, e2;
		OBB::ExtremePointsAlongDirection(edge, (const vec*)&convexHull.v[0], (int)convexHull.v.size(), e1, e2);
		float edgeLength = Abs(Dot((vec)convexHull.v[e1] - convexHull.v[e2], edge));

		edge.PerpendicularBasis(u, v);

		for(size_t k = 0; k < convexHull.v.size(); ++k)
			pts[k] = float2(u.Dot(convexHull.v[k]), v.Dot(convexHull.v[k]));

		float2 rectCenter;
		float2 uDir;
		float2 vDir;
		float minU, maxU, minV, maxV;
		float rectArea = float2::MinAreaRectInPlace(&pts[0], (int)pts.size(), rectCenter, uDir, vDir, minU, maxU, minV, maxV);

		c10 = (maxV - minV) * vDir;
		c20 = (maxU - minU) * uDir;

		float volume = rectArea*edgeLength;
		if (volume + 1e-5f < bestVolume)
		{
			bestVolume = volume;
			edge = (c10.x*u + c10.y*v);
			float len = edge.Normalize();
			if (len <= 0.f)
				edge = u;
			numTimesNotImproved = 0;
			prevSecondChoice = (c20.x*u + c20.y*v);
			len = prevSecondChoice.Normalize();
			if (len <= 0.f)
				prevSecondChoice = u;
			outEdgeA = edge;
			outEdgeB = prevSecondChoice;

// Enable for a dirty hack to experiment with performance.
//#define NO_JIGGLES

#ifdef NO_JIGGLES
			break;
#endif
		}
		else
		{
			++numTimesNotImproved;
			edge = prevSecondChoice;
		}

		if (--numJiggles <= 0)
			break;
	}
	return bestVolume;
}

// Moves the floating point sign bit from src to dst.
#ifdef MATH_SSE
#define MoveSign(dst, src) \
	dst = s4f_x(xor_ps(setx_ps(dst), and_ps(setx_ps(src), simd4fSignBit))); \
	src = s4f_x(abs_ps(setx_ps(src)));
#else
#define MoveSign(dst, src) if (src < 0.f) { dst = -dst; src = -src; }
#endif

int ComputeBasis(const vec &f1a, const vec &f1b,
	const vec &f2a, const vec &f2b,
	const vec &f3a, const vec &f3b,
	vec *n1,
	vec *n2,
	vec *n3)
{
	const float eps = 1e-4f;
	const float angleEps = 1e-3f;

	{
		vec a = f1b;
		vec b = f1a-f1b;
		vec c = f2b;
		vec d = f2a-f2b;
		vec e = f3b;
		vec f = f3a-f3b;

		float g = a.Dot(c)*d.Dot(e) - a.Dot(d)*c.Dot(e);
		float h = a.Dot(c)*d.Dot(f) - a.Dot(d)*c.Dot(f);
		float i = b.Dot(c)*d.Dot(e) - b.Dot(d)*c.Dot(e);
		float j = b.Dot(c)*d.Dot(f) - b.Dot(d)*c.Dot(f);

		float k = g*b.Dot(e) - a.Dot(e)*i;
		float l = h*b.Dot(e) + g*b.Dot(f) - a.Dot(f)*i - a.Dot(e)*j;
		float m = h*b.Dot(f) - a.Dot(f)*j;

		float s = l*l - 4*m*k;

		if (Abs(m) < 1e-5f || Abs(s) < 1e-5f)
		{
			// The equation is linear instead.

			float v = -k / l;
			float t = -(g + h*v) / (i + j*v);
			float u = -(c.Dot(e) + c.Dot(f)*v) / (d.Dot(e) + d.Dot(f)*v);
			int nSolutions = 0;
			// If we happened to divide by zero above, the following checks handle them.
			if (v >= -eps && t >= -eps && u >= -eps && v <= 1.f + eps && t <= 1.f + eps && u <= 1.f + eps)
			{
				n1[0] = (a + b*t).Normalized();
				n2[0] = (c + d*u).Normalized();
				n3[0] = (e + f*v).Normalized();
				if (Abs(n1[0].Dot(n2[0])) < angleEps
					&& Abs(n1[0].Dot(n3[0])) < angleEps
					&& Abs(n2[0].Dot(n3[0])) < angleEps)
					return 1;
				else
					return 0;
			}
			return nSolutions;
		}

		if (s < 0.f)
			return 0; // Discriminant negative, no solutions for v.

		float sgnL = l < 0 ? -1.f : 1.f;
		float V1 = -(l + sgnL*Sqrt(s))/ (2.f*m);
		float V2 = k / (m*V1);

		float T1 = -(g + h*V1) / (i + j*V1);
		float T2 = -(g + h*V2) / (i + j*V2);

		float U1 = -(c.Dot(e) + c.Dot(f)*V1) / (d.Dot(e) + d.Dot(f)*V1);
		float U2 = -(c.Dot(e) + c.Dot(f)*V2) / (d.Dot(e) + d.Dot(f)*V2);

		int nSolutions = 0;
		if (V1 >= -eps && T1 >= -eps && U1 >= -eps && V1 <= 1.f + eps && T1 <= 1.f + eps && U1 <= 1.f + eps)
		{
			n1[nSolutions] = (a + b*T1).Normalized();
			n2[nSolutions] = (c + d*U1).Normalized();
			n3[nSolutions] = (e + f*V1).Normalized();

			if (Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (V2 >= -eps && T2 >= -eps && U2 >= -eps && V2 <= 1.f + eps && T2 <= 1.f + eps && U2 <= 1.f + eps)
		{
			n1[nSolutions] = (a + b*T2).Normalized();
			n2[nSolutions] = (c + d*U2).Normalized();
			n3[nSolutions] = (e + f*V2).Normalized();
			if (Abs(n1[nSolutions].Dot(n2[nSolutions])) < angleEps
				&& Abs(n1[nSolutions].Dot(n3[nSolutions])) < angleEps
				&& Abs(n2[nSolutions].Dot(n3[nSolutions])) < angleEps)
				++nSolutions;
		}
		if (s < 1e-4f && nSolutions == 2)
			 nSolutions = 1;
				
		return nSolutions;
	}
}

// A heuristic(?) that checks of the face normals of two edges are not suitably oriented.
// This is used to skip certain configurations.
static bool AreEdgesBad(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	MARK_UNUSED(f1a);
	MARK_UNUSED(f1b);
	MARK_UNUSED(f2a);
	MARK_UNUSED(f2b);
	return false;
	// Currently disabled. It's not completely certain if there's a form of this heuristic that
	// might be perfect, needs more tweaking.
#if 0 
	float a1 = Abs(f1a.Dot(f1b));
	float a2 = Abs(f2a.Dot(f2b));
	float b1 = Abs(f1a.Dot(f2b));
	float b2 = Abs(f2a.Dot(f1b));
	const float limitEpsilon = 1e-4f;

	if ((a1 > 1.f - limitEpsilon && a2 < limitEpsilon) || (a2 > 1.f - limitEpsilon && a1 < limitEpsilon))
		return true;
	if ((b1 > 1.f - limitEpsilon && b2 < limitEpsilon) || (b2 > 1.f - limitEpsilon && b1 < limitEpsilon))
		return true;

	if ((a1 > 1.f - limitEpsilon || a1 < limitEpsilon) && (a2 > 1.f - limitEpsilon || a2 < limitEpsilon))
		return true;
	if ((b1 > 1.f - limitEpsilon || b1 < limitEpsilon) && (b2 > 1.f - limitEpsilon || b2 < limitEpsilon))
		return true;

	return false;
#endif
}

static bool AreEdgesCompatibleForOBB(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b)
{
	const vec f1a_f1b = f1a-f1b;
	const vec f2a_f2b = f2a-f2b;
	float a = f1b.Dot(f2b);
	float b = (f1a_f1b).Dot(f2b);
	float c = (f2a_f2b).Dot(f1b);
	float d = (f1a_f1b).Dot(f2a_f2b);

	/*
		n1 = f1a*t + f1b*(1-t) = f1b + (f1a-f1b)*t
		n2 = f2a*u + f2b*(1-u) = f2b + (f2a-f2b)*u

		n1.n2 = 0
			f1b.f2b + t*(f1a-f1b).f2b + u*(f2a-f2b).f1b + t*u*(f1a-f1b).(f2a-f2b) = 0
			a + t*b + u*c + t*u*d = 0

		Does this equation have a solution within t & u \in [0,1]?

		// The function f(t,u) = a + t*b + u*c + t*u*d is continuous and bilinear
		// with respect to t and u, so test the four corners to get the minimum
		// and maximum of the function. If minimum <= 0 and
		// maximum >= 0, we know it must have a zero inside t,u \in [0,1].

		t=0: f(t,u)=a+uc   => min: a, max: a+c
		u=0: f(t,u)=a+tb   => min: a, max: a+b
		t=1: f(t,u)=a+uc + b+ud  => min: a+b, max: a+b+c+d
		u=1: f(t,u)=a+tb + c+td  => min: a+c, max: a+b+c+d
	*/
	float ab = a+b;
	float ac = a+c;
	float abcd = ab+c+d;
	float minVal = Min(a, ab, ac, abcd);
	float maxVal = Max(a, ab, ac, abcd);
	return minVal <= 0.f && maxVal >= 0.f;
}

// Enable this to add extra runtime checks to sanity test that the generated OBB is actually valid.
//#define OBB_ASSERT_VALIDITY

// Enable this to add internal debug prints for tracking internal behavior.
//#define OBB_DEBUG_PRINT

// Enable this to print out detailed profiling info.
//#define ENABLE_TIMING

#ifdef ENABLE_TIMING
#define TIMING_TICK(...) __VA_ARGS__
#define TIMING LOGI
#else
#define TIMING(...) ((void)0)
#define TIMING_TICK(...) ((void)0)
#endif

namespace
{
	struct hash_edge
	{
		size_t operator()(const std::pair<int, int> &e) const
		{
			return (e.first << 16) ^ e.second;
		}
	};
}

bool AreCompatibleOpposingEdges(const vec &f1a, const vec &f1b, const vec &f2a, const vec &f2b, vec &outN)
{
	/*
		n1 = f1a*t + f1b*(1-t)
		n2 = f2a*u + f2b*(1-u)
		n1 = -c*n2, where c > 0
		f1a*t + f1b*(1-t) = -c*f2a*u - c*f2b*(1-u)
		f1a*t - f1b*t + cu*f2a + c*f2b - cu*f2b = -f1b
		c*f2b + t*(f1a-f1b) + cu*(f2a-f2b) = -f1b

		M * v = -f1b, where

		M = [ f2b, (f1a-f1b), (f2a-f2b) ] column vectors
		v = [c, t, cu]
	*/

	const float tooCloseToFaceEpsilon = 1e-4f;

	float3x3 A;
	A.SetCol(0, f2b.xyz()); // c
	A.SetCol(1, (f1a - f1b).xyz()); // t
	A.SetCol(2, (f2a - f2b).xyz()); // r = c*u
	float3 x;
	bool success = A.SolveAxb(-f1b.xyz(), x);
	float c = x[0];
	float t = x[1];
	float cu = x[2];
	if (!success || c <= 0.f || t < 0.f || t > 1.f)
		return false;
	float u = cu / c;
	if (t < tooCloseToFaceEpsilon || t > 1.f - tooCloseToFaceEpsilon
		|| u < tooCloseToFaceEpsilon || u > 1.f - tooCloseToFaceEpsilon)
		return false;
	if (cu < 0.f || cu > c)
		return false;
	outN = f1b + (f1a-f1b)*t;
	return true;
}

OBB OBB::OptimalEnclosingOBB(const vec *pointArray, int numPoints)
{
	// Precomputation: Generate the convex hull of the input point set. This is because
	// we need vertex-edge-face connectivity information about the convex hull shape, and
	// this also allows discarding all points in the interior of the input hull, which
	// are irrelevant.
	Polyhedron convexHull = Polyhedron::ConvexHull(pointArray, numPoints);
	if (!pointArray || convexHull.v.size() == 0)
	{
		OBB minOBB;
		minOBB.SetNegativeInfinity();
		return minOBB;
	}
	return OptimalEnclosingOBB(convexHull);
}

bool SortedArrayContains(const std::vector<int> &arr, int i)
{
	size_t left = 0;
	size_t right = arr.size() - 1;
	if (arr[left] == i || arr[right] == i)
		return true;
	if (arr[left] > i || arr[right] < i)
		return false;

	while(left < right)
	{
		size_t middle = (left + right + 1) >> 1;
		if (arr[middle] < i)
			left = i;
		else if (arr[middle] > i)
			right = i;
		else
			return true;
	}
	return false;
}

bool IsVertexAntipodalToEdge(const Polyhedron &convexHull, int vi, const std::vector<int> &neighbors, const vec &f1a, const vec &f1b)
{
	float tMin = 0.f;
	float tMax = 1.f;

	vec v = convexHull.v[vi];
	vec f1b_f1a = f1b-f1a;
	for(size_t i = 0; i < neighbors.size(); ++i)
	{
		/* Is an edge and a vertex compatible to be antipodal?

			n1 = f1b + (f1a-f1b)*t
			e { v-vn }

			n1.e <= 0
			(f1b + (f1a-f1b)*t).e <= 0
			t*(f1a-f1b).e <= -f1b.e 
			if (f1a-f1b).e > 0:
				t <= -f1b.e / (f1a-f1b).e && t \in [0,1]
				-f1b.e / (f1a-f1b).e >= 0
				f1b.e <= 0
			if (f1a-f1b).e < 0:
				t >= -f1b.e / (f1a-f1b).e && t \in [0,1]
				-f1b.e / (f1a-f1b).e <= 1
				-f1b.e >= (f1a-f1b).e
				f1b.e + (f1a-f1b).e <= 0 
			if (f1a-f1b).e == 0:
				0 <= -f1b.e
		*/
		vec e = vec(convexHull.v[neighbors[i]]) - v;
		float s = f1b_f1a.Dot(e);
		float n = f1b.Dot(e);
		const float epsilon = 1e-4f;
		if (s > epsilon)
			tMax = Min(tMax, n / s);
		else if (s < -epsilon)
			tMin = Max(tMin, n / s);
		else if (n < -epsilon)
			return false;

		// The interval of possible solutions for t is now degenerate?
		if (tMax - tMin < -5e-2f) // -1e-3f has been seen to be too strict here.
			return false;
	}
	return true;
}

void FORCE_INLINE TestThreeAdjacentFaces(const vec &n1, const vec &n2, const vec &n3,
	int edgeI, int edgeJ, int edgeK,
	const Polyhedron &convexHull, const std::vector<std::pair<int, int> > &edges,
	const std::vector<std::vector<int> > &antipodalPointsForEdge,
	float *minVolume, OBB *minOBB)
{
	// Compute the most extreme points in each direction.
	float maxN1 = n1.Dot(convexHull.v[edges[edgeI].first]);
	float maxN2 = n2.Dot(convexHull.v[edges[edgeJ].first]);
	float maxN3 = n3.Dot(convexHull.v[edges[edgeK].first]);
	float minN1 = FLOAT_INF;
	float minN2 = FLOAT_INF;
	float minN3 = FLOAT_INF;
	for(size_t l = 0; l < antipodalPointsForEdge[edgeI].size(); ++l) // O(constant)?
		minN1 = Min(minN1, n1.Dot(convexHull.v[antipodalPointsForEdge[edgeI][l]]));
	for(size_t l = 0; l < antipodalPointsForEdge[edgeJ].size(); ++l) // O(constant)?
		minN2 = Min(minN2, n2.Dot(convexHull.v[antipodalPointsForEdge[edgeJ][l]]));
	for(size_t l = 0; l < antipodalPointsForEdge[edgeK].size(); ++l) // O(constant)?
		minN3 = Min(minN3, n3.Dot(convexHull.v[antipodalPointsForEdge[edgeK][l]]));
	float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
	if (volume < *minVolume)
	{
		minOBB->axis[0] = n1;
		minOBB->axis[1] = n2;
		minOBB->axis[2] = n3;
		minOBB->r[0] = (maxN1 - minN1) * 0.5f;
		minOBB->r[1] = (maxN2 - minN2) * 0.5f;
		minOBB->r[2] = (maxN3 - minN3) * 0.5f;
		minOBB->pos = (minN1 + minOBB->r[0])*n1 + (minN2 + minOBB->r[1])*n2 + (minN3 + minOBB->r[2])*n3;
		assert(volume > 0.f);
#ifdef OBB_ASSERT_VALIDITY
		OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB->axis[0], minOBB->axis[1]);
		assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
		*minVolume = volume;
	}
}

OBB OBB::OptimalEnclosingOBB(const Polyhedron &convexHull)
{
	/* Outline of the algorithm:
	  0. Compute the convex hull of the point set (given as input to this function) O(VlogV)
	  1. Compute vertex adjacency data, i.e. given a vertex, return a list of its neighboring vertices. O(V)
	  2. Precompute face normal direction vectors, since these are needed often. (does not affect big-O complexity, just a micro-opt) O(F)
	  3. Compute edge adjacency data, i.e. given an edge, return the two indices of its neighboring faces. O(V)
	  4. Precompute antipodal vertices for each edge. O(A*ElogV), where A is the size of antipodal vertices per edge. A ~ O(1) on average.
	  5. Precompute all sidepodal edges for each edge. O(E*S), where S is the size of sidepodal edges per edge. S ~ O(sqrtE) on average.
	     - Sort the sidepodal edges to a linear order so that it's possible to do fast set intersection computations on them. O(E*S*logS), or O(E*sqrtE*logE).
	  6. Test all configurations where all three edges are on adjacent faces. O(E*S^2) = O(E^2) or if smart with graph search, O(ES) = O(E*sqrtE)?
	  7. Test all configurations where two edges are on opposing faces, and the third one is on a face adjacent to the two. O(E*sqrtE*logV)?
	  8. Test all configurations where two edges are on the same face (OBB aligns with a face of the convex hull). O(F*sqrtE*logV).
	  9. Return the best found OBB.
	*/

	OBB minOBB;
	float minVolume = FLOAT_INF;

	// Handle degenerate planar cases up front.
	if (convexHull.v.size() <= 3 || convexHull.f.size() <= 1)
	{
		// TODO
		LOGW("Convex hull is degenerate and has only %d vertices/%d faces!", (int)convexHull.v.size(), (int)convexHull.f.size());
		minOBB.SetNegativeInfinity();
		return minOBB;
	}
	TIMING_TICK(tick_t t1 = Clock::Tick());
	// Precomputation: For each vertex in the convex hull, compute their neighboring vertices.
	std::vector<std::vector<int> > adjacencyData = convexHull.GenerateVertexAdjacencyData(); // O(V)
	TIMING_TICK(tick_t t2 = Clock::Tick());
	TIMING("Adjacencygeneration: %f msecs", Clock::TimespanToMillisecondsF(t1, t2));

	// Precomputation: Compute normalized face direction vectors for each face of the hull.
	//std::vector<vec_storage> faceNormals;
	VecArray faceNormals;
	faceNormals.reserve(convexHull.NumFaces());
	for(int i = 0; i < convexHull.NumFaces(); ++i) // O(F)
	{
		if (convexHull.f[i].v.size() < 3)
		{
			LOGE("Input convex hull contains a degenerate face %d with only %d vertices! Cannot process this!",
				i, (int)convexHull.f[i].v.size());
			return minOBB;
		}
		vec normal = convexHull.FaceNormal(i);
		faceNormals.push_back(DIR_VEC((float)normal.x, (float)normal.y, (float)normal.z));
	}

	TIMING_TICK(tick_t t23 = Clock::Tick());
	TIMING("Facenormalsgen: %f msecs", Clock::TimespanToMillisecondsF(t2, t23));

#ifdef OBB_ASSERT_VALIDITY
	// For debugging, assert that face normals in the input Polyhedron are pointing in valid directions:
	for(size_t i = 0; i < faceNormals.size(); ++i)
	{
		vec pointOnFace = convexHull.v[convexHull.f[i].v[0]];
		for(size_t j = 0; j < convexHull.v.size(); ++j)
		{
			vec pointDiff = vec(convexHull.v[j]) - pointOnFace;
			float signedDistance = pointDiff.Dot(faceNormals[i]);
			if (signedDistance > 1e-1f)
			{
				// Find how deep there are points on the opposite side.
				int extremeOpposite = convexHull.ExtremeVertex(-vec(faceNormals[i]));
				float signedDistanceOpposite = Dot(vec(convexHull.v[extremeOpposite]) - pointOnFace, faceNormals[i]);
				LOGE("Vertex %d is %f units deep on the wrong side of face %d (which has %d vertices and surface area %f): On the opposite side, extreme signed distance is vtx %d at d %f.", (int)j, signedDistance, 
					(int)i, (int)convexHull.f[i].v.size(), convexHull.FacePolygon(i).Area(), extremeOpposite, signedDistanceOpposite);
				break;
			}
		}
	}
#endif

	// For each edge i, specifies the two vertices v0 and v1 through which that edge goes.
	// This array does not have duplicates, i.e. there only exists one edge index for (v0->v1), and no
	// edge for (v1->v0).
	std::vector<std::pair<int, int> > edges;
	edges.reserve(convexHull.v.size() * 2);
	// For each edge i, specifies the two face indices f0 and f1 that share that edge.
	std::vector<std::pair<int, int> > facesForEdge;
	facesForEdge.reserve(convexHull.v.size()*2);
	// For each vertex pair (v0, v1) through which there is an edge, specifies the index i of the edge that passes through them.
	// This map contains duplicates, so both (v0, v1) and (v1, v0) map to the same edge index.
	// Currently use a O(V^2) array for this data structure for performance.
	// TODO: Add compile option for using unordered_map for O(V) storage, but that's somewhat slower.
	//	std::unordered_map<std::pair<int, int>, int, hash_edge> vertexPairsToEdges;
	unsigned int *vertexPairsToEdges = new unsigned int[convexHull.v.size()*convexHull.v.size()];
	memset(vertexPairsToEdges, 0xFF, sizeof(unsigned int)*convexHull.v.size()*convexHull.v.size());
#define EMPTY_EDGE 0xFFFFFFFFU

	// Precomputation: for each edge through vertices (i,j), we need to know
	// the face indices for the two adjoining faces that share the edge.
	// This is O(V).
	for (size_t i = 0; i < convexHull.f.size(); ++i)
	{
		const Polyhedron::Face &f = convexHull.f[i];
		int v0 = f.v.back();
		for(size_t j = 0; j < f.v.size(); ++j)
		{
			int v1 = f.v[j];
			std::pair<int, int> e = std::make_pair(v0, v1);
			//std::unordered_map<std::pair<int, int>, int, hash_edge>::const_iterator iter = vertexPairsToEdges.find(e);
			//if (iter == vertexPairsToEdges.end())
			if (vertexPairsToEdges[v0*convexHull.v.size()+v1] == EMPTY_EDGE)
			{
				//vertexPairsToEdges[e] = (int)edges.size();
				vertexPairsToEdges[v0*convexHull.v.size()+v1] = (unsigned int)edges.size();
				vertexPairsToEdges[v1*convexHull.v.size()+v0] = (unsigned int)edges.size();
//				vertexPairsToEdges[std::make_pair(v1, v0)] = (int)edges.size(); // Mark that we know we have seen v0->v1 already.
				edges.push_back(e);
				facesForEdge.push_back(std::make_pair((int)i, -1)); // The -1 will be filled once we see the edge v1->v0.
			}
			else
				facesForEdge[vertexPairsToEdges[v0*(int)convexHull.v.size()+v1]/*iter->second*/].second = (int)i;
			v0 = v1;
		}
	}
	TIMING_TICK(tick_t t3 = Clock::Tick());
	TIMING("Adjoiningfaces: %f msecs", Clock::TimespanToMillisecondsF(t23, t3));

	// Throughout the algorithm, internal edges can all be discarded, so provide a helper macro to test for that.
#define IS_INTERNAL_EDGE(i) (reinterpret_cast<vec*>(&faceNormals[facesForEdge[i].first])->Dot(faceNormals[facesForEdge[i].second]) > 1.f - 1e-4f)

	TIMING_TICK(
		int numInternalEdges = 0;
		for(size_t i = 0; i < edges.size(); ++i)
			if (IS_INTERNAL_EDGE(i))
				++numInternalEdges;
	);
	TIMING("%d/%d (%.2f%%) edges are internal and will be ignored.", numInternalEdges, (int)edges.size(), numInternalEdges * 100.0 / edges.size());

#ifdef OBB_DEBUG_PRINT
	for(size_t i = 0; i < convexHull.v.size(); ++i)
		LOGI("v[%d] = %s", (int)i, vec(convexHull.v[i]).ToString().c_str());
	for(size_t i = 0; i < edges.size(); ++i)
		LOGI("e[%d] = %d->%d", (int)i, edges[i].first, edges[i].second);
#endif

	// Throughout the whole algorithm, this array stores an auxiliary structure for performing graph searches
	// on the vertices of the convex hull. Conceptually each index of the array stores a boolean whether we
	// have visited that vertex or not during the current search. However storing such booleans is slow, since
	// we would have to perform a linear-time scan through this array before next search to reset each boolean
	// to unvisited false state. Instead, store a number, called a "color" for each vertex to specify whether
	// that vertex has been visited, and manage a global color counter floodFillVisitColor that represents the
	// visited vertices. At any given time, the vertices that have already been visited have the value
	// floodFillVisited[i] == floodFillVisitColor in them. This gives a win that we can perform constant-time
	// clears of the floodFillVisited array, by simply incrementing the "color" counter to clear the array.
	std::vector<unsigned int> floodFillVisited(convexHull.v.size());
	unsigned int floodFillVisitColor = 1;

	// As a syntactic aid, use the helpers MARK_VISITED(v), HAVE_VISITED_VERTEX(v) and CLEAR_GRAPH_SEARCH to
	// remind of the conceptual meaning of these values.
#define MARK_VERTEX_VISITED(v) (floodFillVisited[(v)] = floodFillVisitColor)
#define HAVE_VISITED_VERTEX(v) (floodFillVisited[(v)] == floodFillVisitColor) // HAVE_VISITED_VERTEX(v) implies HAVE_QUEUED_VERTEX(v)
#define CLEAR_GRAPH_SEARCH() (++floodFillVisitColor)

	// Stores for each edge index i the complete list of antipodal vertices for that edge.
	std::vector<std::vector<int> > antipodalPointsForEdge(edges.size());

	TIMING_TICK(
		tick_t tz = Clock::Tick();
		unsigned long long numSpatialStrips = 0);

	// The currently best variant for establishing a spatially coherent traversal order.
	std::vector<int> spatialFaceOrder;
	std::vector<int> spatialEdgeOrder;
	LCG rng(123);
	{ // Explicit scope for variables that are not needed after this.
		TIMING_TICK(int prevEdgeEnd = -1);
		std::vector<std::pair<int, int> > traverseStackEdges;
		std::vector<unsigned int> visitedEdges(edges.size());
		std::vector<unsigned int> visitedFaces(convexHull.f.size());
		traverseStackEdges.push_back(std::make_pair(0, adjacencyData[0].front()));
		while(!traverseStackEdges.empty())
		{
			std::pair<int, int> e = traverseStackEdges.back();
			traverseStackEdges.pop_back();
			int thisEdge = vertexPairsToEdges[e.first*convexHull.v.size()+e.second];
			if (visitedEdges[thisEdge])
				continue;
			visitedEdges[thisEdge] = 1;
			if (!visitedFaces[facesForEdge[thisEdge].first])
			{
				visitedFaces[facesForEdge[thisEdge].first] = 1;
				spatialFaceOrder.push_back(facesForEdge[thisEdge].first);
			}
			if (!visitedFaces[facesForEdge[thisEdge].second])
			{
				visitedFaces[facesForEdge[thisEdge].second] = 1;
				spatialFaceOrder.push_back(facesForEdge[thisEdge].second);
			}
			TIMING_TICK(
				if (prevEdgeEnd != e.first)
					++numSpatialStrips;
				prevEdgeEnd = e.second;
			);
			if (!IS_INTERNAL_EDGE(thisEdge))
				spatialEdgeOrder.push_back(thisEdge);
			int v0 = e.second;
			size_t sizeBefore = traverseStackEdges.size();
			for(size_t i = 0; i < adjacencyData[v0].size(); ++i)
			{
				int v1 = adjacencyData[v0][i];
				int e1 = vertexPairsToEdges[v0*convexHull.v.size()+v1];
				if (visitedEdges[e1])
					continue;
				traverseStackEdges.push_back(std::make_pair(v0, v1));
			}
			// Take a random adjacent edge.
			int nNewEdges = (int)(traverseStackEdges.size() - sizeBefore);
			if (nNewEdges > 0)
			{
				int r = rng.Int(0, nNewEdges - 1);
				std::swap(traverseStackEdges.back(), traverseStackEdges[sizeBefore+r]);
			}
		}
	}

	TIMING_TICK(tick_t tx = Clock::Tick());
	TIMING("SpatialOrder: %f msecs, search over %d edges is in %llu strips, approx. %f edges/strip",
		Clock::TimespanToMillisecondsF(tz, tx), (int)edges.size(), numSpatialStrips, (double)edges.size()/numSpatialStrips);

	// Stores a memory of yet unvisited vertices for current graph search.
	std::vector<int> traverseStack;

// Enable this to remove a big chunk of intelligent logic in favor of very naive operation to find antipodal vertices.
// Use for debugging/crossreferencing against the smart version.
//#define NAIVE_ANTIPODAL_SEARCH

#ifdef NAIVE_ANTIPODAL_SEARCH
	// A naive O(E*V) algorithm for computing all antipodal points for each edge.
	for (size_t i = 0; i < edges.size(); ++i) // O(E)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];
		for(size_t j = 0; j < convexHull.v.size(); ++j) // O(V)
			if (IsVertexAntipodalToEdge(convexHull, j, adjacencyData[j], f1a, f1b))
				antipodalPointsForEdge[i].push_back(j);
	}
#endif

	// Since we do several extreme vertex searches, and the search directions have a lot of spatial locality,
	// always start the search for the next extreme vertex from the extreme vertex that was found during the
	// previous iteration for the previous edge. This has been profiled to improve overall performance by as
	// much as 15-25%.
	int startingVertex = 0;

	TIMING_TICK(
		unsigned long long numVertexNeighborSearches = 0;
		unsigned long long numVertexNeighborSearchImprovements = 0;
	);

#ifndef NAIVE_ANTIPODAL_SEARCH

	// Precomputation: for each edge, we need to compute the list of potential antipodal points (points on
	// the opposing face of an enclosing OBB of the face that is flush with the given edge of the polyhedron).
	// This is O(E*log(V)) ?
	//for (size_t i = 0; i < edges.size(); ++i) // O(E)
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		float dummy;
		CLEAR_GRAPH_SEARCH(); // ExtremeVertexConvex performs a graph search, initialize the search data structure for it.
		startingVertex = convexHull.ExtremeVertexConvex(adjacencyData, -f1a, floodFillVisited, floodFillVisitColor, dummy, startingVertex); // O(log(V)?
		TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
		TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
		CLEAR_GRAPH_SEARCH(); // Search through the graph for all adjacent antipodal vertices.
		traverseStack.push_back(startingVertex);
		MARK_VERTEX_VISITED(startingVertex);
		while(!traverseStack.empty()) // In amortized analysis, only a constant number of vertices are antipodal points for any edge?
		{
			int v = traverseStack.back();
			traverseStack.pop_back();
			const std::vector<int> &neighbors = adjacencyData[v];
			if (IsVertexAntipodalToEdge(convexHull, v, neighbors, f1a, f1b))
			{
				assume(edges[i].first != v && edges[i].second != v);
				if (edges[i].first == v || edges[i].second == v)
				{
					LOGE("Edge %d: %d->%d is antipodal to vertex %d, which is part of the same edge! This should be possible only if the input is degenerate planar!",
						(int)i, edges[i].first, edges[i].second, v);
					minOBB.SetNegativeInfinity();
					delete[] vertexPairsToEdges;
					return minOBB;
				}
				antipodalPointsForEdge[i].push_back(v);
				for(size_t j = 0; j < neighbors.size(); ++j)
					if (!HAVE_VISITED_VERTEX(neighbors[j]))
					{
						traverseStack.push_back(neighbors[j]);
						MARK_VERTEX_VISITED(neighbors[j]);
					}
			}
		}
		// Robustness: If the above search did not find any antipodal points, add the first found extreme
		// point at least, since it is always an antipodal point. This is known to occur very rarely due
		// to numerical imprecision in the above loop over adjacent edges.
		if (antipodalPointsForEdge[i].empty())
		{
//			LOGW("Due to numerical stability issues(?), could not find an antipodal vertex for edge %d! Check the scale of your input dataset! Good scale is having coordinates range e.g. [0.0,100.0]", edges[i]);
			// Getting here is most likely a bug. Fall back to linear scan, which is very slow.
			for(size_t j = 0; j < convexHull.v.size(); ++j) // O(V)
				if (IsVertexAntipodalToEdge(convexHull, (int)j, adjacencyData[j], f1a, f1b))
					antipodalPointsForEdge[i].push_back((int)j);

//			antipodalPointsForEdge[i].push_back(startingVertex);
		}
	}
#endif

	TIMING_TICK(
		tick_t t4 = Clock::Tick();
		size_t numTotalAntipodals = 0;
		for (size_t i = 0; i < antipodalPointsForEdge.size(); ++i)
			numTotalAntipodals += antipodalPointsForEdge[i].size();
	);
	TIMING("Antipodalpoints: %f msecs (avg edge has %.3f antipodal points)", Clock::TimespanToMillisecondsF(t3, t4), (float)numTotalAntipodals/edges.size());
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/edges.size(), (double)numVertexNeighborSearchImprovements/edges.size());

#ifdef OBB_DEBUG_PRINT
	for (size_t i = 0; i < antipodalPointsForEdge.size(); ++i)
	{
		std::string s;
		for (size_t j = 0; j < antipodalPointsForEdge[i].size(); ++j)
		{
			char str[256];
			sprintf(str, "%d ", (int)antipodalPointsForEdge[i][j]);
			s += str;
		}
		LOGI("Antipodal points for edge %d: %s", (int)i, s.c_str());
	}
#endif

	// Stores for each edge i the list of all sidepodal edge indices j that it can form an OBB with.
	std::vector<std::vector<int> > compatibleEdges(edges.size());

	// Use a O(E*V) data structure for sidepodal vertices.
	unsigned char *sidepodalVertices = new unsigned char[edges.size()*convexHull.v.size()];
	memset(sidepodalVertices, 0, sizeof(unsigned char)*edges.size()*convexHull.v.size());

// Enable this to perform a dumb search for sidepodal edges. Makes sense mostly for debugging the behavior of the smarter version.
//#define NAIVE_SIDEPODAL_SEARCH

#ifdef NAIVE_SIDEPODAL_SEARCH
	// Precomputation: Compute all potential companion edges for each edge.
	// This is O(E^2)
	// Important! And edge can be its own companion edge! So have each edge test itself during iteration.
	for(size_t i = 0; i < edges.size(); ++i) // O(E)
	{
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];
		for(size_t j = i; j < edges.size(); ++j) // O(E)
			if (AreEdgesCompatibleForOBB(f1a, f1b, faceNormals[facesForEdge[j].first], faceNormals[facesForEdge[j].second]))
			{
				compatibleEdges[i].push_back(j);
				sidepodalVertices[i*convexHull.v.size()+edges[j].first] = 1;
				sidepodalVertices[i*convexHull.v.size()+edges[j].second] = 1;

				if (i != j)
				{
					compatibleEdges[j].push_back(i);
					sidepodalVertices[j*convexHull.v.size()+edges[i].first] = 1;
					sidepodalVertices[j*convexHull.v.size()+edges[i].second] = 1;
				}
			}
	}
#else

	// Compute all sidepodal edges for each edge by performing a graph search. The set of sidepodal edges is
	// connected in the graph, which lets us avoid having to iterate over each edge pair of the convex hull.
	// Total running time is O(E*sqrtE).
	TIMING_TICK(
		numVertexNeighborSearches = 0;
		numVertexNeighborSearchImprovements = 0;
	);
//	for(size_t i = 0; i < edges.size(); ++i) // O(E)
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		vec deadDirection = (f1a+f1b)*0.5f;
		vec basis1, basis2;
		deadDirection.PerpendicularBasis(basis1, basis2);

		float dummy;
		vec dir = f1a.Perpendicular();
		CLEAR_GRAPH_SEARCH();
		startingVertex = convexHull.ExtremeVertexConvex(adjacencyData, dir, floodFillVisited, floodFillVisitColor, dummy, startingVertex); // O(log|V|)
		TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
		TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
		CLEAR_GRAPH_SEARCH();
		traverseStack.push_back(startingVertex);
		while(!traverseStack.empty()) // O(sqrt(E))
		{
			int v = traverseStack.back();
			traverseStack.pop_back();
			if (HAVE_VISITED_VERTEX(v))
				continue;
			MARK_VERTEX_VISITED(v);

			const std::vector<int> &n = adjacencyData[v];
			for(size_t j = 0; j < n.size(); ++j)
			{
				int vAdj = n[j];
				if (HAVE_VISITED_VERTEX(vAdj))
					continue;

//				int edge = vertexPairsToEdges[std::make_pair(v, vAdj)];
				int edge = vertexPairsToEdges[v*convexHull.v.size()+vAdj];
//				if (IS_INTERNAL_EDGE(edge))
//					continue; // Edges inside faces with 180 degrees dihedral angles can be ignored.
				if (AreEdgesCompatibleForOBB(f1a, f1b, faceNormals[facesForEdge[edge].first], faceNormals[facesForEdge[edge].second]))
				{
					if ((int)i <= edge)
					{
						if (!IS_INTERNAL_EDGE(edge))
							compatibleEdges[i].push_back(edge);

						sidepodalVertices[i*convexHull.v.size()+edges[edge].first] = 1;
						sidepodalVertices[i*convexHull.v.size()+edges[edge].second] = 1;

//						sidepodalVertices[i].insert(edges[edge].first);
//						sidepodalVertices[i].insert(edges[edge].second);
						if ((int)i != edge)
						{
//							vec f2a = faceNormals[facesForEdge[edge].first];
//							vec f2b = faceNormals[facesForEdge[edge].second];

							if (!IS_INTERNAL_EDGE(edge))
								compatibleEdges[edge].push_back((int)i);
//							sidepodalVertices[edge].insert(edges[i].first);
//							sidepodalVertices[edge].insert(edges[i].second);
							sidepodalVertices[edge*convexHull.v.size()+edges[i].first] = 1;
							sidepodalVertices[edge*convexHull.v.size()+edges[i].second] = 1;
//							sidepodalVerticesForEdge[edge].push_back(edges[i].first);
//							sidepodalVerticesForEdge[edge].push_back(edges[i].second);
//							++numSmallerCompatibleEdgeIndices[edge];
						}
					}
					traverseStack.push_back(vAdj);
				}
			}
		}
	}
#endif

#if 0
	for(size_t i = 0; i < sidepodalVerticesForEdge.size(); ++i) // O(E)
		std::sort(sidepodalVerticesForEdge[i].begin(), sidepodalVerticesForEdge[i].end());
#endif

	TIMING_TICK(
		tick_t t5 = Clock::Tick();
		size_t numTotalEdges = 0;
		for(size_t i = 0; i < compatibleEdges.size(); ++i)
			numTotalEdges += compatibleEdges[i].size();
	);
	TIMING("Companionedges: %f msecs (%d edges have on average %d companion edges each)", Clock::TimespanToMillisecondsF(t4, t5), (int)compatibleEdges.size(), (int)(numTotalEdges / compatibleEdges.size()));
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/edges.size(), (double)numVertexNeighborSearchImprovements/edges.size());
	TIMING_TICK(tick_t ts = Clock::Tick(););
	TIMING_TICK(tick_t tb = Clock::Tick(););
	TIMING("SortCompanionedges: %f msecs", Clock::TimespanToMillisecondsF(ts, tb));
	TIMING_TICK(t5 = Clock::Tick());

#ifdef OBB_DEBUG_PRINT
	for (size_t i = 0; i < edges.size(); ++i)
	{
		std::string s;
		for (size_t j = 0; j < compatibleEdges[i].size(); ++j)
		{
			char str[256];
			sprintf(str, "%d ", (int)compatibleEdges[i][j]);
			s += str;
		}
		LOGI("Edge %d:%d->%d is compatible with: %s", (int)i, edges[i].first, edges[i].second, s.c_str());
	}
#endif

	// Take advantage of spatial locality: start the search for the extreme vertex from the extreme vertex
	// that was found during the previous iteration for the previous edge. This speeds up the search since
	// edge directions have some amount of spatial locality and the next extreme vertex is often close
	// to the previous one. Track two hint variables since we are performing extreme vertex searches to
	// two opposing directions at the same time.
	int extremeVertexSearchHint1 = 0;
	int extremeVertexSearchHint2 = 0;
	int extremeVertexSearchHint3 = 0;
	int extremeVertexSearchHint4 = 0;
	int extremeVertexSearchHint1_b = 0;
	int extremeVertexSearchHint2_b = 0;
	int extremeVertexSearchHint3_b = 0;

	TIMING_TICK(
		unsigned long long numConfigsExplored = 0;
		unsigned long long numBootstrapStepsDone = 0;
		unsigned long long numCommonSidepodalStepsDone = 0;
		unsigned long long numEdgeSqrtEdges = 0;
		);

	// Stores a memory of yet unvisited vertices that are common sidepodal vertices to both currently chosen edges for current graph search.
	std::vector<int> traverseStackCommonSidepodals;

//	for(size_t i = 0; i < edges.size(); ++i) // O(|E|)
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		vec deadDirection = (f1a+f1b)*0.5f;

//		vec e1 = (vec(convexHull.v[edges[i].first]) - vec(convexHull.v[edges[i].second])).Normalized();

		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];

		for(size_t j = 0; j < compatibleEdgesI.size(); ++j) // O(sqrt(|E|))?
		{
			int edgeJ = compatibleEdgesI[j];
			if (edgeJ <= (int)i) continue; // Remove symmetry.
			vec f2a = faceNormals[facesForEdge[edgeJ].first];
			vec f2b = faceNormals[facesForEdge[edgeJ].second];
			if (AreEdgesBad(f1a, f1b, f2a, f2b)) continue;
			TIMING_TICK(++numEdgeSqrtEdges);

			vec deadDirection2 = (f2a+f2b)*0.5f;

			vec searchDir = deadDirection.Cross(deadDirection2);
			float len = searchDir.Normalize();
			if (len == 0.f)
			{
				searchDir = f1a.Cross(f2a);
				len = searchDir.Normalize();
				if (len == 0.f)
					searchDir = f1a.Perpendicular();
			}

			CLEAR_GRAPH_SEARCH();
			float dummy;
			extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, searchDir, floodFillVisited, floodFillVisitColor, dummy, extremeVertexSearchHint1); // O(log|V|)?
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
			CLEAR_GRAPH_SEARCH();
			extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -searchDir, floodFillVisited, floodFillVisitColor, dummy, extremeVertexSearchHint2); // O(log|V|)?
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);

// Enable new smarter search for sidepodal vertices. Should be always on.
#define SECOND_GUESS_SIDEPODALS

			int secondSearch = -1;
#ifdef SECOND_GUESS_SIDEPODALS
			if (sidepodalVertices[edgeJ*convexHull.v.size()+extremeVertexSearchHint1]) traverseStackCommonSidepodals.push_back(extremeVertexSearchHint1);
			else traverseStack.push_back(extremeVertexSearchHint1);
			if (sidepodalVertices[edgeJ*convexHull.v.size()+extremeVertexSearchHint2]) traverseStackCommonSidepodals.push_back(extremeVertexSearchHint2);
			else secondSearch = extremeVertexSearchHint2;//traverseStack.push_back(extremeVertexSearchHint2);
#else
			traverseStackCommonSidepodals.push_back(extremeVertexSearchHint1);
			traverseStackCommonSidepodals.push_back(extremeVertexSearchHint2);
#endif

			// Bootstrap to a good vertex that is sidepodal to both edges.
			CLEAR_GRAPH_SEARCH();
			while(!traverseStack.empty())
			{
				int v = traverseStack.front();
				traverseStack.erase(traverseStack.begin());
				if (HAVE_VISITED_VERTEX(v))
					continue;
				MARK_VERTEX_VISITED(v);
				TIMING_TICK(++numBootstrapStepsDone);
				const std::vector<int> &n = adjacencyData[v];
				for(size_t k = 0; k < n.size(); ++k)
				{
					int vAdj = n[k];
					if (!HAVE_VISITED_VERTEX(vAdj) && sidepodalVertices[i*convexHull.v.size()+vAdj])
					{
						if (sidepodalVertices[edgeJ*convexHull.v.size()+vAdj])
						{
							traverseStack.clear();
							if (secondSearch != -1)
							{
								traverseStack.push_back(secondSearch);
								secondSearch = -1;
								MARK_VERTEX_VISITED(vAdj);
							}
							traverseStackCommonSidepodals.push_back(vAdj);
							break;
						}
						else
							traverseStack.push_back(vAdj);
					}
				}
			}

			CLEAR_GRAPH_SEARCH();
			while(!traverseStackCommonSidepodals.empty())
			{
				TIMING_TICK(++numCommonSidepodalStepsDone);
				int v = traverseStackCommonSidepodals.back();
				traverseStackCommonSidepodals.pop_back();
				if (HAVE_VISITED_VERTEX(v))
					continue;
				MARK_VERTEX_VISITED(v);
				const std::vector<int> &n = adjacencyData[v];
				for(size_t k = 0; k < n.size(); ++k)
				{
					int vAdj = n[k];
//					int edgeK = vertexPairsToEdges[std::make_pair(v, vAdj)];
					int edgeK = vertexPairsToEdges[v*convexHull.v.size()+vAdj];
					if (IS_INTERNAL_EDGE(edgeK))
						continue; // Edges inside faces with 180 degrees dihedral angles can be ignored.
					if (sidepodalVertices[i*convexHull.v.size()+vAdj]
						&& sidepodalVertices[edgeJ*convexHull.v.size()+vAdj])
					{
						if (!HAVE_VISITED_VERTEX(vAdj))
							traverseStackCommonSidepodals.push_back(vAdj);

						if (edgeJ < edgeK)
						{
							// Test edge triplet i, edgeJ, edgeK.
							vec f3a = faceNormals[facesForEdge[edgeK].first];
							vec f3b = faceNormals[facesForEdge[edgeK].second];

							if (!AreEdgesBad(f1a, f1b, f3a, f3b) && !AreEdgesBad(f2a, f2b, f3a, f3b))
							{
								vec n1[2], n2[2], n3[2];
								TIMING_TICK(++numConfigsExplored);
								int nSolutions = ComputeBasis(f1a, f1b, f2a, f2b, f3a, f3b, n1, n2, n3);
								for(int s = 0; s < nSolutions; ++s) // O(constant), nSolutions == 0, 1 or 2.
								{
									TestThreeAdjacentFaces(n1[s], n2[s], n3[s], (int)i, edgeJ, edgeK, convexHull,
										edges, antipodalPointsForEdge, &minVolume, &minOBB);
								}
							}
						}
					}
				}
			}
		}
	}
	TIMING("Edgetriplets: %llu edgesqrts, %llu bootstraps (%.3f/edgesqrt), %llu sidepodals steps (%.3f/edgesqrt), "
		"#third edges: %llu (%.3f/edgesqrt) #vertex steps: %llu (%.3f/edgesqrt) "
		"#vertex improvements: %.3f/edgesqrt",
		numEdgeSqrtEdges,
		numBootstrapStepsDone, (float)numBootstrapStepsDone/numEdgeSqrtEdges,
		numCommonSidepodalStepsDone, (float)numCommonSidepodalStepsDone/numEdgeSqrtEdges,
		numConfigsExplored, (float)numConfigsExplored/numEdgeSqrtEdges,
		numVertexNeighborSearches, (float)numVertexNeighborSearches/numEdgeSqrtEdges,
		(float)numVertexNeighborSearchImprovements/numEdgeSqrtEdges);

	TIMING_TICK(tick_t t6 = Clock::Tick());
	TIMING("Edgetripletconfigs: %f msecs (%d configs)", Clock::TimespanToMillisecondsF(t5, t6), numConfigsExplored);
	TIMING_TICK(
		t6 = Clock::Tick();
		unsigned long long numTwoOpposingFacesConfigs = 0;
		numVertexNeighborSearches = 0;
		numVertexNeighborSearchImprovements = 0;
		);

	std::vector<int> antipodalEdges;
	VecArray antipodalEdgeNormals;

	// Main algorithm body for finding all search directions where the OBB is flush with the edges of the
	// convex hull from two opposing faces. This is O(E*sqrtE*logV)?
//	for (size_t i = 0; i < edges.size(); ++i) // O(E)
///	{
	for(size_t ii = 0; ii < spatialEdgeOrder.size(); ++ii)
	{
		size_t i = (size_t)spatialEdgeOrder[ii];
		vec f1a = faceNormals[facesForEdge[i].first];
		vec f1b = faceNormals[facesForEdge[i].second];

		antipodalEdges.clear();
		antipodalEdgeNormals.clear();

		const std::vector<int> &antipodals = antipodalPointsForEdge[i];
		for(size_t j = 0; j < antipodals.size(); ++j) // O(constant)?
		{
			int antipodalVertex = antipodals[j];
			const std::vector<int> &adjacents = adjacencyData[antipodalVertex];
			for(size_t k = 0; k < adjacents.size(); ++k) // O(constant)?
			{
				int vAdj = adjacents[k];
				if (vAdj < antipodalVertex)
					continue; // We search unordered edges, so no need to process edge (v1, v2) and (v2, v1) twice - take the canonical order to be antipodalVertex < vAdj

//				int edge = vertexPairsToEdges[std::make_pair(antipodalVertex, vAdj)];
				int edge = vertexPairsToEdges[antipodalVertex*convexHull.v.size()+vAdj];
				if ((int)i > edge) // We search pairs of edges, so no need to process twice - take the canonical order to be i < edge.
					continue;
				if (IS_INTERNAL_EDGE(edge))
					continue; // Edges inside faces with 180 degrees dihedral angles can be ignored.

				vec f2a = faceNormals[facesForEdge[edge].first];
				vec f2b = faceNormals[facesForEdge[edge].second];

				vec n;
				bool success = AreCompatibleOpposingEdges(f1a, f1b, f2a, f2b, n);
				if (success)
				{
					antipodalEdges.push_back(edge);
					antipodalEdgeNormals.push_back(n.Normalized());
				}
			}
		}

		const std::vector<int> &compatibleEdgesI = compatibleEdges[i];
		for(size_t j = 0; j < compatibleEdgesI.size(); ++j)
//		for(size_t j = 0; j < antipodalEdges.size(); ++j)
		{
			int edgeJ = compatibleEdgesI[j];
//			const std::vector<int> &compatibleEdgesJ = compatibleEdges[edge];
//			n = n.Normalized();
			for(size_t k = 0; k < antipodalEdges.size(); ++k)
			{
				int edgeK = antipodalEdges[k];

				vec n = antipodalEdgeNormals[k];
				float minN1 = n.Dot(convexHull.v[edges[edgeK].first]);
				float maxN1 = n.Dot(convexHull.v[edges[i].first]);

				// Test all mutual compatible edges.
				vec f3a = faceNormals[facesForEdge[edgeJ].first];
				vec f3b = faceNormals[facesForEdge[edgeJ].second];
				// Is edge3 compatible with direction n?
				// n3 = f3b + (f3a-f3b)*v
				// n1.n3 = 0
				// n1.(f3b + (f3a-f3b)*v) = 0
				// n1.f3b + n1.((f3a-f3b)*v) = 0
				// n1.f3b = (n1.(f3b-f3a))*v
				// If n1.(f3b-f3a) != 0:
				//    v = n1.f3b / n1.(f3b-f3a)
				// If n1.(f3b-f3a) == 0:
				//    n1.f3b must be zero as well, then arbitrary v is ok.
				float num = n.Dot(f3b);
				float denom = n.Dot(f3b-f3a);
				MoveSign(num, denom);

				const float epsilon = 1e-4f;
				if (denom < epsilon)//EqualAbs(denom, 0.f))
				{
					num = EqualAbs(num, 0.f) ? 0.f : -1.f;
					denom = 1.f;
				}

//				if (v >= 0.f - epsilon && v <= 1.f + epsilon)
				if (num >= denom * -epsilon && num <= denom * (1.f + epsilon))
				{
					float v = num / denom;
					vec n3 = (f3b + (f3a - f3b) * v).Normalized();
					vec n2 = n3.Cross(n).Normalized();

					float minN2, maxN2;

					CLEAR_GRAPH_SEARCH();
					int hint = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, (k == 0) ? extremeVertexSearchHint1 : extremeVertexSearchHint1_b); // O(logV)?
					if (k == 0) extremeVertexSearchHint1 = extremeVertexSearchHint1_b = hint;
					else extremeVertexSearchHint1_b = hint;
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);

					CLEAR_GRAPH_SEARCH();
					hint = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, (k == 0) ? extremeVertexSearchHint2 : extremeVertexSearchHint2_b); // O(logV)?
					if (k == 0) extremeVertexSearchHint2 = extremeVertexSearchHint2_b = hint;
					else extremeVertexSearchHint2_b = hint;
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
					minN2 = -minN2;

					float maxN3 = n3.Dot(convexHull.v[edges[edgeJ].first]);

					float minN3 = FLOAT_INF;
					const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edgeJ];
					if (antipodalsEdge3.size() < 20) // If there are very few antipodal vertices, do a very tight loop and just iterate over each.
					{
						for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
							minN3 = Min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));
					}
					else
					{
						// Otherwise perform a spatial locality exploiting graph search.
						CLEAR_GRAPH_SEARCH();
						hint = convexHull.ExtremeVertexConvex(adjacencyData, -n3, floodFillVisited, floodFillVisitColor, minN3, (k == 0) ? extremeVertexSearchHint3 : extremeVertexSearchHint3_b); // O(logV)?
						if (k == 0) extremeVertexSearchHint3 = extremeVertexSearchHint3_b = hint;
						else extremeVertexSearchHint3_b = hint;
						TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
						TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
						minN3 = -minN3;
					}

					float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
					TIMING_TICK(++numTwoOpposingFacesConfigs);
					if (volume < minVolume)
					{
						minOBB.pos = ((minN1 + maxN1) * n + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
						minOBB.axis[0] = n;
						minOBB.axis[1] = n2;
						minOBB.axis[2] = n3;
						minOBB.r[0] = (maxN1 - minN1) * 0.5f;
						minOBB.r[1] = (maxN2 - minN2) * 0.5f;
						minOBB.r[2] = (maxN3 - minN3) * 0.5f;
#ifdef OBB_ASSERT_VALIDITY
						OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB.axis[0], minOBB.axis[1]);
						assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
						minVolume = volume;
					}
				}
			}
		}
	}

	TIMING_TICK(tick_t t7 = Clock::Tick());
	TIMING("Edgepairsforfaces: %f msecs (%llu configs)", Clock::TimespanToMillisecondsF(t6, t7), numTwoOpposingFacesConfigs);
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/numTwoOpposingFacesConfigs, (double)numVertexNeighborSearchImprovements/numTwoOpposingFacesConfigs);
	TIMING_TICK(
		tick_t t72 = Clock::Tick();
		int numTwoSameFacesConfigs = 0;
		numVertexNeighborSearches = 0;
		numVertexNeighborSearchImprovements = 0;
		);

	// Main algorithm body for computing all search directions where the OBB touches two edges on the same face.
	// This is O(F*sqrtE*logV)?
	for(size_t ii = 0; ii < spatialFaceOrder.size(); ++ii) // O(F)
	{
		size_t i = spatialFaceOrder[ii];
		vec n1 = faceNormals[i];

		// Find two edges on the face. Since we have flexibility to choose from multiple edges of the same face,
		// choose two that are possibly most opposing to each other, in the hope that their sets of sidepodal
		// edges are most mutually exclusive as possible, speeding up the search below.
		int e1 = -1;
		int v0 = convexHull.f[i].v.back();
		for(size_t j = 0; j < convexHull.f[i].v.size(); ++j)
		{
			int v1 = convexHull.f[i].v[j];
			int e = vertexPairsToEdges[v0*convexHull.v.size()+v1];
			if (!IS_INTERNAL_EDGE(e))
			{
				e1 = e;
				break;
			}
			v0 = v1;
		}
		if (e1 == -1)
			continue; // All edges of this face were degenerate internal edges! Just skip processing the whole face.

		const std::vector<int> &antipodals = antipodalPointsForEdge[e1];
		const std::vector<int> &compatibleEdgesI = compatibleEdges[e1];

		float maxN1 = n1.Dot(convexHull.v[edges[e1].first]);
		float minN1;

		if (antipodals.size() < 20)
		{
			minN1 = FLOAT_INF;
			for(size_t j = 0; j < antipodals.size(); ++j)
				minN1 = Min(minN1, n1.Dot(convexHull.v[antipodals[j]]));
		}
		else
		{
			CLEAR_GRAPH_SEARCH();
			extremeVertexSearchHint4 = convexHull.ExtremeVertexConvex(adjacencyData, -n1, floodFillVisited, floodFillVisitColor, minN1, extremeVertexSearchHint4); // O(logV)?
			minN1 = -minN1;
			TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
			TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
		}

		for(size_t j = 0; j < compatibleEdgesI.size(); ++j)
		{
			int edge3 = compatibleEdgesI[j];
			// Test all mutual compatible edges.
			vec f3a = faceNormals[facesForEdge[edge3].first];
			vec f3b = faceNormals[facesForEdge[edge3].second];

			// Is edge3 compatible with direction n?
			// n3 = f3b + (f3a-f3b)*v
			// n1.n3 = 0
			// n1.(f3b + (f3a-f3b)*v) = 0
			// n1.f3b + n1.((f3a-f3b)*v) = 0
			// n1.f3b = (n1.(f3b-f3a))*v
			// If n1.(f3b-f3a) != 0:
			//    v = n1.f3b / n1.(f3b-f3a)
			// If n1.(f3b-f3a) == 0:
			//    n1.f3b must be zero as well, then arbitrary v is ok.
			float num = n1.Dot(f3b);
			float denom = n1.Dot(f3b-f3a);
			float v;
			if (!EqualAbs(denom, 0.f))
				v = num / denom;
			else
				v = EqualAbs(num, 0.f) ? 0.f : -1.f;

			const float epsilon = 1e-4f;
			if (v >= 0.f - epsilon && v <= 1.f + epsilon)
			{
				vec n3 = (f3b + (f3a - f3b) * v).Normalized();
				vec n2 = n3.Cross(n1).Normalized();

				float minN2, maxN2;
				CLEAR_GRAPH_SEARCH();
				extremeVertexSearchHint1 = convexHull.ExtremeVertexConvex(adjacencyData, n2, floodFillVisited, floodFillVisitColor, maxN2, extremeVertexSearchHint1); // O(logV)?
				TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
				TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
				CLEAR_GRAPH_SEARCH();
				extremeVertexSearchHint2 = convexHull.ExtremeVertexConvex(adjacencyData, -n2, floodFillVisited, floodFillVisitColor, minN2, extremeVertexSearchHint2); // O(logV)?
				TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
				TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
				minN2 = -minN2;
				float maxN3 = n3.Dot(convexHull.v[edges[edge3].first]);

				const std::vector<int> &antipodalsEdge3 = antipodalPointsForEdge[edge3];
				float minN3 = FLOAT_INF;
				if (antipodalsEdge3.size() < 20) // If there are very few antipodal vertices, do a very tight loop and just iterate over each.
				{
					for(size_t a = 0; a < antipodalsEdge3.size(); ++a)
						minN3 = Min(minN3, n3.Dot(convexHull.v[antipodalsEdge3[a]]));
				}
				else
				{
					// Otherwise perform a spatial locality exploiting graph search.
					CLEAR_GRAPH_SEARCH();
					extremeVertexSearchHint3 = convexHull.ExtremeVertexConvex(adjacencyData, -n3, floodFillVisited, floodFillVisitColor, minN3, extremeVertexSearchHint3); // O(logV)?
					TIMING_TICK(numVertexNeighborSearches += convexHull.numSearchStepsDone);
					TIMING_TICK(numVertexNeighborSearchImprovements += convexHull.numImprovementsMade);
					minN3 = -minN3;
				}

				float volume = (maxN1 - minN1) * (maxN2 - minN2) * (maxN3 - minN3);
				TIMING_TICK(++numTwoSameFacesConfigs);
				if (volume < minVolume)
				{
					minOBB.pos = ((minN1 + maxN1) * n1 + (minN2 + maxN2) * n2 + (minN3 + maxN3) * n3) * 0.5f;
					minOBB.axis[0] = n1;
					minOBB.axis[1] = n2;
					minOBB.axis[2] = n3;
					minOBB.r[0] = (maxN1 - minN1) * 0.5f;
					minOBB.r[1] = (maxN2 - minN2) * 0.5f;
					minOBB.r[2] = (maxN3 - minN3) * 0.5f;
					assert(volume > 0.f);
#ifdef OBB_ASSERT_VALIDITY
					OBB o = OBB::FixedOrientationEnclosingOBB((const vec*)&convexHull.v[0], convexHull.v.size(), minOBB.axis[0], minOBB.axis[1]);
					assert2(EqualRel(o.Volume(), volume), o.Volume(), volume);
#endif
					minVolume = volume;
				}
			}
		}
	}

	TIMING_TICK(tick_t t8 = Clock::Tick());
	TIMING("Faceconfigs: %f msecs (%d configs, %d faces, %f configs/face)", Clock::TimespanToMillisecondsF(t72, t8), numTwoSameFacesConfigs, (int)spatialFaceOrder.size(), (double)numTwoSameFacesConfigs/spatialFaceOrder.size());
	TIMING("%f vertex neighbor searches per edge, %f improvements", (double)numVertexNeighborSearches/numTwoSameFacesConfigs, (double)numVertexNeighborSearchImprovements/numTwoSameFacesConfigs);

	// The search for edge triplets does not follow cross-product orientation, so
	// fix that up at the very last step, if necessary.
	if (minOBB.axis[0].Cross(minOBB.axis[1]).Dot(minOBB.axis[2]) < 0.f)
		minOBB.axis[2] = -minOBB.axis[2];
#ifdef MATH_VEC_IS_FLOAT4
	minOBB.r.w = 0.f;
	minOBB.pos.w = 1.f;
#endif
	delete[] sidepodalVertices;
	delete[] vertexPairsToEdges;
	return minOBB;
}

OBB OBB::FixedOrientationEnclosingOBB(const vec *pointArray, int numPoints, const vec &dir0, const vec &dir1)
{
	assume(dir0.IsNormalized());
	assume(dir1.IsNormalized());

	int d0, d1;
	float mind0, maxd0, mind1, maxd1, mind2, maxd2;
	OBB::ExtremePointsAlongDirection(dir0, pointArray, numPoints, d0, d1, mind0, maxd0);
	OBB::ExtremePointsAlongDirection(dir1, pointArray, numPoints, d0, d1, mind1, maxd1);
	vec edgeC = dir0.Cross(dir1).Normalized();
	OBB::ExtremePointsAlongDirection(edgeC, pointArray, numPoints, d0, d1, mind2, maxd2);
	float rd0 = (maxd0 - mind0) * 0.5f;
	float rd1 = (maxd1 - mind1) * 0.5f;
	float rd2 = (maxd2 - mind2) * 0.5f;
	OBB minOBB;
	minOBB.pos = (mind0 + rd0) * dir0 + (mind1 + rd1) * dir1 + (mind2 + rd2) * edgeC;
	minOBB.axis[0] = dir0;
	minOBB.axis[1] = dir1;
	minOBB.axis[2] = edgeC;
	minOBB.r = DIR_VEC(rd0, rd1, rd2);
	return minOBB;
}

OBB OBB::BruteEnclosingOBB(const vec *pointArray, int numPoints)
{
	Polyhedron convexHull = Polyhedron::ConvexHull(pointArray, numPoints);
	return BruteEnclosingOBB(convexHull);
}

OBB OBB::BruteEnclosingOBB(const Polyhedron &convexPolyhedron)
{
	OBB minOBB;
	if (convexPolyhedron.v.size() == 0)
	{
		minOBB.SetNegativeInfinity();
		return minOBB;
	}

//	std::vector<std::vector<int> > adjacencyData = convexHull.GenerateVertexAdjacencyData();
//	std::vector<int> floodFillVisited(convexHull.v.size());
//	int floodFillVisitColor = 1;

	std::vector<float2> pts;
	pts.resize(convexPolyhedron.v.size());
	float minVolume = FLOAT_INF;
	vec minVolumeEdgeA;
	vec minVolumeEdgeB;

	const int Y = 128;
	const int X = 128;
	for(int y = 0; y < Y; ++y)
		for(int x = 0; x < X; ++x)
		{
			float fx = (float)x / (X-1) * 2.0f - 1.0f;
			float fy = (float)y / (Y-1) * 2.0f - 1.0f;

			float lenSq = fx*fx + fy*fy;
			if (lenSq > 1.00f)
				continue;
			float fz = Sqrt(1.0f - lenSq);

			vec edge = DIR_VEC(fx, fy, fz);

			vec edgeA, edgeB;
			float volume = SmallestOBBVolumeJiggle(edge, convexPolyhedron, pts, /*adjacencyData, floodFillVisited, floodFillVisitColor,*/
				edgeA, edgeB);

			if (volume < minVolume)
			{
				minVolumeEdgeA = edgeA;
				minVolumeEdgeB = edgeB;
				minVolume = volume;
			}
		}

	return FixedOrientationEnclosingOBB((const vec *)&convexPolyhedron.v[0], (int)convexPolyhedron.v.size(), minVolumeEdgeA, minVolumeEdgeB);
}

OBB OBB::Brute2EnclosingOBB(const Polyhedron &convexPolyhedron)
{
	OBB minOBB;
	if (convexPolyhedron.v.size() == 0)
	{
		minOBB.SetNegativeInfinity();
		return minOBB;
	}

	std::vector<std::vector<int> > adjacencyData = convexPolyhedron.GenerateVertexAdjacencyData();
	std::vector<unsigned int> floodFillVisited(convexPolyhedron.v.size());
	int floodFillVisitColor = 1;

	float minVolume = FLOAT_INF;

	int v[6] = {};
	float dst[6] = {};
	const int S = 256;
	float inc = 2.f / (float)S;
	float Z = -1.f;
	for(int z = 0; z < S; ++z)
	{
		float Y = -1.f;
		for(int y = 0; y < S; ++y)
		{
			float X = -1.f;
			for(int x = 0; x < S; ++x)
			{
				float d = 1.f - X*X - Y*Y - Z*Z;
				X += inc;
				if (d < -1e-4f) continue;
				float W = (d > 0.f) ? Sqrt(d) : 0.f;
				Quat q(X,Y,Z,W);
				q.Normalize();

				float4x4 m = q.ToFloat4x4();
				const vec d0 = FLOAT4_TO_DIR(m.Col(0));
				const vec d1 = FLOAT4_TO_DIR(m.Col(1));
				const vec d2 = FLOAT4_TO_DIR(m.Col(2));
				v[0] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d0, floodFillVisited, floodFillVisitColor++, dst[0], v[0]);
				v[1] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d0, floodFillVisited, floodFillVisitColor++, dst[1], v[1]);
				v[2] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d1, floodFillVisited, floodFillVisitColor++, dst[2], v[2]);
				v[3] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d1, floodFillVisited, floodFillVisitColor++, dst[3], v[3]);
				v[4] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d2, floodFillVisited, floodFillVisitColor++, dst[4], v[4]);
				v[5] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d2, floodFillVisited, floodFillVisitColor++, dst[5], v[5]);
				float volume = (dst[0] + dst[1]) * (dst[2] + dst[3]) + (dst[4] + dst[5]);
				if (volume < minVolume)
				{
					minOBB.axis[0] = d0;
					minOBB.axis[1] = d1;
					minOBB.axis[2] = d2;
					minOBB.r[0] = (dst[0] + dst[1]) * 0.5f;
					minOBB.r[1] = (dst[2] + dst[3]) * 0.5f;
					minOBB.r[2] = (dst[4] + dst[5]) * 0.5f;
					minOBB.pos = ((dst[0] - dst[1]) * d0 + (dst[2] - dst[3]) * d1 + (dst[4] - dst[5]) * d2) * 0.5f;
					minVolume = volume;
				}
			}
			Y += inc;
		}
		Z += inc;
	}
	std::vector<float2> pts;
	pts.resize(convexPolyhedron.v.size());
	/*float volume = */SmallestOBBVolumeJiggle(minOBB.axis[2], convexPolyhedron, pts, /*adjacencyData, floodFillVisited, floodFillVisitColor,*/
		minOBB.axis[0], minOBB.axis[1]);
	minOBB = OBB::FixedOrientationEnclosingOBB((const vec*)&convexPolyhedron.v[0], (int)convexPolyhedron.v.size(),
		minOBB.axis[0], minOBB.axis[1]);
	return minOBB;
}

#if defined(_MSC_VER) && defined(MATH_SSE) && _MSC_VER < 1800 // < VS2013
// Work around a VS2010 bug "error C2719: 'q': formal parameter with __declspec(align('16')) won't be aligned"
OBB OBB::Brute3EnclosingOBB(const Polyhedron &convexPolyhedron, const Quat &q)
#else
OBB OBB::Brute3EnclosingOBB(const Polyhedron &convexPolyhedron, Quat q)
#endif
{
	OBB minOBB;
	if (convexPolyhedron.v.size() == 0)
	{
		minOBB.SetNegativeInfinity();
		return minOBB;
	}

	std::vector<std::vector<int> > adjacencyData = convexPolyhedron.GenerateVertexAdjacencyData();
	std::vector<unsigned int> floodFillVisited(convexPolyhedron.v.size());
	int floodFillVisitColor = 1;

	float minVolume = FLOAT_INF;

	static LCG rng(Clock::TickU32());
	int v[6] = {};
	float dst[6] = {};
	float a = 0.f;
	const int nSteps = 1000000;
	const float incr = 2.f * pi / nSteps;
	int nStepsNoProgress = 0;
	vec unitX = q*vec::unitX;
	vec u,w;
	unitX.PerpendicularBasis(u, w);
	Quat r = q;
	while(nStepsNoProgress < nSteps + 100)
	{
		//vec z = u * Cos(a) + w * Sin(a);
		//z.Normalize();
		vec z = vec::RandomDir(rng);
		Quat rot = Quat(z, rng.Float(0.f, 2.f*pi/(360.f*200.f)) /*2.f*pi/(360.f*2000.f)*/);
		Quat test = rot * r;
		test.Normalize();

		float4x4 m = test.ToFloat4x4();
		const vec d0 = FLOAT4_TO_DIR(m.Col(0));
		const vec d1 = FLOAT4_TO_DIR(m.Col(1));
		const vec d2 = FLOAT4_TO_DIR(m.Col(2));
		v[0] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d0, floodFillVisited, floodFillVisitColor++, dst[0], v[0]);
		v[1] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d0, floodFillVisited, floodFillVisitColor++, dst[1], v[1]);
		v[2] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d1, floodFillVisited, floodFillVisitColor++, dst[2], v[2]);
		v[3] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d1, floodFillVisited, floodFillVisitColor++, dst[3], v[3]);
		v[4] = convexPolyhedron.ExtremeVertexConvex(adjacencyData,  d2, floodFillVisited, floodFillVisitColor++, dst[4], v[4]);
		v[5] = convexPolyhedron.ExtremeVertexConvex(adjacencyData, -d2, floodFillVisited, floodFillVisitColor++, dst[5], v[5]);
		float volume = (dst[0] + dst[1]) * (dst[2] + dst[3]) + (dst[4] + dst[5]);
		if (volume < minVolume)
		{
//			LOGI("Improved volume from %f to %f.", volume, minVolume);
			minOBB.axis[0] = d0;
			minOBB.axis[1] = d1;
			minOBB.axis[2] = d2;
			minOBB.r[0] = (dst[0] + dst[1]) * 0.5f;
			minOBB.r[1] = (dst[2] + dst[3]) * 0.5f;
			minOBB.r[2] = (dst[4] + dst[5]) * 0.5f;
			minOBB.pos = ((dst[0] - dst[1]) * d0 + (dst[2] - dst[3]) * d1 + (dst[4] - dst[5]) * d2) * 0.5f;
			minVolume = volume;

			r = test;
			nStepsNoProgress = 0;
			unitX = r*vec::unitX;
			unitX.PerpendicularBasis(u, w);
		}
		else
			++nStepsNoProgress;
		a = Mod(a + incr, 2.f*pi);
	}
	return minOBB;
}

vec OBB::Size() const
{
	return r * 2.f;
}

vec OBB::HalfSize() const
{
	return r;
}

vec OBB::Diagonal() const
{
	return 2.f * HalfDiagonal();
}

vec OBB::HalfDiagonal() const
{
	return axis[0] * r[0] + axis[1] * r[1] + axis[2] * r[2];
}

float3x4 OBB::WorldToLocal() const
{
	float3x4 m = LocalToWorld();
	m.InverseOrthonormal();
	return m;
}

float3x4 OBB::LocalToWorld() const
{
	// To produce a normalized local->world matrix, do the following.
	/*
	float3x4 m;
	vec x = axis[0] * r.x;
	vec y = axis[1] * r.y;
	vec z = axis[2] * r.z;
	m.SetCol(0, 2.f * x);
	m.SetCol(1, 2.f * y);
	m.SetCol(2, 2.f * z);
	m.SetCol(3, pos - x - y - z);
	return m;
	*/

	assume2(axis[0].IsNormalized(), axis[0], axis[0].LengthSq());
	assume2(axis[1].IsNormalized(), axis[1], axis[1].LengthSq());
	assume2(axis[2].IsNormalized(), axis[2], axis[2].LengthSq());
	float3x4 m; ///\todo sse-matrix
	m.SetCol(0, axis[0].ptr());
	m.SetCol(1, axis[1].ptr());
	m.SetCol(2, axis[2].ptr());
	vec p = pos - axis[0] * r.x - axis[1] * r.y - axis[2] * r.z;
	m.SetCol(3, p.ptr());
	assume(m.IsOrthonormal());
	return m;
}

/// The implementation of this function is from Christer Ericson's Real-Time Collision Detection, p.133.
vec OBB::ClosestPoint(const vec &targetPoint) const
{
#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SIMD)
	// Best: 8.833 nsecs / 24 ticks, Avg: 9.044 nsecs, Worst: 9.217 nsecs
	simd4f d = sub_ps(targetPoint.v, pos.v);
	simd4f x = xxxx_ps(r.v);
	simd4f closestPoint = pos.v;
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[0].v), x), neg_ps(x)), axis[0].v, closestPoint);
	simd4f y = yyyy_ps(r.v);
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[1].v), y), neg_ps(y)), axis[1].v, closestPoint);
	simd4f z = zzzz_ps(r.v);
	closestPoint = madd_ps(max_ps(min_ps(dot4_ps(d, axis[2].v), z), neg_ps(z)), axis[2].v, closestPoint);
	return closestPoint;
#else
	// Best: 33.412 nsecs / 89.952 ticks, Avg: 33.804 nsecs, Worst: 34.180 nsecs
	vec d = targetPoint - pos;
	vec closestPoint = pos; // Start at the center point of the OBB.
	for(int i = 0; i < 3; ++i) // Project the target onto the OBB axes and walk towards that point.
		closestPoint += Clamp(Dot(d, axis[i]), -r[i], r[i]) * axis[i];

	return closestPoint;
#endif
}

float OBB::Volume() const
{
	vec size = Size();
	return size.x*size.y*size.z;
}

float OBB::SurfaceArea() const
{
	const vec size = Size();
	return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

vec OBB::RandomPointInside(LCG &rng) const
{
	float f1 = rng.Float();
	float f2 = rng.Float();
	float f3 = rng.Float();
	return PointInside(f1, f2, f3);
}

vec OBB::RandomPointOnSurface(LCG &rng) const
{
	int i = rng.Int(0, 5);
	float f1 = rng.Float();
	float f2 = rng.Float();
	return FacePoint(i, f1, f2);
}

vec OBB::RandomPointOnEdge(LCG &rng) const
{
	int i = rng.Int(0, 11);
	float f = rng.Float();
	return PointOnEdge(i, f);
}

vec OBB::RandomCornerPoint(LCG &rng) const
{
	return CornerPoint(rng.Int(0, 7));
}

void OBB::Translate(const vec &offset)
{
	pos += offset;
}

void OBB::Scale(const vec &centerPoint, float scaleFactor)
{
	return Scale(centerPoint, DIR_VEC_SCALAR(scaleFactor));
}

void OBB::Scale(const vec &centerPoint, const vec &scaleFactor)
{
	///@bug This scales in global axes, not local axes.
	float3x4 transform = float3x4::Scale(scaleFactor, centerPoint);
	Transform(transform);
}

template<typename Matrix>
void OBBTransform(OBB &o, const Matrix &transform)
{
	o.pos = transform.MulPos(o.pos);
	o.axis[0] = transform.MulDir(o.r.x * o.axis[0]);
	o.axis[1] = transform.MulDir(o.r.y * o.axis[1]);
	o.axis[2] = transform.MulDir(o.r.z * o.axis[2]);
	o.r.x = o.axis[0].Normalize();
	o.r.y = o.axis[1].Normalize();
	o.r.z = o.axis[2].Normalize();
}

void OBB::Transform(const float3x3 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float3x4 &transform)
{
	assume(transform.IsColOrthogonal());
	OBBTransform(*this, transform);
}

void OBB::Transform(const float4x4 &transform)
{
	assume(transform.IsColOrthogonal3());
	OBBTransform(*this, transform);
}

void OBB::Transform(const Quat &transform)
{
	OBBTransform(*this, transform.ToFloat3x3());
}

float OBB::Distance(const vec &point) const
{
	///@todo This code can be optimized a bit. See Christer Ericson's Real-Time Collision Detection,
	/// p.134.
	vec closestPoint = ClosestPoint(point);
	return point.Distance(closestPoint);
}

float OBB::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

bool OBB::Contains(const vec &point) const
{
#if defined(MATH_SSE) && defined(MATH_AUTOMATIC_SSE)
// Best: 9.985 nsecs / 26.816 ticks, Avg: 10.112 nsecs, Worst: 11.137 nsecs
	simd4f pt = sub_ps(point.v, pos.v);
	simd4f s1 = mul_ps(pt, axis[0].v);
	simd4f s2 = mul_ps(pt, axis[1].v);
	simd4f s3 = mul_ps(pt, axis[2].v);
	s1 = abs_ps(sum_xyzw_ps(s1));
	s2 = abs_ps(sum_xyzw_ps(s2));
	s3 = abs_ps(sum_xyzw_ps(s3));

	s1 = _mm_sub_ss(s1, r.v);
	s2 = _mm_sub_ss(s2, yyyy_ps(r.v));
	simd4f s12 = _mm_max_ss(s1, s2);
	s3 = _mm_sub_ss(s3, zzzz_ps(r.v));
	s3 = _mm_max_ss(s12, s3);
	return _mm_cvtss_f32(s3) <= 0.f; // Note: This might be micro-optimized further out by switching to a signature "float OBB::SignedDistance(point)" instead.
#else
// Best: 14.978 nsecs / 39.944 ticks, Avg: 15.350 nsecs, Worst: 39.941 nsecs
	vec pt = point - pos;
	return Abs(Dot(pt, axis[0])) <= r[0] &&
	       Abs(Dot(pt, axis[1])) <= r[1] &&
	       Abs(Dot(pt, axis[2])) <= r[2];
#endif
}

bool OBB::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool OBB::Contains(const AABB &aabb) const
{
	// Since both AABB and OBB are convex objects, this OBB contains the AABB
	// if and only if it contains all its corner points.
	for(int i = 0; i < 8; ++i)
	if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool OBB::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool OBB::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool OBB::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

bool OBB::Intersects(const AABB &aabb) const
{
	return Intersects(OBB(aabb));
}

void OBB::Enclose(const vec &point)
{
	vec p = point - pos;
	for(int i = 0; i < 3; ++i)
	{
		assume2(EqualAbs(axis[i].Length(), 1.f), axis[i], axis[i].Length());
		float dist = p.Dot(axis[i]);
		float distanceFromOBB = Abs(dist) - r[i];
		if (distanceFromOBB > 0.f)
		{
			r[i] += distanceFromOBB * 0.5f;
			if (dist > 0.f) ///\todo Optimize out this comparison!
				pos += axis[i] * distanceFromOBB * 0.5f;
			else
				pos -= axis[i] * distanceFromOBB * 0.5f;

			p = point-pos; ///\todo Can we omit this? (redundant since axis[i] are orthonormal?)

			mathassert(EqualAbs(Abs(p.Dot(axis[i])), r[i], 1e-1f));
		}
	}
	// Should now contain the point.
	assume2(Distance(point) <= 1e-3f, point, Distance(point));
}

void OBB::Triangulate(int x, int y, int z, vec *outPos, vec *outNormal, float2 *outUV, bool ccwIsFrontFacing) const
{
	AABB aabb(POINT_VEC_SCALAR(0), r*2.f);
	aabb.Triangulate(x, y, z, outPos, outNormal, outUV, ccwIsFrontFacing);
	float3x4 localToWorld = LocalToWorld();
	assume(localToWorld.HasUnitaryScale()); // Transforming of normals will fail otherwise.
	localToWorld.BatchTransformPos(outPos, NumVerticesInTriangulation(x,y,z), sizeof(vec));
	localToWorld.BatchTransformDir(outNormal, NumVerticesInTriangulation(x,y,z), sizeof(vec));
}

void OBB::ToEdgeList(vec *outPos) const
{
	assume(outPos);
	if (!outPos)
		return;
	for(int i = 0; i < 12; ++i)
	{
		LineSegment edge = Edge(i);
		outPos[i*2] = edge.a;
		outPos[i*2+1] = edge.b;
	}
}

bool OBB::Intersects(const OBB &b, float epsilon) const
{
	assume(pos.IsFinite());
	assume(b.pos.IsFinite());
	assume(vec::AreOrthogonal(axis[0], axis[1], axis[2]));
	assume(vec::AreOrthogonal(b.axis[0], b.axis[1], b.axis[2]));

#if defined(MATH_AUTOMATIC_SSE) && defined(MATH_SSE)
	// SSE4.1:
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 23.913 nsecs / 40.645 ticks, Avg: 26.490 nsecs, Worst: 43.729 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 42.585 nsecs / 72.413 ticks, Avg: 44.373 nsecs, Worst: 70.774 nsecs

	simd4f bLocalToWorld[3];
	mat3x4_transpose((const simd4f*)b.axis, bLocalToWorld);

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	simd4f R[3];
	mat3x4_mul_sse(R, (const simd4f*)axis, bLocalToWorld);

	// Express the translation vector in a's coordinate frame.
	simd4f t = mat3x4_mul_sse((const simd4f*)axis, sub_ps(b.pos, pos));

	// This trashes the w component, which should technically be zero, but this does not matter since
	// AbsR will only be used with direction vectors.
	const vec epsilonxyz = set1_ps(epsilon);
	simd4f AbsR[3];
	AbsR[0] = add_ps(abs_ps(R[0]), epsilonxyz);
	AbsR[1] = add_ps(abs_ps(R[1]), epsilonxyz);
	AbsR[2] = add_ps(abs_ps(R[2]), epsilonxyz);

	// Test the three major axes of this OBB.
	simd4f res = cmpgt_ps(abs_ps(t), add_ps(r, mat3x4_mul_sse(AbsR, b.r)));
	if (!allzero_ps(res)) return false;

	// Test the three major axes of the OBB b.
	simd4f transpR[3];
	mat3x4_transpose(R, transpR);
	vec l = abs_ps(mat3x4_mul_sse(transpR, r));
	simd4f transpAbsR[3];
	mat3x4_transpose(AbsR, transpAbsR);
	vec s = mat3x4_mul_sse(transpAbsR, r);
	res = cmpgt_ps(l, add_ps(s, b.r));
	if (!allzero_ps(res)) return false;

	// Test the 9 different cross-axes.

#define xxxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,0,0))
#define yyyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,1,1))
#define zzzw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,2,2,2))
#define yxxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,0,1))
#define zzyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,2,2))

	// A.x <cross> B.x
	// A.x <cross> B.y
	// A.x <cross> B.z
	simd4f ra = madd_ps(yyyw_ps(r), AbsR[2], mul_ps(zzzw_ps(r), AbsR[1]));
	simd4f rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[0]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[0])));
	simd4f lhs = msub_ps(zzzw_ps(t), R[1], mul_ps(yyyw_ps(t), R[2]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.y <cross> B.x
	// A.y <cross> B.y
	// A.y <cross> B.z
	ra = madd_ps(xxxw_ps(r), AbsR[2], mul_ps(zzzw_ps(r), AbsR[0]));
	rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[1]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[1])));
	lhs = msub_ps(xxxw_ps(t), R[2], mul_ps(zzzw_ps(t), R[0]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	if (!allzero_ps(res)) return false;

	// A.z <cross> B.x
	// A.z <cross> B.y
	// A.z <cross> B.z
	ra = madd_ps(xxxw_ps(r), AbsR[1], mul_ps(yyyw_ps(r), AbsR[0]));
	rb = madd_ps(yxxw_ps(b.r), zzyw_ps(AbsR[2]), mul_ps(zzyw_ps(b.r), yxxw_ps(AbsR[2])));
	lhs = msub_ps(yyyw_ps(t), R[0], mul_ps(xxxw_ps(t), R[1]));
	res = cmpgt_ps(abs_ps(lhs), add_ps(ra, rb));
	return allzero_ps(res) != 0;

#else
	// Benchmark 'OBBIntersectsOBB_Random': OBB::Intersects(OBB) Random
	//    Best: 100.830 nsecs / 171.37 ticks, Avg: 110.533 nsecs, Worst: 155.582 nsecs
	// Benchmark 'OBBIntersectsOBB_Positive': OBB::Intersects(OBB) Positive
	//    Best: 95.771 nsecs / 162.739 ticks, Avg: 107.935 nsecs, Worst: 173.110 nsecs

	// Generate a rotation matrix that transforms from world space to this OBB's coordinate space.
	float3x3 R;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			R[i][j] = Dot(axis[i], b.axis[j]);

	vec t = b.pos - pos;
	// Express the translation vector in a's coordinate frame.
	t = DIR_VEC(Dot(t, axis[0]), Dot(t, axis[1]), Dot(t, axis[2]));

	float3x3 AbsR;
	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			AbsR[i][j] = Abs(R[i][j]) + epsilon;

	// Test the three major axes of this OBB.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[i];
		float rb = DOT3(b.r, AbsR[i]);
		if (Abs(t[i]) > ra + rb)
			return false;
	}

	// Test the three major axes of the OBB b.
	for(int i = 0; i < 3; ++i)
	{
		float ra = r[0] * AbsR[0][i] + r[1] * AbsR[1][i] + r[2] * AbsR[2][i];
		float rb = b.r[i];
		if (Abs(t.x * R[0][i] + t.y * R[1][i] + t.z * R[2][i]) > ra + rb)
			return false;
	}

	// Test the 9 different cross-axes.

	// A.x <cross> B.x
	float ra = r.y * AbsR[2][0] + r.z * AbsR[1][0];
	float rb = b.r.y * AbsR[0][2] + b.r.z * AbsR[0][1];
	if (Abs(t.z * R[1][0] - t.y * R[2][0]) > ra + rb)
		return false;

	// A.x < cross> B.y
	ra = r.y * AbsR[2][1] + r.z * AbsR[1][1];
	rb = b.r.x * AbsR[0][2] + b.r.z * AbsR[0][0];
	if (Abs(t.z * R[1][1] - t.y * R[2][1]) > ra + rb)
		return false;

	// A.x <cross> B.z
	ra = r.y * AbsR[2][2] + r.z * AbsR[1][2];
	rb = b.r.x * AbsR[0][1] + b.r.y * AbsR[0][0];
	if (Abs(t.z * R[1][2] - t.y * R[2][2]) > ra + rb)
		return false;

	// A.y <cross> B.x
	ra = r.x * AbsR[2][0] + r.z * AbsR[0][0];
	rb = b.r.y * AbsR[1][2] + b.r.z * AbsR[1][1];
	if (Abs(t.x * R[2][0] - t.z * R[0][0]) > ra + rb)
		return false;

	// A.y <cross> B.y
	ra = r.x * AbsR[2][1] + r.z * AbsR[0][1];
	rb = b.r.x * AbsR[1][2] + b.r.z * AbsR[1][0];
	if (Abs(t.x * R[2][1] - t.z * R[0][1]) > ra + rb)
		return false;

	// A.y <cross> B.z
	ra = r.x * AbsR[2][2] + r.z * AbsR[0][2];
	rb = b.r.x * AbsR[1][1] + b.r.y * AbsR[1][0];
	if (Abs(t.x * R[2][2] - t.z * R[0][2]) > ra + rb)
		return false;

	// A.z <cross> B.x
	ra = r.x * AbsR[1][0] + r.y * AbsR[0][0];
	rb = b.r.y * AbsR[2][2] + b.r.z * AbsR[2][1];
	if (Abs(t.y * R[0][0] - t.x * R[1][0]) > ra + rb)
		return false;

	// A.z <cross> B.y
	ra = r.x * AbsR[1][1] + r.y * AbsR[0][1];
	rb = b.r.x * AbsR[2][2] + b.r.z * AbsR[2][0];
	if (Abs(t.y * R[0][1] - t.x * R[1][1]) > ra + rb)
		return false;

	// A.z <cross> B.z
	ra = r.x * AbsR[1][2] + r.y * AbsR[0][2];
	rb = b.r.x * AbsR[2][1] + b.r.y * AbsR[2][0];
	if (Abs(t.y * R[0][2] - t.x * R[1][2]) > ra + rb)
		return false;

	// No separating axis exists, so the two OBB don't intersect.
	return true;
#endif
}

/// The implementation of OBB-Plane intersection test follows Christer Ericson's Real-Time Collision Detection, p. 163. [groupSyntax]
bool OBB::Intersects(const Plane &p) const
{
	// Compute the projection interval radius of this OBB onto L(t) = this->pos + x * p.normal;
	float t = r[0] * Abs(Dot(p.normal, axis[0])) +
			  r[1] * Abs(Dot(p.normal, axis[1])) +
			  r[2] * Abs(Dot(p.normal, axis[2]));
	// Compute the distance of this OBB center from the plane.
	float s = Dot(p.normal, pos) - p.d;
	return Abs(s) <= t;
}

bool OBB::Intersects(const Ray &ray) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray localRay = WorldToLocal() * ray;
	return aabb.Intersects(localRay);
}

bool OBB::Intersects(const Ray &ray, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Ray localRay = WorldToLocal() * ray;
	return aabb.Intersects(localRay, dNear, dFar);
}

bool OBB::Intersects(const Line &line) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const Line &line, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Line l = WorldToLocal() * line;
	return aabb.Intersects(l, dNear, dFar);
}

bool OBB::Intersects(const LineSegment &lineSegment) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l);
}

bool OBB::Intersects(const LineSegment &lineSegment, float &dNear, float &dFar) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	LineSegment l = WorldToLocal() * lineSegment;
	return aabb.Intersects(l, dNear, dFar);
}

/// The implementation of the OBB-Sphere intersection test follows Christer Ericson's Real-Time Collision Detection, p. 166. [groupSyntax]
bool OBB::Intersects(const Sphere &sphere, vec *closestPointOnOBB) const
{
	// Find the point on this AABB closest to the sphere center.
	vec pt = ClosestPoint(sphere.pos);

	// If that point is inside sphere, the AABB and sphere intersect.
	if (closestPointOnOBB)
		*closestPointOnOBB = pt;

	return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool OBB::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool OBB::Intersects(const Triangle &triangle) const
{
	AABB aabb(POINT_VEC_SCALAR(0.f), Size());
	Triangle t = WorldToLocal() * triangle;
	return t.Intersects(aabb);
}

bool OBB::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool OBB::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool OBB::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string OBB::ToString() const
{
	char str[256];
	sprintf(str, "OBB(Pos:(%.2f, %.2f, %.2f) Halfsize:(%.2f, %.2f, %.2f) X:(%.2f, %.2f, %.2f) Y:(%.2f, %.2f, %.2f) Z:(%.2f, %.2f, %.2f))",
		pos.x, pos.y, pos.z, r.x, r.y, r.z, axis[0].x, axis[0].y, axis[0].z, axis[1].x, axis[1].y, axis[1].z, axis[2].x, axis[2].y, axis[2].z);
	return str;
}

std::string OBB::SerializeToString() const
{
	std::string s = pos.xyz().SerializeToString() + " "
	              + r.xyz().SerializeToString() + " "
	              + axis[0].xyz().SerializeToString() + " "
	              + axis[1].xyz().SerializeToString() + " "
	              + axis[2].xyz().SerializeToString();
	return s;
}

std::string OBB::SerializeToCodeString() const
{
	return "OBB(" + pos.SerializeToCodeString() + ","
	              + r.SerializeToCodeString() + ","
	              + axis[0].SerializeToCodeString() + ","
	              + axis[1].SerializeToCodeString() + ","
	              + axis[2].SerializeToCodeString() + ")";
}

std::ostream &operator <<(std::ostream &o, const OBB &obb)
{
	o << obb.ToString();
	return o;
}

#endif

OBB OBB::FromString(const char *str, const char **outEndStr)
{
	assume(str);
	if (!str)
		return OBB(vec::nan, vec::nan, vec::nan, vec::nan, vec::nan);
	OBB o;
	MATH_SKIP_WORD(str, "OBB(");
	MATH_SKIP_WORD(str, "Pos:(");
	o.pos = PointVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Halfsize:(");
	o.r = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " X:(");
	o.axis[0] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Y:(");
	o.axis[1] = DirVecFromString(str, &str);
	MATH_SKIP_WORD(str, " Z:(");
	o.axis[2] = DirVecFromString(str, &str);
	if (outEndStr)
		*outEndStr = str;
	return o;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void OBB::Triangulate(VertexBuffer &vb, int x, int y, int z, bool ccwIsFrontFacing) const
{
	Array<vec> position;
	Array<vec> normal;
	Array<float2> uv;
	int numVertices = (x*y+y*z+x*z)*2*6;
	position.Resize_unspecified(numVertices);
	normal.Resize_unspecified(numVertices);
	uv.Resize_unspecified(numVertices);
	Triangulate(x,y,z, &position[0], &normal[0], &uv[0], ccwIsFrontFacing);
	int startIndex = vb.AppendVertices(numVertices);
	for(int i = 0; i < (int)position.size(); ++i)
	{
		vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(position[i]));
		if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			vb.Set(startIndex+i, VDNormal, DIR_TO_FLOAT4(normal[i]));
		if (vb.Declaration()->TypeOffset(VDUV) >= 0)
			vb.SetFloat2(startIndex+i, VDUV, 0, uv[i]);
	}
}

void OBB::ToLineList(VertexBuffer &vb) const
{
	if (vb.Declaration()->HasType(VDNormal))
	{
		// FacePlane() returns plane normals in order -x, +x, -y, +y, -z, +z
		// Cornerpoint() returns points in order
		// 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++. (corresponding the XYZ axis directions).

		// Vertices corresponding to these six faces:
		int verts[6][4] =
		{
			{ 0, 1, 3, 2 }, // -x
			{ 7, 6, 4, 5 }, // +x
			{ 0, 1, 5, 4 }, // -y
			{ 7, 6, 2, 3 }, // +y
			{ 0, 2, 6, 4 }, // -z
			{ 7, 5, 1, 3 }  // +z
		};
		int si = vb.AppendVertices(2*4*6);
		for(int face = 0; face < 6; ++face)
		{
			float4 faceNormal = DIR_TO_FLOAT4(FacePlane(face).normal);
			int v0 = verts[face][3];
			for(int v1i = 0; v1i < 4; ++v1i)
			{
				int v1 = verts[face][v1i];
				vb.Set(si, VDPosition, POINT_TO_FLOAT4(CornerPoint(v0)));
				vb.Set(si++, VDNormal, faceNormal);
				vb.Set(si, VDPosition, POINT_TO_FLOAT4(CornerPoint(v1)));
				vb.Set(si++, VDNormal, faceNormal);
				v0 = v1;
			}
		}
	}
	else
	{
		Array<vec> position;
		position.Resize_unspecified(NumVerticesInEdgeList());
		ToEdgeList(&position[0]);
		int startIndex = vb.AppendVertices((int)position.size());
		for(int i = 0; i < (int)position.size(); ++i)
			vb.Set(startIndex+i, VDPosition, POINT_TO_FLOAT4(position[i]));
	}
}

#endif

OBB operator *(const float3x3 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float3x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const float4x4 &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

OBB operator *(const Quat &transform, const OBB &obb)
{
	OBB o(obb);
	o.Transform(transform);
	return o;
}

MATH_END_NAMESPACE
