/* Copyright 2012 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file PBVolume.h
	@author Jukka Jylänki
	@brief Implements a convex polyhedron data structure. */

#pragma once

#include "../Math/float3.h"
#include "AABB.h"
#include "Plane.h"
#include "Sphere.h"

MATH_BEGIN_NAMESPACE

/// Reports a result from an approximate culling operation.
enum CullTestResult
{
	// The tested objects don't intersect - they are fully disjoint.
	TestOutside,

	// The tested object is at least not fully contained inside the other object, but no other information is known.
	// The objects might intersect or be disjoint.
	TestNotContained,

	// The tested object is fully contained inside the other object.
	TestInside
};

/// PBVolume is a "plane bounded volume", a convex polyhedron represented by a set
/// of planes. The number of planes is fixed at compile time so that compilers are able to perfectly unroll the loops for
/// best performance. As a fixed convention, the plane normals of the volume point outwards from the plane, so the
/// negative halfspaces are inside the convex volume.
template<int N>
class PBVolume
{
public:
	Plane p[N];

	int NumPlanes() const { return N; }

	bool Contains(const vec &point) const
	{
		for(int i = 0; i < N; ++i)
			if (p[i].SignedDistance(point) > 0.f)
				return false;
		return true;
	}

	/// Performs an *approximate* intersection test between this PBVolume and the given AABB.
	/** This function is best used for high-performance object culling purposes, e.g. for frustum-aabb culling, when
		a small percentage of false positives do not matter.
		@return An enum denoting whether the given object is inside or intersects this PBVolume. See the CullTestResult enum 
			for the interpretation of the return values. */
	CullTestResult InsideOrIntersects(const AABB &aabb) const
	{
		CullTestResult result = TestInside;

		for(int i = 0; i < N; ++i)
		{
			vec nPoint;
			vec pPoint;
			nPoint.x = (p[i].normal.x < 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			nPoint.y = (p[i].normal.y < 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			nPoint.z = (p[i].normal.z < 0.f ? aabb.maxPoint.z : aabb.minPoint.z);
#ifdef MATH_VEC_IS_FLOAT4
			nPoint.w = 1.f;
#endif

			pPoint.x = (p[i].normal.x >= 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			pPoint.y = (p[i].normal.y >= 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			pPoint.z = (p[i].normal.z >= 0.f ? aabb.maxPoint.z : aabb.minPoint.z);
#ifdef MATH_VEC_IS_FLOAT4
			pPoint.w = 1.f;
#endif

			/*
			// Find the n and p points of the aabb. (The nearest and farthest corners relative to the plane)
			const vec &sign = npPointsSignLUT[((p[i].normal.z >= 0.f) ? 4 : 0) +
												 ((p[i].normal.y >= 0.f) ? 2 : 0) +
												 ((p[i].normal.x >= 0.f) ? 1 : 0)];
			const vec nPoint = c + sign*r;
			const vec pPoint = c - sign*r;
			*/

			float a = p[i].SignedDistance(nPoint);
			if (a >= 0.f)
				return TestOutside; // The AABB is certainly outside this PBVolume.
			a = p[i].SignedDistance(pPoint);
			if (a >= 0.f)
				result = TestNotContained; // At least one vertex is outside this PBVolume. The whole AABB can't possibly be contained in this PBVolume.
		}

		// We can return here either TestInside or TestNotContained, but it's possible that the AABB was outside the frustum, and we
		// just failed to find a separating axis.
		return result;
	}

	CullTestResult InsideOrIntersects(const Sphere &sphere) const
	{
		CullTestResult result = TestInside;
		for(int i = 0; i < N; ++i)
		{
			float d = p[i].SignedDistance(sphere.pos);
			if (d >= sphere.r)
				return TestOutside;
			else if (d >= -sphere.r)
				result = TestNotContained;
		}
		return result;
	}
};

class Frustum;

PBVolume<6> ToPBVolume(const Frustum &frustum);

MATH_END_NAMESPACE
