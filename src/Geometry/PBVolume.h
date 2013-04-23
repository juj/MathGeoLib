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

MATH_BEGIN_NAMESPACE

enum TestResult
{
	TestOutside, // The tested objects don't touch.
//	TestIntersect, // The tested objects intersect, but neither is contained inside each other.
	TestNotContained, // The tested object is not contained inside the other object, but no other information is known.
	TestInside // The tested object is contained inside the other object.
};

/// PBVolume is a "plane bounded volume", a convex polyhedron represented by a set
/// of planes.
template<int N>
class PBVolume
{
public:
	Plane p[N];

	int NumPlanes() const { return N; }

	/// Returns TestOutside, TestInside or TestNotContained.
	TestResult InsideOrIntersects(const AABB &aabb) const
	{
		TestResult result = TestInside;

		//float3 r = (aabb.maxPoint - aabb.minPoint) * 0.5f;
		//float3 c = aabb.minPoint + r;
		for(int i = 0; i < N; ++i)
		{
			float3 nPoint;// = aabb.minPoint;
			float3 pPoint;// = aabb.maxPoint;
			nPoint.x = (p[i].normal.x < 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			nPoint.y = (p[i].normal.y < 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			nPoint.z = (p[i].normal.z < 0.f ? aabb.maxPoint.z : aabb.minPoint.z);

			pPoint.x = (p[i].normal.x >= 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			pPoint.y = (p[i].normal.y >= 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			pPoint.z = (p[i].normal.z >= 0.f ? aabb.maxPoint.z : aabb.minPoint.z);

			/*
			// Find the n and p points of the aabb. (The nearest and farthest corners relative to the plane)
			const float3 &sign = npPointsSignLUT[((p[i].normal.z >= 0.f) ? 4 : 0) +
												 ((p[i].normal.y >= 0.f) ? 2 : 0) +
												 ((p[i].normal.x >= 0.f) ? 1 : 0)];
			const float3 nPoint = c + sign*r;
			const float3 pPoint = c - sign*r;
			*/

			float a = p[i].SignedDistance(nPoint);
			if (a >= 0.f)
				return TestOutside; // The AABB is certainly outside the frustum.
			a = p[i].SignedDistance(pPoint);
			if (a >= 0.f)
				result = TestNotContained; // At least one vertex is outside this frustum. The whole AABB can't possibly be contained in this frustum.
		}

		// We can return here either TestInside or TestNotContained, but it's possible that the AABB was outside the frustum, and we failed to find
		// a separating axis.
		return result;
	}

	TestResult InsideOrIntersects(const Sphere &sphere) const
	{
		TestResult result = TestInside;
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

MATH_END_NAMESPACE
