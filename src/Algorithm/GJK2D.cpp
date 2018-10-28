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

/** @file GJK2D.cpp
	@author Jukka Jylänki
	@brief Implementation of the Gilbert-Johnson-Keerthi (GJK) convex polygon intersection test in 2D. */
#include "GJK2D.h"
#include "../Geometry/LineSegment2D.h"
#include "../Geometry/Triangle2D.h"
#include "../Math/assume.h"
#include "../Math/MathConstants.h"
#include "../Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

/// This function examines the 2D simplex defined by the array of points in s, and calculates which voronoi region
/// of that simplex the origin is closest to. Based on that information, the function constructs a new 2D simplex
/// that will be used to continue the search, and returns a new search direction for the GJK algorithm.
/** @param s [in, out] An array of points in the simplex. When this function returns, this point array is updated to contain the new 2D search simplex.
	@param n [in, out] The number of points in the array s. When this function returns, this reference is updated to specify how many
	                   points the new 2D search simplex contains.
	@return The new search direction vector. */
vec2d UpdateSimplex2D(vec2d *s, int &n)
{
	assert1(n == 2 || n == 3, n);

	if (n == 2)
	{
		// Four voronoi regions that the origin could be in:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to line segment s[0]->s[1]. XX
		// 3) contained in the line segment s[0]->s[1], and our search is over and the algorithm is now finished. XX

		// By construction of the simplex, the cases 0) and 1) can never occur. Then only the cases marked with XX need to be checked.
#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d0 = s[0].DistanceSq(vec2d::zero);
		float d1 = s[1].DistanceSq(vec2d::zero);
		float d2 = LineSegment2D(s[0], s[1]).DistanceSq(vec2d::zero);
		assert2(d2 <= d0, d2, d0);
		assert2(d2 <= d1, d2, d1);
		// Cannot be in case 0: the step 0 -> 1 must have been toward the zero direction:
		assert(Dot(s[1]-s[0], -s[0]) >= 0.f);
		// Cannot be in case 1: the zero direction cannot be in the voronoi region of vertex s[1].
		assert(Dot(s[1]-s[0], -s[1]) <= 0.f);
#endif

		vec2d d01 = s[1] - s[0];
		vec2d newSearchDir = Perp2D(d01);
		if (newSearchDir.LengthSq() > 1e-7f)
		{
			return Dot(newSearchDir, s[0]) <= 0.f ? newSearchDir : -newSearchDir; // Case 2)
		}
		else
		{
			// Case 3)
			n = 0;
			return vec2d::zero;
		}
	}
	else
	{
		assert(n == 3);
		// Seven voronoi regions:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to vertex s[2].
		// 3) closest to edge s[0]->s[1].
		// 4) closest to edge s[1]->s[2].  XX
		// 5) closest to edge s[0]->s[2].  XX
		// 6) contained in the triangle s[0]->s[1]->s[2], and our search is over and the algorithm is now finished.  XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[2], since that
		// was the last added point. But it cannot be the case 2), since previous search took us deepest towards the direction s[1]->s[2],
		// and case 2) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-3) can never occur. Only the cases marked with XX need to be checked.
#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d[7];
		d[0] = s[0].DistanceSq(vec2d::zero);
		d[1] = s[1].DistanceSq(vec2d::zero);
		d[2] = s[2].DistanceSq(vec2d::zero);
		d[3] = LineSegment2D(s[0], s[1]).DistanceSq(vec2d::zero);
		d[4] = LineSegment2D(s[1], s[2]).DistanceSq(vec2d::zero);
		d[5] = LineSegment2D(s[2], s[0]).DistanceSq(vec2d::zero);
		d[6] = Triangle2D(s[0], s[1], s[2]).DistanceSq(vec2d::zero);

		bool isContainedInTriangle = (d[6] <= 1e-3f); // Are we in case 8)?
		float dist = FLOAT_INF;
		int minDistIndex = -1;
		for(int i = 4; i < 7; ++i)
			if (d[i] < dist)
			{
				dist = d[i];
				minDistIndex = i;
			}

		assert4(isContainedInTriangle || dist <= d[0] + 1e-4f, d[0], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[1] + 1e-4f, d[1], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[2] + 1e-4f, d[2], dist, isContainedInTriangle, minDistIndex);
		assert4(isContainedInTriangle || dist <= d[3] + 1e-4f, d[3], dist, isContainedInTriangle, minDistIndex);
#endif
		vec2d d12 = s[2]-s[1];
		vec2d d02 = s[2]-s[0];

		vec2d e12 = Perp2D(d12);
		if (Dot(d02, e12) <= 0.f) e12 = -e12;
		float t12 = Dot(s[1], e12);
		if (t12 < 0.f)
		{
			// Case 4: Edge 1->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[4] <= dist + 1e-3f * Max(1.f, d[4], dist), d[4], dist, isContainedInTriangle, minDistIndex);
#endif
			vec2d newDir = e12;
			s[0] = s[1];
			s[1] = s[2];
			n = 2;
			return newDir;
		}
		vec2d e02 = Perp2D(d02);
		if (Dot(d12, e02) <= 0.f) e02 = -e02;
		float t02 = Dot(s[0], e02);
		if (t02 < 0.f)
		{
			// Case 5: Edge 0->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[5] <= dist + 1e-3f * Max(1.f, d[5], dist), d[5], dist, isContainedInTriangle, minDistIndex);
#endif
			vec2d newDir = e02;
			s[1] = s[2];
			n = 2;
			return newDir;
		}
		// Case 6) The origin lies directly inside the triangle. For robustness, terminate the search here immediately with success.
#ifdef MATH_ASSERT_CORRECTNESS
		assert4(d[6] <= dist + 1e-3f * Max(1.f, d[6], dist), d[6], dist, isContainedInTriangle, minDistIndex);
#endif
		n = 0;
		return vec2d::zero;
	}
}

MATH_END_NAMESPACE
