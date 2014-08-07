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

/** @file GJK.cpp
	@author Jukka Jylänki
	@brief Implementation of the Gilbert-Johnson-Keerthi (GJK) convex polyhedron intersection test. */
#include "GJK.h"
#include "../Geometry/LineSegment.h"
#include "../Geometry/Triangle.h"
#include "../Geometry/Plane.h"

MATH_BEGIN_NAMESPACE

/// This function examines the simplex defined by the array of points in s, and calculates which voronoi region
/// of that simplex the origin is closest to. Based on that information, the function constructs a new simplex
/// that will be used to continue the search, and returns a new search direction for the GJK algorithm.
/** @param s [in, out] An array of points in the simplex. When this function returns, this point array is updated to contain the new search simplex.
	@param n [in, out] The number of points in the array s. When this function returns, this reference is updated to specify how many
	                   points the new search simplex contains.
	@return The new search direction vector. */
vec UpdateSimplex(vec *s, int &n)
{
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
		float d0 = s[0].DistanceSq(vec::zero);
		float d1 = s[1].DistanceSq(vec::zero);
		float d2 = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		assert2(d2 <= d0, d2, d0);
		assert2(d2 <= d1, d2, d1);
		// Cannot be in case 0: the step 0 -> 1 must have been toward the zero direction:
		assert(Dot(s[1]-s[0], -s[0]) >= 0.f);
		// Cannot be in case 1: the zero direction cannot be in the voronoi region of vertex s[1].
		assert(Dot(s[1]-s[0], -s[1]) <= 0.f);
#endif

		vec d01 = s[1] - s[0];
		vec newSearchDir = Cross(d01, Cross(d01, s[1]));
		if (newSearchDir.LengthSq() > 1e-7f)
			return newSearchDir; // Case 2)
		else
		{
			// Case 3)
			n = 0;
			return vec::zero;
		}
	}
	else if (n == 3)
	{
		// Nine voronoi regions:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to vertex s[2].
		// 3) closest to edge s[0]->s[1].
		// 4) closest to edge s[1]->s[2].  XX
		// 5) closest to edge s[0]->s[2].  XX
		// 6) closest to the triangle s[0]->s[1]->s[2], in the positive side.  XX
		// 7) closest to the triangle s[0]->s[1]->s[2], in the negative side.  XX
		// 8) contained in the triangle s[0]->s[1]->s[2], and our search is over and the algorithm is now finished.  XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[2], since that
		// was the last added point. But it cannot be the case 2), since previous search took us deepest towards the direction s[1]->s[2],
		// and case 2) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-3) can never occur. Only the cases marked with XX need to be checked.
#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d[7];
		d[0] = s[0].DistanceSq(vec::zero);
		d[1] = s[1].DistanceSq(vec::zero);
		d[2] = s[2].DistanceSq(vec::zero);
		d[3] = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		d[4] = LineSegment(s[1], s[2]).DistanceSq(vec::zero);
		d[5] = LineSegment(s[2], s[0]).DistanceSq(vec::zero);
		d[6] = Triangle(s[0], s[1], s[2]).DistanceSq(vec::zero);

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
		vec d12 = s[2]-s[1];
		vec d02 = s[2]-s[0];
		vec triNormal = Cross(d02, d12);

		vec e12 = Cross(d12, triNormal);
		float t12 = Dot(s[1], e12);
		if (t12 < 0.f)
		{
			// Case 4: Edge 1->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[4] <= dist + 1e-3f * Max(1.f, d[4], dist), d[4], dist, isContainedInTriangle, minDistIndex);
#endif
			vec newDir = Cross(d12, Cross(d12, s[1]));
			s[0] = s[1];
			s[1] = s[2];
			n = 2;
			return newDir;
		}
		vec e02 = Cross(triNormal, d02);
		float t02 = Dot(s[0], e02);
		if (t02 < 0.f)
		{
			// Case 5: Edge 0->2 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(d[5] <= dist + 1e-3f * Max(1.f, d[5], dist), d[5], dist, isContainedInTriangle, minDistIndex);
#endif
			vec newDir = Cross(d02, Cross(d02, s[0]));
			s[1] = s[2];
			n = 2;
			return newDir;
		}
		// Cases 6)-8):
#ifdef MATH_ASSERT_CORRECTNESS
		assert4(d[6] <= dist + 1e-3f * Max(1.f, d[6], dist), d[6], dist, isContainedInTriangle, minDistIndex);
#endif
		float scaledSignedDistToTriangle = triNormal.Dot(s[2]);
		float distSq = scaledSignedDistToTriangle*scaledSignedDistToTriangle;
		float scaledEpsilonSq = 1e-6f*triNormal.LengthSq();

		if (distSq > scaledEpsilonSq)
		{
			// The origin is sufficiently far away from the triangle.
			if (scaledSignedDistToTriangle <= 0.f)
				return triNormal; // Case 6)
			else
			{
				// Case 7) Swap s[0] and s[1] so that the normal of Triangle(s[0],s[1],s[2]).PlaneCCW() will always point towards the new search direction.
				std::swap(s[0], s[1]);
				return -triNormal;
			}
		}
		else
		{
			// Case 8) The origin lies directly inside the triangle. For robustness, terminate the search here immediately with success.
			n = 0;
			return vec::zero;
		}
	}
	else // n == 4
	{
		// A tetrahedron defines fifteen voronoi regions:
		//  0) closest to vertex s[0].
		//  1) closest to vertex s[1].
		//  2) closest to vertex s[2].
		//  3) closest to vertex s[3].
		//  4) closest to edge s[0]->s[1].
		//  5) closest to edge s[0]->s[2].
		//  6) closest to edge s[0]->s[3].  XX
		//  7) closest to edge s[1]->s[2].
		//  8) closest to edge s[1]->s[3].  XX
		//  9) closest to edge s[2]->s[3].  XX
		// 10) closest to the triangle s[0]->s[1]->s[2], in the outfacing side.
		// 11) closest to the triangle s[0]->s[1]->s[3], in the outfacing side. XX
		// 12) closest to the triangle s[0]->s[2]->s[3], in the outfacing side. XX
		// 13) closest to the triangle s[1]->s[2]->s[3], in the outfacing side. XX
		// 14) contained inside the tetrahedron simplex, and our search is over and the algorithm is now finished. XX

		// By construction of the simplex, the origin must always be in a voronoi region that includes the point s[3], since that
		// was the last added point. But it cannot be the case 3), since previous search took us deepest towards the direction s[2]->s[3],
		// and case 3) implies we should have been able to go even deeper in that direction, or that the origin is not included in the convex shape,
		// a case which has been checked for already before. Therefore the cases 0)-5), 7) and 10) can never occur and
		// we only need to check cases 6), 8), 9), 11), 12), 13) and 14), marked with XX.

#ifdef MATH_ASSERT_CORRECTNESS
		// Sanity-check that the above reasoning is valid by testing each voronoi region and assert()ing that the ones we assume never to
		// happen never will.
		float d[14];
		d[0] = s[0].DistanceSq(vec::zero);
		d[1] = s[1].DistanceSq(vec::zero);
		d[2] = s[2].DistanceSq(vec::zero);
		d[3] = s[3].DistanceSq(vec::zero);
		d[4] = LineSegment(s[0], s[1]).DistanceSq(vec::zero);
		d[5] = LineSegment(s[0], s[2]).DistanceSq(vec::zero);
		d[6] = LineSegment(s[0], s[3]).DistanceSq(vec::zero);
		d[7] = LineSegment(s[1], s[2]).DistanceSq(vec::zero);
		d[8] = LineSegment(s[1], s[3]).DistanceSq(vec::zero);
		d[9] = LineSegment(s[2], s[3]).DistanceSq(vec::zero);
		d[10] = Triangle(s[0], s[1], s[2]).DistanceSq(vec::zero);
		d[11] = Triangle(s[0], s[1], s[3]).DistanceSq(vec::zero);
		d[12] = Triangle(s[0], s[2], s[3]).DistanceSq(vec::zero);
		d[13] = Triangle(s[1], s[2], s[3]).DistanceSq(vec::zero);

		vec Tri013Normal = Cross(s[1]-s[0], s[3]-s[0]);
		vec Tri023Normal = Cross(s[3]-s[0], s[2]-s[0]);
		vec Tri123Normal = Cross(s[2]-s[1], s[3]-s[1]);
		vec Tri012Normal = Cross(s[2] - s[0], s[1] - s[0]);
		assert(Dot(Tri012Normal, s[3] - s[0]) <= 0.f);
		float InTri012 = Dot(-s[0], Tri012Normal);
		float InTri013 = Dot(-s[3], Tri013Normal);
		float InTri023 = Dot(-s[3], Tri023Normal);
		float InTri123 = Dot(-s[3], Tri123Normal);
		bool insideSimplex = InTri012 <= 0.f && InTri013 <= 0.f && InTri023 <= 0.f && InTri123 <= 0.f;

		float dist = FLOAT_INF;
		int minDistIndex = -1;
		for(int i = 6; i < 14; ++i)
			if (i == 6 || i == 8 || i == 9 || i == 11 || i == 12 || i == 13)
				if (d[i] < dist)
				{
					dist = d[i];
					minDistIndex = i;
				}
		assert4(insideSimplex || dist <= d[0] + 1e-4f * Max(1.f, d[0], dist), d[0], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[1] + 1e-4f * Max(1.f, d[1], dist), d[1], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[2] + 1e-4f * Max(1.f, d[2], dist), d[2], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[4] + 1e-4f * Max(1.f, d[4], dist), d[4], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[5] + 1e-4f * Max(1.f, d[5], dist), d[5], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[7] + 1e-4f * Max(1.f, d[7], dist), d[7], dist, insideSimplex, minDistIndex);
		assert4(insideSimplex || dist <= d[10] + 1e-4f * Max(1.f, d[10], dist), d[10], dist, insideSimplex, minDistIndex);
#endif

		vec d01 = s[1] - s[0];
		vec d02 = s[2] - s[0];
		vec d03 = s[3] - s[0];
		vec tri013Normal = Cross(d01, d03); // Normal of triangle 0->1->3 pointing outwards from the simplex.
		vec tri023Normal = Cross(d03, d02); // Normal of triangle 0->2->3 pointing outwards from the simplex.
		assert(Dot(tri013Normal, d02) <= 0.f);
		assert(Dot(tri023Normal, d01) <= 0.f);

		vec e03_1 = Cross(tri013Normal, d03); // The normal of edge 0->3 on triangle 013.
		vec e03_2 = Cross(d03, tri023Normal); // The normal of edge 0->3 on triangle 023.
		float inE03_1 = Dot(e03_1, s[3]);
		float inE03_2 = Dot(e03_2, s[3]);
		if (inE03_1 <= 0.f && inE03_2 <= 0.f)
		{
			// Case 6) Edge 0->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[6] <= dist + 1e-3f * Max(1.f, d[6], dist), d[6], dist, insideSimplex, minDistIndex);
#endif
			vec newDir = Cross(d03, Cross(d03, s[3]));
			s[1] = s[3];
			n = 2;
			return newDir;
		}

		vec d12 = s[2] - s[1];
		vec d13 = s[3] - s[1];
		vec tri123Normal = Cross(d12, d13);
		assert(Dot(tri123Normal, -d02) <= 0.f);
		vec e13_0 = Cross(d13, tri013Normal);
		vec e13_2 = Cross(tri123Normal, d13);
		float inE13_0 = Dot(e13_0, s[3]);
		float inE13_2 = Dot(e13_2, s[3]);
		if (inE13_0 <= 0.f && inE13_2 <= 0.f)
		{
			// Case 8) Edge 1->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[8] <= dist + 1e-3f * Max(1.f, d[8], dist), d[8], dist, insideSimplex, minDistIndex);
#endif
			vec newDir = Cross(d13, Cross(d13, s[3]));
			s[0] = s[1];
			s[1] = s[3];
			n = 2;
			return newDir;
		}

		vec d23 = s[3] - s[2];
		vec e23_0 = Cross(tri023Normal, d23);
		vec e23_1 = Cross(d23, tri123Normal);
		float inE23_0 = Dot(e23_0, s[3]);
		float inE23_1 = Dot(e23_1, s[3]);
		if (inE23_0 <= 0.f && inE23_1 <= 0.f)
		{
			// Case 9) Edge 2->3 is closest. Simplex degenerates to a line segment.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[9] <= dist + 1e-3f * Max(1.f, d[9], dist), d[9], dist, insideSimplex, minDistIndex);
#endif
			vec newDir = Cross(d23, Cross(d23, s[3]));
			s[0] = s[2];
			s[1] = s[3];
			n = 2;
			return newDir;
		}

		float inTri013 = Dot(s[3], tri013Normal);
		if (inTri013 < 0.f && inE13_0 >= 0.f && inE03_1 >= 0.f)
		{
			// Case 11) Triangle 0->1->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[11] <= dist + 1e-3f * Max(1.f, d[11], dist), d[11], dist, insideSimplex, minDistIndex);
#endif
			s[2] = s[3];
			n = 3;
			return tri013Normal;
		}
		float inTri023 = Dot(s[3], tri023Normal);
		if (inTri023 < 0.f && inE23_0 >= 0.f && inE03_2 >= 0.f)
		{
			// Case 12) Triangle 0->2->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[12] <= dist + 1e-3f * Max(1.f, d[12], dist), d[12], dist, insideSimplex, minDistIndex);
#endif
			s[1] = s[0];
			s[0] = s[2];
			s[2] = s[3];
			n = 3;
			return tri023Normal;
		}
		float inTri123 = Dot(s[3], tri123Normal);
		if (inTri123 < 0.f && inE13_2 >= 0.f && inE23_1 >= 0.f)
		{
			// Case 13) Triangle 1->2->3 is closest.
#ifdef MATH_ASSERT_CORRECTNESS
			assert4(!insideSimplex && d[13] <= dist + 1e-3f * Max(1.f, d[13], dist), d[13], dist, insideSimplex, minDistIndex);
#endif
			s[0] = s[1];
			s[1] = s[2];
			s[2] = s[3];
			n = 3;
			return tri123Normal;
		}

		// Case 14) Not in the voronoi region of any triangle or edge. The origin is contained in the simplex, the search is finished.
		n = 0;
		return vec::zero;
	}
}

MATH_END_NAMESPACE
