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

/// This function examines the simplex defined by the array of points in s, and calculates which voronoi region
/// of that simplex the origin is closest to. Based on that information, the function constructs a new simplex
/// that will be used to continue the search, and returns a new search direction for the GJK algorithm.
vec UpdateSimplex(vec *s, int &n)
{
	// The latest point is so close to zero that for numerical stability, we should just consider having reached the target?
	if (s[n-1].LengthSq() < 1e-8f)
	{
		n = 0;
		return vec::zero;
	}

//	for(int i = 0; i < n; ++i)
//		LOGI("Simplex pt %d: %s", i, s[i].ToString().c_str());
	if (n == 2)
	{
		// Four voronoi regions that the origin could be in:
		// 0) closest to vertex s[0].
		// 1) closest to vertex s[1].
		// 2) closest to line segment s[0]->s[1].
		// 3) contained in the line segment s[0]->s[1], and our search is over and the algorithm is now finished.

		// By construction of the simplex, the cases 0) and 1) can never occur.
#ifdef MATH_ASSERT_CORRECTNESS
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
		vec newSearchDir = Cross(d01, Cross(-s[1], d01));
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
		// and case 2) implies we should have been able to go even deeper in that direction, or that the origin is not included in the simplex,
		// a case which has been checked for already before. Therefore the cases 0)-3) can never occur.
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

		assert(isContainedInTriangle || dist <= d[0] + 1e-4f);
		assert(isContainedInTriangle || dist <= d[1] + 1e-4f);
		assert(isContainedInTriangle || dist <= d[2] + 1e-4f);
		assert(isContainedInTriangle || dist <= d[3] + 1e-4f);
		if (dist < 1e-5f)
		{
			n = 0;
			return vec::zero;
		}
#endif
		vec d01 = (s[1]-s[0]).Normalized();
		vec d12 = (s[2]-s[1]).Normalized();
		vec d02 = s[2]-s[0];
		float len = d02.Normalize();
		if (len < 1e-4f)
		{
			s[0] = s[2];
			n = 2;
			return UpdateSimplex(s, n);
		}
		vec triNormal = Cross(d02, d12).Normalized();
		vec e01 = Cross(d01, triNormal);
		assert(Dot(e01, s[2]-s[0]) <= 0.f);
		assert(EqualAbs(Dot(e01, s[0]), Dot(e01, s[1])));
		vec e02 = Cross(triNormal, d02).Normalized();
		assert(Dot(e02, s[1]-s[0]) <= 0.f);
		assert(EqualAbs(Dot(e02, s[0]), Dot(e02, s[2])));
		vec e12 = Cross(d12, triNormal).Normalized();
		assert(Dot(e12, s[0]-s[1]) <= 0.f);
		assert(EqualAbs(Dot(e12, s[1]), Dot(e12, s[2])));

//		vec zeroDir = -s[2];

//		float t01 = Dot(-s[0], e01);
		float t02 = Dot(-s[0], e02);
		float t12 = Dot(-s[1], e12);

		// We walked towards point s[2] to get to origin, so the origin cannot be on the side of edge 0->1.
//		assert(t01 <= 0.f);
/*
		if (t01 > 0.f && t02 > 0.f)
		{
			n = 1;
			return -s[0];
		}
		if (t01 > 0.f && t12 > 0.f)
		{
			s[0] = s[1];
			n = 1;
			return -s[0];
		}
	*/
		if (t02 > 0.f && t12 > 0.f)
		{
			// s[2] is closest: degenerated into a single point.
			assert(minDistIndex == 2);
			assert(s[2].LengthSq() <= s[0].LengthSq());
			assert(s[2].LengthSq() <= s[1].LengthSq());
			s[0] = s[2];
			n = 1;
			return -s[0];
		}
/*
		if (t01 > 0.f)
		{
			n = 2;
			vec newDir = Cross(e01, Cross(zeroDir, e01));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
*/
		if (t02 > 0.f)
		{
			// Case 5: Edge 0->2 is closest.
			assert(d[5] <= dist + 1e-3f * Max(1.f, d[5], dist));
			s[1] = s[2];
			n = 2;
			vec newDir = Cross(e02, Cross(-s[0], e02));
			assert(Dot(newDir, -s[0]) >= 0.f);
			vec newDir2 = -LineSegment(s[0], s[2]).ClosestPoint(vec::zero);
			newDir.Normalize();
			newDir2.Normalize();
			return newDir2;
		}
		if (t12 > 0.f)
		{
			// Case 4: Edge 1->2 is closest.
			assert(d[4] <= dist + 1e-3f * Max(1.f, d[4], dist));
			s[0] = s[1];
			s[1] = s[2];
			n = 2;
			vec newDir = Cross(e12, Cross(-s[1], e12));
			assert(Dot(newDir, -s[1]) >= 0.f);
			vec newDir2 = -LineSegment(s[0], s[1]).ClosestPoint(vec::zero);
			newDir.Normalize();
			newDir2.Normalize();
			return newDir2;
		}
		// Cases 6)-8):
		assert(d[6] <= dist + 1e-3f * Max(1.f, d[6], dist));
		Triangle t(s[0], s[1], s[2]);
		vec cp = t.ClosestPoint(vec::zero);
		assert(cp.LengthSq() <= s[0].LengthSq());
		assert(cp.LengthSq() <= s[1].LengthSq());
		assert(cp.LengthSq() <= s[2].LengthSq());
		if (Dot(triNormal, -s[2]) >= 0.f)
		{
			// Case 6:
			assert(t.PlaneCCW().IsOnPositiveSide(vec::zero));
			return triNormal;
		}
		else
		{
			// Case 7:
			assert(t.PlaneCW().IsOnPositiveSide(vec::zero));
			// Swap s[0] and s[1] so that the normal of Triangle(s[0],s[1],s[2]).PlaneCCW() will always point towards the new search direction.
			std::swap(s[0], s[1]);
			return -triNormal;
		}
	}
	else // n == 4
	{
		vec d01 = s[1] - s[0];
		vec d02 = s[2] - s[0];
		vec d03 = s[3] - s[0];
		vec d12 = s[2] - s[1];
		vec d13 = s[3] - s[1];
		vec d23 = s[3] - s[2];
		vec tri013Normal = Cross(d01, d03);
		assert(Dot(tri013Normal, d02) <= 0.f);
		vec tri023Normal = Cross(d03, d02);
		assert(Dot(tri023Normal, d01) <= 0.f);
		vec tri123Normal = Cross(d12, d13);
		assert(Dot(tri123Normal, -d02) <= 0.f);
		float inTri013 = Dot(-s[3], tri013Normal);
		float inTri023 = Dot(-s[3], tri023Normal);
		float inTri123 = Dot(-s[3], tri123Normal);

		vec tri012Normal = Cross(d02, d01);
		assert(Dot(tri012Normal, d03) <= 0.f);
		float inTri012 = Dot(-s[0], tri012Normal);
//		assert(inTri012 <= 0.f);

		// A tetrahedron defines fifteen voronoi regions:
		//  0) closest to vertex s[0].
		//  1) closest to vertex s[1].
		//  2) closest to vertex s[2].
		//  3) closest to vertex s[3].      XX
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

		// By construction of the simplex, the origin can lie only in one of the voronoi regions that is related to the most
		// recently added point s[3]. Therefore the cases 0), 1), 2), 4), 5), 7) or 10) can never occur and 
		// we only need to check cases 3), 6), 8), 9), 11), 12), 13) and 14), marked with XX.

		// e03, e13, e23, n013, n023, n123
		// 3): e03+, e13+, e23+
		// 6): e03xn013+, e03x023+
		// 8): e13xn013+, e13x123+
		// 9): e23xn023+, e23x123+

#ifdef MATH_ASSERT_CORRECTNESS
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

//		assert(inTri012 <= 0.f);
		bool insideSimplex = inTri012 <= 0.f && inTri013 <= 0.f && inTri023 <= 0.f && inTri123 <= 0.f;

		float dist = d[2];
		int minDistIndex = 2;
		for(int i = 3; i < 14; ++i)
			if (i == 3 || i == 6 || i == 8 || i == 9 || i == 11 || i == 12 || i == 13 || i == 14)
				if (d[i] < dist)
				{
					dist = d[i];
					minDistIndex = i;
				}
		assert(insideSimplex || dist <= d[0] + 1e-4f);
		assert(insideSimplex || dist <= d[1] + 1e-4f);
		assert(insideSimplex || dist <= d[2] + 1e-4f);
		assert(insideSimplex || dist <= d[4] + 1e-4f);
		assert(insideSimplex || dist <= d[5] + 1e-4f);
		assert(insideSimplex || dist <= d[7] + 1e-4f);
		assert(insideSimplex || dist <= d[10] + 1e-4f);
#endif

		// Which direction is the origin at from the latest added point?
//		vec zeroDir = -s[3];

		float inD01 = Dot(d01, -s[1]) >= 0.f;
		float inD02 = Dot(d02, -s[2]) >= 0.f;
		float inD03 = Dot(d03, -s[3]) >= 0.f;
		float inD12 = Dot(d12, -s[2]) >= 0.f;
		float inD13 = Dot(d13, -s[3]) >= 0.f;
		float inD23 = Dot(d23, -s[3]) >= 0.f;

//		if (inTri013 > 0.f && inTri023 > 0.f && inTri123 > 0.f)
		//if (inD03 && inD13 && inD23)
		if (!insideSimplex && minDistIndex == 3)
		{
			assert(inD03 >= 0.f && inD13 >= 0.f && inD23 >= 0.f);
			// The new point 3 is closest. Simplex degenerates back to a single point.
			assert(!insideSimplex);
			assert(d[3] <= dist);
			s[0] = s[3];
			n = 1;
			return -s[3];
		}
		//if (inTri013 > 0.f && inTri023 > 0.f)
		if (!insideSimplex && minDistIndex == 6)
		{
			// Edge 0->3 is closest. Simplex degenerates to a line segment.
			assert(!insideSimplex);
			assert(d[6] <= dist);
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(-s[0], s[1]-s[0]));
			assert(Dot(newDir, -s[0]) >= 0.f);
			vec newDir2 = -LineSegment(s[0], s[1]).ClosestPoint(vec::zero);
			newDir.Normalize();
			newDir2.Normalize();
			return newDir2;
		}
		//if (inTri013 > 0.f && inTri123 > 0.f)
		if (!insideSimplex && minDistIndex == 8)
		{
			// Edge 1->3 is closest. Simplex degenerates to a line segment.
			assert(!insideSimplex);
			assert(d[8] <= dist);
			s[0] = s[1];
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(-s[0], s[1]-s[0]));
			assert(Dot(newDir, -s[0]) >= 0.f);
			vec newDir2 = -LineSegment(s[0], s[1]).ClosestPoint(vec::zero);
			newDir.Normalize();
			newDir2.Normalize();
			return newDir2;
		}
		//if (inTri123 > 0.f && inTri023 > 0.f)
		if (!insideSimplex && minDistIndex == 9)
		{
			// Edge 2->3 is closest. Simplex degenerates to a line segment.
			assert(!insideSimplex);
			assert(d[9] <= dist);
			s[0] = s[2];
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(-s[0], s[1]-s[0]));
			assert(Dot(newDir, -s[0]) >= 0.f);
			vec newDir2 = -LineSegment(s[0], s[1]).ClosestPoint(vec::zero);
			newDir.Normalize();
			newDir2.Normalize();
			return newDir2;
		}
		//if (inTri013 > 0.f)
		if (!insideSimplex && minDistIndex == 11)
		{
			// Triangle 0->1->3 is closest.
			assert(!insideSimplex);
			assert(d[11] <= dist);
			s[2] = s[3];
			n = 3;
			return tri013Normal;
		}
		//if (inTri023 > 0.f)
		if (!insideSimplex && minDistIndex == 12)
		{
			// Triangle 0->2->3 is closest.
			assert(!insideSimplex);
			assert(d[12] <= dist);
			s[1] = s[0];
			s[0] = s[2];
			s[2] = s[3];
			n = 3;
			return tri023Normal;
		}
		//if (inTri123 > 0.f)
		if (!insideSimplex && minDistIndex == 13)
		{
			// Triangle 1->2->3 is closest.
			assert(!insideSimplex);
			assert(d[13] <= dist);
			s[0] = s[1];
			s[1] = s[2];
			s[2] = s[3];
			n = 3;
			return tri123Normal;
		}
		n = 0;
		return vec::zero;
	}
}
