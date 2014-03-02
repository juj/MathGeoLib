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

vec UpdateSimplex(vec *s, int &n)
{
	// The latest point is so close to zero that for numerical stability, we should just consider having reached the target?
	if (s[n-1].LengthSq() < 1e-8f)
	{
		n = 0;
		return vec::zero;
	}

	if (n == 2)
	{
		assert(LineSegment(s[0], s[1]).Distance(vec::zero) <= s[0].Length());
		assert(LineSegment(s[0], s[1]).Distance(vec::zero) <= s[1].Length());
		assert(Dot(s[1]-s[0], -s[1]) <= 0.f);
		assert(Dot(s[1]-s[0], -s[0]) >= 0.f);

		// Which direction is the origin at from the latest added point?
		vec zeroDir = -s[1];
		float l = zeroDir.Normalize();
		if (l < 1e-4f)
		{
			n = 0;
			return zeroDir;
		}
		// Point s[0] was extended to point s[1] towards direction newDir, and it formed a line segment s[0] -> s[1].
		vec d = s[1] - s[0];
		l = d.Normalize();
		if (l < 1e-4f)
		{
			s[0] = s[1];
			n = 1;
			return zeroDir;
		}

		// Precondition: the step 0 -> 1 must have been toward the zero direction:
		assert(Dot(s[1]-s[0], -s[0]) >= 0.f);
		// Precondition: the zero direction cannot be in the half-space defined by 0 -> 1.
		assert(Dot(s[1]-s[0], -s[1]) <= 0.f);

		vec normal = Cross(zeroDir, d);
		vec newDir = Cross(d, normal);
		assert(newDir.IsPerpendicular(d));
		assert(newDir.IsPerpendicular(normal));
		assert3(Dot(newDir, zeroDir) >= -1e8f, newDir, zeroDir, Dot(newDir, zeroDir));
		float len = newDir.Normalize();
		if (len <= 1e-4f)
			n = 0;
		else
			assert(Dot(newDir, zeroDir) >= 0.f);
		return newDir;
	}
	else if (n == 3)
	{
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
		vec e02 = Cross(triNormal, d02).Normalized();
		assert(Dot(e02, s[1]-s[0]) <= 0.f);
		vec e12 = Cross(d12, triNormal).Normalized();
		assert(Dot(e12, s[0]-s[1]) <= 0.f);

		vec zeroDir = -s[2];

//		float t01 = Dot(zeroDir, e01);
		float t02 = Dot(zeroDir, e02);
		float t12 = Dot(zeroDir, e12);

		// We walked towards point s[2] to get to origin, so the origin cannot be on the side of edge 0->1.
		//assert(t01 >= 0.f);
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
		}*/
		if (t02 > 0.f && t12 > 0.f)
		{
			// Degenerated into a single point.
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
			s[1] = s[2];
			n = 2;
			vec newDir = Cross(e02, Cross(zeroDir, e02));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
		if (t12 > 0.f)
		{
			s[0] = s[1];
			s[1] = s[2];
			n = 2;
			vec newDir = Cross(e12, Cross(zeroDir, e12));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
		Triangle t(s[0], s[1], s[2]);
		vec cp = t.ClosestPoint(vec::zero);
		assert(cp.LengthSq() <= s[0].LengthSq());
		assert(cp.LengthSq() <= s[1].LengthSq());
		assert(cp.LengthSq() <= s[2].LengthSq());
		if (Dot(triNormal, zeroDir) >= 0.f)
			return triNormal;
		else
		{
			std::swap(s[0], s[1]);
			return -triNormal;
		}
	}
	else
	{
		// Which direction is the origin at from the latest added point?
		vec zeroDir = -s[3];

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
		float inTri013 = Dot(zeroDir, tri013Normal);
		float inTri023 = Dot(zeroDir, tri023Normal);
		float inTri123 = Dot(zeroDir, tri123Normal);

		vec tri012Normal = Cross(d02, d01);
		assert(Dot(tri012Normal, d03) <= 0.f);
//		assert(Dot(zeroDir, tri012Normal) <= 0.f);

		if (inTri013 > 0.f && inTri023 > 0.f && inTri123 > 0.f)
		{
			// The new point 3 is closest. Simplex degenerates back to a single point.
			s[0] = s[3];
			n = 1;
			return zeroDir;
		}
		if (inTri013 > 0.f && inTri023 > 0.f)
		{
			// Edge 0->3 is closest. Simplex degenerates to a line segment.
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(zeroDir, s[1]-s[0]));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
		if (inTri013 > 0.f && inTri123 > 0.f)
		{
			// Edge 1->3 is closest. Simplex degenerates to a line segment.
			s[0] = s[1];
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(zeroDir, s[1]-s[0]));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
		if (inTri123 > 0.f && inTri023 > 0.f)
		{
			// Edge 2->3 is closest. Simplex degenerates to a line segment.
			s[0] = s[2];
			s[1] = s[3];
			n = 2;
			vec newDir = Cross(s[1]-s[0], Cross(zeroDir, s[1]-s[0]));
			assert(Dot(newDir, zeroDir) >= 0.f);
			return newDir;
		}
		if (inTri013 > 0.f)
		{
			s[2] = s[3];
			n = 3;
			return tri013Normal;
		}
		if (inTri023 > 0.f)
		{
			s[1] = s[0];
			s[0] = s[2];
			s[2] = s[3];
			n = 3;
			return tri023Normal;
		}
		if (inTri123 > 0.f)
		{
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
