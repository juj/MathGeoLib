/* Copyright Jukka Jylanki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file GJK.h
	@author Jukka Jylanki
	@brief Implementation of the Gilbert-Johnson-Keerthi (GJK) convex polyhedron intersection test. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"

vec UpdateSimplex(vec *s, int &n);

#define SUPPORT(dir) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS)); maxS -= minS;

template<typename A, typename B>
bool GJKIntersect(const A &a, const B &b)
{
	vec support[4];
	float maxS, minS;
	support[0] = a.AnyPointFast() - b.AnyPointFast();
	vec d = -support[0].Normalized();
	int n = 1;
	int nIterations = 50;
	while(nIterations-- > 0)
	{
		vec newSupport = SUPPORT(d);
#ifdef MATH_VEC_IS_FLOAT4
		newSupport.w = 0.f;
#endif
		if (Dot(newSupport, d) < 0.f)
			return false;
//		if (maxS < 0.f)
//			return false;
		support[n++] = newSupport;
		d = UpdateSimplex(support, n);
		d.Normalize();
		if (n == 0)
			return true;
	}
	assume2(false && "GJK intersection test did not converge to a result!", a, b);
	return false; // Report no intersection.
}
