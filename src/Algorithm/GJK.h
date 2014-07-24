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

MATH_BEGIN_NAMESPACE

vec UpdateSimplex(vec *s, int &n);

#define SUPPORT(dir) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS));

template<typename A, typename B>
bool GJKIntersect(const A &a, const B &b)
{
	vec support[4];
	float maxS, minS;
	// Start with an arbitrary point in the Minkowski set shape.
	support[0] = a.AnyPointFast() - b.AnyPointFast();
	vec d = -support[0]; // First search direction is straight toward the origin from the found point.
	if (d.LengthSq() < 1e-7f) // Robustness check: Test if the first arbitrary point we guessed produced the zero vector we are looking for!
		return true;
	int n = 1; // Stores the current number of points in the search simplex.
	int nIterations = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
	while(nIterations-- > 0)
	{
		// Compute the extreme point to the direction d in the Minkowski set shape.
		vec newSupport = SUPPORT(d);
#ifdef MATH_VEC_IS_FLOAT4
		assume(newSupport.w == 0.f);
#endif
		// If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
		// convex shape, and the two convex objects a and b do not share a common point - no intersection!
		if (minS + maxS < 0.f)
			return false;
		// Add the newly evaluated point to the search simplex.
		support[n++] = newSupport;
		// Examine the current simplex, prune a redundant part of it, and produce the next search direction.
		d = UpdateSimplex(support, n);
		if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
			return true;
	}
	assume2(false && "GJK intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
	return false; // Report no intersection.
}

MATH_END_NAMESPACE
