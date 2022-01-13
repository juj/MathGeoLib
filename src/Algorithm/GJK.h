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
#include "../Geometry/AABB.h"

MATH_BEGIN_NAMESPACE

vec UpdateSimplex(vec *s, int &n);

#define SUPPORT(dir, minS, maxS) (a.ExtremePoint(dir, maxS) - b.ExtremePoint(-dir, minS));

template<typename A, typename B>
bool GJKIntersect(const A &a, const B &b)
{
	vec support[4];
	// Start with an arbitrary point in the Minkowski set shape.
	support[0] = a.AnyPointFast() - b.AnyPointFast();
	if (support[0].LengthSq() < 1e-7f) // Robustness check: Test if the first arbitrary point we guessed produced the zero vector we are looking for!
		return true;
	vec d = -support[0]; // First search direction is straight toward the origin from the found point.
	int n = 1; // Stores the current number of points in the search simplex.
	int nIterations = 50; // Robustness check: Limit the maximum number of iterations to perform to avoid infinite loop if types A or B are buggy!
	while(nIterations-- > 0)
	{
		// Compute the extreme point to the direction d in the Minkowski set shape.
		float maxS, minS;
		vec newSupport = SUPPORT(d, minS, maxS);
#ifdef MATH_VEC_IS_FLOAT4
		mgl_assume(newSupport.w == 0.f);
#endif
		// If the most extreme point in that search direction did not walk past the origin, then the origin cannot be contained in the Minkowski
		// convex shape, and the two convex objects a and b do not share a common point - no intersection!
		if (minS + maxS < 0.f)
			return false;
		// Add the newly evaluated point to the search simplex.
		mgl_assert(n < 4);
		support[n++] = newSupport;
		// Examine the current simplex, prune a redundant part of it, and produce the next search direction.
		d = UpdateSimplex(support, n);
		if (n == 0) // Was the origin contained in the current simplex? If so, then the convex shapes a and b do share a common point - intersection!
			return true;
	}
	mgl_assume(false && "GJK intersection test did not converge to a result!");
	// TODO: enable
	//mgl_assume2(false && "GJK intersection test did not converge to a result!", a.SerializeToString(), b.SerializeToString());
	return false; // Report no intersection.
}

// This computes GJK intersection, but by first translating both objects to a coordinate frame that is as closely
// centered around world origin as possible, to gain floating point precision.
template<typename A, typename B>
bool FloatingPointOffsetedGJKIntersect(const A &a, const B &b)
{
	AABB ab = a.MinimalEnclosingAABB();
	AABB bb = b.MinimalEnclosingAABB();
	vec offset = (Min(ab.minPoint, bb.minPoint) + Max(ab.maxPoint, bb.maxPoint)) * 0.5f;
	const vec floatingPointPrecisionOffset = -offset;
	return GJKIntersect(a.Translated(floatingPointPrecisionOffset), b.Translated(floatingPointPrecisionOffset));
}

MATH_END_NAMESPACE
