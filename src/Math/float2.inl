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

/** @file float2.inl
	@author Jukka Jylänki
	@brief */
#include "float2.h"
#include "float3.h"
#include "float4.h"
#include "MathFunc.h"
#include "Swap.h"
#include "../Algorithm/Random/LCG.h"
#include "assume.h"
#include <string.h>
#include <stdlib.h>
#include <locale.h>

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#include <iostream>
#include <utility>
#include <algorithm>
#endif

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Algorithm/Sort/Sort.h"

template<typename T>
int TriCmpPred(const T &a, const T &b)
{
    return (a.x < b.x || (a.x == b.x && a.y < b.y)) ? -1 : 1;
}

#endif

MATH_BEGIN_NAMESPACE

template<typename T>
bool LexSortPred(const T &a, const T &b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

template<typename T>
float PerpDot2D(const T &o, const T &a, const T &b)
{
	// 2D convex hull algorithm is very sensitive to numerical precision, so compute perp-dot products in double precision.
	return (float)(((double)a.x - (double)o.x) * ((double)b.y - (double)o.y) - ((double)a.y - (double)o.y) * ((double)b.x - (double)o.x));

	// Single precision for fast code:
//	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

template<typename T>
float DistSq2D(const T &a, const T &b)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	return dx*dx + dy*dy;
}

/// Implements Andrew's Monotone chain 2D convex hull algorithm.
template<typename T>
int float2_ConvexHullInPlace(T *p, int n)
{
	assert(n >= 0);
	if (n <= 1) return n > 0 ? n : 0;

#ifdef MATH_CONTAINERLIB_SUPPORT
    sort::CocktailSort(p, n, LexSortPred<T>);
#elif defined(MATH_ENABLE_STL_SUPPORT)
	std::sort(p, p + n, LexSortPred<T>);
#else
#error TODO qsort or something
#endif

	const float eps = 1e-6f;

	if (n == 2)
		return DistSq2D(p[0], p[1]) > eps ? 2 : 1;

	T *hull = new T[2*n];

	// Scan left to right for the bottom side.
	hull[0] = p[0];
	hull[1] = p[1];
	int k = 2;

	for(int i = 2; i < n; ++i)
	{
		while (k >= 2 && PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0)
			--k;

		assert(k < 2*n);
		hull[k++] = p[i];
	}

	// Come back scanning right to left for the top side.
	const int start = k;

	for(int i = n-2; i > 0; --i)
	{
		while (k > start && PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0)
			--k;

		assert(k < 2*n);
		hull[k++] = p[i];
	}

	// The first (leftmost) point in the set is always part of the convex hull, but we do not want to add that twice
	// to the output when scanning back, so handle the final point outside the above loop (above loop terminates with
	// i>0 instead of i>=0 because of this). Still, we do want to use this last point to prune out previous added points
	// on the top side, so manually compute this filtering outside the above loop.
	while (k > start && PerpDot2D(hull[k-2], hull[k-1], p[0]) <= 0)
		--k;

	// For numerical stability, clean up the generated convex hull by erasing duplicate entries.
	int h = 0;
	p[0] = hull[0];
	for(int i = 1; i < k; ++i)
	{
		if (DistSq2D(hull[i], p[h]) > eps)
		{
			assert(h+1 < n);
			p[++h] = hull[i];
		}
	}
	while(DistSq2D(p[h], p[0]) < eps && h > 0) --h;

	delete[] hull;
	return h+1;
}

template<typename T>
bool float2_ConvexHullContains(T *hull, int n, const T &point)
{
	assert(n >= 0);
	if (n <= 2) return false; // todo: test vs point or vs line?

	int prev = n-1;
	for(int i = 0; i < n; ++i)
	{
		if (PerpDot2D(hull[prev], hull[i], point) < -1e-6f)
			return false;
		prev = i;
	}
	return true;
}

MATH_END_NAMESPACE
