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

MATH_BEGIN_NAMESPACE

template<typename T>
bool LexSortPred(const T &a, const T &b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

template<typename T>
float PerpDot2D(const T &o, const T &a, const T &b)
{
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
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
	int k = 0;
	// TODO: The hull array should really need only n+1 elements here instead of 2n, but for
	// numerical stability reasons, it seems that sometimes more than n+1 elements get written.
	// Figure if there is a way to avoid this.
	T *hull = new T[2*n];

	std::sort(p, p + n, LexSortPred<T>);

	// Scan left to right for the bottom side.
	for(int i = 0; i < n; ++i)
	{
		while (k >= 2 && PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0)
			--k;
		hull[k++] = p[i];
	}

	// Come back scanning right to left for the top side.
	const int start = k+1;
	for(int i = n-2; i >= 0; --i)
	{
		while (k >= start && PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0)
			--k;
		hull[k++] = p[i];
	}
	assert(k <= 2*n);

	// Due to our construction, the first and last points are the same, so
	// to avoid having duplicate data in the outputted hull, drop the
	// last coordinate.
	--k;
	
	const float eps = 1e-6f;

	// For numerical stability, clean up the generated convex hull by erasing duplicate entries.
	int h = 0;
	p[0] = hull[0];
	for(int i = 1; i < k; ++i)
	{
		if (DistSq2D(hull[i], p[h]) > eps)
			p[++h] = hull[i];
	}
	while(DistSq2D(p[h], p[0]) < eps && h > 0) --h;

	delete[] hull;
	return h+1;
}

MATH_END_NAMESPACE
