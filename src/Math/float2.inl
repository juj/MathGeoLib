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
	T *hull = new T[n+1];

	std::sort(p, p + n, LexSortPred<T>);

	// Scan left to right for the bottom side.
	for (int i = 0; i < n; ++i)
	{
		while (k >= 2 && (PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0 || DistSq2D(p[i], hull[k-1]) < 1e-8f)) --k;
		hull[k++] = p[i];
	}

	// Come back scanning right to left for the top side.
	const int start = k+1;
	for (int i = n-2; i >= 0; --i) {
		while (k >= start && (PerpDot2D(hull[k-2], hull[k-1], p[i]) <= 0 || DistSq2D(p[i], hull[k-1]) < 1e-8f)) --k;
		hull[k++] = p[i];
	}
	assert(k <= n+1);

	// Due to our construction, the first and last points are the same, so
	// to avoid having duplicate data in the outputted hull, drop the
	// last coordinate.
	--k;

	memcpy(p, hull, k*sizeof(T));
	delete[] hull;
	return k;
}

MATH_END_NAMESPACE
