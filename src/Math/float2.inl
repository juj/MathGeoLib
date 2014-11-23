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
class SortByPolarAngle
{
public:
	float2 perspective;

	bool operator()(const T &a, const T &b) const
	{
		const float epsilon = 1e-4f;
		float2 A = float2(a) - float2(perspective);
		float2 B = float2(b) - float2(perspective);
		float da = B.x*A.y - A.x*B.y;
		// If the aimed angle of pred->A is smaller than aimed angle of pred->B, then
		// A should come before; or if the three points pred, A and B lie on the same line,
		// then the closer one of A and B to pred should come before.
		return da < -epsilon || (da < epsilon && A.LengthSq() < B.LengthSq());
	}
};

/** This function implements the Graham's Scan algorithm for finding the convex hull of
	a 2D point set. The running time is O(nlogn). For details, see
	"Introduction to Algorithms, 2nd ed.", by Cormen, Leiserson, Rivest, p.824, or
	a lecture by Shai Simonson: http://www.aduni.org/courses/algorithms/index.php?view=cw , lecture 02-13-01. */
template<typename T>
int float2_ConvexHullInPlace(T *p, int n)
{
	if (n <= 2)
		return n;

	if (n >= 50)
	{
		/* Perform Akl–Toussaint heuristic. The limit n=50 is arbitrary and based on quick profiling:
		 Without heuristic:
		   n=10: 1143 ticks
		   n=50: 8657 ticks
		   n=100: 19533 ticks
		 With heuristic:
		   n=10: 1322 ticks
		   n=50: 6759 ticks
		   n=100: 14448 ticks
		*/
		int minX = 0, minY = 0, maxX = 0, maxY = 0;
		for(int i = 1; i < n; ++i)
		{
			if (p[i].x < p[minX].x) minX = i;
			else if (p[i].x > p[maxX].x) maxX = i;
			if (p[i].y < p[minY].y) minY = i;
			else if (p[i].y > p[maxY].y) maxY = i;
		}
		// Direction vectors which point inside the convex hull.
		float2 e0 = (p[maxX] - p[minY]).Rotated90CCW();
		float2 e1 = (p[maxY] - p[maxX]).Rotated90CCW();
		float2 e2 = (p[minX] - p[maxY]).Rotated90CCW();
		float2 e3 = (p[minY] - p[minX]).Rotated90CCW();

		// Add a small epsilon so that the four extreme points on the convex hull will not get pruned
		// due to floating point imprecision.
		const float eps = 1e-6f;
		float e0_d = e0.Dot(p[minY]) + eps;
		float e1_d = e1.Dot(p[maxX]) + eps;
		float e2_d = e2.Dot(p[maxY]) + eps;
		float e3_d = e3.Dot(p[minX]) + eps;

		for(int i = 0; i < n; ++i)
			if (e0.Dot(p[i]) > e0_d && e1.Dot(p[i]) > e1_d && e2.Dot(p[i]) > e2_d && e3.Dot(p[i]) > e3_d)
				Swap(p[i--], p[--n]);
	}

	// Find the lowest point of the set.
	SortByPolarAngle<T> pred;
	pred.perspective = float2(p[0]);
	int smallestY = 0;
	for(int i = 1; i < n; ++i)
		if (p[i].y < pred.perspective.y || (p[i].y == pred.perspective.y && p[i].x < pred.perspective.x))
		{
			pred.perspective = p[i];
			smallestY = i;
		}
	Swap(p[0], p[smallestY]);

	// For robustness, remove duplicates of the perspective pivot points.
	// This is because duplicates on that element will cause the sorting to be nontransitive and break
	// the whole sort.
	int d = 0;
	for(int i = 1; i < n; ++i)
		if (!p[i].Equals(p[0]))
			p[++d] = p[i];
	n = d+1;

	std::sort(&p[1], &p[n], pred);

	// For robustness, remove duplicate input values.
	d = 0;
	for(int i = 1; i < n; ++i)
		if (!p[i].Equals(p[d]))
			p[++d] = p[i];
	n = d+1;

	int h = 1; // Points to the index of the last point added to the hull so far. The first two points are in the hull to start.

	float2 a = p[h] - p[h-1];
	const float epsilon = 1e-4f;
	for(int i = 2; i < n; ++i)
	{
		// The last two added points determine a line, check which side of that line the next point to be added lies in.
		float2 d = p[i] - p[h-1];
		float dir = d.x*a.y - d.y*a.x;
		// Remove previous points from the convex hull until we have a left turn. Also for numerical stability,
		// in the case of three collinear points, remove the middle point.
		while(dir > epsilon || (dir > -epsilon && d.Dot(d) >= a.Dot(a)))
		{
			--h;
			if (h >= 1)
			{
				a = p[h] - p[h-1];
				d = p[i] - p[h-1];
				dir = d.x*a.y - d.y*a.x;
			}
			else
				break;
		}
		p[++h] = p[i];
		a = p[i] - p[h-1];
	}

	// Return the number of points on the new hull.
	return h+1;
}

MATH_END_NAMESPACE
