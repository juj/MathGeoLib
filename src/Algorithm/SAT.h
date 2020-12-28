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

/** @file SAT.h
	@author Jukka Jylanki
	@brief Implementation of the Separating Axis Theorem -based convex object intersection test. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float2.h"
#include "../Math/float3.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Container/Array.h"
#endif

MATH_BEGIN_NAMESPACE

template<typename A, typename B>
bool SATIntersect(const A &a, const B &b)
{
	vec normals[16];
	int n = a.UniqueFaceNormals(normals);
	assert(n <= 16); ///\todo Make this API safe.
	for(int i = 0; i < n; ++i)
	{
		float amin, amax, bmin, bmax;
		a.ProjectToAxis(normals[i], amin, amax);
		b.ProjectToAxis(normals[i], bmin, bmax);
		if (amax < bmin || bmax < amin)
			return false;
	}
	n = b.UniqueFaceNormals(normals);
	assert(n <= 16); ///\todo Make this API safe.
	for(int i = 0; i < n; ++i)
	{
		float amin, amax, bmin, bmax;
		a.ProjectToAxis(normals[i], amin, amax);
		b.ProjectToAxis(normals[i], bmin, bmax);
		if (amax < bmin || bmax < amin)
			return false;
	}
	vec normals2[16];
	n = a.UniqueEdgeDirections(normals);
	int m = b.UniqueEdgeDirections(normals2);
	for(int i = 0; i < n; ++i)
		for(int j = 0; j < m; ++j)
		{
			float amin, amax, bmin, bmax;
			vec normal = Cross(normals[i], normals2[j]);
			a.ProjectToAxis(normal, amin, amax);
			b.ProjectToAxis(normal, bmin, bmax);
			if (amax < bmin || bmax < amin)
				return false;
		}
	return true;
}

bool SATCollide2D(const float2 *a, int numA, const float2 *b, int numB);

// Returns the penetration distance between the two objects. If > 0, the objects are not colliding.
// If < 0, then the objects penetrate that deep along outCollisionNormal vector.
float SATCollide2D_CollisionPoint(const float2 *a, int numA, const float2 *b, int numB,
                                 float2 &outCollisionPoint, float2 &outCollisionNormal);

#ifdef MATH_CONTAINERLIB_SUPPORT
inline bool SATCollide2D(const Array<float2> &a, const Array<float2> &b)
{
	return SATCollide2D(a.ptr(), (int)a.size(), b.ptr(), (int)b.size());
}
#endif

MATH_END_NAMESPACE
