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

/** @file AABB2D.h
	@author Jukka Jylänki
	@brief 2D Axis-Aligned Bounding Box structure. */
#pragma once

#include <stdio.h>

#include "../Math/vec2d.h"
#include "../Math/float3.h"
#include "../Math/MathConstants.h"

MATH_BEGIN_NAMESPACE

class AABB2D
{
public:

	AABB2D() { }

	AABB2D(const vec2d &minPt, const vec2d &maxPt)
	:minPoint(minPt),
	maxPoint(maxPt)
	{
	}

	vec2d minPoint;
	vec2d maxPoint;

	float Width() const { return maxPoint.x - minPoint.x; }
	float Height() const { return maxPoint.y - minPoint.y; }

	float DistanceSq(const vec2d &pt) const
	{
		vec2d cp = pt.Clamp(minPoint, maxPoint);
		return cp.DistanceSq(pt);
	}

	void SetNegativeInfinity()
	{
		minPoint.SetFromScalar(FLOAT_INF);
		maxPoint.SetFromScalar(-FLOAT_INF);
	}

	void Enclose(const vec2d &point)
	{
		minPoint = Min(minPoint, point);
		maxPoint = Max(maxPoint, point);
	}

	bool Intersects(const AABB2D &rhs) const
	{
		return maxPoint.x >= rhs.minPoint.x &&
		       maxPoint.y >= rhs.minPoint.y &&
		       rhs.maxPoint.x >= minPoint.x &&
		       rhs.maxPoint.y >= minPoint.y;
	}

	bool Contains(const AABB2D &rhs) const
	{
		return rhs.minPoint.x >= minPoint.x && rhs.minPoint.y >= minPoint.y
			&& rhs.maxPoint.x <= maxPoint.x && rhs.maxPoint.y <= maxPoint.y;
	}

	bool Contains(const vec2d &pt) const
	{
		return pt.x >= minPoint.x && pt.y >= minPoint.y
			&& pt.x <= maxPoint.x && pt.y <= maxPoint.y;
	}

	bool Contains(int x, int y) const
	{
		return x >= minPoint.x && y >= minPoint.y
			&& x <= maxPoint.x && y <= maxPoint.y;
	}

	bool IsDegenerate() const
	{
		return minPoint.x >= maxPoint.x || minPoint.y >= maxPoint.y;
	}

	bool HasNegativeVolume() const
	{
		return maxPoint.x < minPoint.x || maxPoint.y < minPoint.y;
	}

	bool IsFinite() const
	{
		return minPoint.IsFinite() && maxPoint.IsFinite() && minPoint.MinElement() > -1e5f && maxPoint.MaxElement() < 1e5f;
	}

	vec2d PosInside(const vec2d &normalizedPos) const
	{
		return minPoint + normalizedPos.Mul(maxPoint - minPoint);
	}

	vec2d ToNormalizedLocalSpace(const vec2d &pt) const
	{
		return (pt - minPoint).Div(maxPoint - minPoint);
	}

	AABB2D operator +(const vec2d &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint + pt;
		a.maxPoint = maxPoint + pt;
		return a;
	}

	AABB2D operator -(const vec2d &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint - pt;
		a.maxPoint = maxPoint - pt;
		return a;
	}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	StringT ToString() const
	{
		char str[256];
		sprintf(str, "AABB2D(Min:(%.2f, %.2f) Max:(%.2f, %.2f))", minPoint.x, minPoint.y, maxPoint.x, maxPoint.y);
		return str;
	}
#endif
};

inline bool Contains(const AABB2D &aabb, const float3 &pt)
{
	return aabb.minPoint.x <= pt.x &&
	       aabb.minPoint.y <= pt.y &&
	       pt.x <= aabb.maxPoint.x &&
	       pt.y <= aabb.maxPoint.y;
}

MATH_END_NAMESPACE
