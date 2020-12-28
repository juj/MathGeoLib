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
#include "../Math/float3x3.h"
#include "../Math/float4x4.h"

MATH_BEGIN_NAMESPACE

class AABB2D;

template<typename Matrix>
void AABB2DTransformAsAABB2D(AABB2D &aabb, Matrix &m);

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

	/// Returns a corner point of this AABB.
	/** This function generates one of the eight corner points of this AABB.
		@param cornerIndex The index of the corner point to generate, in the range [0, 3].
			The points are returned in the order 0: --, 1: -+, 2: +-, 3: ++. (corresponding the XYZ axis directions).
		@see PointInside(), Edge(), PointOnEdge(), FaceCenterPoint(), FacePoint(), GetCornerPoints(). */
	vec2d CornerPoint(int cornerIndex) const
	{
		assume(0 <= cornerIndex && cornerIndex <= 3);
		switch(cornerIndex)
		{
			default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
			case 0: return minPoint;
			case 1: return POINT_VEC2D(minPoint.x, maxPoint.y);
			case 2: return POINT_VEC2D(maxPoint.x, minPoint.y);
			case 3: return maxPoint;
		}
	}

	/// Applies a transformation to this AABB2D.
	/** This function transforms this AABB2D with the given transformation, and then recomputes this AABB2D
		to enclose the resulting oriented bounding box. This transformation is not exact and in general, calling
		this function results in the loosening of the AABB bounds.
		@param transform The transformation to apply to this AABB2D. This function assumes that this
			transformation does not contain shear, nonuniform scaling or perspective properties, i.e. that the fourth
			row of the float4x4 is [0 0 0 1].
		@see Translate(), Scale(), Transform(), classes float3x3, float3x4, float4x4, Quat. */
	void TransformAsAABB(const float3x3 &transform)
	{
		AABB2DTransformAsAABB2D(*this, transform);
	}

	void TransformAsAABB(const float3x4 &transform)
	{
		AABB2DTransformAsAABB2D(*this, transform);
	}

	void TransformAsAABB(const float4x4 &transform)
	{
		AABB2DTransformAsAABB2D(*this, transform);
	}
	/*
	void TransformAsAABB(const Quat &transform)
	{
		vec newCenter = transform.Transform(CenterPoint());
		vec newDir = Abs((transform.Transform(Size()) * 0.5f));
		minPoint = newCenter - newDir;
		maxPoint = newCenter + newDir;
	}
	*/
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

/// See Christer Ericson's Real-time Collision Detection, p. 87, or
/// James Arvo's "Transforming Axis-aligned Bounding Boxes" in Graphics Gems 1, pp. 548-550.
/// http://www.graphicsgems.org/
template<typename Matrix>
void AABB2DTransformAsAABB2D(AABB2D &aabb, Matrix &m)
{
	float ax = m[0][0] * aabb.minPoint.x;
	float bx = m[0][0] * aabb.maxPoint.x;
	float ay = m[0][1] * aabb.minPoint.y;
	float by = m[0][1] * aabb.maxPoint.y;

	float ax2 = m[1][0] * aabb.minPoint.x;
	float bx2 = m[1][0] * aabb.maxPoint.x;
	float ay2 = m[1][1] * aabb.minPoint.y;
	float by2 = m[1][1] * aabb.maxPoint.y;

	aabb.minPoint.x = Min(ax, bx) + Min(ay, by) + m[0][3];
	aabb.maxPoint.x = Max(ax, bx) + Max(ay, by) + m[0][3];
	aabb.minPoint.y = Min(ax2, bx2) + Min(ay2, by2) + m[1][3];
	aabb.maxPoint.y = Max(ax2, bx2) + Max(ay2, by2) + m[1][3];
}

MATH_END_NAMESPACE
