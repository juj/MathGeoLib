#pragma once

#include "Math/float2.h"
#include "Math/float3.h"
#include "Math/MathConstants.h"

struct AABB2D
{
	AABB2D() { }
	AABB2D(const float2 &minPt, const float2 &maxPt)
	:minPoint(minPt),
	maxPoint(maxPt)
	{
	}

	float2 minPoint;
	float2 maxPoint;

	float Width() const { return maxPoint.x - minPoint.x; }
	float Height() const { return maxPoint.y - minPoint.y; }

	float DistanceSq(const float2 &pt) const
	{
		float2 cp = pt.Clamp(minPoint, maxPoint);
		return cp.DistanceSq(pt);
	}

	void SetNegativeInfinity()
	{
		minPoint.SetFromScalar(FLOAT_INF);
		maxPoint.SetFromScalar(-FLOAT_INF);
	}

	void Enclose(const float2 &point)
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

	bool IsDegenerate() const
	{
		return minPoint.x >= maxPoint.x || minPoint.y >= maxPoint.y;
	}

	bool IsFinite() const
	{
		return minPoint.IsFinite() && maxPoint.IsFinite() && minPoint.MinElement() > -1e5f && maxPoint.MaxElement() < 1e5f;
	}

	AABB2D operator +(const float2 &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint + pt;
		a.maxPoint = maxPoint + pt;
		return a;
	}

	AABB2D operator -(const float2 &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint - pt;
		a.maxPoint = maxPoint - pt;
		return a;
	}

#ifdef MATH_ENABLE_STL_SUPPORT
	std::string ToString() const
	{
		char str[256];
		sprintf(str, "AABB2D(Min:(%.2f, %.2f) Max:(%.2f, %.2f))", minPoint.x, minPoint.y, maxPoint.x, maxPoint.y);
		return str;
	}
#endif
};

inline AABB2D GetAABB2D(const float3 &pt) { return AABB2D(pt.xy(), pt.xy()); }

inline bool Contains(const AABB2D &aabb, const float3 &pt)
{
	return aabb.minPoint.x <= pt.x &&
	       aabb.minPoint.y <= pt.y &&
	       pt.x <= aabb.maxPoint.x &&
	       pt.y <= aabb.maxPoint.y;
}
