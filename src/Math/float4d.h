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

/** @file float4d.h
	@author Jukka Jylänki
	@brief A double-precision 4-wide vector. */
#pragma once

#include "../MathBuildConfig.h"

#include "../MathGeoLibFwd.h"
#include "float4_sse.h"
#include "MathFunc.h"

MATH_BEGIN_NAMESPACE

class ALIGN16 float4d
{
public:
#if defined(MATH_SSE2)
	NAMELESS_UNION_BEGIN // Allow nonstandard nameless struct in union extension on MSC.

	union
	{
		struct
		{
#endif
			double x, y, z, w;
#if defined(MATH_SSE2)
		};
		simd2d v[2];
	};
	NAMELESS_UNION_END
#endif

	float4d(){}
	float4d(double x, double y, double z, double w):x(x), y(y), z(z), w(w) {}
	float4d(const float4 &rhs):x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {}
	float4d(const float3 &rhs, float w):x(rhs.x), y(rhs.y), z(rhs.z), w(w) {}

	double Dot(const float4d &rhs) const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		return s2d_y(dot4_pd(v, rhs.v));
#else
		return x*rhs.x + y*rhs.y + z*rhs.z + w*rhs.w;
#endif
	}

	float4d operator +(const float4d &rhs) const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		float4d r;
		r.v[0] = add_pd(v[0], rhs.v[0]);
		r.v[1] = add_pd(v[1], rhs.v[1]);
		return r;
#else
		return float4d(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
#endif
	}

	float4d operator -(const float4d &rhs) const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		float4d r;
		r.v[0] = sub_pd(v[0], rhs.v[0]);
		r.v[1] = sub_pd(v[1], rhs.v[1]);
		return r;
#else
		return float4d(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
#endif
	}

	float4d Cross(const float4d &rhs) const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		float4d r;
		cross_pd(r.v, v, rhs.v);
		return r;
#else
		return float4d(y * rhs.z - z * rhs.y,
									z * rhs.x - x * rhs.z,
									x * rhs.y - y * rhs.x,
									0);
#endif
	}

	double LengthSq() const
	{
		return x*x + y*y + z*z + w*w;
	}

	double Distance4Sq(const float4d &rhs) const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		simd2d sub = sub_pd(v[0], rhs.v[0]);
		simd2d xy = dot2_pd(sub, sub);
		sub = sub_pd(v[1], rhs.v[1]);
		simd2d zw = dot2_pd(sub, sub);
		xy = add_pd(xy, zw);
		return s2d_x(xy);
#else
		double dx = x - rhs.x;
		double dy = y - rhs.y;
		double dz = z - rhs.z;
		double dw = w - rhs.w;
		return dx*dx + dy*dy + dz*dz + dw*dw;
#endif
	}
	double DistanceSq(const float4d &rhs) const { return Distance4Sq(rhs); }

	double Distance3Sq(const float4d &rhs) const
	{
		double dx = x - rhs.x;
		double dy = y - rhs.y;
		double dz = z - rhs.z;
		return dx*dx + dy*dy + dz*dz;
	}

	double Normalize4()
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		simd2d len = dot4_pd(v, v);
		len = _mm_sqrt_pd(len);
		simd2d recipLen = div_pd(set1_pd(1.0), len);
		v[0] = mul_pd(v[0], recipLen);
		v[1] = mul_pd(v[1], recipLen);
		return s2d_x(len);
#else
		double len = sqrt(x*x + y*y + z*z + w*w);
		double recipLen = 1.0 / len;
		x *= recipLen;
		y *= recipLen;
		z *= recipLen;
		w *= recipLen;
		return len;
#endif
	}
	double Normalize() { return Normalize4(); }

	double Normalize3()
	{
		double len = sqrt(x*x + y*y + z*z);
		double recipLen = 1.0 / len;
		x *= recipLen;
		y *= recipLen;
		z *= recipLen;
		return len;
	}

	bool IsFinite() const
	{
		return MATH_NS::IsFinite(x) && MATH_NS::IsFinite(y) && MATH_NS::IsFinite(z) && MATH_NS::IsFinite(w);
	}

	bool IsZero(double epsilonSq = 1e-10) const
	{
		return Dot(*this) <= epsilonSq;
	}

	float4d operator -() const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		float4d r;
		r.v[0] = neg_pd(v[0]);
		r.v[1] = neg_pd(v[1]);
		return r;
#else
		return float4d(-x, -y, -z, -w);
#endif
	}

	float4 ToFloat4() const
	{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
		simd4f lo = _mm_cvtpd_ps(v[0]);
		simd4f hi = _mm_cvtpd_ps(v[1]);
		return _mm_movelh_ps(lo, hi);
#else
		return float4((float)x, (float)y, float(z), float(w));
#endif
	}

	vec ToPointVec() const
	{
		assert1(EqualAbs((float)w, 1.0), w);
		return POINT_VEC((float)x, (float)y, (float)z);
	}

	vec ToDirVec() const
	{
		assert1(EqualAbs((float)w, 0.0), w);
		return DIR_VEC((float)x, (float)y, (float)z);
	}

	bool Equals(const float4d &other, float epsilon = 1e-3f) const
	{
		return MATH_NS::Abs(x - other.x) < epsilon &&
					 MATH_NS::Abs(y - other.y) < epsilon &&
					 MATH_NS::Abs(z - other.z) < epsilon &&
					 MATH_NS::Abs(w - other.w) < epsilon;
	}

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x, y, z, w)".
	StringT ToString() const;
#endif

	static const float4d zero;
	static const float4d one;
	static const float4d unitX;
	static const float4d unitY;
	static const float4d unitZ;
	static const float4d unitW;
	static const float4d nan;
	static const float4d inf;
};

inline float4d operator *(double scalar, const float4d &vec)
{
#if defined(MATH_SSE2) && defined(MATH_AUTOMATIC_SSE)
	float4d r;
	simd2d s = set1_pd(scalar);
	r.v[0] = mul_pd(vec.v[0], s);
	r.v[1] = mul_pd(vec.v[1], s);
	return r;
#else
	return float4d(vec.x*scalar, vec.y*scalar, vec.z*scalar, vec.w*scalar);
#endif
}
inline float4d operator *(const float4d &vec, double scalar)
{
	return scalar * vec;
}

struct double4_storage
{
	double x, y, z, w;
	double4_storage(){}
	double4_storage(const float4d &rhs)
	{
		// Copy with scalar. TODO: Revisit if this is avoidable.
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		// Would like to do the following to get SSE benefit, but
		// Visual Studio generates unaligned temporaries to this struct
		// in debug builds, so can't do that.
		//*reinterpret_cast<float4*>(this) = rhs;
	}
	operator float4d() const { return *reinterpret_cast<const float4d*>(this); }
};

MATH_END_NAMESPACE
