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

/** @file Polyhedron.h
	@author Jukka Jylänki
	@brief The Polyhedron geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"

class float4d
{
public:
	double x, y, z, w;
	float4d(){}
	float4d(double x, double y, double z, double w):x(x), y(y), z(z), w(w) {}
	float4d(const float4 &rhs):x(rhs.x), y(rhs.y), z(rhs.z), w(rhs.w) {}

	double Dot(const float4d &rhs) const
	{
		return x*rhs.x + y*rhs.y + z*rhs.z + w*rhs.w;
	}

	float4d operator -(const float4d &rhs) const
	{
		return float4d(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
	}

	float4d Cross(const float4d &rhs) const
	{
		return float4d(y * rhs.z - z * rhs.y,
									z * rhs.x - x * rhs.z,
									x * rhs.y - y * rhs.x,
									0);
	}

	double DistanceSq(const float4d &rhs) const
	{
		double dx = x - rhs.x;
		double dy = y - rhs.y;
		double dz = z - rhs.z;
		return dx*dx + dy*dy + dz*dz;
	}

	double Normalize()
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
};
