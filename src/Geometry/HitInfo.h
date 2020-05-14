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

/** @file HitInfo.h
	@author Jukka Jylänki
	@brief */
#pragma once

MATH_BEGIN_NAMESPACE

struct HitInfo
{
	enum HitResult
	{
		NoHit,
		Intersect,
		AInsideB,
		BInsideA
	};

	/// Specifies the result of the intersection test.
	HitResult result;

	/// Stores the point of intersection.
	float3 point;

	/// Specifies the surface normal of the 'this' object at the point of intersection.
	float3 normalA;

	/// Specifies the surface normal of the other object at the point of intersection.
	float3 normalB;
};

MATH_END_NAMESPACE
