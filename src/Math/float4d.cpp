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

/** @file float4d.cpp
	@author Jukka Jylänki
	@brief A double-precision 4-wide vector. */
#include "float4d.h"

MATH_BEGIN_NAMESPACE

const float4d float4d::zero = float4d(0,0,0,0);
const float4d float4d::one = float4d(1,1,1,1);
const float4d float4d::unitX = float4d(1,0,0,0);
const float4d float4d::unitY = float4d(0,1,0,0);
const float4d float4d::unitZ = float4d(0,0,1,0);
const float4d float4d::unitW = float4d(0,0,0,1);
const float4d float4d::nan = float4d(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);
const float4d float4d::inf = float4d(FLOAT_INF, FLOAT_INF, FLOAT_INF, FLOAT_INF);

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
StringT float4d::ToString() const
{
	char str[256];
	sprintf(str, "(%.3f, %.3f, %.3f, %.3f)", x, y, z, w);
	return str;
}
#endif

MATH_END_NAMESPACE
