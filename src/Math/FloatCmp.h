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

/** @file FloatCmp.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "MathFunc.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#endif

MATH_BEGIN_NAMESPACE

inline bool Equal(double a, double b, double epsilon = 1e-6)
{
	return Abs(a-b) < epsilon;
}

MATH_END_NAMESPACE
