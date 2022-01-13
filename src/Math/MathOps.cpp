/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file MathOps.cpp
	@author Jukka Jyl�nki
	@brief */
#include "MathFunc.h"
#include "myassert.h"

MATH_BEGIN_NAMESPACE

/** Compares the two values for equality, allowing the given amount of absolute error. */
bool EqualAbs(float a, float b, float epsilon)
{
	return Abs(a-b) < epsilon;
}

float RelativeError(float a, float b)
{
	if (a == b) return 0.f; // Handles the special case where approximation and real are both zero.
	return Abs((a-b)/Max(Abs(a), Abs(b)));
}

bool EqualRel(float a, float b, float maxRelError)
{
	if (a == b) return true; // Handles the special case where a and b are both zero.
	return Abs(a-b) <= maxRelError * Max(Abs(a), Abs(b));
}

inline int ReinterpretFloatAsInt(float a)
{
	union reinterpret_float_as_int
	{
		float f;
		int i;
	};
	reinterpret_float_as_int fi;
	fi.f = a;
	return fi.i;
}

bool EqualUlps(float a, float b, int maxUlps)
{
	mgl_assert(sizeof(float) == sizeof(int));
	mgl_assert(maxUlps >= 0);
	mgl_assert(maxUlps < 4 * 1024 * 1024);

	int intA = ReinterpretFloatAsInt(a);
	if (intA < 0) intA = 0x80000000 - intA;
	int intB = ReinterpretFloatAsInt(b);
	if (intB < 0) intB = 0x80000000 - intB;
	if (Abs(intA - intB) <= maxUlps)
		return true;
	return false;
}

MATH_END_NAMESPACE
