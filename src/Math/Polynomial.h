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

/** @file Polynomial.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "MathNamespace.h"

MATH_BEGIN_NAMESPACE

class Polynomial
{
public:

	/// Solves a quadratic equation ax^2 + bx + c = 0 for x. Returns the number of roots found.
	static int SolveQuadratic(float a, float b, float c, float &root1, float &root2);

#if 0
	/// Solves a cubic equation x^3 + ax^2 + bx +  = 0 for x. Returns the number of roots found.
	static int SolveCubic(float a, float b, float c, float d, float &root1, float &root2, float &root3);

	/// Solves a quartic equation ax^4 + bx^3 + cx^2 + dx + e = 0 for x. Returns the number of roots found.
	static int SolveQuartic(float a, float b, float c, float d, float &root1, float &root2, float &root3, float &root4);
#endif

	// @todo add Newton's method functions.
};

MATH_END_NAMESPACE
