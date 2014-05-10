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

/** @file Polynomial.cpp
	@author Jukka Jylänki
	@brief */
#include "Polynomial.h"
#include "MathFunc.h"

MATH_BEGIN_NAMESPACE

int Polynomial::SolveQuadratic(float a, float b, float c, float &root1, float &root2)
{
	// ax^2 + bx + c == 0 => x = [ -b +/- Sqrt(b^2 - 4ac) ] / 2a.

	///@todo numerical float issues: catastrophic cancellation can occur in the subtraction.
	float radicand = b*b - 4.f * a * c;
	if (radicand < -1e-6f) // Add a small epsilon to allow the radicand to be slightly zero.
		return 0;
	float denom = 1.f / (2.f * a);
	if (radicand < 1e-6f) // Consider the radicand to be zero, and hence only one solution.
	{
		root1 = -b * denom;
		return 1;
	}
	radicand = Sqrt(radicand);
	root1 = (-b + radicand) * denom;
	root2 = (-b - radicand) * denom;
	return 2;
}

#if 0
int Polynomial::SolveCubic(float /*a*/, float /*b*/, float /*c*/, float /*d*/, float & /*root1*/, float & /*root2*/, float & /*root3*/)
{
#ifdef _MSC_VER
#pragma warning(Polynomial::SolveCubic not implemented!)
#else
#warning Polynomial::SolveCubic not implemented!
#endif
	assume(false && "Polynomial::SolveCubic not implemented!"); /// @todo Implement.
	return 0;
}

int Polynomial::SolveQuartic(float /*a*/, float /*b*/, float /*c*/, float /*d*/, float & /*root1*/, float & /*root2*/, float & /*root3*/, float & /*root4*/)
{
#ifdef _MSC_VER
#pragma warning(Polynomial::SolveQuartic not implemented!)
#else
#warning Polynomial::SolveQuartic not implemented!
#endif
	assume(false && "Polynomial::SolveQuartic not implemented!"); /// @todo Implement.
	return 0;
}
#endif

MATH_END_NAMESPACE
