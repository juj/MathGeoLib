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

/** @file LCG.h
	@author Jukka Jylänki
	@brief A linear congruential random number generator. */
#pragma once

#include "../../MathBuildConfig.h"
#include "../../Math/MathNamespace.h"

#include "../../Math/MathTypes.h"

/** @brief A linear congruential random number generator.

	Uses D.H. Lehmer's Linear Congruential Method (1949) for generating random numbers.
	Supports both Multiplicative Congruential Method (increment==0) and
	Mixed Congruential Method (increment!=0)
	It is perhaps the simplest and fastest method to generate pseudo-random numbers on
	a computer. Per default uses the values for Minimal Standard LCG.
	http://en.wikipedia.org/wiki/Linear_congruential_generator
	http://www.math.rutgers.edu/~greenfie/currentcourses/sem090/pdfstuff/jp.pdf

	Pros:
	<ul>
	    <li> Easy to implement.
	    <li> Fast.
	</ul>

	Cons:
	<ul>
	    <li> NOT safe for cryptography because of the easily calculatable sequential
	        correlation between successive calls. A case study:
	        http://www.cigital.com/papers/download/developer_gambling.php

	    <li> Tends to have less random low-order bits (compared to the high-order bits)
	         Thus, NEVER do something like this:

	           u32 numBetween1And10 = 1 + LCGRand.Int() % 10;

	         Instead, take into account EVERY bit of the generated number, like this:

	           u32 numBetween1And10 = 1 + (int)(10.0 * (double)LCGRand.Int()
	                                                      /(LCGRand.Max()+1.0));
	         or simply
	
	           u32 numBetween1And10 = LCGRand.Float(1.f, 10.f);
	</ul> */

MATH_BEGIN_NAMESPACE

class LCG
{
public:
	/// Initializes the generator from the current system clock.
	LCG();
	/// Initializes the generator using a custom seed.
	LCG(u32 seed, u32 multiplier = 69621,
		u32 increment = 0, u32 modulus = 0x7FFFFFFF /* 2^31 - 1 */)
	{
		Seed(seed, multiplier, increment, modulus);
	}

	/// Reinitializes the generator to the new settings.
	void Seed(u32 seed, u32 multiplier = 69621, u32 increment = 0, u32 modulus = 0x7FFFFFFF);

	/// Returns a random integer picked uniformly in the range [0, MaxInt()]
	u32 Int();

	/// Returns the biggest number the generator can yield. (Which is always modulus-1)
	u32 MaxInt() const { return modulus - 1; }

	/// Returns a random integer picked uniformly in the range [0, 2^32-1].
	/// @note The configurable modulus and increment are not used by this function, but are always increment == 0, modulus=2^32.
	u32 IntFast();

	/// Returns a random integer picked uniformly in the range [a, b]
	/** @param a Lower bound, inclusive.
	    @param b Upper bound, inclusive.
	    @return A random integer picked uniformly in the range [a, b] */
	int Int(int a, int b);

	/// Returns a random float picked uniformly in the range [0, 1[.
	float Float();

	/// Returns a random float picked uniformly in the range [0, 1].
	/// @note This is much slower than Float()! Prefer that function instead if possible.
	float Float01Incl();

	/// Returns a random float picked uniformly in the range ]-1, 1[.
	/// @note This function has one more bit of randomness compared to Float(), but has a theoretical bias
	/// towards 0.0, since floating point has two representations for 0 (+0 and -0).
	float FloatNeg1_1();

	/// Returns a random float picked uniformly in the range [a, b[.
	/** @param a Lower bound, inclusive.
	    @param b Upper bound, exclusive.
	    @return A random float picked uniformly in the range [a, b[
	    @Note This function is slower than LCG::FloatIncl(). If you do not care about the open/closed interval, prefer calling FloatIncl() instead.
	    @see Float(), FloatIncl(). */
	float Float(float a, float b);

	/// Returns a random float picked uniformly in the range [a, b].
	/** @param a Lower bound, inclusive.
	    @param b Upper bound, inclusive.
	    @return A random float picked uniformly in the range [a, b] */
	float FloatIncl(float a, float b);

	u32 multiplier;
	u32 increment;
	u32 modulus;

	u32 lastNumber;
};

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(LCG)
Q_DECLARE_METATYPE(LCG*)
#endif

MATH_END_NAMESPACE
