/** @file LCG.h
	@author Jukka Jylänki

	This work is copyrighted material and may NOT be used for any kind of commercial or 
	personal advantage and may NOT be copied or redistributed without prior consent
	of the author(s). 

	@brief A linear congruential random number generator.
*/
#pragma once

#include "Types.h"

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

	/// Returns an integer in the range [0, MaxInt()]
	u32 Int();

	/// Returns the biggest number the generator can yield. (Which is always modulus-1)
	u32 MaxInt() const { return modulus - 1; }

	u32 IntFast();

	/// Returns an integer in the range [a, b]
    /** @param a Lower bound, inclusive.
	    @param b Upper bound, inclusive.
	    @return An integer in the range [a, b] */
	int Int(int a, int b);

	/// Returns a float in the range [0, 1[.
	float Float();

	/// Returns a float in the range [a, b[.
    /** @param a Lower bound, inclusive.
	    @param b Upper bound, exclusive.
	    @return A float in the range [a, b[ */
	float Float(float a, float b);

private:
	u32 multiplier;
	u32 increment;
	u32 modulus;

	u32 lastNumber;
};

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(LCG)
Q_DECLARE_METATYPE(LCG*)
#endif
