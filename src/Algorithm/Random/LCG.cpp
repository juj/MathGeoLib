/** @file LCG.cpp
	@author Jukka Jylänki

	This work is copyrighted material and may NOT be used for any kind of commercial or 
	personal advantage and may NOT be copied or redistributed without prior consent
	of the author(s). 

	@brief A linear congruential random number generator.
*/

#include "Math/MathFunc.h"
#include "Algorithm/Random/LCG.h"
#include "Time/Clock.h"

LCG::LCG()
{
	Seed(Clock::TickU32());
}

/** If you want to give different parameters for the generator, you should remember that:
	- modulus should be prime
	- modulus, increment and the multiplier should all be relative primes (increment can be 0)
	- modulus should be greater than multiplier and increment

	Most often you can leave increment = 0.

	A list of widely used values:
	- Park and Miller (Minimal standard): mul = 16807 (7^5)             mod = 2^31 - 1 (2147483647)
	- Park and Miller #2:                 mul = 48271                   mod = 2^31 - 1
	- Park and Miller #3:                 mul = 69621                   mod = 2^31 - 1
	- SIMSCRIPT:                          mul = 630360016               mod = 2^31 - 1
	- URN12:                              mul = 452807053               mod = 2^31

	Infamous examples (Don't use!):
	- Classical ANSI C                    mul = 1103515245  inc = 12345 mod = 2^31
	- RANDU                               mul = 65539                   mod = 2^31  */
void LCG::Seed(u32 seed, u32 mul, u32 inc, u32 mod)
{
	assert((seed !=0 || inc != 0) && "Initializing LCG with seed=0 && inc=0 results in an infinite series of 0s!");

	lastNumber = seed;
	multiplier = mul;
	increment = inc;
	modulus = mod;
}

/** @return An integer in the range [0, modulus-1] */
u32 LCG::IntFast()
{
	lastNumber = (lastNumber * multiplier) & 0x7FFFFFFFUL;
	return lastNumber;
}

u32 LCG::Int()
{
	/// \todo Convert to using Schrage's method for approximate factorization. (Numerical Recipes in C)

	// Currently we cast everything to 64-bit to avoid overflow, which is quite dumb.

	// Create the new random number
#ifdef WIN32
	unsigned __int64 newNum = ((unsigned __int64)lastNumber * (unsigned __int64)multiplier + (unsigned __int64)increment) % (unsigned __int64)modulus;
//	u32 m = lastNumber * multiplier;
//	u32 i = m + increment;
//	u32 f = i & 0x7FFFFFFF;
//	u32 m = (lastNumber * 214013 + 2531011) & 0x7FFFFFFF;
//	unsigned __int64 newNum = (lastNumber * multiplier + increment) & 0x7FFFFFFF;
#else
	// On console platform, we rely on using smaller sequences.
	unsigned long newNum = ((unsigned long)lastNumber * (unsigned long)multiplier + (unsigned long)increment) % (unsigned long)modulus;
#endif
	// Save the newly generated random number to use as seed for the next one.
//	lastNumber = m;//(u32)newNum;
	lastNumber = (u32)newNum;
	return lastNumber;
}

int LCG::Int(int a, int b)
{
	assert(a <= b && "Error in range!");

//	return a + (int)(Int() * Max()/(b-a));
	int num = a + (int)(Float() * (b-a+1));
//	assert(num >= a);
//	assert(num <= b);
	///\todo Some bug here - the result is not necessarily in the proper range.
	if (num < a)
		num = a;
	if (num > b)
		num = b;
	return num;
}

float LCG::Float()
{
	return (float)Int() / ((float)MaxInt() + 1.f);
}

float LCG::Float(float a, float b)
{
	assume(a <= b && "LCG::Float(a,b): Error in range: b < a!");

	return Float()*(b-a)+a;
}
