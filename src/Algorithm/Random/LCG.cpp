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

/** @file LCG.cpp
	@author Jukka Jylänki
	@brief Implementation of the linear congruential random number generator. */

#include "LCG.h"
#include "../../Math/MathFunc.h"
#include "../../Time/Clock.h"
#include "../../Math/MathTypes.h"
#include "../../Math/myassert.h"

MATH_BEGIN_NAMESPACE

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
	- Park and Miller (Minimal standard): mul = 16807 (7^5)             mod = 2^31 - 1 (2147483647 == 0x7FFFFFFF)
	- Park and Miller #2:                 mul = 48271                   mod = 2^31 - 1
	- Park and Miller #3:                 mul = 69621                   mod = 2^31 - 1
	- SIMSCRIPT:                          mul = 630360016               mod = 2^31 - 1
	- URN12:                              mul = 452807053               mod = 2^31

	Infamous examples (Don't use!):
	- Classical ANSI C                    mul = 1103515245  inc = 12345 mod = 2^31
	- RANDU                               mul = 65539                   mod = 2^31  */
void LCG::Seed(u32 seed, u32 mul, u32 inc, u32 mod)
{
	assume((seed != 0 || inc != 0) && "Initializing LCG with seed=0 && inc=0 results in an infinite series of 0s!");

	lastNumber = seed;
	multiplier = mul;
	increment = inc;
	modulus = mod;
	assert(modulus != 0);
}

u32 LCG::IntFast()
{
// The configurable modulus and increment are not used by this function.
	u32 mul = lastNumber * multiplier;
	lastNumber = mul - (mul <= lastNumber?1:0); // Whenever we overflow, flip by one to avoid even multiplier always producing even results, since modulus is even.
	return lastNumber;
}

u32 LCG::Int()
{
	assert(modulus != 0);
	/// \todo Convert to using Schrage's method for approximate factorization. (Numerical Recipes in C)

	// Currently we cast everything to 64-bit to avoid overflow, which is quite dumb.

	// Create the new random number
//#ifdef WIN32
	u64 newNum = ((u64)lastNumber * (u64)multiplier + (u64)increment) % (u64)modulus;
//	u32 m = lastNumber * multiplier;
//	u32 i = m + increment;
//	u32 f = i & 0x7FFFFFFF;
//	u32 m = (lastNumber * 214013 + 2531011) & 0x7FFFFFFF;
//	unsigned __int64 newNum = (lastNumber * multiplier + increment) & 0x7FFFFFFF;
//#else
	// On console platform, we rely on using smaller sequences.
//	unsigned long newNum = ((unsigned long)lastNumber * (unsigned long)multiplier + (unsigned long)increment) % (unsigned long)modulus;
//#endif
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
	u32 i = ((u32)Int() & 0x007FFFFF /* random mantissa */) | 0x3F800000 /* fixed exponent */;
	float f = ReinterpretAsFloat(i); // f is now in range [1, 2[
	f -= 1.f; // Map to range [0, 1[
	return f;
}

float LCG::Float01Incl()
{
	for(int i = 0; i < 100; ++i)
	{
		u32 val = (u32)Int() & 0x00FFFFFF;
		if (val > 0x800000)
			continue;
		else if (val == 0x800000)
			return 1.0f;
		else
		{
			val |= 0x3F800000;
			return ReinterpretAsFloat(val) - 1.f;
		}
	}
	return Float();
}

float LCG::FloatNeg1_1()
{
	u32 i = (u32)Int();
	u32 one = ((i & 0x00800000) << 8) /* random sign bit */ | 0x3F800000 /* fixed exponent */;
	i = one | (i & 0x007FFFFF) /* random mantissa */;
	float f = ReinterpretAsFloat(i); // f is now in range ]-2, -1[ union [1, 2].
	float fone = ReinterpretAsFloat(one); // +/- 1, of same sign as f.
	return f - fone;
}

float LCG::Float(float a, float b)
{
	assume(a <= b && "LCG::Float(a,b): Error in range: b < a!");

	return Float()*(b-a)+a;
}

MATH_END_NAMESPACE
