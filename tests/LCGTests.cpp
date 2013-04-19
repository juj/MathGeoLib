#include "myassert.h"
#include "MathGeoLib.h"
#include "../tests/TestRunner.h"

RANDOMIZED_TEST(LCG_IntFast)
{
	LCG rng;
	u32 prev = rng.IntFast();
	for(int i = 0; i < 1000; ++i)
	{
		u32 next = rng.IntFast();
		assert(next != prev);
		prev = next;
	}
}

RANDOMIZED_TEST(LCG_Int)
{
	LCG rng;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		int prev = rng.Int();
		int next = rng.Int();
		assert(prev != 0 || next != 0);
		if (prev != next)
			allEqual = false;
	}
	assert(!allEqual);
}

RANDOMIZED_TEST(LCG_Int_A_B)
{
	LCG rng;
	for(int i = 0; i < 1000; ++i)
	{
		int a = rng.Int();
		int b = rng.Int();
		if (b < a)
			Swap(a, b);
		int val = rng.Int(a, b);
		assert(a <= val);
		assert(val <= b);
	}
}

RANDOMIZED_TEST(LCG_Float)
{
	LCG rng;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = rng.Float();
		float f2 = rng.Float();
		assert(f < 1.f);
		assert(f >= 0.f);
		assert(f != 0.f || f2 != 0.f);
		if (f != f2)
			allEqual = false;
	}
	assert(!allEqual);
}

RANDOMIZED_TEST(LCG_Float01Incl)
{
	LCG rng;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = rng.Float01Incl();
		float f2 = rng.Float01Incl();
		assert(f <= 1.f);
		assert(f >= 0.f);
		assert(f != 0.f || f2 != 0.f);
		if (f != f2)
			allEqual = false;
	}
	assert(!allEqual);
}

RANDOMIZED_TEST(LCG_FloatNeg1_1)
{
	LCG rng;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = rng.FloatNeg1_1();
		float f2 = rng.FloatNeg1_1();
		assert(f < 1.f);
		assert(f > -1.f);
		assert(f != 0.f || f2 != 0.f);
		if (f != f2)
			allEqual = false;
	}
	assert(!allEqual);
}

RANDOMIZED_TEST(LCG_Float_A_B)
{
	LCG rng;
	for(int i = 0; i < 1000; ++i)
	{
		float a = rng.Float();
		float b = rng.Float();
		if (b < a)
			Swap(a, b);

		float f = rng.Float(a, b);
		assert(a <= f);
		assert(f < b);
	}
}
