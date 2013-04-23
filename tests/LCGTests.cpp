#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"

RANDOMIZED_TEST(LCG_IntFast)
{
	LCG lcg;
	u32 prev = lcg.IntFast();
	for(int i = 0; i < 1000; ++i)
	{
		u32 next = lcg.IntFast();
		assert(next != prev);
		prev = next;
	}
}

RANDOMIZED_TEST(LCG_Int)
{
	LCG lcg;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		int prev = lcg.Int();
		int next = lcg.Int();
		assert(prev != 0 || next != 0);
		if (prev != next)
			allEqual = false;
	}
	assert(!allEqual);
}

RANDOMIZED_TEST(LCG_Int_A_B)
{
	LCG lcg;
	for(int i = 0; i < 1000; ++i)
	{
		int a = lcg.Int();
		int b = lcg.Int();
		if (b < a)
			Swap(a, b);
		int val = lcg.Int(a, b);
		assert(a <= val);
		assert(val <= b);
	}
}

RANDOMIZED_TEST(LCG_Float)
{
	LCG lcg;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = lcg.Float();
		float f2 = lcg.Float();
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
	LCG lcg;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = lcg.Float01Incl();
		float f2 = lcg.Float01Incl();
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
	LCG lcg;
	bool allEqual = true;
	for(int i = 0; i < 1000; ++i)
	{
		float f = lcg.FloatNeg1_1();
		float f2 = lcg.FloatNeg1_1();
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
	LCG lcg;
	for(int i = 0; i < 1000; ++i)
	{
		float a = lcg.Float();
		float b = lcg.Float();
		if (b < a)
			Swap(a, b);

		float f = lcg.Float(a, b);
		assert(a <= f);
		assert(f < b);
	}
}
