#include "myassert.h"
#include "MathGeoLib.h"
#include "../tests/TestRunner.h"

void TestIntFast()
{
	LCG rng;
	u32 prev = rng.IntFast();
	for(int i = 0; i < 1000; ++i)
	{
		u32 next = rng.IntFast();
		assert(next != prev);
		next = prev;
	}
}

void TestInt()
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

void TestIntAB()
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

void TestFloat()
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

void TestFloatAB()
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
		assert(f <= b);
	}
}

void AddLCGTests()
{
	AddRandomizedTest("LCG::IntFast", TestIntFast);
	AddRandomizedTest("LCG::Int", TestInt);
	AddRandomizedTest("LCG::Int(a,b)", TestIntAB);
	AddRandomizedTest("LCG::Float", TestFloat);
	AddRandomizedTest("LCG::Float(a,b)", TestFloatAB);
}
