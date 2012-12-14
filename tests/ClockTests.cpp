#include "myassert.h"
#include "MathGeoLib.h"
#include "../tests/TestRunner.h"

void TestMonotonousClock()
{
	tick_t maxDiff = 0;
	tick_t prev = Clock::Tick();
	for(int i = 0; i < 10000; ++i)
	{
		tick_t now = Clock::Tick();
		tick_t diff = Clock::TicksInBetween(now, prev);
		assert(diff >= 0);
		assert(diff < Clock::TicksPerSec());
		prev = now;
		maxDiff = Max(diff, maxDiff);
	}

	assert(maxDiff > 0);
}

void AddClockTests()
{
	AddTest("Clock monotonity", TestMonotonousClock);
}
