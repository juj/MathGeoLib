#include "myassert.h"
#include "MathGeoLib.h"
#include "../tests/TestRunner.h"

TEST(TestMonotonousClock)
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

	assert(maxDiff > 0); // The clock must proceed at least some amount.
}

TEST(TestSubMillisecondPrecision)
{
	tick_t ticksPerMillisecond = Clock::TicksPerMillisecond();
	assert(ticksPerMillisecond > 1);

	tick_t minDiff = Clock::TicksPerSec();
	tick_t prev = Clock::Tick();
	for(int i = 0; i < 10000; ++i)
	{
		tick_t now = Clock::Tick();
		tick_t diff = Clock::TicksInBetween(now, prev);
		if (diff > 0)
			minDiff = Min(diff, minDiff);

		prev = now;
	}

	assert(minDiff > 0);
	assert(minDiff < ticksPerMillisecond/2); // Smallest met quantity must be less than half a millisecond.
}
