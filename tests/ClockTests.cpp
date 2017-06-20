#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "SystemInfo.h"

MATH_IGNORE_UNUSED_VARS_WARNING

TEST(MonotonousClock)
{
	tick_t maxDiff = 0;
	tick_t prev = Clock::Tick();
	for(int i = 0; i < 10000; ++i)
	{
		tick_t now = Clock::Tick();
		tick_t diff = Clock::TicksInBetween(now, prev);
		assert(diff < Clock::TicksPerSec());
		prev = now;
		maxDiff = Max(diff, maxDiff);
	}

	assert(maxDiff > 0); // The clock must proceed at least some amount.
}

UNIQUE_TEST(ClockPrecision)
{
	LOGI("Clock::Tick() runs at %llu ticks/second.", Clock::TicksPerSec());
}

UNIQUE_TEST(Clock_RdTsc)
{
	unsigned long long tsc = Clock::Rdtsc();
	unsigned long long tsc2 = Clock::Rdtsc();
	LOGI("Two subsequent calls to rdtsc report %llu and %llu. (delta: %llu)", tsc, tsc2, tsc2 - tsc);
}

UNIQUE_TEST(SubMillisecondPrecision)
{
#ifdef __EMSCRIPTEN__
	if (IsChromeBrowserOnWin32())
	{
		// Newest version failure was observed in is 31.0.1600.1.
		WARN_AND_EXPECT_FAIL("Chrome on Win32 has bad timer resolution: https://code.google.com/p/chromium/issues/detail?id=158234");
	}
	if (IsOperaBrowser() && GetOperaVersion() <= BrowserVersion("12.16"))
		WARN_AND_EXPECT_FAIL("Opera has bad timer resolution and doesn't support window.performance.now().");
	if (IsSafariBrowser() && GetSafariVersion() <= BrowserVersion("6.0.5"))
		WARN_AND_EXPECT_FAIL("Safari has bad timer resolution and doesn't support window.performance.now().");
#endif

	tick_t ticksPerMillisecond = Clock::TicksPerMillisecond();
	MARK_UNUSED(ticksPerMillisecond);
#ifndef MATH_TICK_IS_FLOAT
	assert1(ticksPerMillisecond > 1, (u32)ticksPerMillisecond);
#endif

	tick_t minDiff = Clock::TicksPerSec();
	tick_t prev = Clock::Tick();
	int numTimesZeroDiff = 0;
	int numIters = 50000;
	for(int i = 0; i < numIters; ++i)
	{
		tick_t now = Clock::Tick();
		tick_t diff = Clock::TicksInBetween(now, prev);
		if (diff > 0)
			minDiff = Min(diff, minDiff);
		else
			++numTimesZeroDiff;

		prev = now;
	}

	LOGI("Smallest observed non-zero delta in Clock::Tick() is %d ticks. A zero delta was observed %d times (out of %d tests)", 
		(int)minDiff, numTimesZeroDiff, numIters);
	assert(minDiff > 0);
	assert(minDiff < ticksPerMillisecond/2); // Smallest met quantity must be less than half a millisecond.
}

volatile tick_t dummyGlobalTicks = 0;

BENCHMARK(Clock_Tick, "Time taken by 10x Clock::Tick()s")
{
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
	dummyGlobalTicks += Clock::Tick();
}
BENCHMARK_END;

volatile unsigned long long dummyGlobalTsc = 0;

BENCHMARK(Clock_Rdtsc, "Time taken by 10x Clock::Rdtsc()s")
{
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
	dummyGlobalTsc += Clock::Rdtsc();
}
BENCHMARK_END;
