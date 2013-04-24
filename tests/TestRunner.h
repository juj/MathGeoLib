#pragma once

#include <vector>
#include <string>

#include "../src/Algorithm/Random/LCG.h"
#include "../src/Time/Clock.h"

#define SCALE 1e2f

#define GUARDBAND 1e-2f

extern LCG rng;

typedef void (*TestFunctionPtr)();
struct Test
{
	std::string name;
	std::string description;
	bool isBenchmark;
	bool isRandomized;
	/// If true, this function should not be trialled multiple times (for performance benchmarks), but should
	/// be called only exactly once. Used mostly for very slow tests that would take too long to run multiple times.
	bool runOnlyOnce;
	TestFunctionPtr function;
};

extern volatile int globalPokedData;

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string description = "");
void AddTest(std::string name, TestFunctionPtr function, std::string description = "", bool runOnlyOnce = false);
void AddBenchmark(std::string name, TestFunctionPtr function, std::string description = "");
int RunTests(int numTimes);
/// Returns -2: no tests left to run, -1: failed, 0: success, 1: success with warnings.
int RunOneTest(int numTimes, int numTrials, const char **prefixes);
void PrintTestRunSummary();
std::string FormatTime(double ticks);

class AddTestOp
{
public:
	AddTestOp(const char *name, const char *description, bool isRandomized, bool runOnlyOnce, bool isBenchmark, TestFunctionPtr function)
	{
		if (isBenchmark)
			AddBenchmark(name, function, description);
		else if (isRandomized)
			AddRandomizedTest(name, function, description);
		else
			AddTest(name, function, description, runOnlyOnce);
	}
};

#ifdef MATH_TESTS_EXECUTABLE

#define TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, __FILE__, false, false, false, TestFunc_##name); \
	void TestFunc_##name()

#define RANDOMIZED_TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, __FILE__, true, false, false, TestFunc_##name); \
	void TestFunc_##name()

#define UNIQUE_TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, __FILE__, false, true, false, TestFunc_##name); \
	void TestFunc_##name()

#define BENCHMARK(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, __FILE__, false, false, true, TestFunc_##name); \
	void TestFunc_##name()

#else

// Not running tests - specfiy the test functions, but don't add them at app startup time (they should be DCEd).
#define TEST(name) void TestFunc_##name()
#define RANDOMIZED_TEST(name) void TestFunc_##name()
#define UNIQUE_TEST(name) void TestFunc_##name()
#define BENCHMARK(name) void TestFunc_##name()

#endif

const int testrunner_numTimerTests = 10000;
const int testrunner_numItersPerTest = 1000;

#define TIMER_BEGIN \
{ \
	unsigned long long bestTsc = (unsigned long long)-1; \
	tick_t bestTicks = (tick_t)-1; \
	tick_t accumTicks = 0; \
	tick_t worstTicks = 0; \
	for(int xx = 0; xx < testrunner_numTimerTests; ++xx) \
	{ \
		tick_t start = Clock::Tick(); \
		unsigned long long startTsc = Clock::Rdtsc(); \
		for(int i = 0; i < testrunner_numItersPerTest; ++i) \
		{

#define TIMER_END \
		} \
		unsigned long long endTsc = Clock::Rdtsc(); \
		tick_t end = Clock::Tick(); \
		tick_t elapsedTicks = end - start; \
		tick_t elapsedTsc = endTsc - startTsc; \
		bestTsc = Min(bestTsc, elapsedTsc); \
		bestTicks = Min(bestTicks, elapsedTicks); \
		worstTicks = Max(worstTicks, elapsedTicks); \
		accumTicks += elapsedTicks; \
	} \
	double bestCycles = (double)bestTsc / testrunner_numItersPerTest; \
	LOGI("\n   Best: %s / %g ticks, Avg: %s, Worst: %s", FormatTime((double)bestTicks / testrunner_numItersPerTest).c_str(), bestCycles, \
		FormatTime((double)accumTicks / (testrunner_numTimerTests * testrunner_numItersPerTest)).c_str(), FormatTime((double)worstTicks / testrunner_numItersPerTest).c_str()); \
}
