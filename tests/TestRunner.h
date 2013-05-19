#pragma once

#include <vector>
#include <string>

#include "../src/Algorithm/Random/LCG.h"
#include "../src/Time/Clock.h"

#define SCALE 1e2f

#define GUARDBAND 1e-2f

extern LCG rng;

// The test function can read test info and write test result to t.
struct Test;
typedef void (*TestFunctionPtr)(Test &t);
struct Test
{
	Test()
	{
		isBenchmark = isRandomized = runOnlyOnce = false;
		function = 0;
		numTimesRun = numTrialsPerRun = numPasses = numFails = 0;
		fastestTime = averageTime = worstTime = fastestCycles = 0.0;
	}
	std::string name;
	std::string file;
	std::string description;
	bool isBenchmark;
	bool isRandomized;
	/// If true, this function should not be trialled multiple times (for performance benchmarks), but should
	/// be called only exactly once. Used mostly for very slow tests that would take too long to run multiple times.
	bool runOnlyOnce;
	TestFunctionPtr function;

	// Results:
	int numTimesRun; ///< The total number of times this test was executed.
	int numTrialsPerRun; ///< Each time this test was run, how many trials of it were performed (for benchmarking).
	int numPasses;
	int numFails;
	double fastestTime;
	double averageTime;
	double worstTime;

	double fastestCycles; ///< Raw clock cycles/high-resolution counter times.
};

class JSONReport;

extern volatile int globalPokedData;

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string file = "", std::string description = "");
void AddTest(std::string name, TestFunctionPtr function, std::string file = "", std::string description = "", bool runOnlyOnce = false);
void AddBenchmark(std::string name, TestFunctionPtr function, std::string file = "", std::string description = "");
int RunTests(int numTimes);
/// Returns -2: no tests left to run, -1: failed, 0: success, 1: success with warnings.
int RunOneTest(int numTimes, int numTrials, const char * const *prefixes, JSONReport &jsonReport);
void PrintTestRunSummary();
std::string FormatTime(double ticks);

class AddTestOp
{
public:
	AddTestOp(const char *name, const char *file, const char *description, bool isRandomized, bool runOnlyOnce, bool isBenchmark, TestFunctionPtr function)
	{
		if (isBenchmark)
			AddBenchmark(name, function, file, description);
		else if (isRandomized)
			AddRandomizedTest(name, function, file, description);
		else
			AddTest(name, function, description, file, runOnlyOnce);
	}
};

#define TEST(name) \
	void TestFunc_##name(Test &test); \
	AddTestOp addtestop_##name(#name, __FILE__, "", false, false, false, TestFunc_##name); \
	void TestFunc_##name(Test & /*test*/)

#define RANDOMIZED_TEST(name) \
	void TestFunc_##name(Test &test); \
	AddTestOp addtestop_##name(#name, __FILE__, "", true, false, false, TestFunc_##name); \
	void TestFunc_##name(Test & /*test*/)

#define UNIQUE_TEST(name) \
	void TestFunc_##name(Test &test); \
	AddTestOp addtestop_##name(#name, __FILE__, "", false, true, false, TestFunc_##name); \
	void TestFunc_##name(Test & /*test*/)

#define BENCHMARK(name, description) \
	void BenchmarkFunc_##name(Test &test); \
	AddTestOp addbenchmarkop_##name(#name, __FILE__, description, false, false, true, BenchmarkFunc_##name); \
	void BenchmarkFunc_##name(Test &test) \
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

#define BENCHMARK_END \
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
	test.numTimesRun = testrunner_numTimerTests; \
	test.numTrialsPerRun = testrunner_numItersPerTest; \
	test.fastestCycles = (double)bestTsc / testrunner_numItersPerTest; \
	test.fastestTime = (double)bestTicks / testrunner_numItersPerTest; \
	test.averageTime = (double)accumTicks / (testrunner_numTimerTests * testrunner_numItersPerTest); \
	test.worstTime = (double)worstTicks / testrunner_numItersPerTest; \
	LOGI("\n   Best: %s / %g ticks, Avg: %s, Worst: %s", FormatTime(test.fastestTime).c_str(), test.fastestCycles, \
		FormatTime(test.averageTime).c_str(), FormatTime(test.worstTime).c_str()); \
}

#if defined(_DEBUG) || defined(DEBUG) // In debug mode, it's sensible to run benchmarks only to test they don't crash, so do minimal amount of iterations.
#if defined(EMSCRIPTEN)
const int testrunner_numTimerTests = 1;
const int testrunner_numItersPerTest = 1;
#else
const int testrunner_numTimerTests = 10;
const int testrunner_numItersPerTest = 100;
#endif
#else
const int testrunner_numTimerTests = 100;
const int testrunner_numItersPerTest = 1000;
#endif
