#pragma once

#include <vector>
#include <string>

#include "../src/Algorithm/Random/LCG.h"
#include "../src/Time/Clock.h"

#define SCALE 1e2f

#define GUARDBAND 1e-2f

#ifdef MATH_ENABLE_NAMESPACE
using namespace MATH_NS;
#endif

extern LCG rng;

enum TestResult
{
	TestNotRun,
	TestPassed,
	TestPassedWithWarnings,
	TestFailed
};

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
		result = TestNotRun;
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

	TestResult result;

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
extern int globalTestExpectedToFail;
extern std::string globalTestFailureDescription;

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

#ifndef __EMSCRIPTEN__
#define RDTSC() Clock::Rdtsc()
#else
#define RDTSC() 0
#endif

#if _MSC_VER >= 1700 // Visual Studio 2012
// On VS2012 and newer, benchmarks are not apples-to-apples if some for loops get autovectorized by
// the compiler, and others aren't. For example, measuring scalar sinf() vs MGL SSE Sin() shows that
// VS2012 autovectorizer is able to compute 8 sinf()s in one iteration, but Sin() is always computed fully scalar.
// Therefore in the benchmarks disable autovectorization to provide directly comparable results.
// http://msdn.microsoft.com/en-us/library/hh872235.aspx
#define MGL_PRAGMA_NO_AUTOVECTORIZE __pragma(loop(no_vector))
#else
#define MGL_PRAGMA_NO_AUTOVECTORIZE
#endif

#define BENCHMARK(name, description) \
	void BenchmarkFunc_##name(Test &test); \
	AddTestOp addbenchmarkop_##name(#name, __FILE__, description, false, false, true, BenchmarkFunc_##name); \
	void BenchmarkFunc_##name(Test &test) \
	{ \
		unsigned long long bestTsc = (unsigned long long)-1; \
		tick_t bestTicks = TICK_INF; \
		tick_t accumTicks = 0; \
		tick_t worstTicks = 0; \
		int numWarmupTicks = 1; /* Run a warmup for JIT environments */ \
		MGL_PRAGMA_NO_AUTOVECTORIZE for(int xx = -numWarmupTicks; xx < testrunner_numTimerTests; ++xx) \
		{ \
			tick_t start = Clock::Tick(); \
			unsigned long long startTsc = RDTSC(); \
			MGL_PRAGMA_NO_AUTOVECTORIZE for(int i = 0; i < testrunner_numItersPerTest; ++i) \
			{

#define BENCHMARK_END \
			} \
			unsigned long long endTsc = RDTSC(); \
			tick_t end = Clock::Tick(); \
			tick_t elapsedTicks = end - start; \
			unsigned long long elapsedTsc = (unsigned long long)((startTsc != 0 || endTsc != 0) ? (endTsc - startTsc) : elapsedTicks); \
			/*LOGI("Took %f ticks %s", (float)elapsedTicks, (xx < 0) ? "Ignored (warmup)" : "");*/ \
			if (xx < 0) continue; /* Skip recording measures when warming up. */ \
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

#define BENCHMARK_ITERS(name, numTests_, numIters_, description) \
	void BenchmarkFunc_##name(Test &test); \
	AddTestOp addbenchmarkop_##name(#name, __FILE__, description, false, false, true, BenchmarkFunc_##name); \
	void BenchmarkFunc_##name(Test &test) \
	{ \
		const int numTests = numTests_; \
		const int numIters = numIters_; \
		unsigned long long bestTsc = (unsigned long long)-1; \
		tick_t bestTicks = (tick_t)-1; \
		tick_t accumTicks = 0; \
		tick_t worstTicks = 0; \
		MGL_PRAGMA_NO_AUTOVECTORIZE for(int xx = 0; xx < numTests; ++xx) \
		{ \
			tick_t start = Clock::Tick(); \
			unsigned long long startTsc = Clock::Rdtsc(); \
			MGL_PRAGMA_NO_AUTOVECTORIZE for(int i = 0; i < numIters; ++i) \
			{

#define BENCHMARK_ITERS_END \
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
	test.numTimesRun = numTests; \
	test.numTrialsPerRun = numIters; \
	test.fastestCycles = (double)bestTsc / numIters; \
	test.fastestTime = (double)bestTicks / numIters; \
	test.averageTime = (double)accumTicks / (numTests * numIters); \
	test.worstTime = (double)worstTicks / numIters; \
	LOGI("\n   Best: %s / %g ticks, Avg: %s, Worst: %s", FormatTime(test.fastestTime).c_str(), test.fastestCycles, \
		FormatTime(test.averageTime).c_str(), FormatTime(test.worstTime).c_str()); \
}

class TestSkippedException : public std::exception
{
public:
	explicit TestSkippedException(const char *reason_)
	:reason(reason_)
	{
	}

	virtual ~TestSkippedException() throw()
	{
	}

	std::string reason;

	virtual const char* what() const throw()
	{
		return reason.c_str();
	}
};

#define SKIP_TEST(reason) throw TestSkippedException(reason)
#define WARN_AND_EXPECT_FAIL(reason) { globalTestExpectedToFail = 2; globalTestFailureDescription = reason; }
#define EXPECT_FAIL(reason) { globalTestExpectedToFail = 1; globalTestFailureDescription = reason; }

#if defined(_DEBUG) || defined(DEBUG) // In debug mode, it's sensible to run benchmarks only to test they don't crash, so do minimal amount of iterations.

#ifdef __EMSCRIPTEN__
// Need to be _very_ minimal in debug mode, since we run with SAFE_HEAP=1.
const int testrunner_numTimerTests = 2;
const int testrunner_numItersPerTest = 4;
#else
const int testrunner_numTimerTests = 3;
const int testrunner_numItersPerTest = 10;
#endif

#else
const int testrunner_numTimerTests = 100;
const int testrunner_numItersPerTest = 1000;
#endif

