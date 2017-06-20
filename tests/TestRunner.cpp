#include <stdio.h>
#include <stdlib.h>
#include "TestRunner.h"
#include "../src/Time/Clock.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include "TestData.h"
#include "JSONReport.h"

#include "../src/Math/myassert.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

LCG rng(Clock::TickU32());

std::vector<Test> &Tests()
{
	static std::vector<Test> tests;
	return tests;
}

static int numTestsPassed = 0;
static int numTestsFailed = 0;
static int numTestsWarnings = 0;
static int numTestsRun = 0;

volatile int globalPokedData = 0;

// If true, the currently running test should fail, and succeeding is an error.
// 0 - not expected to fail
// 1 - expected to fail, issue LOGI diagnostics.
// 2 - expected to fail, issue LOGW diagnostics.
int globalTestExpectedToFail = 0;
std::string globalTestFailureDescription = ""; // A custom optional message describing why this test is expected to fail.

void AddTest(std::string name, TestFunctionPtr function, std::string file, std::string description, bool runOnlyOnce)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = false;
	t.runOnlyOnce = runOnlyOnce;
	t.isBenchmark = false;
	t.file = file;
	Tests().push_back(t);
}

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string file, std::string description)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = true;
	t.runOnlyOnce = false;
	t.isBenchmark = false;
	t.file = file;
	Tests().push_back(t);
}

void AddBenchmark(std::string name, TestFunctionPtr function, std::string file, std::string description)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = false;
	t.runOnlyOnce = true;
	t.isBenchmark = true;
	t.file = file;
	Tests().push_back(t);
}

std::string FormatTime(double ticks)
{
	double msecs = ticks * 1000.0 / Clock::TicksPerSec();
	double secs = msecs / 1000.0;
	double usecs = msecs * 1000.0;
	char str[256];
	if (secs >= 1.0)
		sprintf(str, "%.3f secs", (float)secs);
	else if (msecs >= 1.0)
		sprintf(str, "%.3f msecs", (float)msecs);
	else if(usecs >= 1.0)
		sprintf(str, "%.3f usecs", (float)usecs);
	else
		sprintf(str, "%.3f nsecs", (float)(usecs*1000.0));
	return str;
}

// Print text to log without appending a newline to the end, if possible.
#if !defined(LOGI_NL)
#if defined(ANDROID) || defined(NPAPI) || defined(__native_client__)
#define LOGI_NL LOGI
#elif defined(WIN8RT)
void LOGI_NL(const char *format, ...)
{
	const int capacity = 2048;
	char str[capacity];

	va_list args;
	va_start(args, format);

	vsnprintf((char *)str, capacity, format, args);
	str[capacity-1] = 0; // We only support logging a fixed-length string so don't care if we fail/truncate, just make sure we zero-terminate so there won't be any issues.
	OutputDebugStringA(str);
}
#else
#define LOGI_NL printf
#endif
#endif

/// Returns 0: passed, 1: passed with warnings, -1: failed.
int RunTest(Test &t, int numTimesToRun, int numTrialsPerRun, JSONReport &jsonReport)
{
	if (t.runOnlyOnce)
		numTimesToRun = numTrialsPerRun = 1;
	if (!t.isRandomized)
		numTimesToRun = 1;
	if (t.isBenchmark)
	{
		numTimesToRun = numTrialsPerRun = 1;
		LOGI_NL("Benchmark '%s': %s", t.name.c_str(), t.description.c_str());
	}
	else
		LOGI_NL("Testing '%s': ", t.name.c_str());

	std::vector<tick_t> times;
	times.reserve(numTimesToRun);

	t.numFails = 0;
	t.numPasses = 0;
	std::string failReason; // Stores the failure reason of the first failure.
	std::vector<std::string> failReasons;
	globalTestExpectedToFail = 0;
	globalTestFailureDescription = std::string();

	for(int j = 0; j < numTimesToRun; ++j)
	{
		tick_t start = Clock::Tick();
		for(int k = 0; k < numTrialsPerRun; ++k)
//			for(int k = 0; k < (t.isRandomized ? numTrials : 1); ++k)
		{
#ifdef FAIL_USING_EXCEPTIONS
			try
			{
#endif
				t.function(t);
				if (globalTestExpectedToFail)
				{
					globalTestExpectedToFail = 0; // Signal that the following exception reports a failure of this test, and not an expected failure.
					throw std::runtime_error(std::string("This test should have failed due to reason '") + globalTestFailureDescription + "', but it didn't fail!");
				}
#ifdef FAIL_USING_EXCEPTIONS
			}
			catch(const TestSkippedException &e)
			{
				if (failReason.empty())
				{
					failReason = std::string("SKIPPED: ") + e.what();
					LOGW("%s", failReason.c_str());
				}
			}
			catch(const std::exception &e)
			{
				if (globalTestExpectedToFail)
				{
					if (globalTestExpectedToFail == 2)
						LOGE("This test failed as expected. Caught an exception '%s', failure is due to reason '%s'.", e.what(), globalTestFailureDescription.c_str());
					else
						LOGI("This test failed as expected. Caught an exception '%s', failure is due to reason '%s'.", e.what(), globalTestFailureDescription.c_str());
				}
				else
				{
					if (failReason.empty())
						failReason = e.what();
					++t.numFails;
				}
			}
			catch(...)
			{
				++t.numFails;
				LOGE("Error: Received an unknown exception type that is _not_ derived from std::exception! This should not happen!");
			}
#endif
		}
		tick_t end = Clock::Tick();
		times.push_back(end - start);
	}

	t.numPasses = numTimesToRun*numTrialsPerRun - t.numFails;
	std::sort(times.begin(), times.end());

	// Erase outliers. (x% slowest)
	const float rateSlowestToDiscard = 0.05f;
	int numSlowestToDiscard = (int)(times.size() * rateSlowestToDiscard);
	times.erase(times.end() - numSlowestToDiscard, times.end());

	tick_t total = 0;
	for(size_t j = 0; j < times.size(); ++j)
		total += times[j];

	if (!t.isBenchmark)
	{
		if (!times.empty())
		{
			t.fastestTime = (double)times[0] / numTrialsPerRun;
			t.averageTime = (double)total / times.size() / numTrialsPerRun;
			t.worstTime = (double)times.back() / numTrialsPerRun;
			t.numTimesRun = numTimesToRun;
			t.numTrialsPerRun = numTrialsPerRun;
		}
		else
		{
			t.fastestTime = t.averageTime = t.worstTime = -1.0;
			t.numTimesRun = t.numTrialsPerRun = 0;
		}
	}
	float successRate = (t.numPasses + t.numFails > 0) ? (float)t.numPasses * 100.f / (t.numPasses + t.numFails) : 0.f;

	jsonReport.Report(t);

	if (t.isBenchmark && t.numFails == 0) // Benchmarks print themselves.
		return 0; // 0: Success

	int ret = 0; // 0: Success

	if (t.numFails == 0)
	{
		if (t.isRandomized)
			LOGI(" ok (%d passes, 100%%)", t.numPasses);
		else
			LOGI(" ok ");
//		++numTestsPassed;
		t.result = TestPassed;
	}
	else if (successRate >= 95.0f)
	{
		LOGI_NL(" ok ");
		LOGW("Some failures with '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), t.numPasses, successRate);
//		++numTestsPassed;
//		++numWarnings;
		ret = 1; // Success with warnings
		t.result = TestPassedWithWarnings;
	}
	else
	{
		if (t.isRandomized)
			LOGE("FAILED: '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), t.numPasses, successRate);
		else
			LOGE("FAILED: '%s'", failReason.c_str());
		ret = -1; // Failed
		t.result = TestFailed;
	}

	if (!times.empty())
	{
		if (t.runOnlyOnce)
			LOGI("   Elapsed: %s", FormatTime((double)times[0]).c_str());
		else
			LOGI("   Fastest: %s, Average: %s, Slowest: %s", FormatTime(t.fastestTime).c_str(), FormatTime(t.averageTime).c_str(), FormatTime(t.worstTime).c_str());
	}

	return ret;
}

static int nextTestToRun = 0;

bool StringBeginsWithOneOf(const char *str, const char * const *prefixes)
{
	for(const char * const *prefix = prefixes; *prefix; ++prefix)
		if (!strncmp(str, *prefix, strlen(*prefix)))
			return true;

	return false;
}

bool StringContainsOneOf(const char *str, const char * const *prefixes)
{
	for(const char * const *prefix = prefixes; *prefix; ++prefix)
		if (strstr(str, *prefix) != 0)
			return true;

	return false;
}

int RunOneTest(int numTimes, int numTrials, const char * const *prefixes, JSONReport &jsonReport)
{
	std::vector<Test> &tests = Tests();
	while(nextTestToRun < (int)tests.size())
	{
		if (StringBeginsWithOneOf(tests[nextTestToRun].name.c_str(), prefixes) || StringContainsOneOf(tests[nextTestToRun].description.c_str(), prefixes)
			|| StringContainsOneOf(tests[nextTestToRun].file.c_str(), prefixes))
		{
			int ret = RunTest(tests[nextTestToRun], numTimes, numTrials, jsonReport);

			if (ret == 0 || ret == 1)
				++numTestsPassed;
			if (ret == 1)
				++numTestsWarnings;
			if (ret == -1)
				++numTestsFailed;

			++nextTestToRun;
			++numTestsRun;
			return ret;
		}
		++nextTestToRun;
	}

	return -2; // No tests left to run
}

void PrintTestRunSummary()
{
	LOGI("Done. %d tests run. %d passed, of which %d succeeded with warnings. %d failed.", numTestsRun, numTestsPassed, numTestsWarnings, numTestsFailed);
	if (numTestsFailed > 0)
	{
		LOGE_NS("The following tests failed:");
		std::vector<Test> &tests = Tests();
		for (size_t i = 0; i < tests.size(); ++i)
		{
			if (tests[i].result == TestFailed)
				LOGE_NS("   %s", tests[i].name.c_str());
		}
	}
	if (numTestsWarnings > 0)
	{
		LOGW_NS("The following tests had some failures:");
		std::vector<Test> &tests = Tests();
		int totalFailureCount = 0;
		int totalPassCount = 0;
		for (size_t i = 0; i < tests.size(); ++i)
		{
			Test &t = tests[i];
			if (t.numFails > 0 && tests[i].result != TestFailed)
			{
				float successRate = (t.numPasses + t.numFails > 0) ? (float)t.numPasses * 100.f / (t.numPasses + t.numFails) : 0.f;
				LOGW_NS("   %s: %d failures, %d passes (%.2f%% success rate).", tests[i].name.c_str(), tests[i].numFails, tests[i].numPasses, successRate);
				totalFailureCount += t.numFails;
			}
			totalPassCount += t.numPasses;
		}
		LOGW_NS("Total failure count: %d/%d (%f%% of all iterations)", totalFailureCount, totalFailureCount + totalPassCount, totalFailureCount * 100.0 / (totalFailureCount+totalPassCount));
	}
}

int argc_;
char **argv_;
void TestsFinished()
{
	PrintTestRunSummary();

	LOGI("%d", globalPokedData);

	// When --exit0 is passed, we forcibly return 0 and not the number of failed tests.
	// Used by buildbot in valgrind runs to ignore any failures - the failures are detected
	// in a "real" run instead that carry more randomized trial runs.
	for(int i = 1; i < argc_; ++i)
		if (!strcmp(argv_[i], "--exit0"))
			exit(0);
}

int numTotalRuns = 0;
int numTrialsPerTimedBlock = 0;
// A list of test prefixes to include in the run.
std::vector<const char *> prefixes;
JSONReport jsonReport;

void RunNextTest()
{
	RunOneTest(numTotalRuns, numTrialsPerTimedBlock, &prefixes[0], jsonReport);
#ifdef __EMSCRIPTEN__
	if (nextTestToRun >= (int)Tests().size())
	{
		emscripten_cancel_main_loop();
		TestsFinished();
		exit(numTestsFailed);
	}
#endif
}

#ifdef MATH_TESTS_EXECUTABLE
int main(int argc, char **argv)
{
	argc_ = argc;
	argv_ = argv;

	try
	{
		TestData::InitTestData();
	} catch(const std::exception &e)
	{
		LOGE("std::exception was thrown during initialization of test runner data! Unable to launch tests!\n%s", e.what());
		return 99999;
	} catch(...)
	{
		LOGE("Unknown exception was thrown during initialization of test runner data! Unable to launch tests!");
		return 99999;
	}
	numTotalRuns = (argc >= 2) ? atoi(argv[1]) : 100;
	numTrialsPerTimedBlock = (argc >= 3) ? atoi(argv[2]) : 100;
#ifdef __EMSCRIPTEN__
	numTotalRuns = numTrialsPerTimedBlock = 10;
#endif
	for(int i = 3; i < argc; ++i)
	{
		if (argv[i][0] != '-' && argv[i][0] != '/')
			prefixes.push_back(argv[i]);
	}
	if (prefixes.empty())
		prefixes.push_back(""); // Empty prefix runs all tests.
	prefixes.push_back(0); // Sentinel to terminate prefix string list.

	if (numTotalRuns == 0 || numTrialsPerTimedBlock == 0)
	{
		LOGI("Usage: %s <numTotalRuns> <numTrialsPerTimedBlock>", argv[0]); 
		LOGI("   Runs all tests.");
		LOGI("       %s <numTotalRuns> <numTrialsPerTimedBlock> prefix1 prefix2 prefix3...", argv[0]); 
		LOGI("   Runs all tests starting with one of the given prefixes, or residing in one of the named code files.");
		return 0;
	}

	{
		std::string jsonFilename = "test_results.json";
		for(int i = 1; i+1 < argc; ++i)
			if (!strcmp(argv[i], "--json"))
				jsonFilename = argv[i+1]; // Allow overriding the output file name from command line.
		jsonReport.Create(jsonFilename.c_str());
	}

	numTestsRun = numTestsPassed = numTestsWarnings = numTestsFailed = 0;

#ifdef __EMSCRIPTEN__
	emscripten_set_main_loop(&RunNextTest, 0, 0);
#else
	while(nextTestToRun < (int)Tests().size())
		RunNextTest();
	TestsFinished();
	return numTestsFailed; // exit code of 0 denotes a successful run.
#endif
}
#endif
