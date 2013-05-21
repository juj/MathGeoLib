#include <stdio.h>
#include <stdlib.h>
#include "TestRunner.h"
#include "../src/Time/Clock.h"
#include <algorithm>
#include <cstring>
#include "TestData.h"
#include "JSONReport.h"

#include "../src/Math/myassert.h"

LCG rng(Clock::TickU32());

std::vector<Test> &Tests()
{
	static std::vector<Test> tests;
	return tests;
}

static int numTestsPassed = 0;
static int numTestsFailed = 0;
static int numTestsWarnings = 0;

volatile int globalPokedData = 0;

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
#ifdef FAIL_USING_EXCEPTIONS
			}
			catch(const std::runtime_error &e)
			{
				if (!!strcmp(e.what(), "expect failure"))
				{
					if (failReason.empty())
						failReason = e.what();
					++t.numFails;
				}
				else
				{
					LOGI("Caught std::runtime_error thrown from another file as expected.");
				}
			}
			catch(const std::exception &e)
			{
				if (!!strcmp(e.what(), "expect failure"))
				{
					if (failReason.empty())
						failReason = e.what();
					++t.numFails;
				}
				else
				{
					LOGI("Caught std::runtime_error as std::exception thrown from another file as expected.");
				}
			}
			catch(...)
			{
				LOGE("Error: Received an unknown exception type that is _not_ derived from std::exception!");
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
		t.fastestTime = (double)times[0] / numTrialsPerRun;
		t.averageTime = (double)total / times.size() / numTrialsPerRun;
		t.worstTime = (double)times.back() / numTrialsPerRun;
		t.numTimesRun = numTimesToRun;
		t.numTrialsPerRun = numTrialsPerRun;
	}
	float successRate = (float)t.numPasses * 100.f / (t.numPasses + t.numFails);

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
	}
	else if (successRate >= 95.0f)
	{
		LOGI_NL(" ok ");
		LOGW("Some failures with '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), t.numPasses, successRate);
//		++numTestsPassed;
//		++numWarnings;
		ret = 1; // Success with warnings
	}
	else
	{
		if (t.isRandomized)
			LOGE("FAILED: '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), t.numPasses, successRate);
		else
			LOGE("FAILED: '%s'", failReason.c_str());
		ret = -1; // Failed
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
			return ret;
		}
		++nextTestToRun;
	}

	return -2; // No tests left to run
}

// Returns the number of failures.
int RunTests(int numTimes, int numTrials, const char * const *prefixes, JSONReport &jsonReport)
{
	numTestsPassed = numTestsWarnings = numTestsFailed = 0;

	for(size_t i = 0; i < Tests().size(); ++i)
		RunOneTest(numTimes, numTrials, prefixes, jsonReport);

	PrintTestRunSummary();
	return numTestsFailed;
}

void PrintTestRunSummary()
{
	LOGI("Done. %d tests run. %d passed, of which %d succeeded with warnings. %d failed.", (int)Tests().size(), numTestsPassed, numTestsWarnings, numTestsFailed);
}

#ifdef MATH_TESTS_EXECUTABLE
int main(int argc, char **argv)
{
	int numTotalRuns = (argc >= 2) ? atoi(argv[1]) : 100;
	int numTrialsPerTimedBlock = (argc >= 3) ? atoi(argv[2]) : 100;
#ifdef EMSCRIPTEN
	numTotalRuns = numTrialsPerTimedBlock = 10;
#endif
	const char * const noPrefixes[] = { "", 0 };
	const char * const *prefixes = (argc >= 4) ? &argv[3] : noPrefixes;

	if (numTotalRuns == 0 || numTrialsPerTimedBlock == 0)
	{
		LOGI("Usage: %s <numTotalRuns> <numTrialsPerTimedBlock>", argv[0]); 
		LOGI("   Runs all tests.");
		LOGI("       %s <numTotalRuns> <numTrialsPerTimedBlock> prefix1 prefix2 prefix3...", argv[0]); 
		LOGI("   Runs all tests starting with one of the given prefixes, or residing in one of the named code files.");
		return 0;
	}

	JSONReport jsonReport;
	std::string jsonFilename = "test_results.json";
	for(int i = 1; i+1 < argc; ++i)
		if (!strcmp(argv[i], "--json"))
			jsonFilename = argv[i+1]; // Allow overriding the output file name from command line.
	jsonReport.Create(jsonFilename.c_str());

	int numFailures = RunTests(numTotalRuns, numTrialsPerTimedBlock, prefixes, jsonReport);
	LOGI("%d", globalPokedData);

	// When --exit0 is passed, we forcibly return 0 and not the number of failed tests.
	// Used by buildbot in valgrind runs to ignore any failures - the failures are detected
	// in a "real" run instead that carry more randomized trial runs.
	for(int i = 1; i < argc; ++i)
		if (!strcmp(argv[i], "--exit0"))
			return 0;

	return numFailures; // exit code of 0 denotes a successful run.
}
#endif
