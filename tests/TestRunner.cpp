#include <stdio.h>
#include <stdlib.h>
#include "TestRunner.h"
#include "Time/Clock.h"
#include <algorithm>
#include "MathGeoLibTests.h"

#include "myassert.h"

LCG rng(Clock::TickU32());

std::vector<Test> tests;

static int numTestsPassed = 0;
static int numTestsFailed = 0;
static int numTestsWarnings = 0;

volatile int globalPokedData = 0;

void AddTest(std::string name, TestFunctionPtr function, std::string description, bool runOnlyOnce)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = false;
	t.runOnlyOnce = runOnlyOnce;
	tests.push_back(t);
}

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string description)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = true;
	t.runOnlyOnce = false;
	tests.push_back(t);
}

std::string FormatTime(tick_t ticks)
{
	double msecs = Clock::TicksToMillisecondsD(ticks);
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
#if defined(ANDROID) || defined(NPAPI)
#define LOGI_NL LOGI
#else
#define LOGI_NL printf
#endif

/// Returns 0: passed, 1: passed with warnings, -1: failed.
int RunTest(Test &t, int numTimes, int numTrials)
{
	if (t.runOnlyOnce)
		numTimes = numTrials = 1;
	int numTimes1 = numTimes / numTrials;
	LOGI_NL("Testing '%s': ", t.name.c_str());

	std::vector<tick_t> times;
	times.reserve(numTimes);

	int numFails = 0;
	int numPasses = 0;
	std::string failReason; // Stores the failure reason of the first failure.
	std::vector<std::string> failReasons;
	for(int j = 0; j < (t.isRandomized ? numTimes1 : 1); ++j)
	{
		tick_t start = Clock::Tick();
		for(int k = 0; k < numTrials; ++k)
//			for(int k = 0; k < (t.isRandomized ? numTrials : 1); ++k)
		{
			try
			{
				t.function();
			}
			catch(const std::exception &e)
			{
				if (failReason.empty())
					failReason = e.what();
				++numFails;
			}
		}
		tick_t end = Clock::Tick();
		times.push_back(end - start);
	}

	numPasses = (t.isRandomized ? numTimes : 1) - numFails;
	std::sort(times.begin(), times.end());

	// Erase outliers. (x% slowest)
	const float rateSlowestToDiscard = 0.05f;
	int numSlowestToDiscard = (int)(times.size() * rateSlowestToDiscard);
	times.erase(times.end() - numSlowestToDiscard, times.end());

	tick_t total = 0;
	for(size_t j = 0; j < times.size(); ++j)
		total += times[j];

	float successRate = (float)numPasses * 100.f / numTimes;

	int ret = 0; // 0: Success

	if (numFails == 0)
	{
		if (t.isRandomized)
			LOGI(" ok (%d passes, 100%%)", numPasses);
		else
			LOGI(" ok ");
//		++numTestsPassed;
	}
	else if (successRate >= 95.0f)
	{
		LOGI_NL(" ok ");
		LOGW("Some failures with '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), numPasses, successRate);
//		++numTestsPassed;
//		++numWarnings;
		ret = 1; // Success with warnings
	}
	else
	{
		if (t.isRandomized)
			LOGE("FAILED: '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), numPasses, successRate);
		else
			LOGE("FAILED: '%s'", failReason.c_str());
		ret = -1; // Failed
	}

	if (!times.empty())
		LOGI("   Fastest: %s, Average: %s, Slowest: %s", FormatTime(times[0]).c_str(), FormatTime(total / times.size()).c_str(), FormatTime(times.back()).c_str());

	return ret;
}

static int nextTestToRun = 0;

int RunOneTest(int numTimes, int numTrials)
{
	if (nextTestToRun >= (int)tests.size())
		return -2; // No tests left to run
	int ret = RunTest(tests[nextTestToRun++], numTimes, numTrials);

	if (ret == 0 || ret == 1)
		++numTestsPassed;
	if (ret == 1)
		++numTestsWarnings;
	if (ret == -1)
		++numTestsFailed;

	return ret;
}

// Returns the number of failures.
int RunTests(int numTimes, int numTrials)
{
	numTestsPassed = numTestsWarnings = numTestsFailed = 0;

	for(size_t i = 0; i < tests.size(); ++i)
		RunOneTest(numTimes, numTrials);

	PrintTestRunSummary();
	return numTestsFailed;
}

void PrintTestRunSummary()
{
	LOGI("Done. %d tests run. %d passed, of which %d succeeded with warnings. %d failed.", (int)tests.size(), numTestsPassed, numTestsWarnings, numTestsFailed);
}

#ifdef MATH_TESTS_EXECUTABLE
int main(int argc, char **argv)
{
	AddMathGeoLibTests();

	const int numTotalRuns = (argc >= 2) ? atoi(argv[1]) : 10000;
	const int numTrialsPerTimedBlock = (argc >= 3) ? atoi(argv[2]) : 100;

	int numFailures = RunTests(numTotalRuns, numTrialsPerTimedBlock);
	LOGI("%d", globalPokedData);
	return numFailures; // exit code of 0 denotes a successful run.
}
#endif
