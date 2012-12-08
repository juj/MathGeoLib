#include <stdio.h>
#include <stdlib.h>
#include "TestRunner.h"
#include "Time/Clock.h"
#include <Algorithm>

#include "myassert.h"

std::vector<Test> tests;

volatile int globalPokedData = 0;

void AddTest(std::string name, TestFunctionPtr function, bool isRandomized, std::string description)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	t.isRandomized = isRandomized;
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

// Returns the number of failures.
int RunTests(int numTimes)
{
	int numTestsPassed = 0;
	int numWarnings = 0;

	int numTrials = 100;
	int numTimes1 = numTimes / numTrials;

	for(size_t i = 0; i < tests.size(); ++i)
	{
		printf("Testing '%s': ", tests[i].name.c_str());

		std::vector<tick_t> times;
		times.reserve(numTimes);

		int numFails = 0;
		int numPasses = 0;
		std::string failReason; // Stores the failure reason of the first failure.
		std::vector<std::string> failReasons;
		for(int j = 0; j < (tests[i].isRandomized ? numTimes1 : 1); ++j)
		{
			tick_t start = Clock::Tick();
			for(int k = 0; k < (tests[i].isRandomized ? numTrials : 1); ++k)
			{
				try
				{
					tests[i].function();
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

		numPasses = (tests[i].isRandomized ? numTimes : 1) - numFails;
		std::sort(times.begin(), times.end());

		// Erase outliers. (x% slowest)
		const float rateSlowestToDiscard = 0.05f;
		int numSlowestToDiscard = (int)(times.size() * rateSlowestToDiscard);
		times.erase(times.end() - numSlowestToDiscard, times.end());

		tick_t total = 0;
		for(size_t j = 0; j < times.size(); ++j)
			total += times[j];

		float successRate = (float)numPasses * 100.f / numTimes;

		if (numFails == 0)
		{
			if (tests[i].isRandomized)
				LOGI("ok (%d passes, 100%%)", numPasses);
			else
				LOGI("ok");
			++numTestsPassed;
		}
		else if (successRate >= 95.0f)
		{
			printf("ok ");
			LOGW("Some failures with '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), numPasses, successRate);
			++numTestsPassed;
			++numWarnings;
		}
		else
			LOGE("FAILED: '%s' (%d passes, %.2f%% of all tries)", failReason.c_str(), numPasses, successRate);

		if (!times.empty())
			LOGI("   Fastest: %s, Average: %s, Slowest: %s", FormatTime(times[0]).c_str(), FormatTime(total / times.size()).c_str(), FormatTime(times.back()).c_str());
	}

	int numFailures = (int)tests.size() - numTestsPassed;
	LOGI("Done. %d tests run. %d passed, of which %d succeeded with warnings. %d failed.", (int)tests.size(), numTestsPassed, numWarnings, numFailures);
	return numFailures;
}

void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();
void AddMatrixTests();

int main()
{
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();
	AddMatrixTests();

	int numFailures = RunTests(10000);
	LOGI("%d\n", globalPokedData);
	return numFailures; // exit code of 0 denotes a successful run.
}
