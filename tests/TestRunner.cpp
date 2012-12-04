#include <stdio.h>
#include <stdlib.h>
#include "TestRunner.h"

#include "myassert.h"

std::vector<Test> tests;

void AddTest(std::string name, TestFunctionPtr function, std::string description)
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	tests.push_back(t);
}

// Returns the number of failures.
int RunTests(int numTimes)
{
	int numTestsPassed = 0;

	for(size_t i = 0; i < tests.size(); ++i)
	{
		printf("Testing '%s': ", tests[i].name.c_str());
		int numFails = 0;
		int numPasses = 0;
		try
		{
			for(int j = 0; j < numTimes; ++j)
			{
				tests[i].function();
				++numPasses;
			}
		}
		catch(const std::exception &e)
		{
			LOGE("FAILED: '%s' (%d passes)", e.what(), numPasses);
			++numFails;
		}

		if (numFails == 0)
		{
			LOGI("ok (%d passes)", numPasses);
			++numTestsPassed;
		}
	}

	int numFailures = (int)tests.size() - numTestsPassed;
	LOGI("Done. %d tests run. %d passed. %d failed.", (int)tests.size(), numTestsPassed, numFailures);
	return numFailures;
}

void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();

int main()
{
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();

	int numFailures = RunTests(100000);
	return numFailures; // exit code of 0 denotes a successful run.
}
