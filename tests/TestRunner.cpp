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

void RunTests(int numTimes)
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
			printf("FAILED: '%s' (%d passes)\n", e.what(), numPasses);
			++numFails;
		}

		if (numFails == 0)
		{
			printf("ok (%d passes)\n", numPasses);
			++numTestsPassed;
		}
	}

	printf("Done. %d tests run. %d passed. %d failed.\n", (int)tests.size(), numTestsPassed, (tests.size() - numTestsPassed));
}

void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();

int main()
{
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();

	RunTests(1000000);
}
