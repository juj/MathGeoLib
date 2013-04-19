#pragma once

#include <vector>
#include <string>

#include "Algorithm/Random/LCG.h"

#define SCALE 1e2f

#define GUARDBAND 1e-2f

extern LCG rng;

typedef void (*TestFunctionPtr)();
struct Test
{
	std::string name;
	std::string description;
	bool isRandomized;
	/// If true, this function should not be trialled multiple times (for performance benchmarks), but should
	/// be called only exactly once. Used mostly for very slow tests that would take too long to run multiple times.
	bool runOnlyOnce;
	TestFunctionPtr function;
};

extern volatile int globalPokedData;

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string description = "");
void AddTest(std::string name, TestFunctionPtr function, std::string description = "", bool runOnlyOnce = false);
int RunTests(int numTimes);
/// Returns -2: no tests left to run, -1: failed, 0: success, 1: success with warnings.
int RunOneTest(int numTimes, int numTrials);
void PrintTestRunSummary();

class AddTestOp
{
public:
	AddTestOp(const char *name, const char *description, bool isRandomized, bool runOnlyOnce, TestFunctionPtr function)
	{
		if (isRandomized)
			AddRandomizedTest(name, function, description);
		else
			AddTest(name, function, description, runOnlyOnce);
	}
};

#ifdef MATH_TESTS_EXECUTABLE

#define TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, "", false, false, TestFunc_##name); \
	void TestFunc_##name()

#define RANDOMIZED_TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, "", true, false, TestFunc_##name); \
	void TestFunc_##name()

#define UNIQUE_TEST(name) \
	void TestFunc_##name(); \
	AddTestOp addtestop_##name(#name, "", false, true, TestFunc_##name); \
	void TestFunc_##name()

#else

// Not running tests - specfiy the test functions, but don't add them at app startup time (they should be DCEd).
#define TEST(name) void TestFunc_##name()
#define RANDOMIZED_TEST(name) void TestFunc_##name()
#define UNIQUE_TEST(name) void TestFunc_##name()

#endif
