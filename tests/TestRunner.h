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
	TestFunctionPtr function;
};

extern volatile int globalPokedData;

void AddRandomizedTest(std::string name, TestFunctionPtr function, std::string description = "");
void AddTest(std::string name, TestFunctionPtr function, std::string description = "");
int RunTests(int numTimes);
/// Returns -2: no tests left to run, -1: failed, 0: success, 1: success with warnings.
int RunOneTest(int numTimes, int numTrials);
void PrintTestRunSummary();
