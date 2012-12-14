#pragma once

#include <vector>
#include <string>

#include "Algorithm/Random/LCG.h"
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
