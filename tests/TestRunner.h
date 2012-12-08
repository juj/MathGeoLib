#pragma once

#include <vector>
#include <string>

typedef void (*TestFunctionPtr)();
struct Test
{
	std::string name;
	std::string description;
	bool isRandomized;
	TestFunctionPtr function;
};

extern volatile int globalPokedData;

void AddTest(std::string name, TestFunctionPtr function, bool isRandomized = true, std::string description = "");
int RunTests(int numTimes);
