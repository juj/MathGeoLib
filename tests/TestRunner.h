#pragma once

#include <vector>
#include <string>

typedef void (*TestFunctionPtr)();
struct Test
{
	std::string name;
	std::string description;
	TestFunctionPtr function;
};

void AddTest(std::string name, TestFunctionPtr function, std::string description = "");
int RunTests(int numTimes);
