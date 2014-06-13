#pragma once

#include <string>

// Sometimes we want an explicit function call to occur, especially when related to operating with callstack fetching.
#ifdef _MSC_VER
#define NOINLINE __declspec(noinline)
#else
#define NOINLINE __attribute__((noinline))
#endif

std::string GetCallstack(const char *indent = "", const char *ignoreFilter = 0);
