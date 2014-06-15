#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Math/Callstack.h"

std::string NOINLINE CapturingCallstacksWorks()
{
	std::string callstack = GetCallstack();
	return callstack;
}

#if (defined(WIN32) && defined(_MSC_VER) || defined(__APPLE__) || defined(LINUX))
UNIQUE_TEST(CaptureCallstack)
{
	std::string callstack = CapturingCallstacksWorks();
	LOGI("-- begin callstack -- ");
	printf("%s", callstack.c_str());
	LOGI("-- end callstack -- ");
	assert1(callstack.find("CapturingCallstacksWorks") != std::string::npos, callstack);
}
#endif
