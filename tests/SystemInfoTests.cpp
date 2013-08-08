#include <stdio.h>
#include <stdlib.h>
#include <locale.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "SystemInfo.h"
#include "TestRunner.h"

UNIQUE_TEST(GetOSDisplayString)
{
	LOGI("OS: %s", GetOSDisplayString().c_str());
	assert(GetOSDisplayString().length() > 0);
}

UNIQUE_TEST(GetTotalSystemPhysicalMemory)
{
	int mbytes = (int)(GetTotalSystemPhysicalMemory() / 1024 / 1024);
	LOGI("Physical Memory: %d MBytes.", mbytes);
	assert(mbytes > 0);
	assert(mbytes < 128 * 1024); // Arbitrary cap on large values.
}

UNIQUE_TEST(GetProcessorBrandName)
{
	LOGI("Processor: %s", GetProcessorBrandName().c_str());
	assert(GetProcessorBrandName().length() > 0);
}

UNIQUE_TEST(GetProcessorCPUIDString)
{
	LOGI("CPUID: %s", GetProcessorCPUIDString().c_str());
	assert(GetProcessorCPUIDString().length() > 0);
}

UNIQUE_TEST(GetProcessorExtendedCPUIDInfo)
{
	LOGI("Extended CPU Info: %s", GetProcessorExtendedCPUIDInfo().c_str());
	assert(GetProcessorExtendedCPUIDInfo().length() > 0);
}

UNIQUE_TEST(GetCPUSpeedFromRegistry)
{
	LOGI("CPU Speed: %d MHz", (int)GetCPUSpeedFromRegistry(0));
	assert((int)GetCPUSpeedFromRegistry(0) > 0);
	assert((int)GetCPUSpeedFromRegistry(0) < 32 * 1024); // Arbitrary cap on large values.
}
