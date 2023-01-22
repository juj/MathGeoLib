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
	mgl_assert(GetOSDisplayString().length() > 0);
}

UNIQUE_TEST(GetTotalSystemPhysicalMemory)
{
	int mbytes = (int)(GetTotalSystemPhysicalMemory() / 1024 / 1024);
	LOGI("Physical Memory: %d MBytes.", mbytes);
	mgl_assert(mbytes > 0);
	mgl_assert(mbytes < 128 * 1024); // Arbitrary cap on large values.
}

UNIQUE_TEST(GetProcessorBrandName)
{
	LOGI("Processor: %s", GetProcessorBrandName().c_str());
	mgl_assert(GetProcessorBrandName().length() > 0);
}

UNIQUE_TEST(GetProcessorCPUIDString)
{
	LOGI("CPUID: %s", GetProcessorCPUIDString().c_str());
	mgl_assert(GetProcessorCPUIDString().length() > 0);
}

UNIQUE_TEST(GetProcessorExtendedCPUIDInfo)
{
	LOGI("Extended CPU Info: %s", GetProcessorExtendedCPUIDInfo().c_str());
	mgl_assert(GetProcessorExtendedCPUIDInfo().length() > 0);
}

UNIQUE_TEST(GetCPUSpeedFromRegistry)
{
	LOGI("CPU Speed: %d MHz", (int)GetCPUSpeedFromRegistry(0));
	mgl_assert((int)GetCPUSpeedFromRegistry(0) > 0);
	mgl_assert((int)GetCPUSpeedFromRegistry(0) < 32 * 1024); // Arbitrary cap on large values.
}

UNIQUE_TEST(GetMaxSimultaneousThreads)
{
	LOGI("Maximum number of simultaneous threads (number of logical cores): %d", GetMaxSimultaneousThreads());
	mgl_assert(GetMaxSimultaneousThreads() > 0);
}

#ifdef __EMSCRIPTEN__

UNIQUE_TEST(DetectBrowser)
{
	LOGI("Chrome version: %s", GetChromeVersion().ToString().c_str());
	LOGI("Opera version: %s", GetOperaVersion().ToString().c_str());
	LOGI("Safari version: %s", GetSafariVersion().ToString().c_str());
	LOGI("IsChromeBrowser: %s", IsChromeBrowser()?"true":"false");
	LOGI("IsChromeBrowserOnWin32: %s", IsChromeBrowserOnWin32()?"true":"false");
	LOGI("IsOperaBrowser: %s", IsOperaBrowser()?"true":"false");
	LOGI("IsSafariBrowser: %s", IsSafariBrowser()?"true":"false");
}

#endif
