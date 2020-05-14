#include "SystemInfo.h"

#if defined(LINUX) || defined(__APPLE__) || defined(ANDROID)

#include <string>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

std::string TrimLeft(std::string str)
{
	size_t pos = str.find_first_not_of(" \n\r\t");
	if (pos != std::string::npos)
		return str.substr(pos);
	else
		return "";
}

std::string TrimRight(std::string str)
{
	size_t pos = str.find_last_not_of(" \n\r\t");
	if (pos != std::string::npos)
	{
		str.erase(pos+1);
		return str;
	}
	else
		return "";
}

std::string Trim(std::string str)
{
	return TrimLeft(TrimRight(str));
}

// http://stackoverflow.com/questions/646241/c-run-a-system-command-and-get-output
std::string RunProcess(const char *cmd)
{
	FILE *fp = popen(cmd, "r");
	if (!fp)
		return std::string();

	std::stringstream ss;
	char str[1035];
	while(fgets(str, sizeof(str)-1, fp))
		ss << str;

	pclose(fp);

	return TrimRight(ss.str()); // Trim the end of the result to remove \n.
}

std::string FindLine(const std::string &inStr, const char *lineStart)
{
	int lineStartLen = strlen(lineStart);
	size_t idx = inStr.find(lineStart);
	if (idx == std::string::npos)
		return std::string();
	idx += lineStartLen;
	size_t lineEnd = inStr.find("\n", idx);
	if (lineEnd == std::string::npos)
		return inStr.substr(idx);
	else
		return inStr.substr(idx, lineEnd-idx);
}

#endif

#if defined(WIN32) && !defined(WIN8RT)

#include <windows.h>
#include <iphlpapi.h>

#include <tchar.h>
#include <stdio.h>

#ifdef MATH_ENABLE_STL_SUPPORT
#include <sstream>
#endif
#include <iostream>

#ifdef _MSC_VER
#include <intrin.h>

#pragma comment(lib, "User32.lib")
#pragma comment(lib, "IPHLPAPI.lib")
#elif defined (__GNUC__)
#include <cpuid.h>
#endif

std::string ReadRegistryKeyString(const char *registryKey, const char *registryValue)
{
	// Open the key
	HKEY hKey;
	if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, registryKey, 0, KEY_QUERY_VALUE, &hKey) != ERROR_SUCCESS)
		return 0;

	char str[256] = {};
	DWORD dwLen = 255;
	LONG ret = RegQueryValueExA(hKey, registryValue, NULL, NULL, (LPBYTE)str, &dwLen);
	RegCloseKey(hKey);

	if (ret == ERROR_SUCCESS)
		return str;
	else
		return std::string();
}

unsigned int ReadRegistryKeyU32(const char *registryKey, const char *registryValue)
{
	// Open the key
	HKEY hKey;
	if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, registryKey, 0, KEY_QUERY_VALUE, &hKey) != ERROR_SUCCESS)
		return 0;

	unsigned int value = 0;
	DWORD dwLen = 4;
	LONG ret = RegQueryValueExA(hKey, registryValue, NULL, NULL, (LPBYTE)&value, &dwLen);
	RegCloseKey(hKey);

	if (ret == ERROR_SUCCESS)
		return value;
	else
		return 0;
}

std::string GetOSDisplayString()
{
	std::string productName = ReadRegistryKeyString("SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\", "ProductName");
	std::string servicePack = ReadRegistryKeyString("SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\", "CSDVersion");
	std::string bitness = ReadRegistryKeyString("SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\", "BuildLabEx");
	if (bitness.find("amd64") != std::string::npos)
		bitness = "64-bit";
	else
		bitness = "32-bit";

	return productName + " " + bitness + " " + servicePack;
}

unsigned long long GetTotalSystemPhysicalMemory()
{
	MEMORYSTATUSEX statex;
	statex.dwLength = sizeof(statex);
	int ret = GlobalMemoryStatusEx(&statex);
	if (ret == 0)
		return 0;

	return (unsigned long long)statex.ullTotalPhys;
}

void CpuId(int *outInfo, int infoType)
{
#ifdef _MSC_VER
	__cpuid(outInfo, infoType);
#elif defined(__GNUC__)
	__get_cpuid((unsigned int)infoType, (unsigned int*)outInfo, (unsigned int*)outInfo+1, (unsigned int*)outInfo+2, (unsigned int*)outInfo+3);
#else
#warning CpuId not implemented for this compiler!
#endif
}

std::string GetProcessorBrandName()
{
	int CPUInfo[4] = {-1};

	// Calling __cpuid with 0x80000000 as the InfoType argument
	// gets the number of valid extended IDs.
	CpuId(CPUInfo, 0x80000000);
	unsigned int nExIds = CPUInfo[0];

	if (nExIds < 0x80000004)
		 return "Unknown";

	char CPUBrandString[0x40];
	memset(CPUBrandString, 0, sizeof(CPUBrandString));

	// Get the information associated with each extended ID.
	for (unsigned int i = 0x80000002; i <= nExIds && i <= 0x80000004; ++i)
	{
		CpuId(CPUInfo, i);

		// Interpret CPU brand string and cache information.
		if  (i == 0x80000002)
			memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
		else if  (i == 0x80000003)
			memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
		else if  (i == 0x80000004)
			memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
	}

	return CPUBrandString;
}

std::string GetProcessorCPUIDString()
{
	int CPUInfo[4] = {-1};

	// __cpuid with an InfoType argument of 0 returns the number of
	// valid Ids in CPUInfo[0] and the CPU identification string in
	// the other three array elements. The CPU identification string is
	// not in linear order. The code below arranges the information 
	// in a human readable form.
	CpuId(CPUInfo, 0);
	char CPUString[13] = {};
	memcpy(CPUString, CPUInfo+1, sizeof(int));
	memcpy(CPUString+4, CPUInfo+3, sizeof(int));
	memcpy(CPUString+8, CPUInfo+2, sizeof(int));

	return CPUString;
}

std::string GetProcessorExtendedCPUIDInfo()
{
	int CPUInfo[4] = {-1};

	// __cpuid with an InfoType argument of 0 returns the number of
	// valid Ids in CPUInfo[0] and the CPU identification string in
	// the other three array elements. The CPU identification string is
	// not in linear order. The code below arranges the information 
	// in a human readable form.
	CpuId(CPUInfo, 0);
	unsigned nIds = CPUInfo[0];
	char CPUString[13] = {};
	memcpy(CPUString, CPUInfo+1, sizeof(int));
	memcpy(CPUString+4, CPUInfo+3, sizeof(int));
	memcpy(CPUString+8, CPUInfo+2, sizeof(int));

	if (nIds == 0)
		return CPUString;

	CpuId(CPUInfo, 1);

	int nSteppingID = CPUInfo[0] & 0xf;
	int nModel = (CPUInfo[0] >> 4) & 0xf;
	int nFamily = (CPUInfo[0] >> 8) & 0xf;
	//	int nProcessorType = (CPUInfo[0] >> 12) & 0x3;
	int nExtendedmodel = (CPUInfo[0] >> 16) & 0xf;
	int nExtendedfamily = (CPUInfo[0] >> 20) & 0xff;
	//	int nBrandIndex = CPUInfo[1] & 0xff;

	std::stringstream ss;
	ss << CPUString << ", " << "Stepping: " << nSteppingID << ", Model: " << nModel <<
		", Family: " << nFamily << ", Ext.model: " << nExtendedmodel << ", Ext.family: " << nExtendedfamily << ".";

	return ss.str();
}

int GetMaxSimultaneousThreads()
{
	SYSTEM_INFO sysinfo;
	GetSystemInfo(&sysinfo);
	return sysinfo.dwNumberOfProcessors;
}

unsigned long GetCPUSpeedFromRegistry(unsigned long dwCPU)
{
	char str[256];
	sprintf(str, "HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\%lu\\", dwCPU);
	return (unsigned long)ReadRegistryKeyU32(str, "~MHz");
}

#elif defined(LINUX)

std::string GetOSDisplayString()
{
	return RunProcess("lsb_release -ds") + " " + RunProcess("uname -mrs");
}

unsigned long long GetTotalSystemPhysicalMemory()
{
	std::string r = RunProcess("cat /proc/meminfo");
	std::string memTotal = FindLine(r, "MemTotal:");
	int mem = 0;
	int n = sscanf(memTotal.c_str(), "%d", &mem);
	if (n == 1)
		return (unsigned long long)mem * 1024;
	else
		return 0;
}

std::string GetProcessorBrandName()
{
	std::string r = RunProcess("cat /proc/cpuinfo");
	std::string vendor_id = Trim(FindLine(FindLine(r, "vendor_id"),":"));
	// Slightly hacky mechanism to report generic ARM processors that don't have a vendor_id in their /proc/cpuinfo.
	if (vendor_id.empty() && !Trim(FindLine(GetProcessorCPUIDString(), "ARM")).empty())
		return "ARM";
	else
		return "Unknown";
}

std::string GetProcessorCPUIDString()
{
	std::string r = RunProcess("cat /proc/cpuinfo");
	return Trim(FindLine(FindLine(r, "model name"),":"));
}

std::string GetProcessorExtendedCPUIDInfo()
{
	std::string r = RunProcess("cat /proc/cpuinfo");
	std::string stepping = Trim(FindLine(FindLine(r, "stepping"),":"));
	std::string model = Trim(FindLine(FindLine(r, "model"),":"));
	std::string family = Trim(FindLine(FindLine(r, "cpu family"),":"));

	std::stringstream ss;
	ss << GetProcessorBrandName() << ", " << "Stepping: " << stepping << ", Model: " << model <<
		", Family: " << family;
	return ss.str();
}

int GetMaxSimultaneousThreads()
{
	std::string r = RunProcess("lscpu");
	r = TrimRight(FindLine(r, "CPU(s):"));
	int numCPUs = 0;
	int n = sscanf(r.c_str(), "%d", &numCPUs);
	return (n == 1) ? numCPUs : 0;
}

unsigned long GetCPUSpeedFromRegistry(unsigned long /*dwCPU*/)
{
	std::string r = RunProcess("lscpu");
	r = TrimRight(FindLine(r, "CPU MHz:"));
	int mhz = 0;
	int n = sscanf(r.c_str(), "%d", &mhz);
	if (!r.empty() && n == 1)
		return mhz;
	else
	{
		// lscpu did not contain clock speed. Perhaps we are on a Raspberry Pi: http://elinux.org/RPI_vcgencmd_usage
		r = RunProcess("vcgencmd get_config arm_freq");
		r = TrimRight(FindLine(r, "arm_freq="));
		n = sscanf(r.c_str(), "%d", &mhz);
		return mhz;
	}
}

#elif defined(__EMSCRIPTEN__)

#include <emscripten.h>

unsigned long long GetTotalSystemPhysicalMemory() { return (unsigned long long)emscripten_run_script_int("TOTAL_MEMORY"); }
std::string GetProcessorBrandName() { return "n/a"; } 
std::string GetProcessorCPUIDString() { return "n/a"; }
std::string GetProcessorExtendedCPUIDInfo() { return "n/a"; }
unsigned long GetCPUSpeedFromRegistry(unsigned long /*dwCPU*/) { return 1; }
int GetMaxSimultaneousThreads() { return 1; }

bool IsChromeBrowser()
{
	return GetChromeVersion().v[0] > 0;
}

bool IsChromeBrowserOnWin32()
{
	std::string os = GetOSDisplayString();

	return GetChromeVersion().v[0] > 0 && os.find("Win32") != std::string::npos;
}

bool IsOperaBrowser()
{
	std::string os = GetOSDisplayString();
	return GetOperaVersion().v[0] > 0;
}

bool IsSafariBrowser()
{
	std::string os = GetOSDisplayString();
	return GetSafariVersion().v[0] > 0;
}

BrowserVersion GetChromeVersion()
{
	std::string os = GetOSDisplayString();
	size_t idx = os.find("Chrome/");
	if (idx == std::string::npos)
		return BrowserVersion();
	return BrowserVersion(os.substr(idx+strlen("Chrome/")).c_str());
}

BrowserVersion GetOperaVersion()
{
	std::string os = GetOSDisplayString();
	size_t idx = os.find("Opera");
	if (idx == std::string::npos)
		return BrowserVersion();

	idx = os.find("Version/");
	if (idx == std::string::npos)
		return BrowserVersion();
	return BrowserVersion(os.substr(idx+strlen("Version/")).c_str());
}

BrowserVersion GetSafariVersion()
{
	std::string os = GetOSDisplayString();
	size_t idx = os.find("Safari");
	if (idx == std::string::npos)
		return BrowserVersion();

	idx = os.find("Version/");
	if (idx == std::string::npos)
		return BrowserVersion();
	return BrowserVersion(os.substr(idx+strlen("Version/")).c_str());
}

#elif defined(__APPLE__)

extern "C"
{
	char *IosSystemInformation();
}

std::string GetOSDisplayString()
{
#ifdef APPLE_IOS
	char *systemInfo = IosSystemInformation();
	std::string info = systemInfo;
	free(systemInfo);
	return info;
#else
	std::string uname = RunProcess("uname -mrs");

	// http://stackoverflow.com/questions/11072804/mac-os-x-10-8-replacement-for-gestalt-for-testing-os-version-at-runtime/11697362#11697362
	std::string systemVer = RunProcess("cat /System/Library/CoreServices/SystemVersion.plist");
	size_t idx = systemVer.find("<key>ProductVersion</key>");
	if (idx == std::string::npos)
		return uname;
	idx = systemVer.find("<string>", idx);
	if (idx == std::string::npos)
		return uname;
	idx += strlen("<string>");
	size_t endIdx = systemVer.find("</string>", idx);
	if (endIdx == std::string::npos)
		return uname;
	std::string marketingVersion = Trim(systemVer.substr(idx, endIdx-idx));

	uname += " Mac OS X " + marketingVersion;
	int majorVer = 0, minorVer = 0;
	int n = sscanf(marketingVersion.c_str(), "%d.%d", &majorVer, &minorVer);
	if (n != 2)
		return uname;
	switch (majorVer * 100 + minorVer)
	{
		case 1001: return uname + " Puma";
		case 1002: return uname + " Jaguar";
		case 1003: return uname + " Panther";
		case 1004: return uname + " Tiger";
		case 1005: return uname + " Leopard";
		case 1006: return uname + " Snow Leopard";
		case 1007: return uname + " Lion";
		case 1008: return uname + " Mountain Lion";
		default: return uname;
	}
#endif
}

#include <sys/types.h>
#include <sys/sysctl.h>

std::string sysctl_string(const char *sysctl_name)
{
	char str[128] = {};
	size_t size = sizeof(str)-1;
	sysctlbyname(sysctl_name, str, &size, NULL, 0);
	return str;
}

int sysctl_int32(const char *sysctl_name)
{
	int32_t val = 0;
	size_t size = sizeof(val);
	sysctlbyname(sysctl_name, &val, &size, NULL, 0);
	return (int)val;
}

int64_t sysctl_int64(const char *sysctl_name)
{
	int64_t val = 0;
	size_t size = sizeof(val);
	sysctlbyname(sysctl_name, &val, &size, NULL, 0);
	return val;
}

unsigned long long GetTotalSystemPhysicalMemory()
{
	return (unsigned long long)sysctl_int64("hw.memsize");
}

std::string GetProcessorBrandName()
{
#ifdef APPLE_IOS
	std::string m = sysctl_string("hw.machine");
	// http://stackoverflow.com/questions/18414032/how-to-identify-a-hw-machine-identifier-reliable
	if (m == "iPad1,1") return "iPad 1G Wi-Fi/GSM A1219/A1337";
	else if (m == "iPad2,1") return "iPad 2 Wi-Fi A1395";
	else if (m == "iPad2,2") return "iPad 2 GSM A1396";
	else if (m == "iPad2,3") return "iPad 2 CDMA A1397";
	else if (m == "iPad2,4") return "iPad 2 Wi-Fi Rev A A1395";
	else if (m == "iPad2,5") return "iPad mini 1G Wi-Fi A1432";
	else if (m == "iPad2,6") return "iPad mini 1G GSM A1454";
	else if (m == "iPad2,7") return "iPad mini 1G GSM+CDMA A1455";
	else if (m == "iPad3,1") return "iPad 3 Wi-Fi A1416";
	else if (m == "iPad3,2") return "iPad 3 GSM+CDMA A1403";
	else if (m == "iPad3,3") return "iPad 3 GSM A1430";
	else if (m == "iPad3,4") return "iPad 4 Wi-Fi A1458";
	else if (m == "iPad3,5") return "iPad 4 GSM A1459";
	else if (m == "iPad3,6") return "iPad 4 GSM+CDMA A1460";
	else if (m == "iPad4,1") return "iPad Air Wi‑Fi A1474";
	else if (m == "iPad4,2") return "iPad Air Cellular A1475";
	else if (m == "iPad4,3") return "iPad Air Wi-Fi+Cellular A1476";
	else if (m == "iPad4,4") return "iPad mini 2G Wi‑Fi A1489";
	else if (m == "iPad4,5") return "iPad mini 2G Cellular A1517";
	else if (m == "iPhone1,1") return "iPhone 2G GSM A1203";
	else if (m == "iPhone1,2") return "iPhone 3G GSM A1241/A13241";
	else if (m == "iPhone2,1") return "iPhone 3GS GSM A1303/A13251";
	else if (m == "iPhone3,1") return "iPhone 4 GSM A1332";
	else if (m == "iPhone3,2") return "iPhone 4 GSM Rev A -";
	else if (m == "iPhone3,3") return "iPhone 4 CDMA A1349";
	else if (m == "iPhone4,1") return "iPhone 4S GSM+CDMA A1387/A14311";
	else if (m == "iPhone5,1") return "iPhone 5 GSM A1428";
	else if (m == "iPhone5,2") return "iPhone 5 GSM+CDMA A1429/A14421";
	else if (m == "iPhone5,3") return "iPhone 5C GSM A1456/A1532";
	else if (m == "iPhone5,4") return "iPhone 5C Global A1507/A1516/A1526/A1529";
	else if (m == "iPhone6,1") return "iPhone 5S GSM A1433/A1533";
	else if (m == "iPhone6,2") return "iPhone 5S Global A1457/A1518/A1528/A1530";
	else if (m == "iPod1,1") return "iPod touch 1G - A1213";
	else if (m == "iPod2,1") return "iPod touch 2G - A1288";
	else if (m == "iPod3,1") return "iPod touch 3G - A1318";
	else if (m == "iPod4,1") return "iPod touch 4G - A1367";
	else if (m == "iPod5,1") return "iPod touch 5G - A1421/A1509";

#ifndef NDEBUG
	static bool unknownHardwareWarned = false;
	if (!unknownHardwareWarned)
	{
		unknownHardwareWarned = true;
		fprintf(stderr, "Your device has an unidentified hw.machine identifier: %s. Please report it to https://github.com/juj/MathGeoLib/issues . Thanks!\n", m.c_str());
	}
#endif
	return m;
#else
	return sysctl_string("machdep.cpu.vendor");
#endif
}

std::string GetProcessorCPUIDString()
{
#ifdef APPLE_IOS
	return sysctl_string("hw.model");
#else
	return sysctl_string("machdep.cpu.brand_string");
#endif
}

std::string GetProcessorExtendedCPUIDInfo()
{
	char str[1024];
#ifdef APPLE_IOS
	sprintf(str, "Model: %s, Family: %d, Type: %d, Subtype: %d, 64-bit capable: %d, HW fp support: %d, NEON support: %d, NEON hpfp: %d, VFP shortvector: %d",
			sysctl_string("hw.model").c_str(), sysctl_int32("hw.cpufamily"), sysctl_int32("hw.cputype"), sysctl_int32("hw.cpusubtype"), sysctl_int32("hw.cpu64bit_capable"),
			sysctl_int32("hw.optional.floatingpoint"), sysctl_int32("hw.optional.neon"), sysctl_int32("hw.optional.neon_hpfp"), sysctl_int32("hw.optional.vfp_shortvector"));
#else
	sprintf(str, "%s, Stepping: %d, Model: %d, Family: %d, Ext.model: %d, Ext.family: %d.", GetProcessorCPUIDString().c_str(), sysctl_int32("machdep.cpu.stepping"), sysctl_int32("machdep.cpu.model"), sysctl_int32("machdep.cpu.family"), sysctl_int32("machdep.cpu.extmodel"), sysctl_int32("machdep.cpu.extfamily"));
#endif
	return str;
}

unsigned long GetCPUSpeedFromRegistry(unsigned long /*dwCPU*/)
{
	int64_t freq = sysctl_int64("hw.cpufrequency");
#ifdef APPLE_IOS
	if (freq == 0)
	{
		// Apple has stopped from supporting querying the CPU clock speed in new iOS versions: http://stackoverflow.com/questions/8291357/cannot-get-cpu-frequency-in-ios-5
		// Therefore if we fail to get the frequency in above way, make a guess based on the HW version.
		// The guess database is taken from http://en.wikipedia.org/wiki/List_of_iOS_devices
		std::string m = sysctl_string("hw.machine");
		if (m == "iPad1,1") freq = 1000 * 1000 * 1000;
		else if (m == "iPad2,1") freq = 800 * 1000 * 1000;
		else if (m == "iPad2,2") freq = 800 * 1000 * 1000;
		else if (m == "iPad2,3") freq = 800 * 1000 * 1000;
		else if (m == "iPad2,4") freq = 800 * 1000 * 1000;
		else if (m == "iPad2,5") freq = 1000 * 1000 * 1000;
		else if (m == "iPad2,6") freq = 1000 * 1000 * 1000;
		else if (m == "iPad2,7") freq = 1000 * 1000 * 1000;
		else if (m == "iPad3,1") freq = 1000 * 1000 * 1000;
		else if (m == "iPad3,2") freq = 1000 * 1000 * 1000;
		else if (m == "iPad3,3") freq = 1000 * 1000 * 1000;
		else if (m == "iPad3,4") freq = 1400 * 1000 * 1000;
		else if (m == "iPad3,5") freq = 1400 * 1000 * 1000;
		else if (m == "iPad3,6") freq = 1400 * 1000 * 1000;
		else if (m == "iPad4,1") freq = 1400 * 1000 * 1000;
		else if (m == "iPad4,2") freq = 1400 * 1000 * 1000;
		else if (m == "iPad4,3") freq = 1400 * 1000 * 1000;
		else if (m == "iPad4,4") freq = 1300 * 1000 * 1000;
		else if (m == "iPad4,5") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone1,1") freq = 412 * 1000 * 1000;
		else if (m == "iPhone1,2") freq = 412 * 1000 * 1000;
		else if (m == "iPhone2,1") freq = 600 * 1000 * 1000;
		else if (m == "iPhone3,1") freq = 800 * 1000 * 1000;
		else if (m == "iPhone3,2") freq = 800 * 1000 * 1000;
		else if (m == "iPhone3,3") freq = 800 * 1000 * 1000;
		else if (m == "iPhone4,1") freq = 800 * 1000 * 1000;
		else if (m == "iPhone5,1") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone5,2") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone5,3") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone5,4") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone6,1") freq = 1300 * 1000 * 1000;
		else if (m == "iPhone6,2") freq = 1300 * 1000 * 1000;
		else if (m == "iPod1,1") freq = 400 * 1000 * 1000;
		else if (m == "iPod2,1") freq = 533 * 1000 * 1000;
		else if (m == "iPod3,1") freq = 600 * 1000 * 1000;
		else if (m == "iPod4,1") freq = 800 * 1000 * 1000;
		else if (m == "iPod5,1") freq = 800 * 1000 * 1000;
	}

#ifndef NDEBUG
	static bool unknownHardwareWarned = false;
	if (freq == 0 && !unknownHardwareWarned)
	{
		unknownHardwareWarned = true;
		fprintf(stderr, "Your device has an unidentified cpu frequency! Please report it to https://github.com/juj/MathGeoLib/issues . Thanks!\n");
	}
#endif
#endif
	return (unsigned long)(freq / 1000 / 1000);
}

// Returns the maximum number of threads the CPU can simultaneously accommodate.
// E.g. for dualcore hyperthreaded Intel CPUs, this returns 4.
int GetMaxSimultaneousThreads()
{
#ifdef APPLE_IOS
	return (int)sysctl_int32("hw.ncpu");
#else
	return (int)sysctl_int32("machdep.cpu.thread_count");
#endif
}

#elif defined(ANDROID)

unsigned long long GetTotalSystemPhysicalMemory()
{
	std::string r = RunProcess("cat /proc/meminfo");
	std::string memTotal = FindLine(r, "MemTotal:");
	int mem = 0;
	int n = sscanf(memTotal.c_str(), "%d", &mem);
	if (n == 1)
		return (unsigned long long)mem * 1024;
	else
		return 0;
}

std::string GetOSDisplayString()
{
	std::string ver = Trim(RunProcess("getprop ro.build.version.release"));
	std::string apiLevel = Trim(RunProcess("getprop ro.build.version.sdk"));
	if (apiLevel.empty())
		apiLevel = Trim(RunProcess("getprop ro.build.version.sdk_int"));

	std::string os;
	if (!ver.empty())
		os = "Android " + ver + " ";
	if (!apiLevel.empty())
		os += "SDK API Level " + apiLevel + " ";
	os += Trim(RunProcess("getprop ro.build.description"));
	return os;
}

std::string GetProcessorBrandName()
{
	std::string r = RunProcess("cat /proc/cpuinfo");
	return Trim(FindLine(FindLine(r, "Processor"),":"));
}

std::string GetProcessorCPUIDString()
{
	// Note: This is actually HW identifier, not CPU ID.
	std::string manufacturer = RunProcess("getprop ro.product.manufacturer");
	std::string brand = RunProcess("getprop ro.product.brand");
	std::string model = RunProcess("getprop ro.product.model");
	std::string board = RunProcess("getprop ro.product.board");
	std::string device = RunProcess("getprop ro.product.device");
	std::string name = RunProcess("getprop ro.product.name");
	return manufacturer + " " + brand + " " + model + " " + board + " " + device + " " + name;
}

std::string GetProcessorExtendedCPUIDInfo()
{
	std::string r = RunProcess("cat /proc/cpuinfo");
	std::string implementer = Trim(FindLine(FindLine(r, "CPU implementer"),":"));
	std::string arch = Trim(FindLine(FindLine(r, "CPU architecture"),":"));
	std::string variant = Trim(FindLine(FindLine(r, "CPU variant"),":"));
	std::string part = Trim(FindLine(FindLine(r, "CPU part"),":"));
	std::string rev = Trim(FindLine(FindLine(r, "CPU revision"),":"));
	std::string hw = Trim(FindLine(FindLine(r, "Hardware"),":"));

	std::stringstream ss;
	ss << "Hardware: " << hw << ", CPU implementer: " << implementer << ", arch: " << arch << ", variant: " << variant 
		<< ", part: " << part << ", revision: " << rev;
	return ss.str();
}

unsigned long GetCPUSpeedFromRegistry(unsigned long dwCPU)
{
	std::stringstream ss;
	ss << "cat /sys/devices/system/cpu/cpu" << dwCPU << "/cpufreq/cpuinfo_max_freq";
	std::string r = RunProcess(ss.str().c_str());
	int freq = 0;
	int n = sscanf(r.c_str(), "%d", &freq);
	if (n == 1)
		return freq / 1000; // cpuinfo_max_freq seems to be in kHz. Output MHz.
	else
		return 0;
}

int GetMaxSimultaneousThreads()
{
	std::string r = RunProcess("cat /sys/devices/system/cpu/present");
	int nCoresMin = 0, nCoresMax = 0;
	int n = sscanf(r.c_str(), "%d-%d", &nCoresMin, &nCoresMax);
	if (n == 2)
		return nCoresMax - nCoresMin + 1; // The min-max numbers are indices to cpu cores, so +1 for the count.
	else
		return 1;
}

#else

#ifdef _MSC_VER
#pragma warning("SystemInfo.cpp not implemented for the current platform!")
#else
#warning SystemInfo.cpp not implemented for the current platform!
#endif

std::string GetOSDisplayString() { return ""; }
unsigned long long GetTotalSystemPhysicalMemory() { return 0; }
std::string GetProcessorBrandName() { return ""; }
std::string GetProcessorCPUIDString() { return ""; }
std::string GetProcessorExtendedCPUIDInfo() { return ""; }
unsigned long GetCPUSpeedFromRegistry(unsigned long /*dwCPU*/) { return 0; }
int GetMaxSimultaneousThreads() { return 0; }

#endif
