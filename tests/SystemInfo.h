#pragma once

#include <string>
#include <sstream>

std::string GetOSDisplayString();

/// @return The total amount of RAM on the system, in bytes.
unsigned long long GetTotalSystemPhysicalMemory();

/// @return A readable string of the processor as given by cpuid, like "AMD Turion(tm) X2 Dual-Core Mobile RM-74".
std::string GetProcessorBrandName();

/// @return "GenuineIntel" or "AuthenticAMD" or something similar.
std::string GetProcessorCPUIDString();

/// @return Technical information about the processor.
std::string GetProcessorExtendedCPUIDInfo();

/// @return The maximum number of threads the CPU can simultaneously accommodate.
int GetMaxSimultaneousThreads();

/// @return The clock speed of the given core, in MHz. This is nominal, not actual. It is read from registry.
unsigned long GetCPUSpeedFromRegistry(unsigned long dwCPU);

void CpuId(int *outInfo, int infoType);

#ifdef __EMSCRIPTEN__

class BrowserVersion
{
public:
	int v[5];

	BrowserVersion()
	{
		v[0] = v[1] = v[2] = v[3] = v[4] = 0;
	}

	BrowserVersion(int v1, int v2, int v3, int v4)
	{
		v[0] = v1;
		v[1] = v2;
		v[2] = v3;
		v[3] = v4;
		v[4] = 0;
	}

	explicit BrowserVersion(const char *str)
	{
		int n = sscanf(str, "%d.%d.%d.%d.%d", &v[0], &v[1], &v[2], &v[3], &v[4]);
		if (n < 5) v[4] = 0;
		if (n < 4) v[3] = 0;
		if (n < 3) v[2] = 0;
		if (n < 2) v[1] = 0;
		if (n < 1) v[0] = 0;
	}

#define LEXCOMPARE(a,b) if (a < b) return true; else if (a > b) return false;
	bool operator <(const BrowserVersion &rhs) const
	{
		LEXCOMPARE(v[0], rhs.v[0]);
		LEXCOMPARE(v[1], rhs.v[1]);
		LEXCOMPARE(v[2], rhs.v[2]);
		LEXCOMPARE(v[3], rhs.v[3]);
		LEXCOMPARE(v[4], rhs.v[4]);
		return false;
	}
	bool operator >(const BrowserVersion &rhs) const
	{
		return rhs < *this;
	}
	bool operator ==(const BrowserVersion &rhs) const
	{
		return !(rhs < *this || rhs > *this);
	}
	bool operator <=(const BrowserVersion &rhs) const
	{
		return *this < rhs || *this == rhs;
	}
	bool operator >=(const BrowserVersion &rhs) const
	{
		return rhs <= *this;
	}
	std::string ToString() const
	{
		std::stringstream ss;
		ss << v[0] << "." << v[1] << "." << v[2] << "." << v[3] << "." << v[4];
		return ss.str();
	}
};

BrowserVersion GetChromeVersion();
BrowserVersion GetOperaVersion();
BrowserVersion GetSafariVersion();

bool IsChromeBrowser();
bool IsChromeBrowserOnWin32();

bool IsOperaBrowser();
bool IsSafariBrowser();

inline bool IsIE11DeveloperPreview()
{
	static int isIE11DeveloperPreview = -1;
	if (isIE11DeveloperPreview == -1) // Only do this check once, and cache the result for performance.
	{
		std::string os = GetOSDisplayString();
		bool isIE11 = os.find("Trident") != os.npos && os.find("rv:11.0") != os.npos;
		isIE11DeveloperPreview = isIE11 ? 1 : 0;
	}
	return isIE11DeveloperPreview != 0;
}

extern "C"
{
	extern const char *browser_info();
}
inline std::string GetOSDisplayString() { return browser_info(); }

#else
inline bool IsIE11DeveloperPreview() { return false; }
#endif
