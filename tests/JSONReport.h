#include "SystemInfo.h"
#include "../src/Math/myassert.h"

class JSONReport
{
public:
	JSONReport()
	:handle(0)
	{
	}

	~JSONReport()
	{
		Finish();
	}

	static std::string CompilerIdentifier() // http://sourceforge.net/p/predef/wiki/Compilers/
	{
#ifdef _MSC_VER
		char fullVer[256];
		const char *majorVerString = "VS?";
#if _MSC_VER < 1310 /* VS2003 */
		return "Old unsupported VS";
#elif _MSC_VER < 1400
		majorVerString = "VS2003";
#elif _MSC_VER < 1500
		majorVerString = "VS2005";
#elif _MSC_VER < 1600
		majorVerString = "VS2008";
#elif _MSC_VER < 1700
		majorVerString = "VS2010";
#elif _MSC_VER < 1800
		majorVerString = "VS2012";
#else
		majorVerString = "VS2012 (or newer?)";
#endif
		return majorVerString;
	/*
		int majorVer = _MSC_FULL_VER / 100000000;
		int minorVer = (_MSC_FULL_VER / 100000) % 100;
		int patch = _MSC_FULL_VER % 100000;
#if _MSC_VER < 1500)
		if (majorVer == 14)
			majorVerString = "VS2005";
		else if (majorVer == 15)
			majorVerString = "VS2008";
		else if (majorVer == 16)
			majorVerString = "VS2010";
		else if (majorVer == 17)
			majorVerString = "VS2012";
		sprintf(fullVer, "%s (%02d.%02d.%05d)", majorVerString, majorVer, minorVer, patch);

		return fullVer;
#endif*/
#elif defined(EMSCRIPTEN)
		char str[256];
		sprintf(str, "Emscripten Clang %s", __clang_version__);
		return str;
#elif defined(__clang__)
		char str[256];
		sprintf(str, "Clang %s", __clang_version__);
		return str;
#elif defined(__GNUC__)
		char str[256];
		const char *prefix = "";
#if defined(__MINGW32__)
		prefix = "MinGW ";
#endif
		sprintf(str, "%sGCC %d.%d.%d", prefix, __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
		return str;
#else
		return "unknown compiler";
#endif
	}

	static std::string ArchitectureIdentifier()
	{
#if defined(__arm__) || defined(_M_ARM)
		return "ARM";
#elif defined(_M_X64) || defined(__x86_64__)
		return "x86-64bit";
#elif defined(__i386__) || defined(_X86_) || defined(_M_IX86)
		return "x86-32bit";
#else
		return "unknown arch";
#endif
	}
	
	static std::string OSIdentifier()
	{
#ifdef __ANDROID__
		return "Android";
#elif defined(__linux__) || defined(LINUX)
		return "Linux";
#elif defined(APPLE_IOS)
		return "iOS";
#elif defined(APPLE_OSX) || defined(__APPLE__)
		return "OSX";
#elif defined(__unix__)
		return "Unix";
#elif defined(WIN32)
#ifdef WIN8PHONE
		return "Win8Phone";
#elif defined(WIN8)
		return "Win8";
#else
		std::string os = GetOSDisplayString();
		if (os.find("Windows 7") != os.npos)
			return "Windows 7";
		if (os.find("Windows 8") != os.npos)
			return "Windows 8";
		if (os.find("Windows XP") != os.npos)
			return "Windows XP";
		return os;
#endif
#elif defined(EMSCRIPTEN)
		return "Emscripten";
#else
		return "unknown OS";
#endif
	}
	
	static std::string SIMDIdentifier()
	{
		std::string simd;
#ifdef MATH_AVX
		simd = "AVX";
#elif defined(MATH_SSE41)
		simd = "SSE4.1";
#elif defined(MATH_SSE3)
		simd = "SSE3";
#elif defined(MATH_SSE2)
		simd = "SSE2";
#elif defined(MATH_SSE)
		simd = "SSE1";
#elif defined(MATH_NEON)
		simd = "NEON";
#else
		return "No SIMD";
#endif

#ifdef MATH_AUTOMATIC_SSE
		return simd + "(auto)";
#else
		return simd + "(manual)";
#endif
	}

	static std::string ConfigIdentifier()
	{
#ifdef _DEBUG
		return "Debug";
#elif defined(RELEASE)
		return "Release";
#elif defined(RELWITHDEBINFO)
		return "RelWithDebInfo";
#else
		return "";
#endif
	}

	static std::string BuildDescription()
	{
		return OSIdentifier() + " " + CompilerIdentifier() + " " + ArchitectureIdentifier() + " " + SIMDIdentifier() + " " + ConfigIdentifier();
	}

	void Create(const char *filename)
	{
		Finish(); // If we happened to have an old one..

		handle = fopen(filename, "w");
		if (!handle)
		{
			LOGE("Failed to open file '%s'!", filename);
			return;
		}
		fputs("{\n", handle);
		fprintf(handle, "\t\"build\": \"%s\",\n", BuildDescription().c_str());
		fprintf(handle, "\t\"os\": \"%s\",\n", GetOSDisplayString().c_str());
		fprintf(handle, "\t\"ram\": %d,\n", (int)(GetTotalSystemPhysicalMemory()/1024/1024));
		fprintf(handle, "\t\"cpuBrandName\": \"%s\",\n", GetProcessorBrandName().c_str());
		fprintf(handle, "\t\"cpuid\": \"%s\",\n",GetProcessorCPUIDString().c_str());
		fprintf(handle, "\t\"cpuidInfo\": \"%s\",\n",GetProcessorExtendedCPUIDInfo().c_str());
		///\todo This doesn't work on Core i3/i5/i7, so leave it out.
//		fprintf(handle, "\t\"numCores\": %d,\n", GetNumberOfLogicalCPUCores());
		fprintf(handle, "\t\"cpuSpeed\": %d,\n", (int)GetCPUSpeedFromRegistry(0));

		fprintf(handle, "\t\"defines\": {\n");
#ifdef WINVER
		fprintf(handle, "\t\t\"WINVER\": \"0x%X\",\n", WINVER);
#endif
#ifdef WIN8
		fprintf(handle, "\t\t\"WIN8\": true,\n");
#endif
#ifdef MATH_ENABLE_WINXP_SUPPORT
		fprintf(handle, "\t\t\"MATH_ENABLE_WINXP_SUPPORT\": true,\n");
#endif
#ifdef _DEBUG
		fprintf(handle, "\t\t\"_DEBUG\": true,\n");
#endif
#ifdef MATH_SILENT_ASSUME
		fprintf(handle, "\t\t\"MATH_SILENT_ASSUME\": true,\n");
#endif
#ifdef MATH_ENABLE_INSECURE_OPTIMIZATIONS
		fprintf(handle, "\t\t\"MATH_ENABLE_INSECURE_OPTIMIZATIONS\": true,\n");
#endif
#ifdef MATH_ASSERT_CORRECTNESS
		fprintf(handle, "\t\t\"MATH_ASSERT_CORRECTNESS\": true,\n");
#endif
#ifdef FAIL_USING_EXCEPTIONS
		fprintf(handle, "\t\t\"FAIL_USING_EXCEPTIONS\": true,\n");
#endif
#ifdef MATH_ENABLE_STL_SUPPORT
		fprintf(handle, "\t\t\"MATH_ENABLE_STL_SUPPORT\": true,\n");
#endif
#ifdef ANDROID
		fprintf(handle, "\t\t\"ANDROID\": true,\n");
#endif
#ifdef __ARM_ARCH_7A__
		fprintf(handle, "\t\t\"__ARM_ARCH_7A__\": true,\n");
#endif
#ifdef WIN8RT
		fprintf(handle, "\t\t\"WIN8RT\": true,\n");
#endif
#ifdef _M_ARM
		fprintf(handle, "\t\t\"_M_ARM\": true,\n");
#endif
#ifdef MATH_AVX
		fprintf(handle, "\t\t\"MATH_AVX\": true,\n");
#endif
#ifdef MATH_SSE41
		fprintf(handle, "\t\t\"MATH_SSE41\": true,\n");
#endif
#ifdef MATH_SSE3
		fprintf(handle, "\t\t\"MATH_SSE3\": true,\n");
#endif
#ifdef MATH_SSE2
		fprintf(handle, "\t\t\"MATH_SSE2\": true,\n");
#endif
#ifdef MATH_SSE
		fprintf(handle, "\t\t\"MATH_SSE\": true,\n");
#endif
#ifdef MATH_NEON
		fprintf(handle, "\t\t\"MATH_NEON\": true,\n");
#endif
#ifdef _MSC_VER
		fprintf(handle, "\t\t\"_MSC_VER\": %d,\n", _MSC_VER);
#endif
#ifdef __clang__
		fprintf(handle, "\t\t\"__clang__\": true,\n");
#endif
#ifdef __GNUC__
		fprintf(handle, "\t\t\"__GNUC__\": true,\n");
#endif
#ifdef MATH_SIMD
		fprintf(handle, "\t\t\"MATH_SIMD\": true,\n");
#endif
#ifdef MATH_AUTOMATIC_SSE
		fprintf(handle, "\t\t\"MATH_AUTOMATIC_SSE\": true,\n");
#endif
		fprintf(handle, "\t\t\"dummy_swallowcomma\": true\n");
		fprintf(handle, "\t\t},\n");

		fputs("\t\"results\": [\n", handle);
	}

	static double TicksToMicroseconds(double ticks)
	{
		return ticks * 1000000.0 / Clock::TicksPerSec();
	}

	void Report(const Test &t)
	{
		assert(handle);
		if (!handle)
			return;

		fprintf(handle, "\t\t{ \"type\": \"%s\", \"name\": \"%s\", \"numTimesRun\": %d, \"numTrialsPerRun\": %d, \"numPasses\": %d, \"numFails\": %d, \"fastestTime\": %f, \"averageTime\": %f, \"slowestTime\": %f, \"fastestCycles\": %f },\n", 
			t.isBenchmark ? "benchmark" : "test", t.name.c_str(), t.numTimesRun, t.numTrialsPerRun, t.numPasses, t.numFails, 
			TicksToMicroseconds(t.fastestTime), TicksToMicroseconds(t.averageTime), TicksToMicroseconds(t.worstTime), t.fastestCycles);
		fflush(handle);
	}

	void Finish()
	{
		if (handle)
		{
			fputs("\t\t{ \"dummy_commaswallow\": true }\n", handle);
			fputs("\t]\n}\n", handle);
			fclose(handle);
		}
		handle = 0;
	}

private:
	FILE *handle;

	void operator =(const JSONReport &);
	JSONReport(const JSONReport &);
};
