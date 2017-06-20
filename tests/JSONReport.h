#include <list>
#include <string>
#include <algorithm>

#include "SystemInfo.h"
#include "../src/Math/myassert.h"
#include "../src/Math/assume.h"

class JSONReport
{
private:
	// If a file output has not been opened, all reports are queued to this structure instead,
	// and logged to screen at quit time. This allows extracting the results from stdout
	// to a report file.
	std::list<std::string> reports;
	char temp_str[512];
	bool isOpen;

public:
	JSONReport()
	:isOpen(false),
	handle(0)
	{
	}

	~JSONReport()
	{
		Finish();
	}

	static std::string CompilerIdentifier() // http://sourceforge.net/p/predef/wiki/Compilers/
	{
		char str[2048];
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:6326) // C6326: potential comparison of a constant with another constant http://msdn.microsoft.com/en-us/library/tz3zbzw6.aspx
		// http://stackoverflow.com/questions/70013/how-to-detect-if-im-compiling-code-with-visual-studio-2008
		switch(_MSC_VER)
		{
			case 1100: return "MSVC 5.0";
			case 1200: return "MSVC 6.0";
			case 1300: return "MSVC 7.0";
			case 1310: return "VS2003";
			case 1400: return "VS2005";
			case 1500: return "VS2008";
			case 1600: return "VS2010";
			case 1700: return "VS2012";
			case 1800: return "VS2013";
			default:
			{
				sprintf(str, "MSVC ver. %d", _MSC_VER);
				return str;
			}
		}
#pragma warning(pop)
#elif defined(__EMSCRIPTEN__)
		std::string browserVersion = GetOSDisplayString();
		sprintf(str, "%s, Clang %s", browserVersion.c_str(), __clang_version__);
		return str;
#elif defined(__clang__)
		sprintf(str, "Clang %s", __clang_version__);
		return str;
#elif defined(__GNUC__)
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
		return "";
#endif
	}
	
	static std::string OSIdentifier()
	{
#ifdef __ANDROID__
		return "Android";
#elif defined(__native_client__)
		return "Native Client";
#elif defined(__linux__) || defined(LINUX)
		return "Linux";
#elif defined(APPLE_IOS)
		return "iOS";
#elif defined(APPLE_OSX) || defined(__APPLE__)
		return "OSX";
#elif defined(__EMSCRIPTEN__)
		return "Emscripten";
#elif defined(__FLASHPLAYER__)
		return "Flash";
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
#endif

#if defined(MATH_SSE) || defined(MATH_NEON)
#ifdef MATH_AUTOMATIC_SSE
		return simd + "(auto)";
#else
		return simd + "(manual)";
#endif
#elif defined(__native_client__) || defined(__EMSCRIPTEN__) || defined(__FLASHPLAYER__)
		return ""; // These platforms don't have SIMD option, so don't report the absence of it either.
#else
		return "No SIMD";
#endif
	}

	static std::string ConfigIdentifier()
	{
#ifdef _DEBUG
		return "Debug";
#else
		return "";
#endif
	}

	static std::string BuildDescription()
	{
		return OSIdentifier() + " " + CompilerIdentifier() + " " + ArchitectureIdentifier() + " " + SIMDIdentifier() + " " + ConfigIdentifier();
	}

#define LOG_WRITE(...) \
	MULTI_LINE_MACRO_BEGIN \
		sprintf(temp_str, __VA_ARGS__); \
		if (handle) \
			fputs(temp_str, handle); \
		else \
		{ \
			std::string s = temp_str; \
			s.erase(s.find_last_not_of(" \n\r\t")+1); \
			reports.push_back(s); \
		} \
	MULTI_LINE_MACRO_END

	void Create(const char *filename)
	{
		Finish(); // If we happened to have an old one..

#if !defined(ANDROID) && !defined(__EMSCRIPTEN__) && !defined(WIN8PHONE) && !defined(APPLE_IOS) && !defined(NACL) && !defined(NPAPI) // Virtual FS archs output to screen.
		handle = fopen(filename, "w");
		if (!handle)
			LOGE("Failed to open file '%s'!", filename);
#else
		MARK_UNUSED(filename);
#endif

		LOG_WRITE("{\n");
		LOG_WRITE("\t\"build\": \"%s\",\n", BuildDescription().c_str());
		LOG_WRITE("\t\"os\": \"%s\",\n", GetOSDisplayString().c_str());
		LOG_WRITE("\t\"ram\": %d,\n", (int)(GetTotalSystemPhysicalMemory()/1024/1024));
		LOG_WRITE("\t\"cpuBrandName\": \"%s\",\n", GetProcessorBrandName().c_str());
		LOG_WRITE("\t\"cpuid\": \"%s\",\n",GetProcessorCPUIDString().c_str());
		LOG_WRITE("\t\"cpuidInfo\": \"%s\",\n",GetProcessorExtendedCPUIDInfo().c_str());
		///\todo This doesn't work on Core i3/i5/i7, so leave it out.
//		LOG_WRITE("\t\"numCores\": %d,\n", GetNumberOfLogicalCPUCores());
		LOG_WRITE("\t\"cpuSpeed\": %d,\n", (int)GetCPUSpeedFromRegistry(0));

		LOG_WRITE("\t\"defines\": {\n");
#ifdef WINVER
		LOG_WRITE("\t\t\"WINVER\": \"0x%X\",\n", WINVER);
#endif
#ifdef WIN8
		LOG_WRITE( "\t\t\"WIN8\": true,\n");
#endif
#ifdef MATH_ENABLE_WINXP_SUPPORT
		LOG_WRITE("\t\t\"MATH_ENABLE_WINXP_SUPPORT\": true,\n");
#endif
#ifdef _DEBUG
		LOG_WRITE("\t\t\"_DEBUG\": true,\n");
#endif
#ifdef NDEBUG
		LOG_WRITE("\t\t\"NDEBUG\": true,\n");
#endif
#ifdef RELEASE
		LOG_WRITE("\t\t\"RELEASE\": true,\n");
#endif
#ifdef RELWITHDEBINFO
		LOG_WRITE("\t\t\"RELWITHDEBINFO\": true,\n");
#endif
#ifdef OPTIMIZED_RELEASE
		LOG_WRITE("\t\t\"OPTIMIZED_RELEASE\": true,\n");
#endif
#ifdef MATH_SILENT_ASSUME
		LOG_WRITE("\t\t\"MATH_SILENT_ASSUME\": true,\n");
#endif
#ifdef MATH_ENABLE_INSECURE_OPTIMIZATIONS
		LOG_WRITE("\t\t\"MATH_ENABLE_INSECURE_OPTIMIZATIONS\": true,\n");
#endif
#ifdef MATH_ASSERT_CORRECTNESS
		LOG_WRITE("\t\t\"MATH_ASSERT_CORRECTNESS\": true,\n");
#endif
#ifdef FAIL_USING_EXCEPTIONS
		LOG_WRITE("\t\t\"FAIL_USING_EXCEPTIONS\": true,\n");
#endif
#ifdef MATH_ENABLE_STL_SUPPORT
		LOG_WRITE("\t\t\"MATH_ENABLE_STL_SUPPORT\": true,\n");
#endif
#ifdef ANDROID
		LOG_WRITE("\t\t\"ANDROID\": true,\n");
#endif
#ifdef __ARM_ARCH_7A__
		LOG_WRITE("\t\t\"__ARM_ARCH_7A__\": true,\n");
#endif
#ifdef WIN8RT
		LOG_WRITE("\t\t\"WIN8RT\": true,\n");
#endif
#ifdef _M_ARM
		LOG_WRITE("\t\t\"_M_ARM\": true,\n");
#endif
#ifdef MATH_AVX
		LOG_WRITE("\t\t\"MATH_AVX\": true,\n");
#endif
#ifdef MATH_SSE41
		LOG_WRITE("\t\t\"MATH_SSE41\": true,\n");
#endif
#ifdef MATH_SSE3
		LOG_WRITE("\t\t\"MATH_SSE3\": true,\n");
#endif
#ifdef MATH_SSE2
		LOG_WRITE("\t\t\"MATH_SSE2\": true,\n");
#endif
#ifdef MATH_SSE
		LOG_WRITE("\t\t\"MATH_SSE\": true,\n");
#endif
#ifdef MATH_NEON
		LOG_WRITE("\t\t\"MATH_NEON\": true,\n");
#endif
#ifdef _MSC_VER
		LOG_WRITE("\t\t\"_MSC_VER\": %d,\n", _MSC_VER);
#endif
#ifdef __clang__
		LOG_WRITE("\t\t\"__clang__\": true,\n");
#endif
#ifdef __GNUC__
		LOG_WRITE("\t\t\"__GNUC__\": true,\n");
#endif
#ifdef MATH_SIMD
		LOG_WRITE("\t\t\"MATH_SIMD\": true,\n");
#endif
#ifdef MATH_AUTOMATIC_SSE
		LOG_WRITE("\t\t\"MATH_AUTOMATIC_SSE\": true,\n");
#endif
		LOG_WRITE("\t\t\"dummy_swallowcomma\": true\n");
		LOG_WRITE("\t\t},\n");

		LOG_WRITE("\t\"results\": [\n");

		isOpen = true;
	}

	static double TicksToMicroseconds(double ticks)
	{
		return ticks * 1000000.0 / Clock::TicksPerSec();
	}

	static std::string ConvertSlashes(std::string str)
	{
		std::replace(str.begin(), str.end(), '\\', '/');
		return str;
	}

	void Report(const Test &t)
	{
		LOG_WRITE("\t\t{ \"type\": \"%s\", \"name\": \"%s\", \"description\": \"%s\", \"file\": \"%s\", \"numTimesRun\": %d, \"numTrialsPerRun\": %d, \"numPasses\": %d, \"numFails\": %d, \"fastestTime\": %f, \"averageTime\": %f, \"slowestTime\": %f, \"fastestCycles\": %f },\n", 
			t.isBenchmark ? "benchmark" : "test", ConvertSlashes(t.name).c_str(), ConvertSlashes(t.description).c_str(), ConvertSlashes(t.file).c_str(), t.numTimesRun, t.numTrialsPerRun, t.numPasses, t.numFails, 
			TicksToMicroseconds(t.fastestTime), TicksToMicroseconds(t.averageTime), TicksToMicroseconds(t.worstTime), t.fastestCycles);
		if (handle)
			fflush(handle);
	}

	void Finish()
	{
		if (isOpen)
		{
			LOG_WRITE("\t\t{ \"dummy_commaswallow\": true }\n");
			LOG_WRITE("\t]\n}\n");
		}
		if (handle)
			fclose(handle);
		else if (!reports.empty())// Output everything to stdout, since we don't have a file to store the results to.
		{
			LOGI("***** BEGIN FILE test_results.json *****");
			for(std::list<std::string>::iterator iter = reports.begin(); iter != reports.end(); ++iter)
				LOGI("%s", iter->c_str());
			LOGI("***** END FILE test_results.json *****");
			reports.clear();
		}
		handle = 0;
		isOpen = false;
	}

private:
	FILE *handle;

	void operator =(const JSONReport &);
	JSONReport(const JSONReport &);
};

#undef LOG_WRITE
