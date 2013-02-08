/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Log.h
	@author Jukka Jylänki
	@brief The LOG and LOGUSER macros. Provides an unified mechanism for logging. */
#pragma once

// From http://cnicholson.net/2009/03/stupid-c-tricks-dowhile0-and-c4127/
#ifdef _MSC_VER
#define MULTI_LINE_MACRO_BEGIN do { \
	__pragma(warning(push)) \
	__pragma(warning(disable:4127))

#define MULTI_LINE_MACRO_END \
	} while(0) \
	__pragma(warning(pop))

#else

#define MULTI_LINE_MACRO_BEGIN do {
#define MULTI_LINE_MACRO_END } while(0)

#endif

/// A bitfield type that describes single or multiple log channels (each bit represents a channel).
typedef unsigned int LogChannel;

namespace
{
const LogChannel LogUser = 1;        ///< This log channel is free to use for user-level application messages.
const LogChannel LogInfo = 2;        ///< Successful connects and disconnects.
const LogChannel LogError = 4;       ///< All connection-related errors.
const LogChannel LogWarning = 8;       ///< All connection-related errors.
const LogChannel LogObjectAlloc = 16; ///< For debugging: prints information about object allocations an deallocations.
const LogChannel LogData = 32;       ///< For debugging: dumps information about in- and outbound packet data.
const LogChannel LogVerbose = 64;    ///< For debugging: Print detailed internal information.
const LogChannel LogWaits = 128;      ///< Logs all long performance-related waits (thread/mutex blocks) that occur in the system.
}

/// Prints a variadic line to log.
/// @param logChannel The log channel to print to. This variable must have exactly one bit set, and is ANDed against
///                   the currently active channels that can be set with a call to kNet::SetLogChannels.
/// @param filename The C++ file from which the log message occurs, pass in __FILE__.
/// @param lineNumber Pass in __LINE__.
/// @param msg The printf-style message format specifier for the text to print.
void TimeOutputDebugStringVariadic(LogChannel logChannel, const char *filename, int lineNumber, const char *msg, ...);

/// Prints a message to the log. Same as kNet::TimeOutputDebugStringVariadic, but does not use printf formatting.
void TimeOutputDebugString(LogChannel logChannel, const char *filename, int lineNumber, const char *msg);

/// Sets the currently enabled log channels. Pass in a bitwise OR of the log channels you want to enable.
/// If you pass in 0, all logging is disabled. By default, only the channel LogChannelUser is enabled.
void SetLogChannels(LogChannel logChannel);

/// Returns the currently active log channels.
LogChannel GetLogChannels();

/// Returns true if the specified log channel is not suppressed.
//bool IsLogChannelActive(LogChannel channel);

/// Sets the output for log messages. By default all logging is output to std::cout. Setting a log file
/// redirects all logging to that file. Calling this function with a null filename pointer restores
/// logging to target std::cout.
void SetLogFile(const char *filename);

/// When called, sets the runtime to print out all memory leaks at program exit time. Win32-only. On
/// linux, this is a no-op.
void EnableMemoryLeakLoggingAtExit();

enum ConsoleTextColor
{
	ColorRed = 4, // == FOREGROUND_RED in WinCon.h
	ColorGreen  = 2, // == FOREGROUND_GREEN in WinCon.h
	ColorBlue = 1, // == FOREGROUND_BLUE in WinCon.h
	ColorIntensity = 8 // == FOREGROUND_INTENSITY in WinCon.h
};

/// Specifies a new print color for stdout. Only works on Win32, other platforms are no-op implementations.
void SetStdoutTextColor(int newColor);

/// Prints out a variadic message to the log channel User.
#define LOGUSER(msg, ...) ( /*IsLogChannelActive(LogUser) &&*/ (TimeOutputDebugStringVariadic(LogUser, __FILE__, __LINE__, msg, ##__VA_ARGS__), true) )

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : warning: " #desc)

#if defined(NPAPI) && !defined(LOGGING_SUPPORT_DISABLED)
#include <stdio.h>
///\todo Temporary. Implement logmsg as variadic directly instead of this kind of #define workaround.
void logmsg(const char *msg);

#define LOGI(...) \
	MULTI_LINE_MACRO_BEGIN \
		char str____[16384]; \
		sprintf(str____, __VA_ARGS__); \
		logmsg(str____); \
		logmsg("\n"); \
	MULTI_LINE_MACRO_END

#define LOGW(...) \
	MULTI_LINE_MACRO_BEGIN \
		char str____[16384]; \
		logmsg("Warning: "); \
		sprintf(str____, __VA_ARGS__); \
		logmsg(str____); \
		logmsg("\n"); \
	MULTI_LINE_MACRO_END

#define LOGE(...) \
	MULTI_LINE_MACRO_BEGIN \
		char str____[16384]; \
		logmsg("Error: "); \
		sprintf(str____, __VA_ARGS__); \
		logmsg(str____); \
		logmsg("\n"); \
	MULTI_LINE_MACRO_END

#define LOG(channel, ...) \
	MULTI_LINE_MACRO_BEGIN \
		/*if (IsLogChannelActive(channel))*/ \
		{ \
			char str____[16384]; \
			logmsg(#channel); \
			sprintf(str____, __VA_ARGS__); \
			logmsg(str____); \
			logmsg("\n"); \
		} \
	MULTI_LINE_MACRO_END

#elif defined(ANDROID) && !defined(LOGGING_SUPPORT_DISABLED)

/// This will require you to pass '-llog' on the command line to link against the Android logging libraries.
#include <android/log.h>

#ifndef LOG
#define LOG(channel, ...) do { /*if (IsLogChannelActive(channel))*/ (void)__android_log_print(ANDROID_LOG_INFO, "native-activity", __VA_ARGS__); } while(0)
#endif
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "native-activity", __VA_ARGS__))
#define LOGW(...) do { \
	char logfunc_str[512]; \
	sprintf(logfunc_str, __VA_ARGS__); \
	char logfunc_str2[512]; \
	sprintf(logfunc_str2, "Warning: %s", logfunc_str); \
	((void)__android_log_print(ANDROID_LOG_WARN, "native-activity", logfunc_str2)); \
	} while(0)

#define LOGE(...) do { \
	char logfunc_str[512]; \
	sprintf(logfunc_str, __VA_ARGS__); \
	char logfunc_str2[512]; \
	sprintf(logfunc_str2, "Error: %s", logfunc_str); \
	((void)__android_log_print(ANDROID_LOG_ERROR, "native-activity", logfunc_str2)); \
	} while(0)

#elif defined(WIN32) && !defined(LOGGING_SUPPORT_DISABLED)

#include <stdio.h>

#ifndef LOG
#define LOG(channel, ...) \
	MULTI_LINE_MACRO_BEGIN \
		printf(__VA_ARGS__); \
		printf("\n"); \
	MULTI_LINE_MACRO_END
#endif

#define LOGI(...) \
	MULTI_LINE_MACRO_BEGIN \
		printf(__VA_ARGS__); \
		printf("\n"); \
	MULTI_LINE_MACRO_END

#define LOGW(...) \
	MULTI_LINE_MACRO_BEGIN \
		SetStdoutTextColor(ColorRed | ColorGreen | ColorIntensity); \
		printf("Warning: "); printf(__VA_ARGS__); printf("\n"); \
		SetStdoutTextColor(ColorRed | ColorGreen | ColorBlue); \
	MULTI_LINE_MACRO_END

#define LOGE(...) \
	MULTI_LINE_MACRO_BEGIN \
		SetStdoutTextColor(ColorRed | ColorIntensity); \
		printf("Error: "); printf(__VA_ARGS__); printf("\n"); \
		SetStdoutTextColor(ColorRed | ColorGreen | ColorBlue); \
	MULTI_LINE_MACRO_END

#elif (defined(PEPPER) || defined(__APPLE__) || defined(__GNUC__) || defined(EMSCRIPTEN) || defined(__FLASHPLAYER__)) && !defined(LOGGING_SUPPORT_DISABLED)

#include <stdio.h>
/// Prints out a variadic message to the given log channel.
//#define LOG(channel, msg, ...)  ( IsLogChannelActive(channel) && (TimeOutputDebugStringVariadic(channel, __FILE__, __LINE__, msg, ##__VA_ARGS__), true) )
#define LOG(channel, ...) do { printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGI(...) do { printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGW(...) do { printf("Warning: "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGE(...) do { printf("Error: "); printf(__VA_ARGS__); printf("\n"); } while(0)
/*
#define LOGE(msg, ...)  ( kNet::IsLogChannelActive(LogError) && (kNet::TimeOutputDebugStringVariadic(LogError, __FILE__, __LINE__, msg, ##__VA_ARGS__), true) )
#define LOGW(msg, ...)  ( kNet::IsLogChannelActive(LogWarning) && (kNet::TimeOutputDebugStringVariadic(LogWarning, __FILE__, __LINE__, msg, ##__VA_ARGS__), true) )
#define LOGI(msg, ...)  ( kNet::IsLogChannelActive(LogInfo) && (kNet::TimeOutputDebugStringVariadic(LogInfo, __FILE__, __LINE__, msg, ##__VA_ARGS__), true) )
*/

#elif defined(LOGGING_SUPPORT_DISABLED)

/// If kNet logging is disabled, LOG() macro is a no-op. This avoids having to evaluate the arguments of the
/// LOG() call, which improves performance.
#define LOG(...) ((void)0)
#define LOGE(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGI(...) ((void)0)

#else

#error LOG/LOGE/LOGW/LOGI not implemented!
#endif

#ifdef GFXAPI_TRACE_ENABLED
#define TRACE LOGI
#else
#define TRACE(...) ((void)0)
#endif
