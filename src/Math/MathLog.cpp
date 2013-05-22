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

/** @file Log.cpp
	@author Jukka Jylänki
	@brief The LOG and LOGUSER macros. Provides an unified mechanism for logging. */
#include "MathLog.h"

#include <cstring>
#include <cstdarg>
#include <stdio.h>

#if defined(WIN32) && !defined(WIN8RT)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#if defined(ANDROID)
/// This will require you to pass '-llog' on the command line to link against the Android logging libraries.
#include <android/log.h>
#endif

#ifdef WIN8PHONE
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

MATH_BEGIN_NAMESPACE

#if defined(__native_client__) || defined(NPAPI)

extern void PrintToConsole(MathLogChannel channel, const char *str); ///< Implemented in gfxapi to route access to application instance.

#elif defined(ANDROID)

void PrintToConsole(MathLogChannel channel, const char *str)
{
	const int capacity = 512;
	char text[capacity];
	if (channel == MathLogError)
	{
		strcpy(text, "Error: ");
		strncat(text, str, capacity-7);
		text[capacity-1] = 0;
		(void)__android_log_print(ANDROID_LOG_ERROR, "native-activity", text);
	}
	else if (channel == MathLogWarning)
	{
		strcpy(text, "Warning: ");
		strncat(text, str, capacity-9);
		text[capacity-1] = 0;
		(void)__android_log_print(ANDROID_LOG_WARN, "native-activity", text);
	}
	else
		(void)__android_log_print(ANDROID_LOG_INFO, "native-activity", str);
}

#elif defined(WIN8PHONE)

void PrintToConsole(MathLogChannel channel, const char *str)
{
	if (channel == MathLogError)
		OutputDebugStringA("Error: ");
	else if (channel == MathLogWarning)
		OutputDebugStringA("Warning: ");
	OutputDebugStringA(str);
	OutputDebugStringA("\r\n");
}

#elif defined(WIN32) && !defined(WIN8RT)

void PrintToConsole(MathLogChannel channel, const char *str)
{
	if (channel == MathLogError)
	{
		SetConsoleTextAttribute(GetStdHandle(STD_ERROR_HANDLE), FOREGROUND_RED | FOREGROUND_INTENSITY);
		fprintf(stderr, "Error: %s\n", str);
		SetConsoleTextAttribute(GetStdHandle(STD_ERROR_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
	}
	else if (channel == MathLogWarning)
	{
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);
		printf("Warning: %s\n", str);
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
	}
	else
		printf("%s\n", str);
}

#else

void PrintToConsole(MathLogChannel channel, const char *str)
{
	if (channel == MathLogError)
		fprintf(stderr, "Error: %s\n", str);
	else if (channel == MathLogWarning)
		printf("Warning: %s\n", str);
	else
		printf("%s\n", str);
}

#endif

void PrintToConsoleVariadic(MathLogChannel channel, const char *format, ...)
{
	const int capacity = 512;
	char str[capacity];

	va_list args;
	va_start(args, format);

	vsnprintf((char *)str, capacity, format, args);
	str[capacity-1] = 0; // We only support logging a fixed-length string so don't care if we fail/truncate, just make sure we zero-terminate so there won't be any issues.
	PrintToConsole(channel, str);
}

MATH_END_NAMESPACE
