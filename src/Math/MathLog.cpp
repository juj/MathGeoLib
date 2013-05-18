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

#include <cstdarg>

#if defined(WIN32) && !defined(WIN8RT)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

MATH_BEGIN_NAMESPACE

void SetStdoutTextColor(int newColor)
{
#if defined(WIN32) && !defined(WIN8RT) // Win8 metro apps don't have SetConsoleTextAttribute.
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), (WORD)newColor);
#endif
}

#ifdef __native_client__
extern void PrintToConsole(LogChannel channel, const char *str); ///< Implemented in gfxapi to route access to pp:Instance.
#else
void PrintToConsole(LogChannel channel, const char *str)
{
	printf("%s", str);
}
#endif

void PrintToConsoleVariadic(LogChannel channel, const char *format, ...)
{
	const int capacity = 512;
	char str[capacity];

	va_list args;
	va_start(args, format);

	int len = vsnprintf((char *)str, capacity, format, args);
	str[capacity-1] = 0; // Don't care if we fail/truncate, just make sure we zero-terminate so there won't be any issues.
	PrintToConsole(channel, str);
}

MATH_END_NAMESPACE
