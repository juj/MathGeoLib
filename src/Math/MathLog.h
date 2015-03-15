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

#include "MathNamespace.h"

MATH_BEGIN_NAMESPACE

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
typedef unsigned int MathLogChannel;

namespace
{
const MathLogChannel MathLogInfo = 1;
const MathLogChannel MathLogError = 2;
const MathLogChannel MathLogWarning = 4;
const MathLogChannel MathLogErrorNoCallstack = MathLogError|65536;
const MathLogChannel MathLogWarningNoCallstack = MathLogWarning|65536;
}

void PrintToConsoleVariadic(MathLogChannel channel, const char *format, ...);
void PrintToConsole(MathLogChannel channel, const char *str);

#define STRINGIZE_HELPER(x) #x
#define STRINGIZE(x) STRINGIZE_HELPER(x)
#define WARNING(desc) message(__FILE__ "(" STRINGIZE(__LINE__) ") : warning: " #desc)

#ifndef LOGGING_SUPPORT_DISABLED

#define LOGI(...) PrintToConsoleVariadic(MathLogInfo, __VA_ARGS__)
#define LOGW(...) PrintToConsoleVariadic(MathLogWarning, __VA_ARGS__)
#define LOGW_NS(...) PrintToConsoleVariadic(MathLogWarningNoCallstack, __VA_ARGS__)
#define LOGE(...) PrintToConsoleVariadic(MathLogError, __VA_ARGS__)
#define LOGE_NS(...) PrintToConsoleVariadic(MathLogErrorNoCallstack, __VA_ARGS__)
#define LOG(channel, ...) PrintToConsoleVariadic(channel, __VA_ARGS__)

#else

#define LOG(...) ((void)0)
#define LOGW(...) ((void)0)
#define LOGW_NS(...) ((void)0)
#define LOGE(...) ((void)0)
#define LOGE_NS(...) ((void)0)
#define LOGI(...) ((void)0)

#endif

MATH_END_NAMESPACE
