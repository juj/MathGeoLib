/* Copyright 2010 Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Clock.cpp
	@brief */

#if defined(__unix__) || defined(__EMSCRIPTEN__) || defined(ANDROID) || defined(__APPLE__) || defined (__CYGWIN__)
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include "../Math/InclWindows.h"
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

#include "Clock.h"
#include "../Math/myassert.h"
#include "../Math/assume.h"

MATH_BEGIN_NAMESPACE

#ifdef WIN32
u64 Clock::ddwTimerFrequency;
#endif

#ifdef __APPLE__
tick_t Clock::ticksPerSecond = 0;
#endif

tick_t Clock::appStartTime = 0;

Clock impl;

void Clock::InitClockData()
{
	if (appStartTime == 0)
		appStartTime = Tick();

#ifdef WIN32
	if (!QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&ddwTimerFrequency)))
	{
		LOGE("The system doesn't support high-resolution timers!");
		ddwTimerFrequency = (u64)-1;
	}

	if (appStartTime == 0)
	{
		appStartTime = (tick_t)GetTickCount64();
	}
#endif

#ifdef __APPLE__
	mach_timebase_info_data_t timeBaseInfo;
	mach_timebase_info(&timeBaseInfo);
	ticksPerSecond = 1000000000ULL * (uint64_t)timeBaseInfo.denom / (uint64_t)timeBaseInfo.numer;
	assert(ticksPerSecond > (uint64_t)timeBaseInfo.denom/timeBaseInfo.numer); // Guard against overflow if OSX numer/denom change or similar.
#endif
}

Clock::Clock()
{
	InitClockData();
}

void Clock::Sleep(int milliseconds)
{
#ifdef WIN8RT
#pragma warning(Clock::Sleep has not been implemented!)
#elif defined(WIN32)
	::Sleep(milliseconds);
#elif !defined(__EMSCRIPTEN__)
	// http://linux.die.net/man/2/nanosleep
	timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (milliseconds - ts.tv_sec * 1000) * 1000 * 1000;
	int ret = nanosleep(&ts, NULL);
	if (ret == -1)
		LOGI("nanosleep returned -1! Reason: %s(%d).", strerror(errno), (int)errno);
#elif defined(__EMSCRIPTEN_PTHREADS__)
	emscripten_thread_sleep(milliseconds);
#elif defined(__EMSCRIPTEN__)
	/*NO-OP, cannot sleep*/
	MARK_UNUSED(milliseconds);
#else
#warning Clock::Sleep has not been implemented!
#endif
}

int Clock::Year()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wYear;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Month()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wMonth;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Day()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wDay;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Hour()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wHour;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Min()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wMinute;
#else
	///\todo.
	return 0;
#endif
}

int Clock::Sec()
{
#ifdef WIN32
	SYSTEMTIME s;
	GetSystemTime(&s);
	return s.wSecond;
#else
	///\todo.
	return 0;
#endif
}

unsigned long Clock::SystemTime()
{
#ifdef WIN32
	return (unsigned long)GetTickCount64();
#else
	return TickU32();
#endif
}
/*
tick_t Clock::ApplicationStartupTick()
{
	return appStartTime;
}
*/
unsigned long Clock::Time()
{
	return (unsigned long)((Tick() - appStartTime) * 1000 / Clock::TicksPerSec());
}

tick_t Clock::Tick()
{
#if defined(ANDROID)
	struct timespec res;
	clock_gettime(CLOCK_REALTIME, &res);
	return 1000000000ULL*res.tv_sec + (tick_t)res.tv_nsec;
#elif defined(__EMSCRIPTEN__)

#ifdef MATH_TICK_IS_FLOAT
	return (tick_t)emscripten_get_now();
#else
	// emscripten_get_now() returns a wallclock time as a float in milliseconds (1e-3).
	// scale it to microseconds (1e-6) and return as a tick.
	return (tick_t)(((double)emscripten_get_now()) * 1e3);
#endif

#elif defined(WIN32)
	LARGE_INTEGER ddwTimer;
	BOOL success = QueryPerformanceCounter(&ddwTimer);
	assume(success != 0);
	MARK_UNUSED(success);
	return ddwTimer.QuadPart;
#elif defined(__APPLE__)
	return mach_absolute_time();
#elif defined(_POSIX_MONOTONIC_CLOCK)
	timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (tick_t)t.tv_sec * 1000 * 1000 * 1000 + (tick_t)t.tv_nsec;
#elif defined(_POSIX_C_SOURCE)
	timeval t;
	gettimeofday(&t, NULL);
	return (tick_t)t.tv_sec * 1000 * 1000 + (tick_t)t.tv_usec;
#else
	return (tick_t)clock();
#endif
}

unsigned long Clock::TickU32()
{
#ifdef WIN32
	LARGE_INTEGER ddwTimer;
	BOOL success = QueryPerformanceCounter(&ddwTimer);
	assume(success != 0);
	MARK_UNUSED(success);
	return ddwTimer.LowPart;
#else
	return (unsigned long)Tick();
#endif
}

tick_t Clock::TicksPerSec()
{
#if defined(ANDROID)
	return 1000000000ULL; // 1e9 == nanoseconds.
#elif defined(__EMSCRIPTEN__)

#ifdef MATH_TICK_IS_FLOAT
	return (tick_t)1000.0;
#else
	return 1000000ULL; // 1e6 == microseconds.
#endif

#elif defined(WIN32)
	return ddwTimerFrequency;
#elif defined(__APPLE__)
	return ticksPerSecond;
#elif defined(_POSIX_MONOTONIC_CLOCK)
	return 1000 * 1000 * 1000;
#elif defined(_POSIX_C_SOURCE) || defined(__APPLE__)
	return 1000 * 1000;
#else
	return CLOCKS_PER_SEC;
#endif
}

unsigned long long Clock::Rdtsc()
{
#if defined(_MSC_VER) && !defined(WIN8PHONE)
	return __rdtsc();
#elif defined(__x86_64__)
	unsigned hi, lo;
	__asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
	return ((unsigned long long)lo) | (((unsigned long long)hi) << 32);
#elif defined(__i386__) || defined(__X86__) || defined(_X86_)
	unsigned long long int x;
	__asm__ volatile ("rdtsc" : "=A" (x));
	return x;
#else
	return Clock::Tick();
#endif
}

MATH_END_NAMESPACE
