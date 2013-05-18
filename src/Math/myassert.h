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

/** @file myassert.h
	@author Jukka Jylänki
	@brief Control over assert() macro for MathGeoLib. */
#include "MathLog.h"

#include <sstream>

#ifdef assert
#undef assert
#endif

#if defined(OPTIMIZED_RELEASE) && !defined(FAIL_USING_EXCEPTIONS)

#define assert(x)
#define asserteq(x,y)
#define assertcmp(x, cmp, y)

#elif defined(FAIL_USING_EXCEPTIONS)

#include <stdexcept>

#define assert(x) \
	MULTI_LINE_MACRO_BEGIN \
		if (!(x)) \
			throw std::runtime_error(#x); \
	MULTI_LINE_MACRO_END

#define asserteq(x,y) \
	MULTI_LINE_MACRO_BEGIN \
		if ((x) != (y)) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' == '" #y "' failed! (" << (x) << " != " << (y) << "!)"; \
			throw std::runtime_error(std_stringstream.str().c_str()); \
		} \
	MULTI_LINE_MACRO_END

#define assertcmp(x, cmp, y) \
	MULTI_LINE_MACRO_BEGIN \
		if (!((x) cmp (y))) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' " #cmp " '" #y "' failed! (" << (x) << " and " << (y) << "!)"; \
			throw std::runtime_error(std_stringstream.str().c_str()); \
		} \
	MULTI_LINE_MACRO_END

#elif defined(WIN32)

#include <cassert>

#define asserteq(x,y) \
	MULTI_LINE_MACRO_BEGIN \
		if ((x) != (y)) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' == '" #y "' failed! (" << (x) << " != " << (y) << "!)"; \
			LOGE("%s", std_stringstream.str().c_str()); \
			_CrtDebugBreak(); \
		} \
	MULTI_LINE_MACRO_END

#define assertcmp(x, cmp, y) \
	MULTI_LINE_MACRO_BEGIN \
		if (!((x) cmp (y))) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' " #cmp " '" #y "' failed! (" << (x) << " and " << (y) << "!)"; \
			LOGE("%s", std_stringstream.str().c_str()); \
			_CrtDebugBreak(); \
		} \
	MULTI_LINE_MACRO_END

#elif defined(_DEBUG)

#define assert(x) do { if (!(x)) LOGW("Assertion failed: %s",  #x); } while(0)
#define asserteq(x,y) \
	MULTI_LINE_MACRO_BEGIN \
		if ((x) != (y)) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' == '" #y "' failed! (" << (x) << " != " << (y) << "!)"; \
			LOGE("%s", std_stringstream.str().c_str()); \
		} \
	MULTI_LINE_MACRO_END
#define assertcmp(x, cmp, y) \
	MULTI_LINE_MACRO_BEGIN \
		if (!((x) cmp (y))) \
		{ \
			std::stringstream std_stringstream; \
			std_stringstream << "Assertion '" #x "' " #cmp " '" #y "' failed! (" << (x) << " and " << (y) << "!)"; \
			LOGE("%s", std_stringstream.str().c_str()); \
		} \
	MULTI_LINE_MACRO_END

#else

#define assert(x)
#define asserteq(x,y)
#define assertcmp(x, cmp, y)

#endif
