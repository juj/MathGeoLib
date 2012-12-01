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
#include "Log.h"

#ifdef assert
#undef assert
#endif

#ifdef OPTIMIZED_RELEASE
#define assert(x)
#else

#ifdef FAIL_USING_EXCEPTIONS

#include <stdexcept>

#define assert(x) do { if (!(x)) throw std::runtime_error(#x); } while(0)

#else

#ifdef WIN32
#include <cassert>
#else


#ifdef _DEBUG
#define assert(x) do { if (!(x)) LOGW("Assertion failed: " #x); } while(0)
#else
#define assert(x)
#endif

#endif

#endif

#endif
