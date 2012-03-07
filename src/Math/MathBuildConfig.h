/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file MathBuildConfig.h
	@author Jukka Jylänki
	@brief Specifies all build flags for the library. */
#pragma once

#define MATH_ENABLE_STL_SUPPORT
#define MATH_TINYXML_INTEROP
#define KNET_LOGGING_SUPPORT_ENABLED

//#define MATH_AVX

// MATH_AVX implies MATH_SSE41, which implies MATH_SSE3, which implies MATH_SSE2, which implies MATH_SSE.
#ifdef MATH_AVX
#define MATH_SSE41
#define MATH_SSE3
#define MATH_SSE2
#define MATH_SSE
#endif

#ifdef MATH_SSE41
#define MATH_SSE3
#define MATH_SSE2
#define MATH_SSE
#endif

#ifdef MATH_SSE3
#include <intrin.h>
#define MATH_SSE2
#define MATH_SSE
#endif

#ifdef MATH_SSE2
#define MATH_SSE
#endif

#ifdef MATH_SSE
#include <xmmintrin.h>
#endif

#include "Types.h"
