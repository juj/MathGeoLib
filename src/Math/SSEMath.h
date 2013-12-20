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

/** @file SSEMath.h
	@author Jukka Jylänki
	@brief SIMD-enabled math helper utilities. */
#pragma once

#include "../MathBuildConfig.h"
#include "MathNamespace.h"
#include "../MathGeoLibFwd.h"
#include <stdint.h>
#include <cstddef>
#include "Reinterpret.h"
#include "simd.h"

MATH_BEGIN_NAMESPACE

/// Allocates the given amount of memory at the given alignment.
void *AlignedMalloc(size_t size, size_t alignment);

/// \todo This is not really placement-new.
template<typename T>
inline T *AlignedNew(size_t numElements, size_t alignment) { return reinterpret_cast<T*>(AlignedMalloc(numElements*sizeof(T), alignment)); }

/// \todo This is not really placement-new.
template<typename T>
inline T *AlignedNew(size_t numElements) { return AlignedNew<T>(numElements, 16); }

/// Frees memory allocated by AlignedMalloc.
void AlignedFree(void *ptr);

MATH_END_NAMESPACE
