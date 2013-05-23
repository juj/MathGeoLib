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

#ifdef MATH_SIMD // If SSE is not enabled, this whole file will not be included.

#ifdef _MSC_VER
#define ALIGN16 __declspec(align(16))
#define ALIGN32 __declspec(align(32))
#else
#define ALIGN16 __attribute((aligned(16)))
#define ALIGN32 __attribute((aligned(32)))
#endif

#define IS16ALIGNED(x) ((((uintptr_t)(x)) & 0xF) == 0)
#define IS32ALIGNED(x) ((((uintptr_t)(x)) & 0x1F) == 0)

#ifdef MATH_AVX
#define ALIGN_MAT ALIGN32
#define MAT_ALIGNMENT 32
#define IS_MAT_ALIGNED(x) IS32ALIGNED(X)
#else
#define ALIGN_MAT ALIGN16
#define MAT_ALIGNMENT 16
#define IS_MAT_ALIGNED(x) IS16ALIGNED(X)
#endif

//inline float ReinterpretAsFloat(u32 i);

#ifdef MATH_SSE

#ifdef MATH_SSE2
#define set_ps_hex(w, z, y, x) _mm_castsi128_ps(_mm_set_epi32(w, z, y, x))
#define set1_ps_hex(x) _mm_castsi128_ps(_mm_set1_epi32(x))
#else
#define set_ps_hex(w, z, y, x) _mm_set_ps(ReinterpretAsFloat(w), ReinterpretAsFloat(z), ReinterpretAsFloat(y), ReinterpretAsFloat(x)))
#define set1_ps_hex(x) _mm_set1_ps(ReinterpretAsFloat(x))
#endif

#if defined(MATH_SSE2) && !defined(MATH_AVX) // We can use the pshufd instruction, which was introduced in SSE2 32-bit integer ops.
/// Swizzles/permutes a single SSE register into another SSE register. Requires SSE2.
#define shuffle1_ps(reg, shuffle) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((reg)), (shuffle)))
#else // We only have SSE 1, so must use the slightly worse shufps instruction, which always destroys the input operand - or we have AVX where we can use this operation without destroying input
#define shuffle1_ps(reg, shuffle) _mm_shuffle_ps((reg), (reg), (shuffle))
#endif

const u32 andMaskOne = 0xFFFFFFFF;
const float andMaskOneF = ReinterpretAsFloat(andMaskOne);

/// A SSE mask register with x = y = z = 0xFFFFFFFF and w = 0x0.
const __m128 sseMaskXYZ = _mm_set_ps(0.f, andMaskOneF, andMaskOneF, andMaskOneF);

const __m128 sseSignMask = _mm_set1_ps(-0.f); // -0.f = 1 << 31
const __m128 sseSignMask3 = _mm_set_ps(0.f, -0.f, -0.f, -0.f); // -0.f = 1 << 31
#ifdef MATH_AVX
const __m256 sseSignMask256 = _mm256_set1_ps(-0.f); // -0.f = 1 << 31
#endif

const __m128 sseEpsilonFloat = _mm_set1_ps(1e-4f);

const __m128 sseZero = _mm_set1_ps(0.f);
const __m128 sseOne = _mm_set1_ps(1.f);

///\todo Benchmark which one is better!
//#define negate_ps(x) _mm_xor_ps(x, sseSignMask)
#define negate_ps(x) _mm_sub_ps(_mm_setzero_ps(), x)

#define negate3_ps(x) _mm_xor_ps(x, sseSignMask3)

#define abs_ps(x) _mm_andnot_ps(sseSignMask, x)
#ifdef MATH_AVX
#define abs_ps256(x) _mm256_andnot_ps(sseSignMask256, x)
#endif

/// Returns the lowest element of the given sse register as a float.
/// @note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast", since
/// the resulting float is represented in an XMM register as well. Check the disassembly to confirm!
FORCE_INLINE float M128_TO_FLOAT(__m128 sse)
{
	float ret;
	_mm_store_ss(&ret, sse);
	return ret;
}

/// Returns a SSE variable with the given float f in the lowest index. The three higher indices are set to zero.
/** @note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast" if the given 
	float is already in a register, since it will lie in an XMM register already. Check the disassembly to confirm!
	@note Detected on VS2010 32-bit + AVX that this generates a vmovss+vxorps+vmovss instruction triple!
	@note Never use this function if you need to generate a 4-vector [f,f,f,f]. Instead, use _mm_set1_ps(&f), which
		generates a vmovss+vhufps and no redundant vxorps+vmovss! */
FORCE_INLINE __m128 FLOAT_TO_M128(float f)
{
	return _mm_load_ss(&f);
}

// If mask[i] == 0, then output index i from a, otherwise mask[i] must be 0xFFFFFFFF, and output index i from b.
FORCE_INLINE __m128 cmov_ps(__m128 a, __m128 b, __m128 mask)
{
#ifdef MATH_SSE41 // SSE 4.1 offers conditional copying between registers with the blendvps instruction.
	return _mm_blendv_ps(a, b, mask);
#else // If not on SSE 4.1, use conditional masking.
	b = _mm_and_ps(mask, b); // Where mask is 1, output b.
	a = _mm_andnot_ps(mask, a); // Where mask is 0, output a.
	return _mm_or_ps(a, b);
#endif
}

// Given four scalar SS FP registers, packs the four values into a single SP FP register.
//inline __m128 pack_4ss_to_ps(__m128 x, __m128 y, __m128 z, __m128 w) // VS2010 BUG! Can't use this signature!
FORCE_INLINE __m128 pack_4ss_to_ps(__m128 x, __m128 y, __m128 z, const __m128 &w)
{
	__m128 xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	__m128 zw = _mm_movelh_ps(z, w); // zw = [ _, w, _, z]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]
}

#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
FORCE_INLINE __m128 modf_ps(__m128 x, __m128 mod)
{
	// x % mod == x - floor(x/mod)*mod
	// floor(x/mod) = integerpart(x/mod)
	__m128 ints = _mm_div_ps(x, mod);
	__m128 integerpart = _mm_round_ps(ints, _MM_FROUND_TO_ZERO);
	return _mm_sub_ps(x, _mm_mul_ps(integerpart, mod));
}
#endif

#endif // ~MATH_SSE

/*
inline std::string ToString(simd4f vec)
{
	float *v = (float*)&vec;
	char str[256];
	sprintf(str, "[%f, %f, %f, %f]", v[3], v[2], v[1], v[0]);
	return str;
}
*/
#else // ~MATH_SIMD

#define ALIGN16
#define ALIGN32
#define ALIGN_MAT
#define IS_MAT_ALIGNED(x) true

#endif // ~MATH_SIMD

MATH_END_NAMESPACE
