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

/** @file simd.h
	@author Jukka Jylänki
	@brief Generic abstraction layer over different SIMD instruction sets. */
#pragma once

#pragma once

#include "../MathBuildConfig.h"
#include "MathNamespace.h"
#include "../MathGeoLibFwd.h"
#include <stdint.h>
#include <cstddef>
#include "Reinterpret.h"

#ifdef MATH_SIMD // If SSE is not enabled, this whole file will not be included.

MATH_BEGIN_NAMESPACE

#ifdef _MSC_VER
#define ALIGN16 __declspec(align(16))
#define ALIGN32 __declspec(align(32))
#define ALIGN64 __declspec(align(64))
#else
#define ALIGN16 __attribute((aligned(16)))
#define ALIGN32 __attribute((aligned(32)))
#define ALIGN64 __attribute((aligned(64)))
#endif

#define IS16ALIGNED(x) ((((uintptr_t)(x)) & 0xF) == 0)
#define IS32ALIGNED(x) ((((uintptr_t)(x)) & 0x1F) == 0)
#define IS64ALIGNED(x) ((((uintptr_t)(x)) & 0x3F) == 0)

#ifdef MATH_AVX
#define ALIGN_MAT ALIGN32
#define MAT_ALIGNMENT 32
#define IS_MAT_ALIGNED(x) IS32ALIGNED(X)
#else
#define ALIGN_MAT ALIGN16
#define MAT_ALIGNMENT 16
#define IS_MAT_ALIGNED(x) IS16ALIGNED(X)
#endif

#ifdef MATH_SSE

#define simd4f __m128
#define simd4i __m128i

#define add_ps _mm_add_ps
#define sub_ps _mm_sub_ps
#define mul_ps _mm_mul_ps
#define div_ps _mm_div_ps
#define set1_ps _mm_set1_ps
#define set_ps _mm_set_ps
const simd4f simd4fSignBit = set1_ps(-0.f); // -0.f = 1 << 31
#define abs_ps(x) _mm_andnot_ps(simd4fSignBit, (x))
#define zero_ps() _mm_setzero_ps()
#define min_ps _mm_min_ps
#define max_ps _mm_max_ps
#define s4f_to_s4i(s4f) _mm_castps_si128((s4f))
#define s4i_to_s4f(s4i) _mm_castsi128_ps((s4i))
#define and_ps _mm_and_ps
#define andnot_ps _mm_andnot_ps
#define or_ps _mm_or_ps
#define xor_ps _mm_xor_ps

#if defined(MATH_SSE2) && !defined(MATH_AVX) // We can use the pshufd instruction, which was introduced in SSE2 32-bit integer ops.
/// Swizzles/permutes a single SSE register into another SSE register. Requires SSE2.
#define shuffle1_ps(reg, shuffle) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((reg)), (shuffle)))
#else // We only have SSE 1, so must use the slightly worse shufps instruction, which always destroys the input operand - or we have AVX where we can use this operation without destroying input
#define shuffle1_ps(reg, shuffle) _mm_shuffle_ps((reg), (reg), (shuffle))
#endif

/// Returns the lowest element of the given sse register as a float.
/// @note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast", since
/// the resulting float is represented in an XMM register as well. Check the disassembly to confirm!
FORCE_INLINE float s4f_x(simd4f s4f)
{
	float ret;
	_mm_store_ss(&ret, s4f);
	return ret;
}
#define s4f_y(s4f) s4f_x(shuffle1_ps((s4f), _MM_SHUFFLE(1,1,1,1)))
#define s4f_z(s4f) s4f_x(shuffle1_ps((s4f), _MM_SHUFFLE(2,2,2,2)))
#define s4f_w(s4f) s4f_x(shuffle1_ps((s4f), _MM_SHUFFLE(3,3,3,3)))

#ifdef MATH_SSE2
#define set_ps_hex(w, z, y, x) _mm_castsi128_ps(_mm_set_epi32(w, z, y, x))
#define set1_ps_hex(x) _mm_castsi128_ps(_mm_set1_epi32(x))
#else
#define set_ps_hex(w, z, y, x) _mm_set_ps(ReinterpretAsFloat(w), ReinterpretAsFloat(z), ReinterpretAsFloat(y), ReinterpretAsFloat(x))
#define set1_ps_hex(x) _mm_set1_ps(ReinterpretAsFloat(x))
#endif

const float andMaskOneF = ReinterpretAsFloat(0xFFFFFFFFU);
/// A SSE mask register with x = y = z = 0xFFFFFFFF and w = 0x0.
const __m128 sseMaskXYZ = set_ps(0.f, andMaskOneF, andMaskOneF, andMaskOneF);
const __m128 sseSignMask3 = set_ps(0.f, -0.f, -0.f, -0.f); // -0.f = 1 << 31
#ifdef MATH_AVX
const __m256 sseSignMask256 = _mm256_set1_ps(-0.f); // -0.f = 1 << 31
#endif
#define negate3_ps(x) xor_ps(x, sseSignMask3)
#ifdef MATH_AVX
#define abs_ps256(x) _mm256_andnot_ps(sseSignMask256, x)
#endif

/// Returns the simd vector [0, 0, 0, f], that is, a SSE variable with the given float f in the lowest index. 
/** The three higher indices are set to zero.
	@note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast" if the given 
	float is already in a register, since it will lie in an XMM register already. Check the disassembly to confirm!
	@note Detected on VS2010 32-bit + AVX that this generates a vmovss+vxorps+vmovss instruction triple!
	@note Never use this function if you need to generate a 4-vector [f,f,f,f]. Instead, use set1_ps(f), which
		generates a vmovss+vhufps and no redundant vxorps+vmovss! */
FORCE_INLINE simd4f setx_ps(float f)
{
	return _mm_load_ss(&f);
}

// Given four scalar SS FP registers, packs the four values into a single SP FP register.
//inline simd4f pack_4ss_to_ps(simd4f x, simd4f y, simd4f z, simd4f w) // VS2010 BUG! Can't use this signature!
FORCE_INLINE simd4f pack_4ss_to_ps(simd4f x, simd4f y, simd4f z, const simd4f &w)
{
	simd4f xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	simd4f zw = _mm_movelh_ps(z, w); // zw = [ _, w, _, z]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]
}

#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
FORCE_INLINE simd4f modf_ps(simd4f x, simd4f mod)
{
	// x % mod == x - floor(x/mod)*mod
	// floor(x/mod) = integerpart(x/mod)
	simd4f ints = _mm_div_ps(x, mod);
	simd4f integerpart = _mm_round_ps(ints, _MM_FROUND_TO_ZERO);
	return _mm_sub_ps(x, _mm_mul_ps(integerpart, mod));
}
#endif

#elif defined(MATH_NEON)

#define simd4f float32x4_t
#define simd4i int32x4_t

#define add_ps vaddq_f32
#define sub_ps vsubq_f32
#define mul_ps vmulq_f32
#define min_ps vminq_f32
#define max_ps vmaxq_f32
#define s4f_to_s4i(s4f) vreinterpretq_u32_f32((s4f))
#define s4i_to_s4f(s4i) vreinterpretq_f32_u32((s4i))
#define and_ps(x, y) s4i_to_s4f(vandq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define andnot_ps(x, y) s4i_to_s4f(vbicq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define or_ps(x, y) s4i_to_s4f(vorrq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define xor_ps(x, y) s4i_to_s4f(veorq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define ornot_ps(x, y) s4i_to_s4f(vornq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))

#define s4f_x(vec) vget_lane_f32(vget_low_f32((vec)), 0)
#define s4f_y(vec) vget_lane_f32(vget_low_f32((vec)), 1)
#define s4f_z(vec) vget_lane_f32(vget_high_f32((vec)), 0)
#define s4f_w(vec) vget_lane_f32(vget_high_f32((vec)), 1)

// NEON doesn't have a divide instruction. Do reciprocal + one step of Newton-Rhapson.
FORCE_INLINE simd4f div_ps(simd4f vec, simd4f vec2)
{
	simd4f rcp = vrecpeq_f32(vec2);
	rcp = mul_ps(vrecpsq_f32(vec2, rcp), rcp);
	return mul_ps(vec, rcp);
}

#define set1_ps vdupq_n_f32
#define abs_ps vabsq_f32
#define zero_ps() set1_ps(0.f) // TODO: Is there anything better than this?

#ifdef _MSC_VER
#define set_ps_const(w,z,y,x) {{ (u64)ReinterpretAsU32(x) | (((u64)ReinterpretAsU32(y)) << 32), (u64)ReinterpretAsU32(z) | (((u64)ReinterpretAsU32(w)) << 32) }}
#define set_ps_hex_const(w,z,y,x) {{ (u64)(x) | (((u64)(y)) << 32), (u64)(z) | (((u64)(w)) << 32) }}
#else
#define set_ps_const(w,z,y,x) { x, y, z, w }
#define set_ps_hex_const(w,z,y,x) { ReinterpretAsFloat(x), ReinterpretAsFloat(y), ReinterpretAsFloat(z), ReinterpretAsFloat(w) }
#endif

FORCE_INLINE simd4f set_ps(float w, float z, float y, float x)
{
//	const ALIGN16 float32_t d[4] = { x, y, z, w };
//	return vld1q_f32(d);
	float32x4_t c = set_ps_const(w,z,y,x);
	return c;
}
FORCE_INLINE simd4f set_ps_hex(u32 w, u32 z, u32 y, u32 x)
{
//	const ALIGN16 u32 d[4] = { x, y, z, w };
//	return vld1q_f32((const float*)d);
	float32x4_t c = set_ps_hex_const(w,z,y,x);
	return c;
}

#endif

// TODO: Which is better codegen - use simd4fZero constant everywhere, or explicitly refer to zero_ps() everywhere,
//       or does it matter?
const simd4f simd4fZero     = zero_ps();
const simd4f simd4fOne      = set1_ps(1.f);
const simd4f simd4fMinusOne = set1_ps(-1.f);
const simd4f simd4fEpsilon  = set1_ps(1e-4f);

///\todo Benchmark which one is better!
//#define negate_ps(x) _mm_xor_ps(x, sseSignMask)
//#define negate_ps(x) _mm_sub_ps(_mm_setzero_ps(), x)

#define negate_ps(x) sub_ps(zero_ps(), (x))

// If mask[i] == 0, then output index i from a, otherwise mask[i] must be 0xFFFFFFFF, and output index i from b.
FORCE_INLINE simd4f cmov_ps(simd4f a, simd4f b, simd4f mask)
{
#ifdef MATH_SSE41 // SSE 4.1 offers conditional copying between registers with the blendvps instruction.
	return _mm_blendv_ps(a, b, mask);
#else // If not on SSE 4.1, use conditional masking.
	b = and_ps(mask, b); // Where mask is 1, output b.
	a = andnot_ps(mask, a); // Where mask is 0, output a.
	return or_ps(a, b);
#endif
}

MATH_END_NAMESPACE

#else // ~MATH_SIMD

#define ALIGN16
#define ALIGN32
#define ALIGN64
#define ALIGN_MAT
#define IS_MAT_ALIGNED(x) true

#endif // ~MATH_SIMD
