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

#include "../MathBuildConfig.h"
#include "MathNamespace.h"
#include "../MathGeoLibFwd.h"
#include <stdint.h>
#include <cstddef>
#include "Reinterpret.h"
#ifdef MATH_SSE41
#include <smmintrin.h>
#endif

#ifdef MATH_SIMD // If SSE is not enabled, this whole file will not be included.

MATH_BEGIN_NAMESPACE

#ifdef MATH_SSE

#define simd4f __m128
#define simd4i __m128i

#define add_ps _mm_add_ps
#define sub_ps _mm_sub_ps
#define mul_ps _mm_mul_ps
#define div_ps _mm_div_ps
#define set1_ps _mm_set1_ps
/// Sets the vector in order (w, z, y, x).
#define set_ps _mm_set_ps
static const simd4f simd4fSignBit = set1_ps(-0.f); // -0.f = 1 << 31
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
#define storeu_ps _mm_storeu_ps
#define store_ps _mm_store_ps
#define loadu_ps _mm_loadu_ps
#define load_ps _mm_load_ps
#define load1_ps _mm_load1_ps
#define stream_ps _mm_stream_ps

#if defined(MATH_SSE2) && !defined(MATH_AVX) // We can use the pshufd instruction, which was introduced in SSE2 32-bit integer ops.
/// Swizzles/permutes a single SSE register into another SSE register. Requires SSE2.
#define shuffle1_ps(reg, shuffle) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((reg)), (shuffle)))
#else // We only have SSE 1, so must use the slightly worse shufps instruction, which always destroys the input operand - or we have AVX where we can use this operation without destroying input
#define shuffle1_ps(reg, shuffle) _mm_shuffle_ps((simd4f)(reg), (simd4f)(reg), (shuffle))
#endif

#define xxxx_ps(x) shuffle1_ps((x), _MM_SHUFFLE(0,0,0,0))
#define yyyy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,1,1,1))
#define zzzz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,2,2,2))
#define wwww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,3,3))

#ifdef MATH_SSE41
#define allzero_ps(x) _mm_testz_si128(_mm_castps_si128((x)), _mm_castps_si128((x)))
#elif defined(MATH_SSE)
// Given a input vector of either 0xFFFFFFFF or 0, returns a nonzero integer of all lanes were zero.
// Warning: this SSE1 version is more like "all finite without nans" instead of "allzero", because
// it does not detect finite non-zero floats. Call only for inputs that are either all 0xFFFFFFFF or 0.
int FORCE_INLINE allzero_ps(simd4f x)
{
	simd4f y = yyyy_ps(x);
	x = or_ps(x, y);
	y = _mm_movehl_ps(y, x);
	x = or_ps(x, y);
	return _mm_ucomige_ss(x, x);
}
#endif

static inline __m128 load_vec3(const float *ptr, float w)
{
	__m128 low = _mm_loadl_pi(_mm_setzero_ps(), (const __m64*)ptr); // [_ _ y x]
	__m128 high = _mm_load_ss(ptr + 2); // [_ _ _ z]
	high = _mm_unpacklo_ps(high, _mm_set_ss(w)); // [_ _ w z]
	return _mm_movelh_ps(low, high);
}

static inline void store_vec3(float *ptr, simd4f v)
{
	_mm_storel_pi((__m64*)ptr, v);
	v = _mm_movehl_ps(v, v);
	_mm_store_ss(ptr+2, v);
}

// Note: Unlike SSE1 _mm_rcp_ps, which has a measured relative error of 3e-4f, the
//       rcp_ps function is more precise, and has a measured relative error of 7e-6f.
static inline __m128 rcp_ps(__m128 x)
{
	simd4f e = _mm_rcp_ps(x);
	// Do one iteration of Newton-Rhapson: e_n = 2*e - x*e^2
	// Note: This is not as precise as computing the exact 1.0f / x, but
	// not because of too few N-R iterations, but because of single-precision
	// FP errors being accumulated during the iterations. Even with having,
	// say, 10 iterations, this function cannot get the exact result 1.0f / x
	// but carries a small (order of 7e-6f) relative error.
	return sub_ps(add_ps(e, e), mul_ps(x, mul_ps(e,e)));
}

// Note: Unlike SSE1 _mm_rsqrt_ps, which has a measured relative error of 1e-4f, the
//       rsqrt_ps function is more precise, and has a measured relative error of 1e-8f.
static inline __m128 rsqrt_ps(__m128 x)
{
	// _mm_rsqrt_ps is not precise, so do one iteration of Newton-Rhapson to correct: e_n = e + 0.5 * (e - x * e^3)
	simd4f e = _mm_rsqrt_ps(x);
	simd4f e3 = mul_ps(mul_ps(e,e),e);
	simd4f half = set1_ps(0.5f);
	return add_ps(e, mul_ps(half, sub_ps(e, mul_ps(x, e3))));
}

#define sqrt_ps _mm_sqrt_ps
#define cmpeq_ps _mm_cmpeq_ps
#define cmpge_ps _mm_cmpge_ps
#define cmpgt_ps _mm_cmpgt_ps
#define cmple_ps _mm_cmple_ps
#define cmplt_ps _mm_cmplt_ps
#define negate3_ps(x) xor_ps(x, sseSignMask3)

/// Returns the lowest element of the given sse register as a float.
/// @note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast", since
/// the resulting float is represented in an XMM register as well.
#define s4f_x(s4f) _mm_cvtss_f32((s4f))
// Given a 4-channel single-precision simd4f variable [w z y x], returns the second channel 'y' as a float.
#define s4f_y(s4f) _mm_cvtss_f32(shuffle1_ps((s4f), _MM_SHUFFLE(1,1,1,1)))
// Given a 4-channel single-precision simd4f variable [w z y x], returns the third channel 'z' as a float.
// @note The intrinsic _mm_movehl_ps() is theoretically better than _mm_unpackhi_ps() or a shuffle to extract the z channel because it has throughput latency of 0.33 compared to 1.0 that the
//       other have. However using _mm_movehl_ps() is tricky since in a blind use, VS2013 compiler easily emits a redundant movaps when targeting pre-AVX, so it might not be worth it.
#define s4f_z(s4f) _mm_cvtss_f32(_mm_unpackhi_ps((s4f), (s4f)))
// Given a 4-channel single-precision simd4f variable [w z y x], returns the fourth channel 'w' as a float.
#define s4f_w(s4f) _mm_cvtss_f32(shuffle1_ps((s4f), _MM_SHUFFLE(3,3,3,3)))

#ifdef MATH_SSE2
#define set_ps_hex(w, z, y, x) _mm_castsi128_ps(_mm_set_epi32(w, z, y, x))
#define set1_ps_hex(x) _mm_castsi128_ps(_mm_set1_epi32(x))
#else
#define set_ps_hex(w, z, y, x) _mm_set_ps(ReinterpretAsFloat(w), ReinterpretAsFloat(z), ReinterpretAsFloat(y), ReinterpretAsFloat(x))
#define set1_ps_hex(x) _mm_set1_ps(ReinterpretAsFloat(x))
#endif

/// Returns the simd vector [_, _, _, f], that is, a SSE variable with the given float f in the lowest index.
/** The three higher indices should all be treated undefined.
	@note When compiling with /arch:SSE or newer, it is expected that this function is a no-op "cast" if the given 
	float is already in a register, since it will lie in an XMM register already. Check the disassembly to confirm!
	@note Never use this function if you need to generate a 4-vector [f,f,f,f]. Instead, use set1_ps(f), which
		generates a vmovss+vhufps and no redundant vxorps+vmovss! */
FORCE_INLINE simd4f setx_ps(float f)
{
	// On VS2010+AVX generates vmovss+vxorps+vmovss
	// return _mm_load_ss(&f);

#if _MSC_VER < 1700 // == VS2012
	// On VS2010+AVX generates vmovss+vshufps (to broadcast the single element to all channels). Best performance so far for VS2010.
	// On VS2013 generates a vbroadcastss instruction.
	return set1_ps(f);
#else
	// On VS2010+AVX is the same as _mm_load_ss, i.e. vmovss+vxorps+vmovss
	// On VS2013, this is the perfect thing - a single vmovss instruction!
	return _mm_set_ss(f);
#endif

	// On VS2010+AVX generates vmovss reg <- mem, vmovss alignedmem <- reg, vmovaps reg <- alignedmem, so is the worst!
	// simd4f s;
	// s.m128_f32[0] = f;
	// return s;
}

/// Returns a direction vector (w == 0) with xyz all set to the same scalar value.
FORCE_INLINE simd4f dir_from_scalar_ps(float scalar)
{
	return set_ps(0.f, scalar, scalar, scalar);
}

/// Returns a position vector (w == 1) with xyz all set to the same scalar value.
FORCE_INLINE simd4f pos_from_scalar_ps(float scalar)
{
	return set_ps(1.f, scalar, scalar, scalar);
}

// Given four scalar SS FP registers, packs the four values into a single SP FP register.
//inline simd4f pack_4ss_to_ps(simd4f x, simd4f y, simd4f z, simd4f w) // VS2010 BUG! Can't use this signature!
FORCE_INLINE simd4f pack_4ss_to_ps(simd4f x, simd4f y, simd4f z, const simd4f &w)
{
	simd4f xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	simd4f zw = _mm_movelh_ps(z, w); // zw = [ _, w, _, z]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]
}

// Given m = [W z y x], and a float w, returns [w z y x]. That is, sets the highest channel of the given 4-channel vector.
static FORCE_INLINE __m128 setw_ps(__m128 m, float w)
{
	__m128 hi = _mm_movehl_ps(m, m); // [W z W z]
	hi = _mm_unpacklo_ps(hi, _mm_set_ss(w)); // [0 W w z]
	return _mm_movelh_ps(m, hi); // [w z y x]
}

#ifdef MATH_SSE2
FORCE_INLINE simd4f modf_ps(simd4f x, simd4f mod)
{
	// x % mod == x - floor(x/mod)*mod
	// floor(x/mod) = integerpart(x/mod)
	simd4f ints = _mm_div_ps(x, mod);
#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
	simd4f integerpart = _mm_round_ps(ints, _MM_FROUND_TO_ZERO);
#else
	simd4f integerpart = _mm_cvtepi32_ps(_mm_cvttps_epi32(ints));
#endif
	return _mm_sub_ps(x, _mm_mul_ps(integerpart, mod));
}
#endif

#elif defined(MATH_NEON)

#include <arm_neon.h>

#define simd4f float32x4_t
#define simd4i int32x4_t

#define add_ps vaddq_f32
#define sub_ps vsubq_f32
#define mul_ps vmulq_f32
#define div_ps(a, b) ((a) / (b))
#define min_ps vminq_f32
#define max_ps vmaxq_f32
#define s4f_to_s4i(s4f) vreinterpretq_u32_f32((s4f))
#define s4i_to_s4f(s4i) vreinterpretq_f32_u32((s4i))
#define and_ps(x, y) s4i_to_s4f(vandq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define andnot_ps(x, y) s4i_to_s4f(vbicq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define or_ps(x, y) s4i_to_s4f(vorrq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define xor_ps(x, y) s4i_to_s4f(veorq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define ornot_ps(x, y) s4i_to_s4f(vornq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))

#define s4f_x(vec) vgetq_lane_f32((vec), 0)
#define s4f_y(vec) vgetq_lane_f32((vec), 1)
#define s4f_z(vec) vgetq_lane_f32((vec), 0)
#define s4f_w(vec) vgetq_lane_f32((vec), 1)

#define set1_ps vdupq_n_f32
#define setx_ps vdupq_n_f32
#define abs_ps vabsq_f32
#define zero_ps() vdupq_n_f32(0.f)

#define storeu_ps vst1q_f32
#define store_ps vst1q_f32
#define loadu_ps vld1q_f32
#define load_ps vld1q_f32
#define load1_ps(ptr) vdupq_n_f32(*(float*)(ptr))
#define stream_ps vst1q_f32
static inline simd4f rcp_ps(simd4f x)
{
	simd4f e = vrecpeq_f32(x);
	e = vmulq_f32(e, vrecpsq_f32(x, e));
	e = vmulq_f32(e, vrecpsq_f32(x, e));
	return e;
}

static inline simd4f rsqrt_ps(simd4f x)
{
	simd4f e = vrsqrteq_f32(x);
	e = vmulq_f32(e, vrsqrtsq_f32(x, vmulq_f32(e, e)));
	e = vmulq_f32(e, vrsqrtsq_f32(x, vmulq_f32(e, e)));
	return e;
}

static inline simd4f sqrt_ps(simd4f x) { return mul_ps(x, rsqrt_ps(x)); }

#define cmpeq_ps(a, b) vreinterpretq_f32_u32(vceqq_u32(vreinterpretq_u32_f32((a)), vreinterpretq_u32_f32((b))))
#define cmpge_ps(a, b) vreinterpretq_f32_u32(vcgeq_u32(vreinterpretq_u32_f32((a)), vreinterpretq_u32_f32((b))))
#define cmpgt_ps(a, b) vreinterpretq_f32_u32(vcgtq_u32(vreinterpretq_u32_f32((a)), vreinterpretq_u32_f32((b))))
#define cmple_ps(a, b) vreinterpretq_f32_u32(vcleq_u32(vreinterpretq_u32_f32((a)), vreinterpretq_u32_f32((b))))
#define cmplt_ps(a, b) vreinterpretq_f32_u32(vcltq_u32(vreinterpretq_u32_f32((a)), vreinterpretq_u32_f32((b))))

// This might not be the most efficient form, and typically it is better to avoid this in NEON, and instead
// prefer the scattering/gathering loads and stores instead.
#define _MM_TRANSPOSE4_PS(a,b,c,d) do { \
	float32x4x2_t m1 = vuzpq_f32((a), (c)); \
	float32x4x2_t m2 = vuzpq_f32((b), (d)); \
	float32x4x2_t m3 = vtrnq_f32(m1.val[0], m2.val[0]); \
	float32x4x2_t m4 = vtrnq_f32(m1.val[1], m2.val[1]); \
	(a) = m3.val[0]; \
	(b) = m4.val[0]; \
	(c) = m3.val[1]; \
	(d) = m4.val[1]; } while(0)

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

FORCE_INLINE simd4f dir_from_scalar_ps(float scalar)
{
	return vsetq_lane_f32(0.f, vdupq_n_f32(scalar), 3);
}

/// Returns a position vector (w == 1) with xyz all set to the same scalar value.
FORCE_INLINE simd4f pos_from_scalar_ps(float scalar)
{
	return vsetq_lane_f32(1.f, vdupq_n_f32(scalar), 3);
}

#endif // ~MATH_NEON

// TODO: Which is better codegen - use simd4fZero constant everywhere, or explicitly refer to zero_ps() everywhere,
//       or does it matter?
const simd4f simd4fZero     = zero_ps();
const simd4f simd4fOne      = set1_ps(1.f);
const simd4f simd4fMinusOne = set1_ps(-1.f);
const simd4f simd4fEpsilon  = set1_ps(1e-4f);

// NOTE: This version was benchmarked to be minutely better than the sub_ps(zero_ps) version on a SSE 4.1 capable system in OBB::ClosestPoint(point):
// sub_ps&zero_ps: Best: 8.833 nsecs / 24 ticks, Avg: 9.044 nsecs, Worst: 9.217 nsecs
// xor_ps&signMask: Best: 8.833 nsecs / 23.768 ticks, Avg: 8.975 nsecs, Worst: 9.601 nsecs
// However the memory load is still worrying, so using the zero_ps still for now.
//#define negate_ps(x) _mm_xor_ps(x, sseSignMask)

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

static const float andMaskOneF = ReinterpretAsFloat(0xFFFFFFFFU);
/// A SSE mask register with x = y = z = 0xFFFFFFFF and w = 0x0.
static const simd4f sseMaskXYZ = set_ps(0.f, andMaskOneF, andMaskOneF, andMaskOneF);
static const simd4f sseSignMask3 = set_ps(0.f, -0.f, -0.f, -0.f); // -0.f = 1 << 31
static const simd4f sseSignMask = set_ps(-0.f, -0.f, -0.f, -0.f); // -0.f = 1 << 31
#ifdef MATH_AVX
static const __m256 sseSignMask256 = _mm256_set1_ps(-0.f); // -0.f = 1 << 31
#endif
#ifdef MATH_AVX
#define abs_ps256(x) _mm256_andnot_ps(sseSignMask256, x)
#endif

MATH_END_NAMESPACE

#endif // ~MATH_SIMD
