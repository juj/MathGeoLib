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
#define muls_ps(vec, scalar) mul_ps((vec), set1_ps((scalar)))
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

// NOTE: This version was benchmarked to be minutely better than the sub_ps(zero_ps) version on a SSE 4.1 capable system in OBB::ClosestPoint(point):
// sub_ps&zero_ps: Best: 8.833 nsecs / 24 ticks, Avg: 9.044 nsecs, Worst: 9.217 nsecs
// xor_ps&signMask: Best: 8.833 nsecs / 23.768 ticks, Avg: 8.975 nsecs, Worst: 9.601 nsecs
// However the memory load is still worrying, so using the zero_ps still for now.
//#define neg_ps(x) _mm_xor_ps((x), sseSignMask)
#define neg_ps(x) sub_ps(zero_ps(), (x))

#if defined(MATH_SSE2) && !defined(MATH_AVX) // We can use the pshufd instruction, which was introduced in SSE2 32-bit integer ops.
/// Swizzles/permutes a single SSE register into another SSE register. Requires SSE2. This has the advantage of not destroying the input operand, but the disadvantage is that it requires a
/// float->int->float pipe transition, which costs a clock cycle. Profiling shows this to be a very small win.
#define shuffle1_ps(reg, shuffle) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((reg)), (shuffle)))
#else // We only have SSE 1, so must use the slightly worse shufps instruction, which always destroys the input operand - or we have AVX where we can use this operation without destroying input
#define shuffle1_ps(reg, shuffle) _mm_shuffle_ps((simd4f)(reg), (simd4f)(reg), (shuffle))
#endif

// Broadcast a single channel to all channels
#define xxxx_ps(x) shuffle1_ps((x), _MM_SHUFFLE(0,0,0,0))
#define yyyy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,1,1,1))
#define zzzz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,2,2,2))
#define wwww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,3,3))
// Duplicate lo to hi or hi to lo
#define xyxy_ps(x) _mm_movelh_ps((x), (x))
#define zwzw_ps(x) _mm_movehl_ps((x), (x))
// Swap elements in low pair, high pair, or both
#define yxzw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,2,0,1))
#define xywz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,3,1,0))
#define yxwz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,3,0,1))
// Swap low pair with high pair
#define zwxy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,0,3,2))
// Rotate lanes x->y, y->z, z->w, w->x or the other way around
#define yzwx_ps(x) shuffle1_ps((x), _MM_SHUFFLE(0,3,2,1))
#define wxyz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,1,0,3))
// Swap second and third element
#define xzyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,2,0))
// Swap second and third element, and then swap elements in low and high pairs
#define zxwy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,3,0,2))
// Reverse whole vector
#define wzyx_ps(x) shuffle1_ps((x), _MM_SHUFFLE(0,1,2,3))
// Duplicate two elements
#define xxyy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,1,0,0)) // This is the same as _mm_unpacklo_ps, but pshufd is probably better(?)
#define xxww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,0,0))
#define yyzz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,2,1,1))
#define zzww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,2,2)) // This the same as _mm_unpackhi_ps, but pshufd is probably better(?)
#define xzxz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,0,2,0))
#define ywyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,3,1))
#ifdef MATH_SSE3
// _mm_moveldup_ps and _mm_movehdup_ps are better than shuffle, since they don't destroy the input operands (under non-AVX).
#define xxzz_ps(x) _mm_moveldup_ps((x))
#define yyww_ps(x) _mm_movehdup_ps((x))
#else
#define xxzz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,2,0,0))
#define yyww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,1,1))
#endif
#define yxxy_ps(x) shuffle1_ps((x), _MM_SHUFFLE(1,0,0,1))
#define wxxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,0,3))
// Rotate x->y->z or the other direction, but leave w intact in the highest channel.
#define yzxw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,0,2,1))
#define zxyw_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,1,0,2))
// Other random-looking swizzles
#define ywxz_ps(x) shuffle1_ps((x), _MM_SHUFFLE(2,0,3,1))
#define zwww_ps(x) shuffle1_ps((x), _MM_SHUFFLE(3,3,3,2))
#define zzyx_ps(x) shuffle1_ps((x), _MM_SHUFFLE(0,1,2,2))

// Duplicates the lowest channel of 'a' twice to low pair, and lowest channel of 'b' twice, to high pair. I.e. returns [b.x, b.x, a.x, a.x].
#define axx_bxx_ps(a, b) _mm_shuffle_ps((a), (b), _MM_SHUFFLE(0,0,0,0))
#define ayy_byy_ps(a, b) _mm_shuffle_ps((a), (b), _MM_SHUFFLE(1,1,1,1)) // [b.y, b.y, a.y, a.y]
#define azz_bzz_ps(a, b) _mm_shuffle_ps((a), (b), _MM_SHUFFLE(2,2,2,2)) // [b.z, b.z, a.z, a.z]

#ifdef MATH_SSE2
#define simd2d __m128d

#define add_pd _mm_add_pd
#define sub_pd _mm_sub_pd
#define mul_pd _mm_mul_pd
#define div_pd _mm_div_pd
#define set1_pd _mm_set1_pd
/// Sets the vector in order (y, x).
#define set_pd _mm_set_pd
static const simd2d simd2dSignBit = set1_pd(-0.f); // -0.f = 1 << 31
#define abs_pd(x) _mm_andnot_pd(simd2dSignBit, (x))
#define zero_pd() _mm_setzero_pd()
#define min_pd _mm_min_pd
#define max_pd _mm_max_pd
#define s2d_to_s4i(s2d) _mm_castpd_si128((s4f))
#define s4i_to_s2d(s2i) _mm_castsi128_pd((s4i))
#define and_pd _mm_and_pd
#define andnot_pd _mm_andnot_pd
#define or_pd _mm_or_pd
#define xor_pd _mm_xor_pd
#define storeu_pd _mm_storeu_pd
#define store_pd _mm_store_pd
#define loadu_pd _mm_loadu_pd
#define load_pd _mm_load_pd
#ifdef MATH_SSE3
// _mm_loaddup_pd == movddup, single instruction
#define load1_pd _mm_loaddup_pd
#else
// _mm_load1_pd == movaps + shuffle, two instructions
#define load1_pd _mm_load1_pd
#endif
#define stream_pd _mm_stream_pd
#define neg_pd(x) xor_pd((x), simd2dSignBit)

#define shuffle1_pd(reg, shuffle) _mm_shuffle_pd((reg), (reg), (shuffle))
#ifdef MATH_SSE3
// _mm_movedup_pd is better than _mm_unpacklo_pd, since it does not destroy the source operand (under non-AVX).
#define xx_pd(x) _mm_movedup_pd((x))
#else
#define xx_pd(x) _mm_unpacklo_pd((x), (x))
#endif
#define yy_pd(x) _mm_unpackhi_pd((x), (x))
#define yx_pd(x) shuffle1_pd((x), _MM_SHUFFLE2(0, 1))
#endif

#ifdef MATH_SSE41
// Returns true if all the bits in the given float vector are zero, when interpreted as an integer.
// Warning: this SSE1 version is more like "all finite without nans" instead of "allzero", because
// it does not detect finite non-zero floats. Call only for inputs that are either all 0xFFFFFFFF or 0.
#define allzero_ps(x) _mm_testz_si128(_mm_castps_si128((x)), _mm_castps_si128((x)))
// Returns true if all the bits in the given float are set.
#define allone_ps(x) _mm_test_all_ones(_mm_castps_si128((x)))
// Returns true if all the bits in (a&b) are zero.
#define a_and_b_allzero_ps(a, b) _mm_testz_si128(_mm_castps_si128(a), _mm_castps_si128(b))
// Returns true if all the bits in (a&~b) are zero.
#define a_and_not_b_allzero_ps(a, b) _mm_testc_si128(_mm_castps_si128(a), _mm_castps_si128(b))
#else
int FORCE_INLINE allzero_ps(simd4f x)
{
	simd4f y = _mm_movehl_ps(x, x);
	x = or_ps(x, y);
#if defined(MATH_SSE2) && defined(MATH_64BIT)
	return _mm_cvtsi128_si64(_mm_castps_si128(x)) == 0;
#else
	y = yyyy_ps(x);
	x = or_ps(x, y);
#ifdef _DEBUG
	// In this construction in SSE1, we can't detect NaNs, so test that those don't occur.
	assume(ReinterpretAsU32(_mm_cvtss_f32(x)) == 0xFFFFFFFF || !IsNan(ReinterpretAsU32(_mm_cvtss_f32(x))));
#endif
	return _mm_ucomige_ss(x, x);
#endif
}
int FORCE_INLINE allone_ps(simd4f x)
{
	simd4f y = _mm_movehl_ps(x, x);
	x = and_ps(x, y);
#if defined(MATH_SSE2) && defined(MATH_64BIT)
	return _mm_cvtsi128_si64(_mm_castps_si128(x)) == -1LL;
#else
	y = yyyy_ps(x);
	x = and_ps(x, y);
#ifdef _DEBUG
	// In this construction in SSE1, we can't detect NaNs, so test that those don't occur.
	assume(ReinterpretAsU32(_mm_cvtss_f32(x)) == 0xFFFFFFFF || !IsNan(ReinterpretAsU32(_mm_cvtss_f32(x))));
#endif
	return !_mm_ucomige_ss(x, x);
#endif
}
#define a_and_b_allzero_ps(a, b) allzero_ps(and_ps((a), (b)))
#define a_and_not_b_allzero_ps(a, b) allzero_ps(andnot_ps((a), (b)))
#endif
// For convenience, define the following aliases to help ease reading SIMD conditionals.
#define anyzero_ps(x) (!allone_ps((x))
#define anyone_ps(x) (!allzero_ps(x))

// Multiply-add. These are otherwise identical, except that the FMA version is specced to have
// better precision with respect to rounding.
#ifdef MATH_FMA
// Multiply-add: a*b + c
#define madd_ps _mm_fmadd_ps
// Multiply-negate-add: c - a*b
#define mnadd_ps _mm_fnmadd_ps
// Multiply-sub: a*b - c
#define msub_ps _mm_fmsub_ps
// Multiply-negate-sub: -c - a*b
#define mnsub_ps _mm_fnmsub_ps
#else
#define madd_ps(a, b, c) add_ps(mul_ps((a), (b)), (c))
#define mnadd_ps(a, b, c) sub_ps((c), mul_ps((a), (b)))
#define msub_ps(a, b, c) sub_ps(mul_ps((a), (b)), (c))
#define mnsub_ps(a, b, c) sub_ps(neg_ps(mul_ps((a), (b))), (c))
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

#define comieq_ss _mm_comieq_ss

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

#define s2d_x(s2d) _mm_cvtsd_f64((s2d))
#define s2d_y(s2d) _mm_cvtsd_f64(_mm_unpackhi_pd((s2d), (s2d)))

// Given a 4-channel single-precision simd4f variable [w z y x], returns the second channel 'y' as a float.
#define s4f_y(s4f) _mm_cvtss_f32(shuffle1_ps((s4f), _MM_SHUFFLE(1,1,1,1)))

#define set_ps_hex(w, z, y, x) _mm_castsi128_ps(_mm_set_epi32(w, z, y, x))
#define set1_ps_hex(x) _mm_castsi128_ps(_mm_set1_epi32(x))

#elif defined(__EMSCRIPTEN__)
// Workaround a JS engine limitation that it's not possible to store arbitrary bit patterns in floats since JS engines destroy them while canonicalizing NaNs.
FORCE_INLINE __m128 set_ps_hex(u32 w, u32 z, u32 y, u32 x)
{
	union {
		u32 v[4];
		__m128 m;
	} u;
	u.v[0] = x;
	u.v[1] = y;
	u.v[2] = z;
	u.v[3] = w;
	return u.m;
}
FORCE_INLINE __m128 set1_ps_hex(u32 x)
{
	union {
		u32 v[4];
		__m128 m;
	} u;
	u.v[0] = u.v[1] = u.v[2] = u.v[3] = x;
	return u.m;
}

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

#if _MSC_VER < 1700 // < VS2012
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

/// Replaces the x channel (lowest lane) in the given vector with the given float x, i.e. returns [vec.w, vec.z, vec.y, x]
FORCE_INLINE simd4f setx_ps(const simd4f &vec, float x)
{
	return _mm_move_ss(vec, _mm_set_ss(x));
}

/// Replaces the y channel in the given vector with the given float y, i.e. returns [vec.w, vec.z, y, vec.x]
FORCE_INLINE simd4f sety_ps(const simd4f &vec, float y)
{
	simd4f tmp = _mm_set_ss(y); // [0, 0, 0, y]
	tmp = _mm_shuffle_ps(tmp, vec, _MM_SHUFFLE(0, 0, 0, 0)); // [vec.x, vec.x, y, y]
	return _mm_shuffle_ps(tmp, vec, _MM_SHUFFLE(3, 2, 0, 2)); // [vec.w, vec.z, y, vec.x]
}

/// Replaces the z channel in the given vector with the given float z, i.e. returns [vec.w, z, vec.y, vec.x]
FORCE_INLINE simd4f setz_ps(const simd4f &vec, float z)
{
	simd4f tmp = _mm_set_ss(z); // [0, 0, 0, z]
	tmp = _mm_shuffle_ps(tmp, vec, _MM_SHUFFLE(3, 3, 0, 0)); // [vec.w, vec.w, z, z]
	return _mm_shuffle_ps(vec, tmp, _MM_SHUFFLE(3, 0, 1, 0)); // [vec.w, z, vec.y, vec.x]
}

/// Replaces the w channel in the given vector with the given float w, i.e. returns [w, vec.z, vec.y, vec.x]
FORCE_INLINE simd4f setw_ps(const simd4f &vec, float w)
{
	simd4f tmp = _mm_set_ss(w); // [0, 0, 0, w]
	tmp = _mm_shuffle_ps(tmp, vec, _MM_SHUFFLE(2, 2, 0, 0)); // [vec.z, vec.z, w, w]
	return _mm_shuffle_ps(vec, tmp, _MM_SHUFFLE(0, 3, 1, 0)); // [w, vec.z, vec.y, vec.x]
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

// Returns the vector [a.x+a.y+a.z+a.w, b.x+b.y+b.z+b.w, c.x+c.y+c.z+c.w, d.x+d.y+d.z+d.w]
#if defined(_MSC_VER) && defined(MATH_SSE) && _MSC_VER < 1800 // < VS2013
// Work around a VS2010 bug "error C2719: 'd': formal parameter with __declspec(align('16')) won't be aligned"
FORCE_INLINE simd4f hadd4_ps(simd4f a, simd4f b, simd4f c, const simd4f &d)
#else
FORCE_INLINE simd4f hadd4_ps(simd4f a, simd4f b, simd4f c, simd4f d)
#endif
{
	simd4f t0 = _mm_unpacklo_ps(a, b);
	simd4f t1 = _mm_unpackhi_ps(a, b);
	t0 = add_ps(t0, t1);
	simd4f t2 = _mm_unpacklo_ps(c, d);
	simd4f t3 = _mm_unpackhi_ps(c, d);
	t2 = add_ps(t2, t3);
	return add_ps(_mm_movelh_ps(t0, t2), _mm_movehl_ps(t2, t0));
}

#elif defined(MATH_NEON)

#include <arm_neon.h>

#define simd4f float32x4_t
#define simd4i int32x4_t

#define add_ps vaddq_f32
#define sub_ps vsubq_f32
#define neg_ps vnegq_f32
#define mul_ps vmulq_f32
#define muls_ps vmulq_n_f32
#define min_ps vminq_f32
#define max_ps vmaxq_f32
#define s4f_to_s4i(s4f) vreinterpretq_u32_f32((s4f))
#define s4i_to_s4f(s4i) vreinterpretq_f32_u32((s4i))
#define and_ps(x, y) s4i_to_s4f(vandq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define andnot_ps(x, y) s4i_to_s4f(vbicq_u32(s4f_to_s4i(y), s4f_to_s4i(x)))
#define or_ps(x, y) s4i_to_s4f(vorrq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define xor_ps(x, y) s4i_to_s4f(veorq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))
#define ornot_ps(x, y) s4i_to_s4f(vornq_u32(s4f_to_s4i(x), s4f_to_s4i(y)))

#define s4f_x(vec) vgetq_lane_f32((vec), 0)
#define s4f_y(vec) vgetq_lane_f32((vec), 1)
#define s4f_z(vec) vgetq_lane_f32((vec), 2)
#define s4f_w(vec) vgetq_lane_f32((vec), 3)

// These are all expected to compile down to a single 1-cycle instruction. Reference: http://community.arm.com/groups/processors/blog/2012/03/13/coding-for-neon--part-5-rearranging-vectors

// Broadcast a single lane to all lanes
#define xxxx_ps(a) vdupq_lane_f32(vget_low_f32((a)), 0)
#define yyyy_ps(a) vdupq_lane_f32(vget_low_f32((a)), 1)
#define zzzz_ps(a) vdupq_lane_f32(vget_high_f32((a)), 0)
#define wwww_ps(a) vdupq_lane_f32(vget_high_f32((a)), 1)
// Duplicate lo to hi or hi to lo
FORCE_INLINE simd4f xyxy_ps(simd4f vec) { float32x2_t xy = vget_low_f32(vec); return vcombine_f32(xy, xy); }
FORCE_INLINE simd4f zwzw_ps(simd4f vec) { float32x2_t zw = vget_high_f32(vec); return vcombine_f32(zw, zw); }
// Swap elements in low pair, high pair, or both
FORCE_INLINE simd4f yxzw_ps(simd4f vec) { float32x2_t xy = vget_low_f32(vec); float32x2_t zw = vget_high_f32(vec); return vcombine_f32(vrev64_f32(xy), zw); }
FORCE_INLINE simd4f xywz_ps(simd4f vec) { float32x2_t xy = vget_low_f32(vec); float32x2_t zw = vget_high_f32(vec); return vcombine_f32(xy, vrev64_f32(zw)); }
#define yxwz_ps vrev64q_f32
// Swap low pair with high pair
FORCE_INLINE simd4f zwxy_ps(simd4f vec) { float32x2_t xy = vget_low_f32(vec); float32x2_t zw = vget_high_f32(vec); return vcombine_f32(zw, xy); } // Should compiled down to a single "vswp d0, d1" instruction
// Rotate lanes x->y, y->z, z->w, w->x or the other way around
FORCE_INLINE simd4f yzwx_ps(simd4f vec) { uint32x4_t i = s4f_to_s4i(vec); return s4i_to_s4f(vextq_u32(i, i, 1)); }
FORCE_INLINE simd4f wxyz_ps(simd4f vec) { uint32x4_t i = s4f_to_s4i(vec); return s4i_to_s4f(vextq_u32(i, i, 3)); }
// Swap second and third element
FORCE_INLINE simd4f xzyw_ps(simd4f vec) { float32x2x2_t v = vtrn_f32(vget_low_f32(vec), vget_high_f32(vec)); return vcombine_f32(v.val[0], v.val[1]); }
// Swap second and third element, and then swap elements in low and high pairs
FORCE_INLINE simd4f zxwy_ps(simd4f vec) { float32x2x2_t v = vtrn_f32(vget_high_f32(vec), vget_low_f32(vec)); return vcombine_f32(v.val[0], v.val[1]); }
// Reverse whole vector (2 cycles, vrev64q followed by vswp)
#define wzyx_ps(vec) zwxy_ps(yxwz_ps((vec)))

// These are not quite 1-cycle swizzles, since they require duplicating the simd vector to another duplicate register to run, at least in GCC 4.6. (TODO: This might be avoidable with inline asm?)
FORCE_INLINE simd4f xxyy_ps(simd4f vec) { return vzipq_f32(vec, vec).val[0]; } // Also could be done with vget_low_f32+v{uzp/vzip/vtrn}_f32, would that be better?
#define yyzz_ps(vec) xxyy_ps(yzwx_ps((vec)))
FORCE_INLINE simd4f zzww_ps(simd4f vec) { return vzipq_f32(vec, vec).val[1]; }
FORCE_INLINE simd4f xxww_ps(simd4f vec) { float32x2_t xx = vdup_lane_f32(vget_low_f32(vec), 0); float32x2_t ww = vdup_lane_f32(vget_high_f32(vec), 1); return vcombine_f32(xx, ww); }
FORCE_INLINE simd4f xxzz_ps(simd4f vec) { return vtrnq_f32(vec, vec).val[0]; }
FORCE_INLINE simd4f yyww_ps(simd4f vec) { return vtrnq_f32(vec, vec).val[1]; }
FORCE_INLINE simd4f xzxz_ps(simd4f vec) { return vuzpq_f32(vec, vec).val[0]; }
FORCE_INLINE simd4f ywyw_ps(simd4f vec) { return vuzpq_f32(vec, vec).val[1]; }
#define yxxy_ps(vec) wxyz_ps(xxyy_ps((vec)))
#define wxxw_ps(vec) wxyz_ps(xxww_ps((vec)))
// Rotate x->y->z or the other direction, but leave w intact in the highest channel.
#define yzxw_ps(vec) xywz_ps(yzwx_ps(vec))
#define zxyw_ps(vec) xywz_ps(zxwy_ps(vec))
// Other random-looking swizzles
#define ywxz_ps(vec) xzyw_ps(yxwz_ps((vec)))
// TODO: Fix something better here
#define zwww_ps(vec) set_ps(s4f_w((vec)), s4f_w((vec)), s4f_w((vec)), s4f_z((vec)))
#define zzyx_ps(vec) set_ps(s4f_x((vec)), s4f_y((vec)), s4f_z((vec)), s4f_z((vec)))

// Duplicates the lowest channel of 'a' twice to low pair, and lowest channel of 'b' twice, to high pair. I.e. returns [b.x, b.x, a.x, a.x].
#define axx_bxx_ps(a, b) xxzz_ps(vcombine_f32(vget_low_f32((a)), vget_low_f32((b))))
#define ayy_byy_ps(a, b) yyww_ps(vcombine_f32(vget_low_f32((a)), vget_low_f32((b)))) // [b.y, b.y, a.y, a.y]
#define azz_bzz_ps(a, b) xxzz_ps(vcombine_f32(vget_high_f32((a)), vget_high_f32((b)))) // [b.z, b.z, a.z, a.z]

#define set1_ps vdupq_n_f32

FORCE_INLINE simd4f setx_ps(float f)
{
	return vdupq_n_f32(f);
}

FORCE_INLINE simd4f setx_ps(const simd4f &vec, float x)
{
	return vsetq_lane_f32(x, vec, 0);
}

#define sety_ps(vec, y) vsetq_lane_f32((y), (vec), 1)
#define setz_ps(vec, z) vsetq_lane_f32((z), (vec), 2)
#define setw_ps(vec, w) vsetq_lane_f32((w), (vec), 3)

#define abs_ps vabsq_f32
#define zero_ps() vdupq_n_f32(0.f)

#define storeu_ps vst1q_f32
#define store_ps vst1q_f32
#define loadu_ps vld1q_f32
#define load_ps vld1q_f32
#define load1_ps(ptr) vdupq_n_f32(*(float*)(ptr))
#define stream_ps vst1q_f32

#if defined(MATH_VFPv4) || defined(MATH_NEONv2)
// Multiply-add: a*b + c
#define madd_ps vmlaq_f32
// Multiply-negate-add: c - a*b == -(-c + a*b)
#define mnadd_ps(a, b, c) neg_ps(vmlsq_f32((a), (b), (c)))
// Multiply-sub: a*b - c
#define msub_ps vmlsq_f32
// Multiply-negate-sub: -c - a*b = -(c + a*b)
#define mnsub_ps(a, b, c) neg_ps(vmlaq_f32((a), (b), (c)))
#else
#define madd_ps(a, b, c) add_ps(mul_ps((a), (b)), (c))
#define mnadd_ps(a, b, c) add_ps(neg_ps(mul_ps((a), (b))), (c))
#define msub_ps(a, b, c) sub_ps(mul_ps((a), (b)), (c))
#define mnsub_ps(a, b, c) sub_ps(neg_ps(mul_ps((a), (b))), (c))
#endif

static FORCE_INLINE simd4f rcp_ps(simd4f x)
{
	simd4f e = vrecpeq_f32(x);
	e = vmulq_f32(e, vrecpsq_f32(x, e));
	e = vmulq_f32(e, vrecpsq_f32(x, e));
	return e;
}

#define div_ps(num, denom) mul_ps((num), rcp_ps((denom)))

static FORCE_INLINE simd4f rsqrt_ps(simd4f x)
{
	simd4f e = vrsqrteq_f32(x);
	e = vmulq_f32(e, vrsqrtsq_f32(x, vmulq_f32(e, e)));
	e = vmulq_f32(e, vrsqrtsq_f32(x, vmulq_f32(e, e)));
	return e;
}

#define cmpeq_ps(a, b) vreinterpretq_f32_u32(vceqq_f32((a), (b)))
#define cmpge_ps(a, b) vreinterpretq_f32_u32(vcgeq_f32((a), (b)))
#define cmpgt_ps(a, b) vreinterpretq_f32_u32(vcgtq_f32((a), (b)))
#define cmple_ps(a, b) vreinterpretq_f32_u32(vcleq_f32((a), (b)))
#define cmplt_ps(a, b) vreinterpretq_f32_u32(vcltq_f32((a), (b)))
#define comieq_ss(a, b) ((s4f_x((a)) == s4f_x((b))) ? 1 : 0)

FORCE_INLINE simd4f cmov_ps(simd4f a, simd4f b, simd4f mask);

static inline simd4f sqrt_ps(simd4f x) { return cmov_ps(mul_ps(x, rsqrt_ps(x)), zero_ps(), cmpeq_ps(x, zero_ps())); }

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

FORCE_INLINE simd4f load_vec3(const float *ptr, float w)
{
	float32x2_t low = vld1_f32(ptr); // [y x]
	float32x2_t high = (float32x2_t) { ptr[2], w }; // [w z]
	return vcombine_f32(low, high); // [w z y x]
}

FORCE_INLINE void store_vec3(float *ptr, simd4f v)
{
	vst1_f32(ptr, vget_low_f32(v)); // store x & y
	vst1q_lane_f32(ptr+2, v, 2); // store z
}

FORCE_INLINE uint32_t allzero_ps(simd4f v)
{
	float32x2_t xy = vget_low_f32(v);
	float32x2_t zw = vget_high_f32(v);
	uint32x2_t orr = vorr_u32(vreinterpret_u32_f32(xy), vreinterpret_u32_f32(zw));
	uint32_t a = vget_lane_u32(orr, 0);
	uint32_t b = vget_lane_u32(orr, 1);
	return ((a|b) == 0) ? 1 : 0;
}

#define a_and_b_allzero_ps(a, b) allzero_ps(and_ps((a), (b)))

FORCE_INLINE uint32_t allone_ps(simd4f v)
{
	float32x2_t xy = vget_low_f32(v);
	float32x2_t zw = vget_high_f32(v);
	uint32x2_t andd = vand_u32(vreinterpret_u32_f32(xy), vreinterpret_u32_f32(zw));
	uint32_t a = vget_lane_u32(andd, 0);
	uint32_t b = vget_lane_u32(andd, 1);
	return ((a&b) == 0xFFFFFFFFU) ? 1 : 0;
}

#define anyzero_ps(x) (!allone_ps((x))
#define anyone_ps(x) (!allzero_ps(x))

FORCE_INLINE simd4f hadd4_ps(simd4f a, simd4f b, simd4f c, simd4f d)
{
	// Most likely possible to do better.
	_MM_TRANSPOSE4_PS(a, b, c, d);
	return add_ps(add_ps(a, b), add_ps(c, d));
}

#endif // ~MATH_NEON

// TODO: Which is better codegen - use simd4fZero constant everywhere, or explicitly refer to zero_ps() everywhere,
//       or does it matter?
const simd4f simd4fZero     = zero_ps();
const simd4f simd4fOne      = set1_ps(1.f);
const simd4f simd4fMinusOne = set1_ps(-1.f);
const simd4f simd4fEpsilon  = set1_ps(1e-4f);

#define neg3_ps(x) xor_ps((x), sseSignMask3)

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

/// A SSE mask register with x = y = z = 0xFFFFFFFF and w = 0x0.
static const simd4f sseMaskXYZ = set_ps_hex(0, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU);
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
