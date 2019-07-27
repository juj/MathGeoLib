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

/** @file float4_neon.h
	@author Jukka Jylänki
	@brief ARM NEON code for float4-related computations. */

#pragma once

#ifdef MATH_SIMD

#ifdef MATH_NEON
#include <arm_neon.h>
#endif

#include "float4_sse.h"

MATH_BEGIN_NAMESPACE

#ifdef ANDROID
FORCE_INLINE void vec4_add_float_asm(const void *vec, float f, void *out)
{
	asm(
		"\t vld1.32 {d0, d1} [%1] \n"
		"\t vdup.32 q1, [%2] \n"
		"\t vadd.f32 q0, q0, q1 \n"
		"\t vst1.32 {d0, d1}, [%0] \n"
		: /* no outputs by value */
		:"r"(out), "r"(vec), "r"(f)
		:"q0", "q1");
}
#endif

FORCE_INLINE simd4f cross_ps(simd4f a, simd4f b)
{
	simd4f a_xzy = yzxw_ps(a); // = [a.w, a.x, a.z, a.y]
	simd4f b_xzy = yzxw_ps(b); // = [b.w, b.x, b.z, b.y]

	simd4f y_yxz = mul_ps(a_xzy, b); // [a.w*b.w, a.z*b.x, a.y*b.z, a.x*b.y]

	return yzxw_ps(msub_ps(b_xzy, a, y_yxz)); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]
}

FORCE_INLINE simd4f dot4_ps(simd4f a, simd4f b);

FORCE_INLINE void basis_ps(simd4f v, simd4f *outB, simd4f *outC)
{
	// Convert v to nonnegative (here could use mul_ps(v, v) for same effect to avoid loading
	// memory, but does not seem to have any measurable difference in profiling)
	simd4f a = abs_ps(v);
	simd4f a_min = min_ps(a, min_ps(yzxw_ps(a), zxyw_ps(a))); // Horizontal min of x,y,z
	simd4f q = cmple_ps(a, a_min); // Mask 0xFFFFFFFF to channels that contain the min element.
	// The mask in q represents a vector (1,0,0), (0,1,0), and (0,0,1) that is the one that's most perpendicular to input vector.
	simd4f v_xzy = yzxw_ps(v);
	simd4f q_xzy = yzxw_ps(q);
	simd4f b_yxz = sub_ps(and_ps(q_xzy, v), and_ps(v_xzy, q));
	simd4f b = yzxw_ps(b_yxz);
	simd4f c = msub_ps(b_yxz, v_xzy, zxyw_ps(mul_ps(v, b_yxz)));
	*outB = mul_ps(b, rsqrt_ps(dot3_ps(b, b)));
	*outC = mul_ps(c, rsqrt_ps(dot3_ps(c, c)));
}

#ifdef MATH_NEON
inline std::string ToString(uint8x8x2_t vec)
{
	uint8_t *v = (uint8_t*)&vec;
	char str[256];
	sprintf(str, "[%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X | %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X]",
		(int)v[15], (int)v[14], (int)v[13], (int)v[12], (int)v[11], (int)v[10], (int)v[9], (int)v[8], (int)v[7], (int)v[6], (int)v[5], (int)v[4], (int)v[3], (int)v[2], (int)v[1], (int)v[0]);
	return str;
}
#endif

#if defined(MATH_AVX) || defined(MATH_NEON)
FORCE_INLINE simd4f vec4_permute(simd4f vec, int i, int j, int k, int l)
{
#ifdef MATH_AVX
	return _mm_permutevar_ps(vec, _mm_set_epi32(l, k, j, i));
#elif defined(MATH_NEON)
	// N.B. Don't use: This has been benchmarked to be 3x slower than scalar CPU version!
	const uint8_t I = (uint8_t)i << 2;
	const uint8_t J = (uint8_t)j << 2;
	const uint8_t K = (uint8_t)k << 2;
	const uint8_t L = (uint8_t)l << 2;
	const ALIGN16 uint8_t indexData[16] = { I, I+1, I+2, I+3, J, J+1, J+2, J+3, K, K+1, K+2, K+3, L, L+1, L+2, L+3 };
	uint8x8x2_t indices;// = *(uint8x8x2_t*)&indexData;//vld2_u8(indexData);
	indices.val[0] = vld1_u8(indexData);
	indices.val[1] = vld1_u8(indexData+8);
	uint8x8x2_t src = *(uint8x8x2_t*)&vec;//vreinterpretq_s8_f32(vec);
	uint8x8x2_t dst;
	dst.val[0] = vtbl2_u8(src, indices.val[0]);
	dst.val[1] = vtbl2_u8(src, indices.val[1]);
	return *(simd4f*)&dst;
#endif
}
#endif

#ifdef MATH_NEON

FORCE_INLINE simd4f sum_xyzw_ps(simd4f vec)
{
	float32x2_t r = vadd_f32(vget_high_f32(vec), vget_low_f32(vec));
	r = vpadd_f32(r, r);
	return vcombine_f32(r, r);
}

FORCE_INLINE simd4f sum_xyz_ps3(simd4f m)
{
	return sum_xyzw_ps(vsetq_lane_f32(0.f, m, 3));
}
#define sum_xyz_ps sum_xyz_ps3

FORCE_INLINE float sum_xyzw_float(simd4f vec)
{
	return vgetq_lane_f32(sum_xyzw_ps(vec), 0);
}

FORCE_INLINE float mul_xyzw_float(simd4f vec)
{
	float32x2_t lo = vget_low_f32(vec);
	float32x2_t hi = vget_high_f32(vec);
	float32x2_t mul = vmul_f32(lo, hi);
	return vget_lane_f32(mul, 0) * vget_lane_f32(mul, 1); ///\todo Can this be optimized somehow?
}

FORCE_INLINE float sum_xyz_float(simd4f vec)
{
	return sum_xyzw_float(vsetq_lane_f32(0.f, vec, 3));
}

FORCE_INLINE float dot4_float(simd4f a, simd4f b)
{
	simd4f mul = mul_ps(a, b);
	return sum_xyzw_float(mul);
}

FORCE_INLINE float dot3_float(simd4f a, simd4f b)
{
	simd4f mul = mul_ps(a, b);
	return sum_xyz_float(mul);
}

FORCE_INLINE simd4f dot4_ps(simd4f a, simd4f b)
{
	return set1_ps(dot4_float(a, b));
}

FORCE_INLINE simd4f dot3_ps(simd4f a, simd4f b)
{
	return set1_ps(dot3_float(a, b));
}
#define dot3_ps3 dot3_ps

#endif // ~MATH_NEON

#ifdef ANDROID
FORCE_INLINE float vec4_length_sq_float_asm(const simd4f *vec)
{
	float len;
	asm(
		"\t vld1.32 {d0, d1}, [%1] \n"
		"\t vmul.f32 q0, q0, q0 \n"    // q0 = [ww zz yy xx]
		"\t vpadd.f32 d0, d0, d1 \n"   // d0 = [ww+zz yy+xx]
		"\t vadd.f32 %0, s0, s1 \n"    // s0 = [xx+yy+zz+ww]   ///\todo Doesn't work?
		:"=w"(len)
		:"r"(vec)
		:"q0");
	return len;
}

FORCE_INLINE void vec4_length_sq_ps_asm(const simd4f *vec, simd4f *out)
{
	asm(
		"\t vld1.32 {d0, d1}, [%1] \n"
		"\t vmul.f32 q0, q0, q0 \n"    // q0 = [ww zz yy xx]
		"\t vpadd.f32 d0, d0, d1 \n"   // d0 = [ww+zz yy+xx]
		"\t vrev64.32 d1, d0 \n"       // d1 = [yy+xx ww+zz]
		"\t vadd.f32 d0, d0, d1 \n"    // d0 = [xx+yy+ww+zz xx+yy+ww+zz]
		"\t vmov.f32 d1, d0 \n"
		"\t vst1.32 {d0, d1}, [%0] \n"
		:
		:"r"(out), "r"(vec)
		:"memory", "q0");
}
#endif

FORCE_INLINE float vec4_length_sq_float(simd4f vec)
{
	return dot4_float(vec, vec);
}

FORCE_INLINE simd4f vec4_length_sq_ps(simd4f vec)
{
	return dot4_ps(vec, vec);
}

FORCE_INLINE float vec3_length_sq_float(simd4f vec)
{
	return dot3_float(vec, vec);
}

FORCE_INLINE simd4f vec3_length_sq_ps(simd4f vec)
{
	return dot3_ps(vec, vec);
}

FORCE_INLINE float vec4_length_float(simd4f vec)
{
	return s4f_x(sqrt_ps(dot4_ps(vec, vec)));
}

FORCE_INLINE simd4f vec4_length_ps(simd4f vec)
{
	return sqrt_ps(dot4_ps(vec, vec));
}

FORCE_INLINE simd4f vec4_normalize(simd4f vec)
{
	return mul_ps(vec, rsqrt_ps(vec4_length_sq_ps(vec)));
}

FORCE_INLINE float vec3_length_float(simd4f vec)
{
	return s4f_x(sqrt_ps(dot3_ps3(vec, vec)));
}

FORCE_INLINE simd4f vec3_length_ps(simd4f vec)
{
	return sqrt_ps(dot3_ps(vec, vec));
}

FORCE_INLINE simd4f vec3_length_ps3(simd4f vec)
{
	return sqrt_ps(dot3_ps3(vec, vec));
}

FORCE_INLINE simd4f vec3_normalize(simd4f vec)
{
	return mul_ps(vec, rsqrt_ps(vec3_length_sq_ps(vec)));
}

// T should be a 4-vector containing the lerp weight [w,w,w,w] in all channels.
FORCE_INLINE simd4f vec4_lerp(simd4f a, simd4f b, simd4f t)
{
	// a*(1-t) + b*t = a - t*a + t*b = a + t*(b-a)
	return madd_ps(t, sub_ps(b, a), a);
}

FORCE_INLINE simd4f vec4_lerp(simd4f a, simd4f b, float t)
{
	return vec4_lerp(a, b, set1_ps(t));
}

MATH_END_NAMESPACE

#endif // ~MATH_SIMD
