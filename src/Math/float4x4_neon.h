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

#include "SSEMath.h"
#include "float4_neon.h"

// Multiplies m1 * m2, where m1 and m2 are stored in row-major format.
FORCE_INLINE void mat4x4_mul_mat4x4(simd4f *out, const simd4f *m1, const simd4f *m2)
{
#if defined(MATH_NEON)
	simd4f r1 = vmulq_lane_f32(m2[0], vget_low_f32(m1[0]), 0);
	simd4f r2 = vmulq_lane_f32(m2[0], vget_low_f32(m1[1]), 0);
	simd4f r3 = vmulq_lane_f32(m2[0], vget_low_f32(m1[2]), 0);
	simd4f r4 = vmulq_lane_f32(m2[0], vget_low_f32(m1[3]), 0);

	r1 = vmlaq_lane_f32(r1, m2[1], vget_low_f32(m1[0]), 1);
	r2 = vmlaq_lane_f32(r2, m2[1], vget_low_f32(m1[1]), 1);
	r3 = vmlaq_lane_f32(r3, m2[1], vget_low_f32(m1[2]), 1);
	r4 = vmlaq_lane_f32(r4, m2[1], vget_low_f32(m1[3]), 1);

	r1 = vmlaq_lane_f32(r1, m2[2], vget_high_f32(m1[0]), 0);
	r2 = vmlaq_lane_f32(r2, m2[2], vget_high_f32(m1[1]), 0);
	r3 = vmlaq_lane_f32(r3, m2[2], vget_high_f32(m1[2]), 0);
	r4 = vmlaq_lane_f32(r4, m2[2], vget_high_f32(m1[3]), 0);

	r1 = vmlaq_lane_f32(r1, m2[3], vget_high_f32(m1[0]), 1);
	r2 = vmlaq_lane_f32(r2, m2[3], vget_high_f32(m1[1]), 1);
	r3 = vmlaq_lane_f32(r3, m2[3], vget_high_f32(m1[2]), 1);
	r4 = vmlaq_lane_f32(r4, m2[3], vget_high_f32(m1[3]), 1);

	out[0] = r1;
	out[1] = r2;
	out[2] = r3;
	out[3] = r4;
#else
	mat4x4_mul_sse(out, m1, m2);
#endif
}

FORCE_INLINE void mat4x4_transpose(simd4f *out, const simd4f *mat)
{
#ifdef MATH_NEON
	float32x4x4_t m = vld4q_f32((const float32_t*)mat);
	vst1q_f32((float32_t*)out, m.val[0]);
	vst1q_f32((float32_t*)out+4, m.val[1]);
	vst1q_f32((float32_t*)out+8, m.val[2]);
	vst1q_f32((float32_t*)out+12, m.val[3]);
#else
	__m128 tmp0 = _mm_unpacklo_ps(mat[0], mat[1]);
	__m128 tmp2 = _mm_unpacklo_ps(mat[2], mat[3]);
	__m128 tmp1 = _mm_unpackhi_ps(mat[0], mat[1]);
	__m128 tmp3 = _mm_unpackhi_ps(mat[2], mat[3]);
	out[0] = _mm_movelh_ps(tmp0, tmp2);
	out[1] = _mm_movehl_ps(tmp2, tmp0);
	out[2] = _mm_movelh_ps(tmp1, tmp3);
	out[3] = _mm_movehl_ps(tmp3, tmp1);
#endif
}

FORCE_INLINE void mat4x4_set(simd4f *mat, float _00, float _01, float _02, float _03,
                                          float _10, float _11, float _12, float _13,
                                          float _20, float _21, float _22, float _23,
                                          float _30, float _31, float _32, float _33)
{
#ifdef MATH_AVX
	__m256 *mat2 = (__m256*)mat;
	mat2[0] = _mm256_set_ps(_13, _12, _11, _10, _03, _02, _01, _00);
	mat2[1] = _mm256_set_ps(_33, _32, _31, _30, _23, _22, _21, _20);
#else
	mat[0] = set_ps(_03, _02, _01, _00);
	mat[1] = set_ps(_13, _12, _11, _10);
	mat[2] = set_ps(_23, _22, _21, _20);
	mat[3] = set_ps(_33, _32, _31, _30);
#endif
}

FORCE_INLINE void mat4x4_mul_float(simd4f *out, simd4f *mat, float scalar)
{
#ifdef MATH_AVX
	__m256 s = _mm256_set1_ps(scalar);
	__m256 *o = (__m256*)out;
	__m256 *i = (__m256*)mat;
	o[0] = _mm256_mul_ps(i[0], s);
	o[1] = _mm256_mul_ps(i[1], s);
#else
	simd4f v = set1_ps(scalar);
	out[0] = mul_ps(mat[0], v);
	out[1] = mul_ps(mat[1], v);
	out[2] = mul_ps(mat[2], v);
	out[3] = mul_ps(mat[3], v);
#endif
}

FORCE_INLINE void mat4x4_div_float(simd4f *out, simd4f *mat, float scalar)
{
#ifdef MATH_AVX
	__m256 *o = (__m256*)out;
	__m256 *i = (__m256*)mat;
	__m256 s = _mm256_set1_ps(scalar);
	__m256 one = _mm256_set1_ps(1.f);
	s = _mm256_div_ps(one, s);
	o[0] = _mm256_mul_ps(i[0], s);
	o[1] = _mm256_mul_ps(i[1], s);
#else
	simd4f s = set1_ps(scalar);
	simd4f one = set1_ps(1.f);
	s = div_ps(one, s);
	out[0] = mul_ps(mat[0], s);
	out[1] = mul_ps(mat[1], s);
	out[2] = mul_ps(mat[2], s);
	out[3] = mul_ps(mat[3], s);
#endif
}

FORCE_INLINE void mat4x4_add_mat4x4(simd4f *out, simd4f *m1, simd4f *m2)
{
#ifdef MATH_AVX
	__m256 *o = (__m256*)out;
	__m256 *i1 = (__m256*)m1;
	__m256 *i2 = (__m256*)m2;
	o[0] = _mm256_add_ps(i1[0], i2[0]);
	o[1] = _mm256_add_ps(i1[1], i2[1]);
#else
	out[0] = add_ps(m1[0], m2[0]);
	out[1] = add_ps(m1[1], m2[1]);
	out[2] = add_ps(m1[2], m2[2]);
	out[3] = add_ps(m1[3], m2[3]);
#endif
}

FORCE_INLINE void mat4x4_sub_mat4x4(simd4f *out, simd4f *m1, simd4f *m2)
{
#ifdef MATH_AVX
	__m256 *o = (__m256*)out;
	__m256 *i1 = (__m256*)m1;
	__m256 *i2 = (__m256*)m2;
	o[0] = _mm256_sub_ps(i1[0], i2[0]);
	o[1] = _mm256_sub_ps(i1[1], i2[1]);
#else
	out[0] = sub_ps(m1[0], m2[0]);
	out[1] = sub_ps(m1[1], m2[1]);
	out[2] = sub_ps(m1[2], m2[2]);
	out[3] = sub_ps(m1[3], m2[3]);
#endif
}

FORCE_INLINE void mat4x4_negate(simd4f *out, simd4f *mat)
{
#ifdef MATH_AVX
	__m256 zero = _mm256_setzero_ps();
	__m256 *o = (__m256*)out;
	__m256 *m = (__m256*)mat;
	o[0] = _mm256_sub_ps(zero, m[0]);
	o[1] = _mm256_sub_ps(zero, m[1]);
#else
	out[0] = negate_ps(mat[0]);
	out[1] = negate_ps(mat[1]);
	out[2] = negate_ps(mat[2]);
	out[3] = negate_ps(mat[3]);
#endif
}

#endif
