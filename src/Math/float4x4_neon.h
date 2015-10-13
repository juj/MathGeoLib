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
#include "float4x4_sse.h"

MATH_BEGIN_NAMESPACE

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
// Multiplies mat * vec, where mat is a matrix in row-major format.
FORCE_INLINE simd4f mat4x4_mul_vec4(const simd4f *mat, simd4f vec)
{
#ifdef MATH_NEON
	// Transpose matrix at load time to get in registers in column-major format.
	float32x4x4_t m = vld4q_f32((const float32_t*)mat);
	simd4f ret = vmulq_lane_f32(m.val[0], vget_low_f32(vec), 0);
	ret = vmlaq_lane_f32(ret, m.val[1], vget_low_f32(vec), 1);
	ret = vmlaq_lane_f32(ret, m.val[2], vget_high_f32(vec), 0);
	return vmlaq_lane_f32(ret, m.val[3], vget_high_f32(vec), 1);
#elif defined(MATH_SSE3)
	return mat4x4_mul_sse3(mat, vec);
#else
	return mat4x4_mul_sse1(mat, vec);
#endif
}
#endif

// Multiplies vec * mat, where mat is a matrix in row-major format.
FORCE_INLINE simd4f vec4_mul_mat4x4(simd4f vec, const simd4f *mat)
{
#ifdef MATH_NEON
	simd4f ret = vmulq_lane_f32(mat[0], vget_low_f32(vec), 0);
	ret = vmlaq_lane_f32(ret, mat[1], vget_low_f32(vec), 1);
	ret = vmlaq_lane_f32(ret, mat[2], vget_high_f32(vec), 0);
	return vmlaq_lane_f32(ret, mat[3], vget_high_f32(vec), 1);
#else
	return colmajor_mat4x4_mul_sse1(mat, vec);
#endif
}

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

#ifdef ANDROID
FORCE_INLINE void mat4x4_mul_mat4x4_asm(simd4f *out, const simd4f *m1, const simd4f *m2)
{
	asm(
		"\t vldmia %1, {q4-q7} \n"
		"\t vldmia %2, {q8-q11} \n"
		"\t vmul.f32 q0, q8, d8[0] \n"
		"\t vmul.f32 q1, q8, d10[0] \n"
		"\t vmul.f32 q2, q8, d12[0] \n"
		"\t vmul.f32 q3, q8, d14[0] \n"
		"\t vmla.f32 q0, q9, d8[1] \n"
		"\t vmla.f32 q1, q9, d10[1] \n"
		"\t vmla.f32 q2, q9, d12[1] \n"
		"\t vmla.f32 q3, q9, d14[1] \n"
		"\t vmla.f32 q0, q10, d9[0] \n"
		"\t vmla.f32 q1, q10, d11[0] \n"
		"\t vmla.f32 q2, q10, d13[0] \n"
		"\t vmla.f32 q3, q10, d15[0] \n"
		"\t vmla.f32 q0, q11, d9[1] \n"
		"\t vmla.f32 q1, q11, d11[1] \n"
		"\t vmla.f32 q2, q11, d13[1] \n"
		"\t vmla.f32 q3, q11, d15[1] \n"
		"\t vstmia %0, {q0-q3} \n"
	: /* no outputs by value */
	: "r"(out), "r"(m1), "r"(m2)
	: "memory", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q11");
}
#endif

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
FORCE_INLINE void mat4x4_transpose(simd4f *out, const simd4f *mat)
{
#ifdef MATH_NEON
	float32x4x4_t m = vld4q_f32((const float32_t*)mat);
	vst1q_f32((float32_t*)out, m.val[0]);
	vst1q_f32((float32_t*)out+4, m.val[1]);
	vst1q_f32((float32_t*)out+8, m.val[2]);
	vst1q_f32((float32_t*)out+12, m.val[3]);
#else

	// Work around Visual Studio AVX codegen issue and avoid movelh and movehl altogether,
	// they seem to produce fishy results even when /GL is not enabled. Related: https://connect.microsoft.com/VisualStudio/feedback/details/814682/visual-studio-2013-x64-compiler-generates-faulty-code-with-gl-o2-arch-avx-flags-enabled
#ifdef MATH_AVX
	__m128 tmp0 = _mm_shuffle_ps(mat[0], mat[1], 0x44);
	__m128 tmp2 = _mm_shuffle_ps(mat[0], mat[1], 0xEE);
	__m128 tmp1 = _mm_shuffle_ps(mat[2], mat[3], 0x44);
	__m128 tmp3 = _mm_shuffle_ps(mat[2], mat[3], 0xEE);
	out[0] = _mm_shuffle_ps(tmp0, tmp1, 0x88);
	out[1] = _mm_shuffle_ps(tmp0, tmp1, 0xDD);
	out[2] = _mm_shuffle_ps(tmp2, tmp3, 0x88);
	out[3] = _mm_shuffle_ps(tmp2, tmp3, 0xDD);
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

#endif
}
#endif

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

FORCE_INLINE void mat4x4_mul_float(simd4f *out, const simd4f *mat, float scalar)
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

FORCE_INLINE void mat4x4_div_float(simd4f *out, const simd4f *mat, float scalar)
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

FORCE_INLINE void mat4x4_add_mat4x4(simd4f *out, const simd4f *m1, const simd4f *m2)
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

FORCE_INLINE void mat4x4_sub_mat4x4(simd4f *out, const simd4f *m1, const simd4f *m2)
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

FORCE_INLINE void mat4x4_negate(simd4f *out, const simd4f *mat)
{
#ifdef MATH_AVX
	__m256 zero = _mm256_setzero_ps();
	__m256 *o = (__m256*)out;
	__m256 *m = (__m256*)mat;
	o[0] = _mm256_sub_ps(zero, m[0]);
	o[1] = _mm256_sub_ps(zero, m[1]);
#else
	out[0] = neg_ps(mat[0]);
	out[1] = neg_ps(mat[1]);
	out[2] = neg_ps(mat[2]);
	out[3] = neg_ps(mat[3]);
#endif
}

MATH_END_NAMESPACE

#endif
