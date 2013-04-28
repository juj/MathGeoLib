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

#endif
