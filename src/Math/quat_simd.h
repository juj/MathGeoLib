#pragma once

#include "SSEMath.h"

MATH_BEGIN_NAMESPACE

#ifdef MATH_SSE

/// Converts a quaternion to a row-major matrix.
/// From http://renderfeather.googlecode.com/hg-history/034a1900d6e8b6c92440382658d2b01fc732c5de/Doc/optimized%20Matrix%20quaternion%20conversion.pdf
void quat_to_mat4x4(__m128 q, __m128 t, __m128 *m)
{
	// Constants:
	const u32 sign = 0x80000000UL;
	const __m128 sseX0 = set_ps_hex(sign, sign, sign, 0);
	const __m128 sseX1 = set_ps_hex(sign, sign, 0, sign);
	__m128 one = _mm_set_ps(0, 0, 0, 1);

#if 0 // The original code converted a quaternion into an hybrid of rotation/translation (bug?)
	__m128 q2 = _mm_add_ps(q, q);                                 // [2w 2z 2y 2x]
	__m128 yxxy = shuffle1_ps(q, _MM_SHUFFLE(1, 0, 0, 1));        // [ y  x  x  y]
	__m128 yyzz2 = shuffle1_ps(q2, _MM_SHUFFLE(2, 2, 1, 1));      // [2z 2z 2y 2y]
	__m128 yy_xy_xz_yz_2 = _mm_mul_ps(yxxy, yyzz2);               // [2yz 2xz 2xy 2yy]
	
	__m128 zwww = shuffle1_ps(q, _MM_SHUFFLE(3, 3, 3, 2));        // [w w w z]
	__m128 zzyx2 = shuffle1_ps(q2, _MM_SHUFFLE(0, 1, 2, 2));      // [2x 2y 2z 2z]
	__m128 zz_wz_wy_wx_2 = _mm_mul_ps(zwww, zzyx2);               // [2xw 2yw 2zw 2zz]

	__m128 xx2 = _mm_mul_ss(q, q2);                               // [2xx]

	// Calculate last two elements of the third row.
	__m128 one_m_xx2 = _mm_sub_ss(one, xx2);                      // [0 0 0 1-2xx]
	__m128 one_m_xx_yy_2 = _mm_sub_ss(one_m_xx2, yy_xy_xz_yz_2);  // [0 0 0 1-2xx-2yy]
	__m128 one_m_xx_yy_2_0_tz_tw = _mm_shuffle_ps(one_m_xx_yy_2, t, _MM_SHUFFLE(3, 2, 1, 0)); // [tw tz 0 1-2xx-2yy]

	// Calculate first row
	__m128 m_yy_xy_xz_yz_2 = _mm_xor_ps(yy_xy_xz_yz_2, sseX0);     // [-2yz -2xz -2xy   2yy]
	__m128 m_zz_wz_wy_wx_2 = _mm_xor_ps(zz_wz_wy_wx_2, sseX1);     // [-2xw -2yw  2zw  -2zz]
	__m128 m_zz_one_wz_wy_wx_2 = _mm_add_ss(m_zz_wz_wy_wx_2, one); // [-2xw -2yw  2zw 1-2zz]
	__m128 first_row = _mm_sub_ps(m_zz_one_wz_wy_wx_2, m_yy_xy_xz_yz_2); // [2yz-2xw 2xz-2yw 2xy+2zw 1-2zz-2yy]
	m[0] = first_row;
	_mm_store_ss((float*)m+3, t);

	// Calculate second row
	__m128 s1 = _mm_move_ss(m_yy_xy_xz_yz_2, xx2);                // [-2yz -2xz -2xy 2xx]
	__m128 s2 = _mm_xor_ps(m_zz_one_wz_wy_wx_2, sseX0);           // [2xw 2yw -2zw 1-2zz]
	__m128 s3 = _mm_sub_ps(s2, s1);                               // [2xw+2yz 2yw+2xz 2xy-2zw 1-2zz-2xx]
	__m128 t_yzwx = shuffle1_ps(t, _MM_SHUFFLE(0, 3, 2, 1));      // [tx tw tz ty]
	__m128 second_row = shuffle1_ps(s3, _MM_SHUFFLE(2, 3, 0, 1)); // [2yw+2xz 2xw+2yz 1-2zz-2xx 2xy-2zw]
	m[1] = second_row;
	_mm_store_ss((float*)m+7, t_yzwx);

	// Calculate third row
	__m128 t1 = _mm_movehl_ps(first_row, second_row);             // [2yz-2xw 2xz-2yw 2yw+2xz 2xw+2yz]
	__m128 t2 = _mm_shuffle_ps(t1, one_m_xx_yy_2_0_tz_tw, _MM_SHUFFLE(2, 0, 3, 1)); // [tz 1-2xx-2yy 2yz-2xw 2yw+2xz]
	m[2] = t2;
	m[3] = _mm_set_ps(1, 0, 0, 0);
#else

	__m128 q2 = _mm_add_ps(q, q);                                 // [2w 2z 2y 2x]
	__m128 yxxy = shuffle1_ps(q, _MM_SHUFFLE(1, 0, 0, 1));        // [ y  x  x  y]
	__m128 yyzz2 = shuffle1_ps(q2, _MM_SHUFFLE(2, 2, 1, 1));      // [2z 2z 2y 2y]
	__m128 yy_xy_xz_yz_2 = _mm_mul_ps(yxxy, yyzz2);               // [2yz 2xz 2xy 2yy]
	
	__m128 zwww = shuffle1_ps(q, _MM_SHUFFLE(3, 3, 3, 2));        // [w w w z]
	__m128 zzyx2 = shuffle1_ps(q2, _MM_SHUFFLE(0, 1, 2, 2));      // [2x 2y 2z 2z]
	__m128 zz_wz_wy_wx_2 = _mm_mul_ps(zwww, zzyx2);               // [2xw 2yw 2zw 2zz]

	__m128 xx2 = _mm_mul_ss(q, q2);                               // [2xx]

	// Calculate last two elements of the third row.
	__m128 one_m_xx2 = _mm_sub_ss(one, xx2);                      // [0 0 0 1-2xx]
	__m128 one_m_xx_yy_2 = _mm_sub_ss(one_m_xx2, yy_xy_xz_yz_2);  // [0 0 0 1-2xx-2yy]
	__m128 one_m_xx_yy_2_0_tz_tw = one_m_xx_yy_2;//_mm_shuffle_ps(one_m_xx_yy_2, t, _MM_SHUFFLE(3, 2, 1, 0)); // [tw tz 0 1-2xx-2yy]

	// Calculate first row
	__m128 m_yy_xy_xz_yz_2 = _mm_xor_ps(yy_xy_xz_yz_2, sseX0);     // [-2yz -2xz -2xy   2yy]
	__m128 m_zz_wz_wy_wx_2 = _mm_xor_ps(zz_wz_wy_wx_2, sseX1);     // [-2xw -2yw  2zw  -2zz]
	__m128 m_zz_one_wz_wy_wx_2 = _mm_add_ss(m_zz_wz_wy_wx_2, one); // [-2xw -2yw  2zw 1-2zz]
	__m128 first_row = _mm_sub_ps(m_zz_one_wz_wy_wx_2, m_yy_xy_xz_yz_2); // [2yz-2xw 2xz-2yw 2xy+2zw 1-2zz-2yy]

	// Calculate second row
	__m128 s1 = _mm_move_ss(m_yy_xy_xz_yz_2, xx2);                // [-2yz -2xz -2xy 2xx]
	__m128 s2 = _mm_xor_ps(m_zz_one_wz_wy_wx_2, sseX0);           // [2xw 2yw -2zw 1-2zz]
	__m128 s3 = _mm_sub_ps(s2, s1);                               // [2xw+2yz 2yw+2xz 2xy-2zw 1-2zz-2xx]
	__m128 second_row = shuffle1_ps(s3, _MM_SHUFFLE(2, 3, 0, 1)); // [2yw+2xz 2xw+2yz 1-2zz-2xx 2xy-2zw]

	// Calculate third row
	__m128 t1 = _mm_movehl_ps(first_row, second_row);             // [2yz-2xw 2xz-2yw 2yw+2xz 2xw+2yz]
	__m128 third_row = _mm_shuffle_ps(t1, one_m_xx_yy_2_0_tz_tw, _MM_SHUFFLE(2, 0, 3, 1)); // [0 1-2xx-2yy 2yz-2xw 2yw+2xz]

	__m128 tmp0 = _mm_unpacklo_ps(first_row, second_row);
	__m128 tmp2 = _mm_unpacklo_ps(third_row, t);
	__m128 tmp1 = _mm_unpackhi_ps(first_row, second_row);
	__m128 tmp3 = _mm_unpackhi_ps(third_row, t);
	m[0] = _mm_movelh_ps(tmp0, tmp2);
	m[1] = _mm_movehl_ps(tmp2, tmp0);
	m[2] = _mm_movelh_ps(tmp1, tmp3);
	m[3] = _mm_set_ps(1, 0, 0, 0);
#endif
}

FORCE_INLINE simd4f quat_transform_vec4(simd4f quat, simd4f vec)
{
	__m128 W = shuffle1_ps(quat, _MM_SHUFFLE(3,3,3,3));

//	__m128 qxv = cross_ps(q, vec.v);
	__m128 a_xzy = shuffle1_ps(quat, _MM_SHUFFLE(3, 0, 2, 1)); // a_xzy = [a.w, a.x, a.z, a.y]
	__m128 b_yxz = shuffle1_ps(vec, _MM_SHUFFLE(3, 1, 0, 2)); // b_yxz = [b.w, b.y, b.x, b.z]
	__m128 a_yxz = shuffle1_ps(quat, _MM_SHUFFLE(3, 1, 0, 2)); // a_yxz = [a.w, a.y, a.x, a.z]
	__m128 b_xzy = shuffle1_ps(vec, _MM_SHUFFLE(3, 0, 2, 1)); // b_xzy = [b.w, b.x, b.z, b.y]
	__m128 x = _mm_mul_ps(a_xzy, b_yxz); // [a.w*b.w, a.x*b.y, a.z*b.x, a.y*b.z]
	__m128 y = _mm_mul_ps(a_yxz, b_xzy); // [a.w*b.w, a.y*b.x, a.x*b.z, a.z*b.y]
	__m128 qxv = _mm_sub_ps(x, y); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]

	__m128 Wv = _mm_mul_ps(W, vec);
	__m128 s = _mm_add_ps(qxv, Wv);

//	s = cross_ps(q, s);
	__m128 s_yxz = shuffle1_ps(s, _MM_SHUFFLE(3, 1, 0, 2)); // b_yxz = [b.w, b.y, b.x, b.z]
	__m128 s_xzy = shuffle1_ps(s, _MM_SHUFFLE(3, 0, 2, 1)); // b_xzy = [b.w, b.x, b.z, b.y]
	x = _mm_mul_ps(a_xzy, s_yxz); // [a.w*b.w, a.x*b.y, a.z*b.x, a.y*b.z]
	y = _mm_mul_ps(a_yxz, s_xzy); // [a.w*b.w, a.y*b.x, a.x*b.z, a.z*b.y]
	s = _mm_sub_ps(x, y); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]

	s = _mm_add_ps(s, s);
	s = _mm_add_ps(s, vec);
	return s;
}

#endif // ~MATH_SSE

MATH_END_NAMESPACE
