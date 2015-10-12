#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_SIMD

#include "SSEMath.h"
#include "float4_neon.h"

MATH_BEGIN_NAMESPACE

#ifdef MATH_SSE

/// Converts a quaternion to a row-major matrix.
inline void quat_to_mat3x4(simd4f q, simd4f t, simd4f *m)
{
	simd4f one = set_ps(0, 0, 0, 1);
	const simd4f sseX1 = set_ps_hex((int)0x80000000UL, (int)0x80000000UL, 0, (int)0x80000000UL); // [-, -, + -]
	simd4f q2 = add_ps(q, q);                                                     // [2w 2z 2y 2x]
	simd4f t2 = _mm_add_ss(xor_ps(mul_ps(zwww_ps(q), zzyx_ps(q2)), sseX1), one);  // [-2xw -2yw  2zw 1-2zz]
	const simd4f sseX0 = yzwx_ps(sseX1);                                          // [-, -, -, +]
	simd4f t0 = mul_ps(yxxy_ps(q), yyzz_ps(q2));                                  // [ 2yz  2xz  2xy 2yy]
	simd4f t1 = xor_ps(t0, sseX0);                                                // [-2yz -2xz -2xy 2yy]
	simd4f r0 = sub_ps(t2, t1);                                                   // [2yz-2xw 2xz-2yw 2xy+2zw 1-2zz-2yy]
	simd4f xx2 = _mm_mul_ss(q, q2);                                               // [2xx]
	simd4f r1 = sub_ps(xor_ps(t2, sseX0), _mm_move_ss(t1, xx2));
	r1 = yxwz_ps(r1);                                                             // [2yw+2xz 2xw+2yz 1-2zz-2xx 2xy-2zw]
	simd4f r2 = _mm_shuffle_ps(_mm_movehl_ps(r0, r1), _mm_sub_ss(_mm_sub_ss(one, xx2), t0), _MM_SHUFFLE(2, 0, 3, 1)); // [0 1-2xx-2yy 2yz-2xw 2yw+2xz]
	simd4f tmp0 = _mm_unpacklo_ps(r0, r1);
	simd4f tmp2 = _mm_unpacklo_ps(r2, t);
	simd4f tmp1 = _mm_unpackhi_ps(r0, r1);
	simd4f tmp3 = _mm_unpackhi_ps(r2, t);
	m[0] = _mm_movelh_ps(tmp0, tmp2);
	m[1] = _mm_movehl_ps(tmp2, tmp0);
	m[2] = _mm_movelh_ps(tmp1, tmp3);
}

FORCE_INLINE void quat_to_mat4x4(simd4f q, simd4f t, simd4f *m)
{
	quat_to_mat3x4(q, t, m);
	m[3] = set_ps(1.f, 0.f, 0.f, 0.f);
}

#endif // ~MATH_SSE

FORCE_INLINE simd4f quat_transform_vec4(simd4f quat, simd4f vec)
{
	const simd4f W = wwww_ps(quat);
	const simd4f a_yzx = yzxw_ps(quat);
	simd4f x = mul_ps(quat, yzxw_ps(vec));
	simd4f qxv = mnadd_ps(a_yzx, vec, x);
	simd4f Wv = mul_ps(W, vec);
	simd4f s = add_ps(qxv, zxyw_ps(Wv));
	simd4f y = zxyw_ps(mul_ps(quat, s)); // [a.w*b.w, a.y*b.x, a.x*b.z, a.z*b.y]
	s = msub_ps(a_yzx, s, y); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]
	s = add_ps(s, s);
	s = add_ps(s, vec);
	return s;
}

#if defined(ANDROID) && defined(MATH_NEON)
inline void quat_mul_quat_asm(const void *q1, const void *q2, void *out)
{
/*	return Quat(x*r.w + y*r.z - z*r.y + w*r.x,
	           -x*r.z + y*r.w + z*r.x + w*r.y,
	            x*r.y - y*r.x + z*r.w + w*r.z,
	           -x*r.x - y*r.y - z*r.z + w*r.w); */
#ifdef _DEBUG
	assert(IS16ALIGNED(out));
	assert(IS16ALIGNED(q1));
	assert(IS16ALIGNED(q2));
	assert(IS16ALIGNED(sx));
	assert(IS16ALIGNED(sy));
	assert(IS16ALIGNED(sz));
#endif
	///\todo 128-bit aligned loads: [%1,:128]
	asm(
		"\t vld1.32 {d0, d1}, [%1]\n"    // q0 = quat1.xyzw
		"\t vmov.i32 d12, #0\n"          // q6.lo = 0
		"\t vmov.i32 d13, #0x80000000\n" // q6.hi = [- - + +] = 'signy'
		"\t vld1.32 {d8, d9}, [%2]\n"    // q4 = quat2.xyzw [%2]
		"\t vdup.32 q1, d0[1]\n"         // q1 = q0[1] = quat1.yyyy = 'Y'
		"\t vdup.32 q2, d1[0]\n"         // q2 = q0[2] = quat1.zzzz = 'Z'
		"\t vshl.i64 d10, d13, #32\n"    // q5.lo = q6.hi = [- +]
		"\t vdup.32 q3, d1[1]\n"         // q3 = q0[3] = quat1.wwww = 'W'
		"\t vdup.32 q0, d0[0]\n"         // q0 = q0[0] = quat1.xxxx = 'X'
		"\t vmov d11, d10\n"             // q5.hi = q5.lo = [- + - +] = 'signx'
		"\t vmov d15, d10\n"             // q7.hi = q5.lo = [- +]
		"\t vshr.u64 d14, d10, #32\n"    // q7.lo = q5.lo = [- + + -] = 'signz'

		"\t vmov d18, d9\n"              // q9.lo = q4.hi
		"\t vmov d19, d8\n"              // q9.hi = q4.lo, q9 = quat2.zwxy = 'q2 for Y'

		"\t veor q0, q0, q5\n"           // q0 = X*signx = [-x x -x x]
		"\t veor q1, q1, q6\n"           // q1 = Y*signy = [-y -y y y]
		"\t veor q2, q2, q7\n"           // q2 = Z*signz = [-z z z -z]

		"\t vrev64.32 q10, q9\n"         // q10 = quat2.wzyx = 'q2 for X'

		"\t vmul.f32 q0, q0, q10\n"      // q0 = X*signx * quat2
		"\t vmul.f32 q11, q1, q9\n"       // q0 += Y*signy * quat2
		"\t vrev64.32 q8, q4\n"          // q8 = quat2.yxwz  = 'q2 for Z'
		"\t vmla.f32 q0, q2, q8\n"       // q0 += Z*signz * quat2
		"\t vmla.f32 q11, q3, q4\n"       // q0 += W       * quat2

		"\t vadd.f32 q0, q0, q11\n"
		"\t vst1.32	{d0, d1}, [%0]\n"    // store output
	: /* no outputs by value */
	: [out]"r"(out), [quat1]"r"(q1), [quat2]"r"(q2)
	: "memory", "q11", "q10", "q9", "q8", "q7", "q6", "q5", "q4", "q3", "q2", "q1", "q0");
}
#endif

FORCE_INLINE simd4f quat_mul_quat(simd4f q1, simd4f q2)
{
/*
	return Quat(x*r.w + y*r.z - z*r.y + w*r.x,
	           -x*r.z + y*r.w + z*r.x + w*r.y,
	            x*r.y - y*r.x + z*r.w + w*r.z,
	           -x*r.x - y*r.y - z*r.z + w*r.w); */
#if defined(MATH_SSE)
	const simd4f signy = set_ps_hex(0x80000000u, 0x80000000u, 0, 0); // [- - + +]
	const simd4f signz = wxxw_ps(signy);   // [- + + -]

	simd4f X = xxxx_ps(q1);
	simd4f Y = xor_ps(signy, yyyy_ps(q1));
	simd4f Z = xor_ps(signz, zzzz_ps(q1));
	simd4f W = wwww_ps(q1);

	simd4f r1 = wzyx_ps(q2); // [x,y,z,w]
	simd4f r2 = zwxy_ps(q2); // [y,x,w,z]
	simd4f r3 = yxwz_ps(q2); // [z,w,x,y]
	// simd4f r4 = q2;

	simd4f out = mul_ps(X, r1);
#ifdef MATH_FMA
	// _mm_fmsubadd_ps can avoid one shuffle and xor(!)
	out = _mm_fmsubadd_ps(Y, r2, out);
#else
	const simd4f signx = xzxz_ps(signy); // [- + - +]
	out = madd_ps(Y, r2, xor_ps(signx, out));
#endif
	out = madd_ps(Z, r3, out);
	out = madd_ps(W, q2, out);
	return out;
#elif defined(ANDROID) && defined(MATH_NEON)
	simd4f ret;
	quat_mul_quat_asm(&q1, &q2, &ret);
	return ret;
#elif defined(MATH_NEON)
	static const float32x4_t signx = set_ps_hex_const(0x80000000u, 0, 0x80000000u, 0);
	static const float32x4_t signy = set_ps_hex_const(0x80000000u, 0x80000000u, 0, 0);
	static const float32x4_t signz = set_ps_hex_const(0x80000000u, 0, 0, 0x80000000u);

	const float32_t *q1f = (const float32_t *)&q1;
	float32x4_t X = xor_ps(signx, vdupq_n_f32(q1f[0]));
	float32x4_t Y = xor_ps(signy, vdupq_n_f32(q1f[1]));
	float32x4_t Z = xor_ps(signz, vdupq_n_f32(q1f[2]));
	float32x4_t W = vdupq_n_f32(q1f[3]);

	float32x4_t r3 = vrev64q_f32(q2); // [z,w,x,y]
	float32x4_t r1 = vcombine_f32(vget_high_f32(r3), vget_low_f32(r3)); // [x,y,z,w]
	float32x4_t r2 = vrev64q_f32(r1); // [y,x,w,z]

	float32x4_t ret = mul_ps(X, r1);
	ret = vmlaq_f32(ret, Y, r2);
	ret = vmlaq_f32(ret, Z, r3);
	ret = vmlaq_f32(ret, W, q2);
	return ret;
#endif
}

FORCE_INLINE simd4f quat_div_quat(simd4f q1, simd4f q2)
{
/*
	return Quat(x*r.w - y*r.z + z*r.y - w*r.x,
	            x*r.z + y*r.w - z*r.x - w*r.y,
	           -x*r.y + y*r.x + z*r.w - w*r.z,
	            x*r.x + y*r.y + z*r.z + w*r.w); */

	const simd4f signx = set_ps_hex(0x80000000u, 0, 0x80000000u, 0); // [- + - +]
	const simd4f signy = xxww_ps(signx);   // [- - + +]
	const simd4f signz = wxxw_ps(signx);   // [- + + -]

	simd4f X = xor_ps(signx, xxxx_ps(q1));
	simd4f Y = xor_ps(signy, yyyy_ps(q1));
	simd4f Z = xor_ps(signz, zzzz_ps(q1));
	simd4f W = wwww_ps(q1);

	q2 = neg3_ps(q2);
	simd4f r1 = wzyx_ps(q2); // [x,y,z,w]
	simd4f r2 = zwxy_ps(q2); // [y,x,w,z]
	simd4f r3 = yxwz_ps(q2); // [z,w,x,y]
	// simd4f r4 = q2;

	simd4f out = mul_ps(X, r1);
	out = madd_ps(Y, r2, out);
	out = madd_ps(Z, r3, out);
	out = madd_ps(W, q2, out);
	return out;
}

MATH_END_NAMESPACE

#endif // ~MATH_SIMD
