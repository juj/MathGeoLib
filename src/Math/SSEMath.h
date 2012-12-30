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

#include "MathBuildConfig.h"

#ifdef MATH_SSE // If SSE is not enabled, this whole file will not be included.

#define ALIGN16 __declspec(align(16))

MATH_BEGIN_NAMESPACE

#ifdef MATH_SSE2 // We can use the pshufd instruction, which was introduced in SSE2 32-bit integer ops.

/// Swizzles/permutes a single SSE register into another SSE register. Requires SSE2.
#define _mm_shuffle1_ps(reg, shuffle) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((reg)), (shuffle)))

#else // We only have SSE 1, so must use the slightly worse shufps instruction, which always destroys the input operand (when AVX is not available)

#define _mm_shuffle1_ps(reg, shuffle) _mm_shuffle_ps((reg), (reg), (shuffle))

#endif

static const u32 andMaskOne = 0xFFFFFFFF;
static const float andMaskOneF = *(float*)&andMaskOne;

/// Returns a SSE mask register with x = y = z = 0xFFFFFFFF and w = 0x0.
/// Warning: _mm_setr_ps is slow!
inline __m128 SSEMaskXYZ()
{
	return _mm_setr_ps(andMaskOneF,andMaskOneF,andMaskOneF,0.f);
}

inline __m128 _mm_sum_xyz_ps(__m128 m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = _mm_and_ps(m, SSEMaskXYZ()); // Clear w to zero.
	m = _mm_hadd_ps(m, m); // m = (x+y, z+w, x+y, z+w).
	m = _mm_hadd_ps(m, m); // m = (x+y+z+w, x+y+z+w, x+y+z+w, x+y+z+w).
	return m; // Each index of the output will contain the sum x+y+z.
#else // We only have SSE 1, and must individually shuffle.
	__m128 Y = _mm_shuffle1_ps(m, _MM_SHUFFLE(1,1,1,1)); // Load Y to lowest index. (others don't matter)
	__m128 Z = _mm_shuffle1_ps(m, _MM_SHUFFLE(2,2,2,2)); // Load Z to lowest index. (others don't matter)
	__m128 XYZ = _mm_add_ss(m, _mm_add_ss(Y, Z));
	return XYZ; // Only the lowest index of the output will contain x+y+z.
#endif
}

/// The returned SP FP contains x+y+z+w in all channels of the vector.
inline __m128 _mm_sum_xyzw_ps(__m128 m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = _mm_hadd_ps(m, m); // m = (x+y, z+w, x+y, z+w).
	m = _mm_hadd_ps(m, m); // m = (x+y+z+w, x+y+z+w, x+y+z+w, x+y+z+w).
	return m; // Each index of the output will contain the sum x+y+z+w.
#else // We only have SSE 1, and must individually shuffle.
	__m128 v2 = _mm_shuffle1_ps(m, _MM_SHUFFLE(1,0,3,2)); // = [y, x, w, z]
	v2 = _mm_add_ps(v2, m); // = [w+y, z+x, y+w, x+z]
	__m128 v3 = _mm_shuffle1_ps(v2, _MM_SHUFFLE(0,3,2,1)); // = [x+z, w+y, z+x, y+w]
	return _mm_add_ps(v2, v3); // = [w+y+x+z, z+x+w+y, y+w+z+x, x+z+y+w]
#endif
}

inline __m128 _mm_mul_xyzw_ps(__m128 v)
{
	__m128 v2 = _mm_shuffle1_ps(v, _MM_SHUFFLE(1, 0, 3, 2)); // v2 = [y, x, w, z]
	v2 = _mm_mul_ps(v, v2); // v2 = [w*y, z*x, y*w, x*z]
	__m128 v3 = _mm_shuffle1_ps(v2, _MM_SHUFFLE(2, 1, 0, 3)); // v3 = [z*x, y*w, x*z, w*y]
	return _mm_mul_ps(v2, v3); // v3 = [w*y*z*x, z*x*y*w, y*w*x*z, x*z*w*y]
}

const __m128 sign_mask = _mm_set1_ps(-0.f); // -0.f = 1 << 31

inline __m128 _mm_abs_ps(__m128 x)
{
    return _mm_andnot_ps(sign_mask, x);
}

inline __m128 _mm_dot3_ps(__m128 a, __m128 b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	__m128 v2 = _mm_dp_ps(a, b, 0x70 | 0x0F); // Choose to multiply x, y and z (0x70 = 0111 0000), and store the output to all indices (0x0F == 0000 1111).
	return v2;
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	__m128 v2 = _mm_mul_ps(a, b);
	return _mm_sum_xyz_ps(v2);
#endif
}

/// The dot product is stored in each channel of the returned vector.
inline __m128 _mm_dot4_ps(__m128 a, __m128 b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	__m128 v2 = _mm_dp_ps(a, b, 0xF0 | 0x0F); // Choose to multiply x, y, z and w (0xF0 = 1111 0000), and store the output to all indices (0x0F == 0000 1111).
	return v2;
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	__m128 v2 = _mm_mul_ps(a, b);
	return _mm_sum_xyzw_ps(v2);
#endif
}

inline float M128_TO_FLOAT(__m128 sse)
{
	float ret;
	_mm_store_ss(&ret, sse);
	return ret;
}

const __m128 epsilonFloat = _mm_set1_ps(1e-4f);

// If mask[i] == 0, then output index i from a, otherwise mask[i] must be 0xFFFFFFFF, and output index i from b.
inline __m128 _mm_cmov_ps(__m128 a, __m128 b, __m128 mask)
{
#ifdef MATH_SSE41 // SSE 4.1 offers conditional copying between registers with the blendvps instruction.
	return _mm_blendv_ps(a, b, mask);
#else // If not on SSE 4.1, use conditional masking.
	b = _mm_and_ps(mask, b); // Where mask is 1, output b.
	a = _mm_andnot_ps(mask, a); // Where mask is 0, output a.
	return _mm_or_ps(a, b);
#endif
}

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
inline __m128 _mm_mat4x4_mul_ps_sse41(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_dp_ps(matrix[0], vector, 0xF0 | 0x0F); // Choose to multiply x, y, z and w (0xF0 = 1111 0000), and store the output to all indices (0x0F == 0000 1111).
	__m128 y = _mm_dp_ps(matrix[1], vector, 0xF0 | 0x0F);
	__m128 z = _mm_dp_ps(matrix[2], vector, 0xF0 | 0x0F);
	__m128 w = _mm_dp_ps(matrix[3], vector, 0xF0 | 0x0F);

	__m128 xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	__m128 zw = _mm_movelh_ps(z, w); // zw = [ _, w, _, z]

	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]
}
#endif

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
#ifdef MATH_SSE3 // If we have SSE3, we can repeatedly use haddps to accumulate the result.
inline __m128 _mm_mat4x4_mul_ps_sse3(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_mul_ps(matrix[0], vector);
	__m128 y = _mm_mul_ps(matrix[1], vector);
	__m128 z = _mm_mul_ps(matrix[2], vector);
	__m128 w = _mm_mul_ps(matrix[3], vector);
	__m128 tmp1 = _mm_hadd_ps(x, y); // = [y2+y3, y0+y1, x2+x3, x0+x1]
	__m128 tmp2 = _mm_hadd_ps(z, w); // = [w2+w3, w0+w1, z2+z3, z0+z1]

	return _mm_hadd_ps(tmp1, tmp2); // = [w0+w1+w2+w3, z0+z1+z2+z3, y0+y1+y2+y3, x0+x1+x2+x3]
}
#endif

#ifdef MATH_SSE
inline __m128 _mm_mat4x4_mul_ps_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_mul_ps(matrix[0], vector);
	__m128 y = _mm_mul_ps(matrix[1], vector);
	__m128 z = _mm_mul_ps(matrix[2], vector);
	__m128 w = _mm_mul_ps(matrix[3], vector);
	_MM_TRANSPOSE4_PS(x, y, z, w); // Contains 2x unpacklo's, 2x unpackhi's, 2x movelh's and 2x movehl's. (or 8 shuffles, depending on the compiler)

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

inline __m128 _mm_colmajor_mat4x4_mul_ps_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(0,0,0,0));
	__m128 y = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(1,1,1,1));
	__m128 z = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2,2,2,2));
	__m128 w = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(3,3,3,3));
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);
	w = _mm_mul_ps(w, matrix[3]);

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

inline __m128 _mm_colmajor_mat4x4_mul_ps_sse1_2(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_shuffle1_ps(vector, _MM_SHUFFLE(0,0,0,0));
	__m128 y = _mm_shuffle1_ps(vector, _MM_SHUFFLE(1,1,1,1));
	__m128 z = _mm_shuffle1_ps(vector, _MM_SHUFFLE(2,2,2,2));
	__m128 w = _mm_shuffle1_ps(vector, _MM_SHUFFLE(3,3,3,3));
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);
	w = _mm_mul_ps(w, matrix[3]);

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

#endif

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 _mm_mat4x4_mul_ps(const __m128 *matrix, __m128 vector)
{
#ifdef MATH_SSE41
	return _mm_mat4x4_mul_ps_sse41(matrix, vector);
#elif defined(MATH_SSE3)
	return _mm_mat4x4_mul_ps_sse3(matrix, vector);
#else
	return _mm_mat4x4_mul_ps_sse1(matrix, vector);
#endif
}

/// Compute the product M*v, where M is a 3x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 _mm_mat3x4_mul_ps(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_dot4_ps(matrix[0], vector);
	__m128 y = _mm_dot4_ps(matrix[1], vector);
	__m128 z = _mm_dot4_ps(matrix[2], vector);

	// Take the 'w' component of the vector unmodified.
	__m128 xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	__m128 zw = _mm_movehl_ps(vector, z); // zw = [ w, _, z, _]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(3, 1, 2, 0)); // ret = [w, z, y, x]
}

inline float3 _mm_mat3x4_mul_ps_float3(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_dot4_ps(matrix[0], vector);
	__m128 y = _mm_dot4_ps(matrix[1], vector);
	__m128 z = _mm_dot4_ps(matrix[2], vector);

	return float3(M128_TO_FLOAT(x), M128_TO_FLOAT(y), M128_TO_FLOAT(z));
}

// Given four scalar SS FP registers, packs the four values into a single SP FP register.
inline __m128 _mm_pack_4ss_to_ps(__m128 x, __m128 y, __m128 z, const __m128 &w)
{
	__m128 xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	__m128 zw = _mm_movelh_ps(z, w); // zw = [ _, w, _, z]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]
}

#define _mm_transpose_matrix_intel(row0, row1, row2, row3) \
	__m128 tmp0, tmp1, tmp2, tmp3; \
	tmp0 = _mm_unpacklo_ps(row0, row1); \
	tmp2 = _mm_unpacklo_ps(row2, row3); \
	tmp1 = _mm_unpackhi_ps(row0, row1); \
	tmp3 = _mm_unpackhi_ps(row2, row3); \
	row0 = _mm_movelh_ps(tmp0, tmp2); \
	row1 = _mm_movehl_ps(tmp2, tmp0); \
	row2 = _mm_movelh_ps(tmp1, tmp3); \
	row3 = _mm_movehl_ps(tmp3, tmp1);

__forceinline void _mm_mat4x4_mul_ps_dpps(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	// Transpose m2:
	// m2[0] = [ 03, 02, 01, 00 ]     [ 30, 20, 10, 00 ]
	// m2[1] = [ 13, 12, 11, 10 ] --> [ 31, 21, 11, 01 ]
	// m2[2] = [ 23, 22, 21, 20 ] --> [ 32, 22, 12, 02 ]
	//         [ 33, 32, 31, 30 ]     [ 33, 23, 13, 03 ]

	__m128 low1 = _mm_movelh_ps(m2[0], m2[1]); // = [ 11, 10, 01, 00 ]
	__m128 low2 = _mm_movelh_ps(m2[2], m2[3]); // = [ 31, 30, 21, 20 ]
	__m128 hi1 = _mm_movehl_ps(m2[1], m2[0]);  // = [ 13, 12, 03, 02 ]
	__m128 hi2 = _mm_movehl_ps(m2[3], m2[2]);  // = [ 33, 32, 23, 22 ]

	__m128 row1 = _mm_shuffle_ps(low1, low2, _MM_SHUFFLE(2, 0, 2, 0)); // = [30, 20, 10, 00]
	__m128 row2 = _mm_shuffle_ps(low1, low2, _MM_SHUFFLE(3, 1, 3, 1)); // = [31, 21, 11, 01]
	__m128 row3 = _mm_shuffle_ps(hi1, hi2, _MM_SHUFFLE(2, 0, 2, 0));   // = [32, 22, 12, 02]
	__m128 row4 = _mm_shuffle_ps(hi1, hi2, _MM_SHUFFLE(3, 1, 3, 1));   // = [33, 23, 13, 03]

	__m128 _00 = _mm_dot4_ps(m1[0], row1);
	__m128 _01 = _mm_dot4_ps(m1[0], row2);
	__m128 _02 = _mm_dot4_ps(m1[0], row3);
	__m128 _03 = _mm_dot4_ps(m1[0], row4);
	out[0] = _mm_pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = _mm_dot4_ps(m1[1], row1);
	__m128 _11 = _mm_dot4_ps(m1[1], row2);
	__m128 _12 = _mm_dot4_ps(m1[1], row3);
	__m128 _13 = _mm_dot4_ps(m1[1], row4);
	out[1] = _mm_pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = _mm_dot4_ps(m1[2], row1);
	__m128 _21 = _mm_dot4_ps(m1[2], row2);
	__m128 _22 = _mm_dot4_ps(m1[2], row3);
	__m128 _23 = _mm_dot4_ps(m1[2], row4);
	out[2] = _mm_pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = _mm_dot4_ps(m1[3], row1);
	__m128 _31 = _mm_dot4_ps(m1[3], row2);
	__m128 _32 = _mm_dot4_ps(m1[3], row3);
	__m128 _33 = _mm_dot4_ps(m1[3], row4);
	out[3] = _mm_pack_4ss_to_ps(_30, _31, _32, _33);
}

__forceinline void _mm_mat4x4_mul_ps_dpps_2(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	// Transpose m2:
	// m2[0] = [ 03, 02, 01, 00 ]     [ 30, 20, 10, 00 ]
	// m2[1] = [ 13, 12, 11, 10 ] --> [ 31, 21, 11, 01 ]
	// m2[2] = [ 23, 22, 21, 20 ] --> [ 32, 22, 12, 02 ]
	//         [ 33, 32, 31, 30 ]     [ 33, 23, 13, 03 ]
	__m128 row1 = m2[0];
	__m128 row2 = m2[1];
	__m128 row3 = m2[2];
	__m128 row4 = m2[3];
	_mm_transpose_matrix_intel(row1, row2, row3, row4);

	__m128 _00 = _mm_dot4_ps(m1[0], row1);
	__m128 _01 = _mm_dot4_ps(m1[0], row2);
	__m128 _02 = _mm_dot4_ps(m1[0], row3);
	__m128 _03 = _mm_dot4_ps(m1[0], row4);
	out[0] = _mm_pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = _mm_dot4_ps(m1[1], row1);
	__m128 _11 = _mm_dot4_ps(m1[1], row2);
	__m128 _12 = _mm_dot4_ps(m1[1], row3);
	__m128 _13 = _mm_dot4_ps(m1[1], row4);
	out[1] = _mm_pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = _mm_dot4_ps(m1[2], row1);
	__m128 _21 = _mm_dot4_ps(m1[2], row2);
	__m128 _22 = _mm_dot4_ps(m1[2], row3);
	__m128 _23 = _mm_dot4_ps(m1[2], row4);
	out[2] = _mm_pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = _mm_dot4_ps(m1[3], row1);
	__m128 _31 = _mm_dot4_ps(m1[3], row2);
	__m128 _32 = _mm_dot4_ps(m1[3], row3);
	__m128 _33 = _mm_dot4_ps(m1[3], row4);
	out[3] = _mm_pack_4ss_to_ps(_30, _31, _32, _33);
}

__forceinline void _mm_mat4x4_mul_ps_dpps_3(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	// Transpose m2:
	// m2[0] = [ 03, 02, 01, 00 ]     [ 30, 20, 10, 00 ]
	// m2[1] = [ 13, 12, 11, 10 ] --> [ 31, 21, 11, 01 ]
	// m2[2] = [ 23, 22, 21, 20 ] --> [ 32, 22, 12, 02 ]
	//         [ 33, 32, 31, 30 ]     [ 33, 23, 13, 03 ]

	__m128 low1 = _mm_movelh_ps(m2[0], m2[1]); // = [ 11, 10, 01, 00 ]
	__m128 low2 = _mm_movelh_ps(m2[2], m2[3]); // = [ 31, 30, 21, 20 ]
	__m128 hi1 = _mm_movehl_ps(m2[1], m2[0]);  // = [ 13, 12, 03, 02 ]
	__m128 hi2 = _mm_movehl_ps(m2[3], m2[2]);  // = [ 33, 32, 23, 22 ]

	__m128 row1 = _mm_shuffle_ps(low1, low2, _MM_SHUFFLE(2, 0, 2, 0)); // = [30, 20, 10, 00]
	__m128 row2 = _mm_shuffle_ps(low1, low2, _MM_SHUFFLE(3, 1, 3, 1)); // = [31, 21, 11, 01]
	__m128 row3 = _mm_shuffle_ps(hi1, hi2, _MM_SHUFFLE(2, 0, 2, 0));   // = [32, 22, 12, 02]
	__m128 row4 = _mm_shuffle_ps(hi1, hi2, _MM_SHUFFLE(3, 1, 3, 1));   // = [33, 23, 13, 03]

	__m128 _00 = _mm_dot4_ps(m1[0], row1);
	__m128 _01 = _mm_dot4_ps(m1[0], row2);
	__m128 _02 = _mm_dot4_ps(m1[0], row3);
	__m128 _03 = _mm_dot4_ps(m1[0], row4);

	__m128 xy = _mm_movelh_ps(_00, _01); // xy = [ _, y, _, x]
	__m128 zw = _mm_movelh_ps(_02, _03); // zw = [ _, w, _, z]
	out[0] = _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[0] = _mm_pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = _mm_dot4_ps(m1[1], row1);
	__m128 _11 = _mm_dot4_ps(m1[1], row2);
	__m128 _12 = _mm_dot4_ps(m1[1], row3);
	__m128 _13 = _mm_dot4_ps(m1[1], row4);

	__m128 xy2 = _mm_movelh_ps(_10, _11); // xy = [ _, y, _, x]
	__m128 zw2 = _mm_movelh_ps(_12, _13); // zw = [ _, w, _, z]
	out[1] = _mm_shuffle_ps(xy2, zw2, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[1] = _mm_pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = _mm_dot4_ps(m1[2], row1);
	__m128 _21 = _mm_dot4_ps(m1[2], row2);
	__m128 _22 = _mm_dot4_ps(m1[2], row3);
	__m128 _23 = _mm_dot4_ps(m1[2], row4);

	__m128 xy3 = _mm_movelh_ps(_20, _21); // xy = [ _, y, _, x]
	__m128 zw3 = _mm_movelh_ps(_22, _23); // zw = [ _, w, _, z]
	out[2] = _mm_shuffle_ps(xy3, zw3, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[2] = _mm_pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = _mm_dot4_ps(m1[3], row1);
	__m128 _31 = _mm_dot4_ps(m1[3], row2);
	__m128 _32 = _mm_dot4_ps(m1[3], row3);
	__m128 _33 = _mm_dot4_ps(m1[3], row4);

	__m128 xy4 = _mm_movelh_ps(_30, _31); // xy = [ _, y, _, x]
	__m128 zw4 = _mm_movelh_ps(_32, _33); // zw = [ _, w, _, z]
	out[3] = _mm_shuffle_ps(xy4, zw4, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[3] = _mm_pack_4ss_to_ps(_30, _31, _32, _33);
}

__forceinline void _mm_mat4x4_mul_ps(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	__m128 s0 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(0,0,0,0));
	__m128 s1 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(1,1,1,1));
	__m128 s2 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(2,2,2,2));
	__m128 s3 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(3,3,3,3));
	__m128 r0 = _mm_mul_ps(s0, m2[0]);
	__m128 r1 = _mm_mul_ps(s1, m2[1]);
	__m128 r2 = _mm_mul_ps(s2, m2[2]);
	__m128 r3 = _mm_mul_ps(s3, m2[3]);
	out[0] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[1] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[2] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle1_ps(m1[3], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle1_ps(m1[3], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle1_ps(m1[3], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle1_ps(m1[3], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[3] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

__forceinline void _mm_mat4x4_mul_ps_2(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	__m128 s0 = _mm_shuffle_ps(m1[0], m1[0], _MM_SHUFFLE(0,0,0,0));
	__m128 s1 = _mm_shuffle_ps(m1[0], m1[0], _MM_SHUFFLE(1,1,1,1));
	__m128 s2 = _mm_shuffle_ps(m1[0], m1[0], _MM_SHUFFLE(2,2,2,2));
	__m128 s3 = _mm_shuffle_ps(m1[0], m1[0], _MM_SHUFFLE(3,3,3,3));
	__m128 r0 = _mm_mul_ps(s0, m2[0]);
	__m128 r1 = _mm_mul_ps(s1, m2[1]);
	__m128 r2 = _mm_mul_ps(s2, m2[2]);
	__m128 r3 = _mm_mul_ps(s3, m2[3]);
	out[0] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle_ps(m1[1], m1[1], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle_ps(m1[1], m1[1], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle_ps(m1[1], m1[1], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle_ps(m1[1], m1[1], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[1] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle_ps(m1[2], m1[2], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle_ps(m1[2], m1[2], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle_ps(m1[2], m1[2], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle_ps(m1[2], m1[2], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[2] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle_ps(m1[3], m1[3], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle_ps(m1[3], m1[3], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle_ps(m1[3], m1[3], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle_ps(m1[3], m1[3], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[3] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

inline void _mm_mat3x4_mul_ps(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	const __m128 m2_3 = _mm_set_ps(1.f, 0.f, 0.f, 0.f);

	__m128 s0 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(0,0,0,0));
	__m128 s1 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(1,1,1,1));
	__m128 s2 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(2,2,2,2));
	__m128 s3 = _mm_shuffle1_ps(m1[0], _MM_SHUFFLE(3,3,3,3));
	__m128 r0 = _mm_mul_ps(s0, m2[0]);
	__m128 r1 = _mm_mul_ps(s1, m2[1]);
	__m128 r2 = _mm_mul_ps(s2, m2[2]);
	__m128 r3 = _mm_mul_ps(s3, m2_3);
	out[0] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle1_ps(m1[1], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2_3);
	out[1] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(0,0,0,0));
	s1 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(1,1,1,1));
	s2 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(2,2,2,2));
	s3 = _mm_shuffle1_ps(m1[2], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2_3);
	out[2] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

MATH_END_NAMESPACE

#else // ~MATH_SSE

#define ALIGN16

#endif // ~MATH_SSE
