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

/** @file float4x4_sse.h
	@author Jukka Jylänki
	@brief SSE code for float4x4-related computations. */
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_SSE

#include "SSEMath.h"
#include "float4_sse.h"

MATH_BEGIN_NAMESPACE

void quat_to_mat4x4(__m128 q, __m128 t, __m128 *m);

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
inline __m128 mat4x4_mul_sse41(const __m128 *matrix, __m128 vector)
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
inline __m128 mat4x4_mul_sse3(const __m128 *matrix, __m128 vector)
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

inline __m128 mat4x4_mul_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_mul_ps(matrix[0], vector);
	__m128 y = _mm_mul_ps(matrix[1], vector);
	__m128 z = _mm_mul_ps(matrix[2], vector);
	__m128 w = _mm_mul_ps(matrix[3], vector);
	_MM_TRANSPOSE4_PS(x, y, z, w); // Contains 2x unpacklo's, 2x unpackhi's, 2x movelh's and 2x movehl's. (or 8 shuffles, depending on the compiler)

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

inline __m128 colmajor_mat4x4_mul_sse1(const __m128 *matrix, __m128 vector)
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

/// Transforms a direction vector (w == 0) by the given matrix in column-major format.
inline __m128 colmajor_mat4x4_muldir_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(0,0,0,0));
	__m128 y = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(1,1,1,1));
	__m128 z = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2,2,2,2));
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);

	return _mm_add_ps(_mm_add_ps(x, y), z);
}

inline __m128 colmajor_mat4x4_mul_sse1_2(const __m128 *matrix, __m128 vector)
{
	__m128 x = shuffle1_ps(vector, _MM_SHUFFLE(0,0,0,0));
	__m128 y = shuffle1_ps(vector, _MM_SHUFFLE(1,1,1,1));
	__m128 z = shuffle1_ps(vector, _MM_SHUFFLE(2,2,2,2));
	__m128 w = shuffle1_ps(vector, _MM_SHUFFLE(3,3,3,3));
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);
	w = _mm_mul_ps(w, matrix[3]);

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 mat4x4_mul_sse(const __m128 *matrix, __m128 vector)
{
#ifdef MATH_SSE41
	return mat4x4_mul_sse41(matrix, vector);
#elif defined(MATH_SSE3)
	return mat4x4_mul_sse3(matrix, vector);
#else
	return mat4x4_mul_sse1(matrix, vector);
#endif
}

/// Compute the product M*v, where M is a 3x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 mat3x4_mul_sse(const __m128 *matrix, __m128 vector)
{
	__m128 x = dot4_ps(matrix[0], vector);
	__m128 y = dot4_ps(matrix[1], vector);
	__m128 z = dot4_ps(matrix[2], vector);

	// Take the 'w' component of the vector unmodified.
	__m128 xy = _mm_movelh_ps(x, y); // xy = [ _, y, _, x]
	__m128 zw = _mm_movehl_ps(vector, z); // zw = [ w, _, z, _]
	return _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(3, 1, 2, 0)); // ret = [w, z, y, x]
}

inline float3 mat3x4_mul_vec(const __m128 *matrix, __m128 vector)
{
	__m128 x = dot4_ps(matrix[0], vector);
	__m128 y = dot4_ps(matrix[1], vector);
	__m128 z = dot4_ps(matrix[2], vector);

	return float3(M128_TO_FLOAT(x), M128_TO_FLOAT(y), M128_TO_FLOAT(z));
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

FORCE_INLINE void mat4x4_mul_dpps(__m128 *out, const __m128 *m1, const __m128 *m2)
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

	__m128 _00 = dot4_ps(m1[0], row1);
	__m128 _01 = dot4_ps(m1[0], row2);
	__m128 _02 = dot4_ps(m1[0], row3);
	__m128 _03 = dot4_ps(m1[0], row4);
	out[0] = pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = dot4_ps(m1[1], row1);
	__m128 _11 = dot4_ps(m1[1], row2);
	__m128 _12 = dot4_ps(m1[1], row3);
	__m128 _13 = dot4_ps(m1[1], row4);
	out[1] = pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = dot4_ps(m1[2], row1);
	__m128 _21 = dot4_ps(m1[2], row2);
	__m128 _22 = dot4_ps(m1[2], row3);
	__m128 _23 = dot4_ps(m1[2], row4);
	out[2] = pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = dot4_ps(m1[3], row1);
	__m128 _31 = dot4_ps(m1[3], row2);
	__m128 _32 = dot4_ps(m1[3], row3);
	__m128 _33 = dot4_ps(m1[3], row4);
	out[3] = pack_4ss_to_ps(_30, _31, _32, _33);
}

FORCE_INLINE void mat4x4_mul_dpps_2(__m128 *out, const __m128 *m1, const __m128 *m2)
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

	__m128 _00 = dot4_ps(m1[0], row1);
	__m128 _01 = dot4_ps(m1[0], row2);
	__m128 _02 = dot4_ps(m1[0], row3);
	__m128 _03 = dot4_ps(m1[0], row4);
	out[0] = pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = dot4_ps(m1[1], row1);
	__m128 _11 = dot4_ps(m1[1], row2);
	__m128 _12 = dot4_ps(m1[1], row3);
	__m128 _13 = dot4_ps(m1[1], row4);
	out[1] = pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = dot4_ps(m1[2], row1);
	__m128 _21 = dot4_ps(m1[2], row2);
	__m128 _22 = dot4_ps(m1[2], row3);
	__m128 _23 = dot4_ps(m1[2], row4);
	out[2] = pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = dot4_ps(m1[3], row1);
	__m128 _31 = dot4_ps(m1[3], row2);
	__m128 _32 = dot4_ps(m1[3], row3);
	__m128 _33 = dot4_ps(m1[3], row4);
	out[3] = pack_4ss_to_ps(_30, _31, _32, _33);
}

FORCE_INLINE void mat4x4_mul_dpps_3(__m128 *out, const __m128 *m1, const __m128 *m2)
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

	__m128 _00 = dot4_ps(m1[0], row1);
	__m128 _01 = dot4_ps(m1[0], row2);
	__m128 _02 = dot4_ps(m1[0], row3);
	__m128 _03 = dot4_ps(m1[0], row4);

	__m128 xy = _mm_movelh_ps(_00, _01); // xy = [ _, y, _, x]
	__m128 zw = _mm_movelh_ps(_02, _03); // zw = [ _, w, _, z]
	out[0] = _mm_shuffle_ps(xy, zw, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[0] = pack_4ss_to_ps(_00, _01, _02, _03);

	__m128 _10 = dot4_ps(m1[1], row1);
	__m128 _11 = dot4_ps(m1[1], row2);
	__m128 _12 = dot4_ps(m1[1], row3);
	__m128 _13 = dot4_ps(m1[1], row4);

	__m128 xy2 = _mm_movelh_ps(_10, _11); // xy = [ _, y, _, x]
	__m128 zw2 = _mm_movelh_ps(_12, _13); // zw = [ _, w, _, z]
	out[1] = _mm_shuffle_ps(xy2, zw2, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[1] = pack_4ss_to_ps(_10, _11, _12, _13);

	__m128 _20 = dot4_ps(m1[2], row1);
	__m128 _21 = dot4_ps(m1[2], row2);
	__m128 _22 = dot4_ps(m1[2], row3);
	__m128 _23 = dot4_ps(m1[2], row4);

	__m128 xy3 = _mm_movelh_ps(_20, _21); // xy = [ _, y, _, x]
	__m128 zw3 = _mm_movelh_ps(_22, _23); // zw = [ _, w, _, z]
	out[2] = _mm_shuffle_ps(xy3, zw3, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[2] = pack_4ss_to_ps(_20, _21, _22, _23);

	__m128 _30 = dot4_ps(m1[3], row1);
	__m128 _31 = dot4_ps(m1[3], row2);
	__m128 _32 = dot4_ps(m1[3], row3);
	__m128 _33 = dot4_ps(m1[3], row4);

	__m128 xy4 = _mm_movelh_ps(_30, _31); // xy = [ _, y, _, x]
	__m128 zw4 = _mm_movelh_ps(_32, _33); // zw = [ _, w, _, z]
	out[3] = _mm_shuffle_ps(xy4, zw4, _MM_SHUFFLE(2, 0, 2, 0)); // ret = [w, z, y, x]

//	out[3] = pack_4ss_to_ps(_30, _31, _32, _33);
}

FORCE_INLINE void mat4x4_mul_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	__m128 s0 = shuffle1_ps(m1[0], _MM_SHUFFLE(0,0,0,0));
	__m128 s1 = shuffle1_ps(m1[0], _MM_SHUFFLE(1,1,1,1));
	__m128 s2 = shuffle1_ps(m1[0], _MM_SHUFFLE(2,2,2,2));
	__m128 s3 = shuffle1_ps(m1[0], _MM_SHUFFLE(3,3,3,3));
	__m128 r0 = _mm_mul_ps(s0, m2[0]);
	__m128 r1 = _mm_mul_ps(s1, m2[1]);
	__m128 r2 = _mm_mul_ps(s2, m2[2]);
	__m128 r3 = _mm_mul_ps(s3, m2[3]);
	out[0] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = shuffle1_ps(m1[1], _MM_SHUFFLE(0,0,0,0));
	s1 = shuffle1_ps(m1[1], _MM_SHUFFLE(1,1,1,1));
	s2 = shuffle1_ps(m1[1], _MM_SHUFFLE(2,2,2,2));
	s3 = shuffle1_ps(m1[1], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[1] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = shuffle1_ps(m1[2], _MM_SHUFFLE(0,0,0,0));
	s1 = shuffle1_ps(m1[2], _MM_SHUFFLE(1,1,1,1));
	s2 = shuffle1_ps(m1[2], _MM_SHUFFLE(2,2,2,2));
	s3 = shuffle1_ps(m1[2], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[2] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = shuffle1_ps(m1[3], _MM_SHUFFLE(0,0,0,0));
	s1 = shuffle1_ps(m1[3], _MM_SHUFFLE(1,1,1,1));
	s2 = shuffle1_ps(m1[3], _MM_SHUFFLE(2,2,2,2));
	s3 = shuffle1_ps(m1[3], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2[3]);
	out[3] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

FORCE_INLINE void mat4x4_mul_sse_2(__m128 *out, const __m128 *m1, const __m128 *m2)
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

inline void mat3x4_mul_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	const __m128 m2_3 = _mm_set_ps(1.f, 0.f, 0.f, 0.f);

	__m128 s0 = shuffle1_ps(m1[0], _MM_SHUFFLE(0,0,0,0));
	__m128 s1 = shuffle1_ps(m1[0], _MM_SHUFFLE(1,1,1,1));
	__m128 s2 = shuffle1_ps(m1[0], _MM_SHUFFLE(2,2,2,2));
	__m128 s3 = shuffle1_ps(m1[0], _MM_SHUFFLE(3,3,3,3));
	__m128 r0 = _mm_mul_ps(s0, m2[0]);
	__m128 r1 = _mm_mul_ps(s1, m2[1]);
	__m128 r2 = _mm_mul_ps(s2, m2[2]);
	__m128 r3 = _mm_mul_ps(s3, m2_3);
	out[0] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = shuffle1_ps(m1[1], _MM_SHUFFLE(0,0,0,0));
	s1 = shuffle1_ps(m1[1], _MM_SHUFFLE(1,1,1,1));
	s2 = shuffle1_ps(m1[1], _MM_SHUFFLE(2,2,2,2));
	s3 = shuffle1_ps(m1[1], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2_3);
	out[1] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));

	s0 = shuffle1_ps(m1[2], _MM_SHUFFLE(0,0,0,0));
	s1 = shuffle1_ps(m1[2], _MM_SHUFFLE(1,1,1,1));
	s2 = shuffle1_ps(m1[2], _MM_SHUFFLE(2,2,2,2));
	s3 = shuffle1_ps(m1[2], _MM_SHUFFLE(3,3,3,3));
	r0 = _mm_mul_ps(s0, m2[0]);
	r1 = _mm_mul_ps(s1, m2[1]);
	r2 = _mm_mul_ps(s2, m2[2]);
	r3 = _mm_mul_ps(s3, m2_3);
	out[2] = _mm_add_ps(_mm_add_ps(r0, r1), _mm_add_ps(r2, r3));
}

// Computes the inverse of a 4x4 matrix via direct cofactor expansion.
/// Returns the determinant of the original matrix, and zero on failure.
#define MAT_COFACTOR(mat, i, j) \
	_mm_sub_ps(_mm_mul_ps(_mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(j,j,j,j)), \
	           shuffle1_ps(_mm_shuffle_ps(mat[3], mat[2], _MM_SHUFFLE(i,i,i,i)), _MM_SHUFFLE(2,0,0,0))), \
	           _mm_mul_ps(shuffle1_ps(_mm_shuffle_ps(mat[3], mat[2], _MM_SHUFFLE(j,j,j,j)), _MM_SHUFFLE(2,0,0,0)), \
	           _mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(i,i,i,i))))
FORCE_INLINE float mat4x4_inverse(const __m128 *mat, __m128 *out)
{
	__m128 f1 = MAT_COFACTOR(mat, 3, 2);
	__m128 f2 = MAT_COFACTOR(mat, 3, 1);
	__m128 f3 = MAT_COFACTOR(mat, 2, 1);
	__m128 f4 = MAT_COFACTOR(mat, 3, 0);
	__m128 f5 = MAT_COFACTOR(mat, 2, 0);
	__m128 f6 = MAT_COFACTOR(mat, 1, 0);
	__m128 v1 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(0,0,0,0)), _MM_SHUFFLE(2,2,2,0));
	__m128 v2 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(1,1,1,1)), _MM_SHUFFLE(2,2,2,0));
	__m128 v3 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(2,2,2,2)), _MM_SHUFFLE(2,2,2,0));
	__m128 v4 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(3,3,3,3)), _MM_SHUFFLE(2,2,2,0));
	const __m128 s1 = _mm_set_ps(-0.0f,  0.0f, -0.0f,  0.0f);
	const __m128 s2 = _mm_set_ps( 0.0f, -0.0f,  0.0f, -0.0f);
	__m128 r1 = _mm_xor_ps(s1, _mm_add_ps(_mm_sub_ps(_mm_mul_ps(v2, f1), _mm_mul_ps(v3, f2)), _mm_mul_ps(v4, f3)));
	__m128 r2 = _mm_xor_ps(s2, _mm_add_ps(_mm_sub_ps(_mm_mul_ps(v1, f1), _mm_mul_ps(v3, f4)), _mm_mul_ps(v4, f5)));
	__m128 r3 = _mm_xor_ps(s1, _mm_add_ps(_mm_sub_ps(_mm_mul_ps(v1, f2), _mm_mul_ps(v2, f4)), _mm_mul_ps(v4, f6)));
	__m128 r4 = _mm_xor_ps(s2, _mm_add_ps(_mm_sub_ps(_mm_mul_ps(v1, f3), _mm_mul_ps(v2, f5)), _mm_mul_ps(v3, f6)));
	__m128 det = dot4_ps(mat[0], _mm_movelh_ps(_mm_unpacklo_ps(r1, r2), _mm_unpacklo_ps(r3, r4)));
	__m128 rcp = _mm_rcp_ps(det);
	out[0] = _mm_mul_ps(r1, rcp);
	out[1] = _mm_mul_ps(r2, rcp);
	out[2] = _mm_mul_ps(r3, rcp);
	out[3] = _mm_mul_ps(r4, rcp);
	return M128_TO_FLOAT(det);
}

/// Inverts a 3x4 affine transformation matrix (in row-major format) that only consists of rotation (+possibly mirroring) and translation.
FORCE_INLINE void mat3x4_inverse_orthonormal(__m128 *mat, __m128 *out)
{
	// mat[0]: [tx,02,01,00]
	// mat[1]: [ty,12,11,10]
	// mat[2]: [tz,22,21,20]
	// mat[3]: assumed to be [1,0,0,0] - not read.

	// First get the translation part (tx,ty,tz) from the original matrix,
	// and compute T=-M^(-1).
	__m128 tmp1 = _mm_unpackhi_ps(mat[0], mat[1]);                      // [ty,tx,12,02]
	__m128 xyz = _mm_shuffle_ps(tmp1, mat[2], _MM_SHUFFLE(3, 3, 3, 2)); // [ _,tz,ty,tx]
	__m128 vec = negate_ps(colmajor_mat4x4_muldir_sse1(mat, xyz));      // [ _,Tz,Ty,Tx]

	__m128 tmp0 = _mm_unpacklo_ps(mat[0], mat[1]); // [11,01,10,00]
	//     tmp1 = computed already above              [ty,tx,12,02]
	__m128 tmp2 = _mm_unpacklo_ps(mat[2], vec);    // [Ty,21,Tx,20]
	__m128 tmp3 = _mm_unpackhi_ps(mat[2], vec);    // [ _,23,Tz,22]

	out[0] = _mm_movelh_ps(tmp0, tmp2);            // [Tx,20,10,00]
	out[1] = _mm_movehl_ps(tmp2, tmp0);            // [Ty,21,11,01]
	out[2] = _mm_movelh_ps(tmp1, tmp3);            // [Tz,22 12,02]
 // out[3] = assumed to be [1,0,0,0] - no need to write back.
}

FORCE_INLINE __m128 NewtonRhapsonRecipStep(__m128 recip, __m128 estimate)
{
	// Do one iteration of Newton-Rhapson:
	// e_n = 2*e - x*e^2
	__m128 e2 = _mm_mul_ps(estimate, estimate);
	return _mm_sub_ps(_mm_add_ps(estimate, estimate), _mm_mul_ps(recip, e2));
}

FORCE_INLINE __m128 NewtonRhapsonRecip(__m128 recip)
{
	__m128 estimate = _mm_rcp_ps(recip);
	return NewtonRhapsonRecipStep(recip, estimate);
}

/// Computes the determinant of a 4x4 matrix. 
inline float mat4x4_determinant(const __m128 *row)
{
	__m128 s = shuffle1_ps(NewtonRhapsonRecip(row[0]), _MM_SHUFFLE(0,0,0,0));
	// row[0].x has a factor of the final determinant.
	__m128 row0 = _mm_mul_ps(s, row[0]);
	s = shuffle1_ps(row[1], _MM_SHUFFLE(0,0,0,0));
	__m128 row1 = _mm_sub_ps(row[1], _mm_mul_ps(s, row0));
	s = shuffle1_ps(row[2], _MM_SHUFFLE(0,0,0,0));
	__m128 row2 = _mm_sub_ps(row[2], _mm_mul_ps(s, row0));
	s = shuffle1_ps(row[3], _MM_SHUFFLE(0,0,0,0));
	__m128 row3 = _mm_sub_ps(row[3], _mm_mul_ps(s, row0));

	// row1.y has a factor of the final determinant.
	s = shuffle1_ps(NewtonRhapsonRecip(row1), _MM_SHUFFLE(1,1,1,1));
	__m128 row1_1 = _mm_mul_ps(s, row1);
	s = shuffle1_ps(row2, _MM_SHUFFLE(1,1,1,1));
	__m128 row2_1 = _mm_sub_ps(row2, _mm_mul_ps(s, row1_1));
	s = shuffle1_ps(row3, _MM_SHUFFLE(1,1,1,1));
	__m128 row3_1 = _mm_sub_ps(row3, _mm_mul_ps(s, row1_1));

	// Now we are left with a 2x2 matrix in row2_1.zw and row3_1.zw.
	// D = row2_1.z * row3_1.w - row2_1.w * row3_1.z.
	__m128 r1 = shuffle1_ps(row2_1, _MM_SHUFFLE(2,3,1,0));
	__m128 r = _mm_mul_ps(r1, row3_1);
	__m128 a = shuffle1_ps(r, _MM_SHUFFLE(3,3,3,3));
	__m128 b = shuffle1_ps(r, _MM_SHUFFLE(2,2,2,2));
	__m128 d1 = _mm_sub_ss(a, b);
	__m128 d2 = row[0];
	__m128 d3 = shuffle1_ps(row1, _MM_SHUFFLE(1,1,1,1));
	__m128 d = _mm_mul_ss(d1, _mm_mul_ss(d2, d3));
	return M128_TO_FLOAT(d);
}

/// Computes the determinant of a 3x4 matrix stored in row-major format. (Treated as a square matrix with last row [0,0,0,1])
inline float mat3x4_determinant(const __m128 *row)
{
	__m128 s = shuffle1_ps(NewtonRhapsonRecip(row[0]), _MM_SHUFFLE(0,0,0,0));
	// row[0].x has a factor of the final determinant.
	__m128 row0 = _mm_mul_ps(s, row[0]);
	s = shuffle1_ps(row[1], _MM_SHUFFLE(0,0,0,0));
	__m128 row1 = _mm_sub_ps(row[1], _mm_mul_ps(s, row0));
	s = shuffle1_ps(row[2], _MM_SHUFFLE(0,0,0,0));
	__m128 row2 = _mm_sub_ps(row[2], _mm_mul_ps(s, row0));

	// Now we are left with a 2x2 matrix in row1.yz and row2.yz.
	// D = row1.y * row2.z - row2.y * row1.z.
	__m128 r1 = shuffle1_ps(row1, _MM_SHUFFLE(3,1,2,0));
	__m128 r = _mm_mul_ps(r1, row2);
	__m128 a = shuffle1_ps(r, _MM_SHUFFLE(2,2,2,2));
	__m128 b = shuffle1_ps(r, _MM_SHUFFLE(1,1,1,1));
	__m128 d1 = _mm_sub_ss(a, b);
	__m128 d2 = row[0];
	__m128 d = _mm_mul_ss(d1, d2);
	return M128_TO_FLOAT(d);
}

inline void mat3x4_transpose(const __m128 *src, __m128 *dst)
{
	__m128 src3 = _mm_setzero_ps(); // w component should be 1, but since it won't get stored, it doesn't matter, so we can just create zeros.
	__m128 tmp0 = _mm_unpacklo_ps(src[0], src[1]);
	__m128 tmp2 = _mm_unpacklo_ps(src[2], src3);
	__m128 tmp1 = _mm_unpackhi_ps(src[0], src[1]);
	__m128 tmp3 = _mm_unpackhi_ps(src[2], src3);
	dst[0] = _mm_movelh_ps(tmp0, tmp2);
	dst[1] = _mm_movehl_ps(tmp2, tmp0);
	dst[2] = _mm_movelh_ps(tmp1, tmp3);
}

MATH_END_NAMESPACE

#endif // ~MATH_SSE
