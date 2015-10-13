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
	__m128 t0 = _mm_unpacklo_ps(x, y);
	__m128 t1 = _mm_unpackhi_ps(x, y);
	t0 = _mm_add_ps(t0, t1);
	__m128 z = _mm_mul_ps(matrix[2], vector);
	__m128 w = _mm_mul_ps(matrix[3], vector);
	__m128 t2 = _mm_unpacklo_ps(z, w);
	__m128 t3 = _mm_unpackhi_ps(z, w);
	t2 = _mm_add_ps(t2, t3);
	return _mm_add_ps(_mm_movelh_ps(t0, t2), _mm_movehl_ps(t2, t0));
}

inline __m128 colmajor_mat4x4_mul_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = xxxx_ps(vector);
	__m128 y = yyyy_ps(vector);
	__m128 z = zzzz_ps(vector);
	__m128 w = wwww_ps(vector);
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);
	w = _mm_mul_ps(w, matrix[3]);

	return _mm_add_ps(_mm_add_ps(x, y), _mm_add_ps(z, w));
}

#if 0
/// Transforms a direction vector (w == 0) by the given matrix in column-major format.
inline __m128 colmajor_mat4x4_muldir_sse1(const __m128 *matrix, __m128 vector)
{
	__m128 x = xxxx_ps(vector);
	__m128 y = yyyy_ps(vector);
	__m128 z = zzzz_ps(vector);
	x = _mm_mul_ps(x, matrix[0]);
	y = _mm_mul_ps(y, matrix[1]);
	z = _mm_mul_ps(z, matrix[2]);

	return _mm_add_ps(_mm_add_ps(x, y), z);
}
#endif

/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 mat4x4_mul_sse(const __m128 *matrix, __m128 vector)
{
// Disabled: The SSE 4.1 version has been profiled to be 2 clock cycles slower than the SSE 3 version.
//#ifdef MATH_SSE41
//	return mat4x4_mul_sse41(matrix, vector);

#if defined(MATH_SSE3)
	return mat4x4_mul_sse3(matrix, vector);
#else
	return mat4x4_mul_sse1(matrix, vector);
#endif
}

/// Compute the product M*v, where M is a 3x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
inline __m128 mat3x4_mul_sse(const __m128 *matrix, __m128 vector)
{
	__m128 x = _mm_mul_ps(matrix[0], vector);
	__m128 y = _mm_mul_ps(matrix[1], vector);
	__m128 t0 = _mm_unpacklo_ps(x, y);
	__m128 t1 = _mm_unpackhi_ps(x, y);
	t0 = _mm_add_ps(t0, t1);
	__m128 z = _mm_mul_ps(matrix[2], vector);
	__m128 w = _mm_mul_ps(_mm_set_ps(1.f, 0.f, 0.f, 0.f), vector);
	__m128 t2 = _mm_unpacklo_ps(z, w);
	__m128 t3 = _mm_unpackhi_ps(z, w);
	t2 = _mm_add_ps(t2, t3);
	return _mm_add_ps(_mm_movelh_ps(t0, t2), _mm_movehl_ps(t2, t0));
}

inline float3 mat3x4_mul_vec(const __m128 *matrix, __m128 vector)
{
	__m128 vec = mat3x4_mul_sse(matrix, vector);
	return float3(s4f_x(vec), s4f_y(vec), s4f_z(vec));
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

FORCE_INLINE void mat4x4_mul_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
#ifdef MATH_64BIT // In 64-bit, we have lots of SIMD registers, so use as many as possible.
// 64-bit, SSE4.1:
// Benchmark 'mat4x4_mul_sse': test against float4x4_op_mul
//   Best: 5.354 nsecs / 17.144 ticks, Avg: 5.672 nsecs, Worst: 6.023 nsecs
	__m128 m1_0 = m1[0];
	__m128 m1_1 = m1[1];
	__m128 m1_2 = m1[2];
	__m128 m1_3 = m1[3];
	__m128 m = m2[0];
	__m128 o1 = mul_ps(xxxx_ps(m1_0), m);
	__m128 o2 = mul_ps(xxxx_ps(m1_1), m);
	__m128 o3 = mul_ps(xxxx_ps(m1_2), m);
	__m128 o4 = mul_ps(xxxx_ps(m1_3), m);
	m = m2[1];
	o1 = add_ps(mul_ps(yyyy_ps(m1_0), m), o1);
	o2 = add_ps(mul_ps(yyyy_ps(m1_1), m), o2);
	o3 = add_ps(mul_ps(yyyy_ps(m1_2), m), o3);
	o4 = add_ps(mul_ps(yyyy_ps(m1_3), m), o4);
	m = m2[2];
	o1 = add_ps(mul_ps(zzzz_ps(m1_0), m), o1);
	o2 = add_ps(mul_ps(zzzz_ps(m1_1), m), o2);
	o3 = add_ps(mul_ps(zzzz_ps(m1_2), m), o3);
	o4 = add_ps(mul_ps(zzzz_ps(m1_3), m), o4);
	m = m2[3];
	out[0] = add_ps(mul_ps(wwww_ps(m1_0), m), o1);
	out[1] = add_ps(mul_ps(wwww_ps(m1_1), m), o2);
	out[2] = add_ps(mul_ps(wwww_ps(m1_2), m), o3);
	out[3] = add_ps(mul_ps(wwww_ps(m1_3), m), o4);
#else // Targeting 32-bit, use as few registers as possible to avoid spilling.
// 32-bit, SSE4.1:
// Benchmark 'mat4x4_mul_sse': test against float4x4_op_mul
//   Best: 6.692 nsecs / 21.34 ticks, Avg: 7.104 nsecs, Worst: 9.704 nsecs
	out[0] = add_ps(add_ps(mul_ps(xxxx_ps(m1[0]), m2[0]), mul_ps(yyyy_ps(m1[0]), m2[1])), add_ps(mul_ps(zzzz_ps(m1[0]), m2[2]), mul_ps(wwww_ps(m1[0]), m2[3])));
	out[1] = add_ps(add_ps(mul_ps(xxxx_ps(m1[1]), m2[0]), mul_ps(yyyy_ps(m1[1]), m2[1])), add_ps(mul_ps(zzzz_ps(m1[1]), m2[2]), mul_ps(wwww_ps(m1[1]), m2[3])));
	out[2] = add_ps(add_ps(mul_ps(xxxx_ps(m1[2]), m2[0]), mul_ps(yyyy_ps(m1[2]), m2[1])), add_ps(mul_ps(zzzz_ps(m1[2]), m2[2]), mul_ps(wwww_ps(m1[2]), m2[3])));
	out[3] = add_ps(add_ps(mul_ps(xxxx_ps(m1[3]), m2[0]), mul_ps(yyyy_ps(m1[3]), m2[1])), add_ps(mul_ps(zzzz_ps(m1[3]), m2[2]), mul_ps(wwww_ps(m1[3]), m2[3])));
#endif
}

inline void mat3x4_mul_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	const __m128 m2_3 = set_ps(1.f, 0.f, 0.f, 0.f);

	__m128 r0 = mul_ps(xxxx_ps(m1[0]), m2[0]);
	__m128 r1 = mul_ps(yyyy_ps(m1[0]), m2[1]);
	__m128 r2 = mul_ps(zzzz_ps(m1[0]), m2[2]);
	__m128 r3 = mul_ps(m1[0], m2_3);
	out[0] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	r0 = mul_ps(xxxx_ps(m1[1]), m2[0]);
	r1 = mul_ps(yyyy_ps(m1[1]), m2[1]);
	r2 = mul_ps(zzzz_ps(m1[1]), m2[2]);
	r3 = mul_ps(m1[1], m2_3);
	out[1] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	r0 = mul_ps(xxxx_ps(m1[2]), m2[0]);
	r1 = mul_ps(yyyy_ps(m1[2]), m2[1]);
	r2 = mul_ps(zzzz_ps(m1[2]), m2[2]);
	r3 = mul_ps(m1[2], m2_3);
	out[2] = add_ps(add_ps(r0, r1), add_ps(r2, r3));
}

// Multiplies a 4x4 matrix by a 3x4 matrix, producing a 4x4 matrix.
// @param out A 4x4 matrix (4 x __m128)
// @param m1 left-hand side matrix (4 x __m128)
// @param m2 right-hand side matrix (3 x __m128)
inline void mat4x4_mul_mat3x4_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	const __m128 m2_3 = set_ps(1.f, 0.f, 0.f, 0.f);

	__m128 r0 = mul_ps(xxxx_ps(m1[0]), m2[0]);
	__m128 r1 = mul_ps(yyyy_ps(m1[0]), m2[1]);
	__m128 r2 = mul_ps(zzzz_ps(m1[0]), m2[2]);
	__m128 r3 = mul_ps(m1[0], m2_3);
	out[0] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	r0 = mul_ps(xxxx_ps(m1[1]), m2[0]);
	r1 = mul_ps(yyyy_ps(m1[1]), m2[1]);
	r2 = mul_ps(zzzz_ps(m1[1]), m2[2]);
	r3 = mul_ps(m1[1], m2_3);
	out[1] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	r0 = mul_ps(xxxx_ps(m1[2]), m2[0]);
	r1 = mul_ps(yyyy_ps(m1[2]), m2[1]);
	r2 = mul_ps(zzzz_ps(m1[2]), m2[2]);
	r3 = mul_ps(m1[2], m2_3);
	out[2] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	r0 = mul_ps(xxxx_ps(m1[3]), m2[0]);
	r1 = mul_ps(yyyy_ps(m1[3]), m2[1]);
	r2 = mul_ps(zzzz_ps(m1[3]), m2[2]);
	r3 = mul_ps(m1[3], m2_3);
	out[3] = add_ps(add_ps(r0, r1), add_ps(r2, r3));
}

// Multiplies a 3x4 matrix by a 4x4 matrix, producing a 4x4 matrix.
// @param out A 4x4 matrix (4 x __m128)
// @param m1 left-hand side matrix (3 x __m128)
// @param m2 right-hand side matrix (4 x __m128)
inline void mat3x4_mul_mat4x4_sse(__m128 *out, const __m128 *m1, const __m128 *m2)
{
	__m128 s0 = xxxx_ps(m1[0]);
	__m128 s1 = yyyy_ps(m1[0]);
	__m128 s2 = zzzz_ps(m1[0]);
	__m128 s3 = wwww_ps(m1[0]);
	__m128 r0 = mul_ps(s0, m2[0]);
	__m128 r1 = mul_ps(s1, m2[1]);
	__m128 r2 = mul_ps(s2, m2[2]);
	__m128 r3 = mul_ps(s3, m2[3]);
	out[0] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	s0 = xxxx_ps(m1[1]);
	s1 = yyyy_ps(m1[1]);
	s2 = zzzz_ps(m1[1]);
	s3 = wwww_ps(m1[1]);
	r0 = mul_ps(s0, m2[0]);
	r1 = mul_ps(s1, m2[1]);
	r2 = mul_ps(s2, m2[2]);
	r3 = mul_ps(s3, m2[3]);
	out[1] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	s0 = xxxx_ps(m1[2]);
	s1 = yyyy_ps(m1[2]);
	s2 = zzzz_ps(m1[2]);
	s3 = wwww_ps(m1[2]);
	r0 = mul_ps(s0, m2[0]);
	r1 = mul_ps(s1, m2[1]);
	r2 = mul_ps(s2, m2[2]);
	r3 = mul_ps(s3, m2[3]);
	out[2] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	out[3] = m2[3];
}

// Multiplies a 3x3 matrix by a 4x4 matrix, producing a 4x4 matrix.
// @param out A 4x4 matrix (4 x __m128)
// @param m1 left-hand side matrix (3x3 floats)
// @param m2 right-hand side matrix (4 x __m128)
inline void mat3x3_mul_mat4x4_sse(__m128 *out, const float *m1, const __m128 *m2)
{
	__m128 m1_0 = loadu_ps(m1);

	__m128 s0 = xxxx_ps(m1_0);
	__m128 s1 = yyyy_ps(m1_0);
	__m128 s2 = zzzz_ps(m1_0);
	__m128 r0 = mul_ps(s0, m2[0]);
	__m128 r1 = mul_ps(s1, m2[1]);
	__m128 r2 = mul_ps(s2, m2[2]);
	out[0] = add_ps(add_ps(r0, r1), r2);

	s0 = wwww_ps(m1_0);
	__m128 m1_1 = loadu_ps(m1+4);
	s1 = xxxx_ps(m1_1);
	s2 = yyyy_ps(m1_1);
	r0 = mul_ps(s0, m2[0]);
	r1 = mul_ps(s1, m2[1]);
	r2 = mul_ps(s2, m2[2]);
	out[1] = add_ps(add_ps(r0, r1), r2);

	s0 = zzzz_ps(m1_1);
	s1 = wwww_ps(m1_1);
	s2 = load1_ps(m1+8);
	r0 = mul_ps(s0, m2[0]);
	r1 = mul_ps(s1, m2[1]);
	r2 = mul_ps(s2, m2[2]);
	out[2] = add_ps(add_ps(r0, r1), r2);

	out[3] = m2[3];
}

// Multiplies a 4x4 matrix by a 3x3 matrix, producing a 4x4 matrix.
// @param out A 4x4 matrix (4 x __m128)
// @param m1 left-hand side matrix (4 x __m128)
// @param m2 right-hand side matrix (3x3 floats)
inline void mat4x4_mul_mat3x3_sse(__m128 *out, const __m128 *m1, const float *m2)
{
	__m128 m2_0 = load_vec3(m2, 0.f);
	__m128 m2_1 = load_vec3(m2+3, 0.f);
	__m128 m2_2 = load_vec3(m2+6, 0.f);
	const __m128 m2_3 = set_ps(1.f, 0.f, 0.f, 0.f);

	__m128 s0 = xxxx_ps(m1[0]);
	__m128 s1 = yyyy_ps(m1[0]);
	__m128 s2 = zzzz_ps(m1[0]);
	__m128 s3 = wwww_ps(m1[0]);
	__m128 r0 = mul_ps(s0, m2_0);
	__m128 r1 = mul_ps(s1, m2_1);
	__m128 r2 = mul_ps(s2, m2_2);
	__m128 r3 = mul_ps(s3, m2_3);
	out[0] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	s0 = xxxx_ps(m1[1]);
	s1 = yyyy_ps(m1[1]);
	s2 = zzzz_ps(m1[1]);
	s3 = wwww_ps(m1[1]);
	r0 = mul_ps(s0, m2_0);
	r1 = mul_ps(s1, m2_1);
	r2 = mul_ps(s2, m2_2);
	r3 = mul_ps(s3, m2_3);
	out[1] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	s0 = xxxx_ps(m1[2]);
	s1 = yyyy_ps(m1[2]);
	s2 = zzzz_ps(m1[2]);
	s3 = wwww_ps(m1[2]);
	r0 = mul_ps(s0, m2_0);
	r1 = mul_ps(s1, m2_1);
	r2 = mul_ps(s2, m2_2);
	r3 = mul_ps(s3, m2_3);
	out[2] = add_ps(add_ps(r0, r1), add_ps(r2, r3));

	s0 = xxxx_ps(m1[3]);
	s1 = yyyy_ps(m1[3]);
	s2 = zzzz_ps(m1[3]);
	s3 = wwww_ps(m1[3]);
	r0 = mul_ps(s0, m2_0);
	r1 = mul_ps(s1, m2_1);
	r2 = mul_ps(s2, m2_2);
	r3 = mul_ps(s3, m2_3);
	out[3] = add_ps(add_ps(r0, r1), add_ps(r2, r3));
}
// Computes the inverse of a 4x4 matrix via direct cofactor expansion.
/// Returns the determinant of the original matrix, and zero on failure.
#define MAT_COFACTOR(mat, i, j) \
	msub_ps(_mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(j,j,j,j)), \
	        shuffle1_ps(_mm_shuffle_ps(mat[3], mat[2], _MM_SHUFFLE(i,i,i,i)), _MM_SHUFFLE(2,0,0,0)), \
	        mul_ps(shuffle1_ps(_mm_shuffle_ps(mat[3], mat[2], _MM_SHUFFLE(j,j,j,j)), _MM_SHUFFLE(2,0,0,0)), \
	               _mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(i,i,i,i))))
FORCE_INLINE float mat4x4_inverse(const simd4f *mat, simd4f *out)
{
	simd4f f1 = MAT_COFACTOR(mat, 3, 2);
	simd4f f2 = MAT_COFACTOR(mat, 3, 1);
	simd4f f3 = MAT_COFACTOR(mat, 2, 1);
	simd4f f4 = MAT_COFACTOR(mat, 3, 0);
	simd4f f5 = MAT_COFACTOR(mat, 2, 0);
	simd4f f6 = MAT_COFACTOR(mat, 1, 0);
	simd4f v1 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(0,0,0,0)), _MM_SHUFFLE(2,2,2,0));
	simd4f v2 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(1,1,1,1)), _MM_SHUFFLE(2,2,2,0));
	simd4f v3 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(2,2,2,2)), _MM_SHUFFLE(2,2,2,0));
	simd4f v4 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(3,3,3,3)), _MM_SHUFFLE(2,2,2,0));
	const simd4f s1 = set_ps(-0.0f,  0.0f, -0.0f,  0.0f);
	const simd4f s2 = wzyx_ps(s1); // [+ - + -]
	simd4f r1 = xor_ps(s1, madd_ps(v4, f3, msub_ps(v2, f1, mul_ps(v3, f2))));
	simd4f r2 = xor_ps(s2, madd_ps(v4, f5, msub_ps(v1, f1, mul_ps(v3, f4))));
	simd4f r3 = xor_ps(s1, madd_ps(v4, f6, msub_ps(v1, f2, mul_ps(v2, f4))));
	simd4f r4 = xor_ps(s2, madd_ps(v3, f6, msub_ps(v1, f3, mul_ps(v2, f5))));
	simd4f det = dot4_ps(mat[0], _mm_movelh_ps(_mm_unpacklo_ps(r1, r2), _mm_unpacklo_ps(r3, r4)));
	simd4f rcp = rcp_ps(det);
	out[0] = mul_ps(r1, rcp);
	out[1] = mul_ps(r2, rcp);
	out[2] = mul_ps(r3, rcp);
	out[3] = mul_ps(r4, rcp);
	return s4f_x(det);
}

#define MAT3x4_COFACTOR(mat, i, j) \
	msub_ps(_mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(j,j,j,j)), \
	        shuffle1_ps(_mm_shuffle_ps(mat_3, mat[2], _MM_SHUFFLE(i,i,i,i)), _MM_SHUFFLE(2,0,0,0)), \
	        mul_ps(shuffle1_ps(_mm_shuffle_ps(mat_3, mat[2], _MM_SHUFFLE(j,j,j,j)), _MM_SHUFFLE(2,0,0,0)), \
	               _mm_shuffle_ps(mat[2], mat[1], _MM_SHUFFLE(i,i,i,i))))
FORCE_INLINE float mat3x4_inverse(const simd4f *mat, simd4f *out)
{
	///\todo There might be a way to exploit here that the last row is a [0,0,0,1], and avoid some computations.
	simd4f mat_3 = set_ps(1.f, 0.f, 0.f, 0.f);
	simd4f f1 = MAT3x4_COFACTOR(mat, 3, 2);
	simd4f f2 = MAT3x4_COFACTOR(mat, 3, 1);
	simd4f f3 = MAT3x4_COFACTOR(mat, 2, 1);
	simd4f f4 = MAT3x4_COFACTOR(mat, 3, 0);
	simd4f f5 = MAT3x4_COFACTOR(mat, 2, 0);
	simd4f f6 = MAT3x4_COFACTOR(mat, 1, 0);
	simd4f v1 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(0,0,0,0)), _MM_SHUFFLE(2,2,2,0));
	simd4f v2 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(1,1,1,1)), _MM_SHUFFLE(2,2,2,0));
	simd4f v3 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(2,2,2,2)), _MM_SHUFFLE(2,2,2,0));
	simd4f v4 = shuffle1_ps(_mm_shuffle_ps(mat[1], mat[0], _MM_SHUFFLE(3,3,3,3)), _MM_SHUFFLE(2,2,2,0));
	const simd4f s1 = set_ps(-0.0f,  0.0f, -0.0f,  0.0f);
	const simd4f s2 = wzyx_ps(s1); // [+ - + -]
	simd4f r1 = xor_ps(s1, madd_ps(v4, f3, msub_ps(v2, f1, mul_ps(v3, f2))));
	simd4f r2 = xor_ps(s2, madd_ps(v4, f5, msub_ps(v1, f1, mul_ps(v3, f4))));
	simd4f r3 = xor_ps(s1, madd_ps(v4, f6, msub_ps(v1, f2, mul_ps(v2, f4))));
	simd4f r4 = xor_ps(s2, madd_ps(v3, f6, msub_ps(v1, f3, mul_ps(v2, f5))));
	simd4f det = dot4_ps(mat[0], _mm_movelh_ps(_mm_unpacklo_ps(r1, r2), _mm_unpacklo_ps(r3, r4)));
	simd4f rcp = _mm_rcp_ps(det);
	out[0] = mul_ps(r1, rcp);
	out[1] = mul_ps(r2, rcp);
	out[2] = mul_ps(r3, rcp);
	return s4f_x(det);
}

/// Inverts a 3x4 affine transformation matrix (in row-major format) that only consists of rotation (+possibly mirroring) and translation.
FORCE_INLINE void mat3x4_inverse_orthonormal(const simd4f *mat, simd4f *out)
{
	// mat[0]: [tx,02,01,00]
	// mat[1]: [ty,12,11,10]
	// mat[2]: [tz,22,21,20]
	// mat[3]: assumed to be [1,0,0,0] - not read.

	// First get the translation part (tx,ty,tz) from the original matrix,
	// and compute T=-M^(-1).
	simd4f tx = wwww_ps(mat[0]);
	simd4f ty = wwww_ps(mat[1]);
	simd4f tz = wwww_ps(mat[2]);
	simd4f vec = mul_ps(tx, mat[0]);
	vec = madd_ps(ty, mat[1], vec);
	vec = mnsub_ps(tz, mat[2], vec);

	simd4f tmp0 = _mm_unpacklo_ps(mat[0], mat[1]); // [11,01,10,00]
	simd4f tmp1 = _mm_unpackhi_ps(mat[0], mat[1]); // [ty,tx,12,02]
	simd4f tmp2 = _mm_unpacklo_ps(mat[2], vec);    // [Ty,21,Tx,20]
	simd4f tmp3 = _mm_unpackhi_ps(mat[2], vec);    // [ _,23,Tz,22]

	out[0] = _mm_movelh_ps(tmp0, tmp2);            // [Tx,20,10,00]
	out[1] = _mm_movehl_ps(tmp2, tmp0);            // [Ty,21,11,01]
	out[2] = _mm_movelh_ps(tmp1, tmp3);            // [Tz,22 12,02]
	// out[3] = assumed to be [1,0,0,0] - no need to write back.
}

/// Inverts a 3x4 affine transformation matrix (in row-major format) that only consists of rotation (+possibly mirroring), uniform scale and translation.
FORCE_INLINE void mat3x4_inverse_orthogonal_uniformscale(const simd4f *mat, simd4f *out)
{
	// mat[0]: [tx,02,01,00]
	// mat[1]: [ty,12,11,10]
	// mat[2]: [tz,22,21,20]
	// mat[3]: assumed to be [1,0,0,0] - not read.

	simd4f scale = rcp_ps(sum_xyz_ps(mul_ps(mat[0], mat[0])));

	// First get the translation part (tx,ty,tz) from the original matrix,
	// and compute T=-M^(-1).
	simd4f tx = wwww_ps(mat[0]);
	simd4f ty = wwww_ps(mat[1]);
	simd4f tz = wwww_ps(mat[2]);
	simd4f m0 = mul_ps(scale, mat[0]);
	simd4f m1 = mul_ps(scale, mat[1]);
	simd4f m2 = mul_ps(scale, mat[2]);
	simd4f vec = mul_ps(tx, m0);
	vec = madd_ps(ty, m1, vec);
	vec = mnsub_ps(tz, m2, vec);

	simd4f tmp0 = _mm_unpacklo_ps(m0, m1); // [11,01,10,00]
	simd4f tmp1 = _mm_unpackhi_ps(m0, m1); // [ty,tx,12,02]
	simd4f tmp2 = _mm_unpacklo_ps(m2, vec);    // [Ty,21,Tx,20]
	simd4f tmp3 = _mm_unpackhi_ps(m2, vec);    // [ _,23,Tz,22]

	out[0] = _mm_movelh_ps(tmp0, tmp2);            // [Tx,20,10,00]
	out[1] = _mm_movehl_ps(tmp2, tmp0);            // [Ty,21,11,01]
	out[2] = _mm_movelh_ps(tmp1, tmp3);            // [Tz,22 12,02]
 // out[3] = assumed to be [1,0,0,0] - no need to write back.
}

FORCE_INLINE void mat3x4_inverse_colorthogonal(const simd4f *mat, simd4f *out)
{
	// mat[0]: [tx,02,01,00]
	// mat[1]: [ty,12,11,10]
	// mat[2]: [tz,22,21,20]
	// mat[3]: assumed to be [1,0,0,0] - not read.

	simd4f scale = rcp_ps(madd_ps(mat[0], mat[0], madd_ps(mat[1], mat[1], mul_ps(mat[2], mat[2]))));

	// First get the translation part (tx,ty,tz) from the original matrix,
	// and compute T=-M^(-1).
	simd4f tx = wwww_ps(mat[0]);
	simd4f ty = wwww_ps(mat[1]);
	simd4f tz = wwww_ps(mat[2]);
	simd4f m0 = mul_ps(scale, mat[0]);
	simd4f m1 = mul_ps(scale, mat[1]);
	simd4f m2 = mul_ps(scale, mat[2]);
	simd4f vec = mul_ps(tx, m0);
	vec = madd_ps(ty, m1, vec);
	vec = mnsub_ps(tz, m2, vec);

	simd4f tmp0 = _mm_unpacklo_ps(m0, m1); // [11,01,10,00]
	simd4f tmp1 = _mm_unpackhi_ps(m0, m1); // [ty,tx,12,02]
	simd4f tmp2 = _mm_unpacklo_ps(m2, vec);    // [Ty,21,Tx,20]
	simd4f tmp3 = _mm_unpackhi_ps(m2, vec);    // [ _,23,Tz,22]

	out[0] = _mm_movelh_ps(tmp0, tmp2);            // [Tx,20,10,00]
	out[1] = _mm_movehl_ps(tmp2, tmp0);            // [Ty,21,11,01]
	out[2] = _mm_movelh_ps(tmp1, tmp3);            // [Tz,22 12,02]
	// out[3] = assumed to be [1,0,0,0] - no need to write back.
}

#endif

#ifdef MATH_SIMD

inline float mat4x4_determinant(const simd4f *row)
{
	simd4f s = xxxx_ps(rcp_ps(row[0]));
	// row[0].x has a factor of the final determinant.
	simd4f row0 = mul_ps(s, row[0]);
	s = xxxx_ps(row[1]);
	simd4f row1 = mnadd_ps(s, row0, row[1]);
	s = xxxx_ps(row[2]);
	simd4f row2 = mnadd_ps(s, row0, row[2]);
	s = xxxx_ps(row[3]);
	simd4f row3 = mnadd_ps(s, row0, row[3]);

	// row1.y has a factor of the final determinant.
	s = yyyy_ps(rcp_ps(row1));
	simd4f row1_1 = mul_ps(s, row1);
	s = yyyy_ps(row2);
	simd4f row2_1 = mnadd_ps(s, row1_1, row2);
	s = yyyy_ps(row3);
	simd4f row3_1 = mnadd_ps(s, row1_1, row3);

	// Now we are left with a 2x2 matrix in row2_1.zw and row3_1.zw.
	// D = row2_1.z * row3_1.w - row2_1.w * row3_1.z.
	simd4f r1 = xywz_ps(row2_1);
	simd4f r = mul_ps(r1, row3_1);
	simd4f a = wwww_ps(r);
	simd4f b = zzzz_ps(r);
	simd4f d1 = sub_ps(a, b); // TODO: sub_ss
	simd4f d2 = row[0];
	simd4f d3 = yyyy_ps(row1);
	simd4f d = mul_ps(d1, mul_ps(d2, d3)); // TODO: mul_ss + sub_ss
	return s4f_x(d);
}

/// Computes the determinant of a 3x4 matrix stored in row-major format. (Treated as a square matrix with last row [0,0,0,1])
inline float mat3x4_determinant(const simd4f *row)
{
	simd4f s = xxxx_ps(rcp_ps(row[0]));
	// row[0].x has a factor of the final determinant.
	simd4f row0 = mul_ps(s, row[0]);
	s = xxxx_ps(row[1]);
	simd4f row1 = mnadd_ps(s, row0, row[1]);
	s = xxxx_ps(row[2]);
	simd4f row2 = mnadd_ps(s, row0, row[2]);

	// Now we are left with a 2x2 matrix in row1.yz and row2.yz.
	// D = row1.y * row2.z - row2.y * row1.z.
	simd4f r1 = xzyw_ps(row1);
	simd4f r = mul_ps(r1, row2);
	simd4f a = zzzz_ps(r);
	simd4f b = yyyy_ps(r);
	simd4f d1 = sub_ps(a, b); // TODO: sub_ss
	simd4f d2 = row[0];
	simd4f d = mul_ps(d1, d2); // TODO: mul_ss
	return s4f_x(d);
}

#endif
#ifdef MATH_SSE

inline void mat3x4_transpose(const simd4f *src, simd4f *dst)
{
	simd4f src3 = zero_ps(); // w component should be 1, but since it won't get stored, it doesn't matter, so we can just create zeros.
	simd4f tmp0 = _mm_unpacklo_ps(src[0], src[1]);
	simd4f tmp2 = _mm_unpacklo_ps(src[2], src3);
	simd4f tmp1 = _mm_unpackhi_ps(src[0], src[1]);
	simd4f tmp3 = _mm_unpackhi_ps(src[2], src3);
	dst[0] = _mm_movelh_ps(tmp0, tmp2);
	dst[1] = _mm_movehl_ps(tmp2, tmp0);
	dst[2] = _mm_movelh_ps(tmp1, tmp3);
}

MATH_END_NAMESPACE

#endif // ~MATH_SSE
