#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/SSEMath.h"
#include "../src/Math/float4x4_sse.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace MATH_NS::TestData;

// In debug mode, we test the correctness of the SSE code. In release mode, we benchmark performance.
#ifdef _DEBUG
#define TEST_SSE_CORRECTNESS
#endif

#if 0 // This code generates a listing of all SSE shuffle possibilities.
char Arg(float f)
{
	int i = RoundInt(f);
	switch(i)
	{
	case 100: return 'x';
	case 101: return 'y';
	case 102: return 'z';
	case 103: return 'w';
	case 200: return 'a';
	case 201: return 'b';
	case 202: return 'c';
	case 203: return 'd';
	case 0: return '0';
	default: return '-';
	}
}

#define UNARY_SSE128(op) \
	{ \
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100); \
	__m128 ret = op(arg1); \
	float4 f = ret; \
	printf("%s([wzyx])=[%c%c%c%c]\n", #op, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x)); \
	}

#define BINARY_SSE128(op) \
	{ \
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100); \
	__m128 arg2 = _mm_set_ps(203, 202, 201, 200); \
	__m128 ret = op(arg1, arg2); \
	float4 f = ret; \
	printf("%s([wzyx], [dcba])=[%c%c%c%c]\n", #op, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x)); \
	}

template<int imm>
void Shuffle_ps()
{
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100);
	__m128 arg2 = _mm_set_ps(203, 202, 201, 200);
	__m128 ret = _mm_shuffle_ps(arg1, arg2, imm);
	float4 f = ret;
	printf("%s([wzyx], [dcba], 0x%X)=[%c%c%c%c]\n", "_mm_shuffle_ps", imm, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x));
	Shuffle_ps<imm+1>();
}

template<>
void Shuffle_ps<256>()
{
}

template<int imm>
void Permute_ps()
{
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100);
	__m128 ret = _mm_permute_ps(arg1, imm);
	float4 f = ret;
	printf("%s([wzyx], 0x%X)=[%c%c%c%c]\n", "_mm_permute_ps", imm, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x));
	Permute_ps<imm+1>();
}

template<>
void Permute_ps<256>()
{
}

template<int imm>
void Blend_ps()
{
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100);
	__m128 arg2 = _mm_set_ps(203, 202, 201, 200);
	__m128 ret = _mm_blend_ps(arg1, arg2, imm);
	float4 f = ret;
	printf("%s([wzyx], [dcba], 0x%X)=[%c%c%c%c]\n", "_mm_blend_ps", imm, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x));
	Blend_ps<imm+1>();
}

template<>
void Blend_ps<16>()
{
}

template<int imm>
void Extract_ps()
{
	__m128 arg1 = _mm_set_ps(103, 102, 101, 100);
	__m128 ret = _mm_extract_ps(arg1, imm);
	float4 f = ret;
	printf("%s([wzyx], 0x%X)=[%c%c%c%c]\n", "_mm_extract_ps", imm, Arg(f.w), Arg(f.z), Arg(f.y), Arg(f.x));
	Extract_ps<imm+1>();
}

template<>
void Extract_ps<4>()
{
}

void SolveShuffles()
{
	BINARY_SSE128(_mm_unpacklo_ps);
	BINARY_SSE128(_mm_unpackhi_ps);
	BINARY_SSE128(_mm_move_ss);
	BINARY_SSE128(_mm_movehl_ps);
	BINARY_SSE128(_mm_movelh_ps);
	Shuffle_ps<0>();
	UNARY_SSE128(_mm_moveldup_ps);
	UNARY_SSE128(_mm_movehdup_ps);
//	UNARY_SSE128(_mm_broadcastss_ps); // AVX 2
	Blend_ps<0>();
	Permute_ps<0>();
}

UNIQUE_TEST(sse_solve_shuffle)
{
	SolveShuffles();
}
#endif

#ifdef MATH_SSE

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

#endif

#ifdef MATH_AVX

// Experimental test to use AVX 8-wide registers for matrix*vector
// Assumes matrix is stored in row-major.
inline __m128 mat4x4_mul_avx(const __m256 *matrix, __m128 vector)
{
	__m256 r01 = matrix[0]; // [ row1, row0 ]
	__m256 r23 = matrix[1]; // [ row3, row2 ]

	__m256 v = _mm256_broadcast_ps(&vector); // [w, z, y, x, w, z, y, x]

	__m256 mul1 = _mm256_mul_ps(r01, v); // [ Y4,Y3,Y2,Y1, X4,X3,X2,X1 ]
	__m256 mul2 = _mm256_mul_ps(r23, v); // [ W4,W3,W2,W1, Z4,Z3,Z2,Z1 ]

	__m256 res = _mm256_hadd_ps(mul1, mul2); // [ W4+W3, W2+W1, Y4+Y3, Y2+Y1, Z4+Z3, Z2+Z1, X4+X3, X2+X1 ]
	res = _mm256_hadd_ps(res, res); // [ W, Y, W, Y, Z, X, Z, X ]

	__m128 hi = _mm256_extractf128_ps(res, 1); // [W, Y, W, Y]
	__m128 lo = _mm256_castps256_ps128(res);  // [Z, X, Z, X]

	__m128 ret = _mm_unpacklo_ps(lo, hi); // [W, Z, Y, X]
	return ret;
}

inline __m128 mat4x4_mul_avx_2(const __m256 *matrix, __m128 vector)
{
	__m256 r01 = matrix[0]; // [ row1, row0 ]
	__m256 r23 = matrix[1]; // [ row3, row2 ]

	__m256 v = _mm256_broadcast_ps(&vector); // [w, z, y, x, w, z, y, x]

	__m256 dp1 = _mm256_dp_ps(r01, v, 0xF1); // [0, 0, 0, Y, 0, 0, 0, X]
	__m256 dp2 = _mm256_dp_ps(r23, v, 0xF2); // [0, 0, W, 0, 0, 0, Z, 0]

	__m256 dp = _mm256_add_ps(dp1, dp2);     // [0, 0, W, Y, 0, 0, Z, X]

	__m128 wy = _mm256_extractf128_ps(dp, 1); // [0, 0, W, Y]

	__m128 xz = _mm256_castps256_ps128(dp);  // [0, 0, Z, X]
	return _mm_unpacklo_ps(xz, wy); // [ W, Z, Y, X]
}

inline __m128 colmajor_mat4x4_mul_avx(const __m256 *matrix, __m128 vector)
{
	__m256 x = _mm256_castps128_ps256(xxxx_ps(vector));
	__m128 y = yyyy_ps(vector);
	__m256 z = _mm256_castps128_ps256(zzzz_ps(vector));
	__m128 w = wwww_ps(vector);

	__m256 yx = _mm256_insertf128_ps(x, y, 1); // [yyyyxxxx]
	__m256 wz = _mm256_insertf128_ps(z, w, 1); // [wwwwzzzz]

	yx = _mm256_mul_ps(matrix[0], yx);
	wz = _mm256_mul_ps(matrix[1], wz);

	__m256 wzyx = _mm256_add_ps(wz, yx);
	__m128 hi = _mm256_extractf128_ps(wzyx, 1);
	__m128 lo = _mm256_castps256_ps128(wzyx);
	return _mm_add_ps(hi, lo);
}

inline __m128 colmajor_mat4x4_mul_avx_2(const __m256 *matrix, __m128 vector)
{
	__m128 zwxy = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2,3,0,1));						// Latency: 1, Throughput: 1
	__m256 v = _mm256_insertf128_ps(_mm256_castps128_ps256(vector), zwxy, 1); // [zwxywzyx] // ?
	__m256 yx = _mm256_shuffle_ps(v, v, _MM_SHUFFLE(0,0,0,0)); // [yyyyxxxx]				// Latency: 1, Throughput: 1
	__m256 wz = _mm256_shuffle_ps(v, v, _MM_SHUFFLE(2,2,2,2)); // [wwwwzzzz]				// Latency: 1, Throughput: 1

	yx = _mm256_mul_ps(matrix[0], yx); // Latency: 5, Throughput: 1
	wz = _mm256_mul_ps(matrix[1], wz);

	__m256 wzyx = _mm256_add_ps(wz, yx); // Latency: 3, Throughput: 1
	__m128 hi = _mm256_extractf128_ps(wzyx, 1); // Latency: 3, Throughput: 1
	__m128 lo = _mm256_castps256_ps128(wzyx); // -
	return _mm_add_ps(hi, lo); // Latency: 3, Throughput: 1
}

BENCHMARK(float3_cross, "float3::Cross")
{
	v2[i].Float3Part() = v[i].Float3Part().Cross(v2[i].Float3Part());
}
BENCHMARK_END;

BENCHMARK(float4_cross, "float4::Cross3")
{
	v2[i] = v[i].Cross3(v2[i]);
}
BENCHMARK_END;

BENCHMARK(mat4x4_mul_avx, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_avx((__m256*)&m[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_avx)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = mat4x4_mul_avx((__m256*)&m, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

BENCHMARK(mat4x4_mul_avx_2, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_avx_2((__m256*)&m[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_avx_2)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = mat4x4_mul_avx_2((__m256*)&m, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

#endif // ~AVX

#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
/// Compute the product M*v, where M is a 4x4 matrix denoted by an array of 4 __m128's, and v is a 4x1 vector.
// Note: There is an indication that this version is slower than the SSE3 version. Be sure to profile.
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

BENCHMARK(mat4x4_mul_sse41, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_sse41(m[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_sse41)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = mat4x4_mul_sse41(m.row, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

#endif

#ifdef MATH_SSE3

BENCHMARK(mat4x4_mul_sse3, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_sse3(m[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_sse3)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = mat4x4_mul_sse3(m.row, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

#endif

#ifdef MATH_SSE

BENCHMARK(mat4x4_mul_sse1, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_sse1(m[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_sse1)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = mat4x4_mul_sse1(m.row, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

BENCHMARK(colmajor_mat4x4_mul_sse1, "test against float4x4_mul_float4")
{
	v2[i] = colmajor_mat4x4_mul_sse1(tpm[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(colmajor_mat4x4_mul_sse1)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 tpm = m.Transposed();
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = colmajor_mat4x4_mul_sse1(tpm.row, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

#ifdef MATH_AVX

BENCHMARK(colmajor_mat4x4_mul_avx, "test against float4x4_mul_float4")
{
	v2[i] = colmajor_mat4x4_mul_avx((__m256*)tpm[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(colmajor_mat4x4_mul_avx)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 tpm = m.Transposed();
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = colmajor_mat4x4_mul_avx(tpm.row2, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

BENCHMARK(colmajor_mat4x4_mul_avx_2, "test against float4x4_mul_float4")
{
	v2[i] = colmajor_mat4x4_mul_avx_2((__m256*)tpm[i].row, v[i]);
}
BENCHMARK_END;

RANDOMIZED_TEST(colmajor_mat4x4_mul_avx_2)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 tpm = m.Transposed();
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 res = colmajor_mat4x4_mul_avx_2(tpm.row2, v);
	float4 res2 = m*v;
	assert(res.Equals(res2));
}

#endif

BENCHMARK(mat4x4_mul_dpps, "test against float4x4_op_mul")
{
	mat4x4_mul_dpps(m2[i].row, m[0].row, m[i].row);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_dpps)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 res;
	mat4x4_mul_dpps(res.row, m.row, m2.row);
	float4x4 res2 = m*m2;
	assert(res.Equals(res2));
}

BENCHMARK(mat4x4_mul_dpps_2, "test against float4x4_op_mul")
{
	mat4x4_mul_dpps(m2[i].row, m[0].row, m[i].row);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_dpps_2)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 res;
	mat4x4_mul_dpps_2(res.row, m.row, m2.row);
	float4x4 res2 = m*m2;
	assert(res.Equals(res2));
}

BENCHMARK(mat4x4_mul_dpps_3, "test against float4x4_op_mul")
{
	mat4x4_mul_dpps(m2[i].row, m[0].row, m[i].row);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_dpps_3)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 res;
	mat4x4_mul_dpps_3(res.row, m.row, m2.row);
	float4x4 res2 = m*m2;
	assert(res.Equals(res2));
}

#ifdef MATH_FMA

FORCE_INLINE void mat4x4_mul_fma(__m128 *out, const __m128 *m1, const __m128 *m2)
{
#ifdef MATH_64BIT // In 64-bit, we have lots of SIMD registers, so use as many as possible.
    // 64-bit, SSE4.1, FMA:
    // Benchmark 'mat4x4_mul_fma': test against float4x4_op_mul
    //   Best: 5.019 nsecs / 16.068 ticks, Avg: 5.320 nsecs, Worst: 7.362 nsecs
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
    o1 = madd_ps(yyyy_ps(m1_0), m, o1);
    o2 = madd_ps(yyyy_ps(m1_1), m, o2);
    o3 = madd_ps(yyyy_ps(m1_2), m, o3);
    o4 = madd_ps(yyyy_ps(m1_3), m, o4);
    m = m2[2];
    o1 = madd_ps(zzzz_ps(m1_0), m, o1);
    o2 = madd_ps(zzzz_ps(m1_1), m, o2);
    o3 = madd_ps(zzzz_ps(m1_2), m, o3);
    o4 = madd_ps(zzzz_ps(m1_3), m, o4);
    m = m2[3];
    out[0] = madd_ps(wwww_ps(m1_0), m, o1);
    out[1] = madd_ps(wwww_ps(m1_1), m, o2);
    out[2] = madd_ps(wwww_ps(m1_2), m, o3);
    out[3] = madd_ps(wwww_ps(m1_3), m, o4);
#else // Targeting 32-bit, use as few registers as possible to avoid spilling.
    // 32-bit, SSE4.1:
    // Benchmark 'mat4x4_mul_fma': test against float4x4_op_mul
    //   Best: 6.358 nsecs / 19.42 ticks, Avg: 6.461 nsecs, Worst: 9.369 nsecs
    out[0] = madd_ps(wwww_ps(m1[0]), m2[3], madd_ps(zzzz_ps(m1[0]), m2[2], madd_ps(yyyy_ps(m1[0]), m2[1], mul_ps(xxxx_ps(m1[0]), m2[0]))));
    out[1] = madd_ps(wwww_ps(m1[1]), m2[3], madd_ps(zzzz_ps(m1[1]), m2[2], madd_ps(yyyy_ps(m1[1]), m2[1], mul_ps(xxxx_ps(m1[1]), m2[0]))));
    out[2] = madd_ps(wwww_ps(m1[2]), m2[3], madd_ps(zzzz_ps(m1[2]), m2[2], madd_ps(yyyy_ps(m1[2]), m2[1], mul_ps(xxxx_ps(m1[2]), m2[0]))));
    out[3] = madd_ps(wwww_ps(m1[3]), m2[3], madd_ps(zzzz_ps(m1[3]), m2[2], madd_ps(yyyy_ps(m1[3]), m2[1], mul_ps(xxxx_ps(m1[3]), m2[0]))));
#endif
}

BENCHMARK(mat4x4_mul_fma, "test against float4x4_op_mul")
{
	mat4x4_mul_fma(m2[i].row, m[0].row, m[i].row);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_fma)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 res;
	mat4x4_mul_fma(res.row, m.row, m2.row);
	float4x4 res2 = m*m2;
	assert(res.Equals(res2));
}
#endif

BENCHMARK(mat4x4_mul_sse, "test against float4x4_op_mul")
{
	mat4x4_mul_sse(m2[i].row, m[0].row, m[i].row);
}
BENCHMARK_END;

RANDOMIZED_TEST(mat4x4_mul_sse)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 res;
	mat4x4_mul_sse(res.row, m.row, m2.row);
	float4x4 res2 = m*m2;
	assert(res.Equals(res2));
}

#endif

BENCHMARK(float3_LengthSq, "float3::LengthSq")
{
	f[i] = v[i].Float3Part().LengthSq();
}
BENCHMARK_END;

BENCHMARK(float3_Length, "float3::Length")
{
	f[i] = v[i].Float3Part().Length();
}
BENCHMARK_END;

BENCHMARK(float3_Normalize, "float3::Normalize")
{
	v[i].Float3Part().Normalize();
}
BENCHMARK_END;

BENCHMARK(float4_Normalize3, "float4::Normalize3")
{
	v[i].Normalize3();
}
BENCHMARK_END;

BENCHMARK(float4_Normalize4, "float4::Normalize4")
{
	v[i].Normalize4();
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(float4_Normalize4_Fast_SSE, "test against float4_Normalize4")
{
	v[i].Normalize4_Fast_SSE();
}
BENCHMARK_END;

BENCHMARK(load_vec3, "load_vec3")
{
	// Best: 1.536 nsecs / 4.032 ticks, Avg: 1.544 nsecs, Worst: 1.920 nsecs
	v[i] = load_vec3(v2[i].ptr(), 1.f);
}
BENCHMARK_END;

BENCHMARK(load_vec3_scalar, "load_vec3_scalar")
{
	// Best: 5.761 nsecs / 15.536 ticks, Avg: 5.930 nsecs, Worst: 8.833 nsecs
	const float *ptr = v2[i].ptr();
	v[i] = float4(ptr[2], ptr[0], ptr[1], 1.f);
}
BENCHMARK_END;

BENCHMARK(vec4_set_scalar, "vec4_set_scalar")
{
	const float *ptr = v2[i].ptr();
	v[i] = float4(ptr[3], ptr[0], ptr[4], ptr[2]);
	v[i].v = mul_ps(v[i].v, v[i].v);
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(vec4_set_simd, "vec4_set_simd")
{
	const float *ptr = v2[i].ptr();
	simd4f x = _mm_set_ss(ptr[3]);
	simd4f y = _mm_set_ss(ptr[0]);
	simd4f z = _mm_set_ss(ptr[4]);
	simd4f w = _mm_set_ss(ptr[2]);
	simd4f xy = _mm_unpacklo_ps(x, y);
	simd4f zw = _mm_unpacklo_ps(z, w);
	simd4f xyzw = _mm_movelh_ps(xy, zw);
	v[i].v = mul_ps(xyzw, xyzw);
}
BENCHMARK_END;
#endif

BENCHMARK(vec4_set_simd_2, "vec4_set_simd_2")
{
	const float *ptr = v2[i].ptr();
	simd4f xyzw = set_ps(ptr[3], ptr[0], ptr[4], ptr[2]);
	v[i].v = mul_ps(xyzw, xyzw);
}
BENCHMARK_END;

BENCHMARK(load_vec3_xyz, "load_vec3_xyz")
{
	// Best: 5.761 nsecs / 15.496 ticks, Avg: 6.122 nsecs, Worst: 6.529 nsecs
	const float3 &f3 = reinterpret_cast<const float3&>(v2[i]);
	v[i] = float4(f3, 1.f);
}
BENCHMARK_END;

#endif
