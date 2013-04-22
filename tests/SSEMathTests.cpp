#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

#include "Math/SSEMath.h"

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

float *FloatArray()
{
	LCG lcg;
	static float *arr;
	static bool initialized = false;
	if (!initialized)
	{
		arr = new float[testrunner_numItersPerTest+32];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (float*)a;
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = lcg.Float(-10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4x4 *MatrixArray()
{
	LCG lcg;
	static float4x4 *arr;
	static bool initialized = false;
	if (!initialized)
	{
		arr = new float4x4[testrunner_numItersPerTest+1];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (float4x4*)a;
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = float4x4::RandomGeneral(lcg, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4x4 *MatrixArray2()
{
	LCG lcg;
	static float4x4 *arr;
	static bool initialized = false;
	if (!initialized)
	{
		arr = new float4x4[testrunner_numItersPerTest+1];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (float4x4*)a;
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = float4x4::RandomGeneral(lcg, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4x4 *TransposedMatrixArray()
{
	static float4x4 *arr;
	static bool initialized = false;
	if (!initialized)
	{
		arr = new float4x4[testrunner_numItersPerTest+1];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (float4x4*)a;
		float4x4 *m = MatrixArray();
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = m[i].Transposed();
		initialized = true;
	}
	return arr;
}

float4 *VectorArray()
{
	LCG lcg;
	static float4 arr[testrunner_numItersPerTest];
	static bool initialized = false;
	if (!initialized)
	{
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4 *VectorArray2()
{
	LCG lcg;
	static float4 arr[testrunner_numItersPerTest];
	static bool initialized = false;
	if (!initialized)
	{
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float *f = FloatArray();
float4x4 *m = MatrixArray();
float4x4 *m2 = MatrixArray();
float4x4 *tm = TransposedMatrixArray();
float4 *v = VectorArray();
float4 *v2 = VectorArray2();

#ifdef MATH_AVX

// Experimental test to use AVX 8-wide registers for matrix*vector
// Assumes matrix is stored in row-major.
inline __m128 _mm_mat4x4_mul_ps_avx(const __m256 *matrix, __m128 vector)
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

inline __m128 _mm_mat4x4_mul_ps_avx_2(const __m256 *matrix, __m128 vector)
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

inline __m128 _mm_colmajor_mat4x4_mul_ps_avx(const __m256 *matrix, __m128 vector)
{
	__m256 x = _mm256_castps128_ps256(_mm_shuffle_ps(vector, vector, _MM_SHUFFLE(0,0,0,0)));
	__m128 y = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(1,1,1,1));
	__m256 z = _mm256_castps128_ps256(_mm_shuffle_ps(vector, vector, _MM_SHUFFLE(2,2,2,2)));
	__m128 w = _mm_shuffle_ps(vector, vector, _MM_SHUFFLE(3,3,3,3));

	__m256 yx = _mm256_insertf128_ps(x, y, 1); // [yyyyxxxx]
	__m256 wz = _mm256_insertf128_ps(z, w, 1); // [wwwwzzzz]

	yx = _mm256_mul_ps(matrix[0], yx);
	wz = _mm256_mul_ps(matrix[1], wz);

	__m256 wzyx = _mm256_add_ps(wz, yx);
	__m128 hi = _mm256_extractf128_ps(wzyx, 1);
	__m128 lo = _mm256_castps256_ps128(wzyx);
	return _mm_add_ps(hi, lo);
}

inline __m128 _mm_colmajor_mat4x4_mul_ps_avx_2(const __m256 *matrix, __m128 vector)
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

BENCHMARK(sse_float3_cross)
{
	TIMER_BEGIN
	{
		v2[i].Float3Part() = v[i].Float3Part().Cross(v2[i].Float3Part());
	}
	TIMER_END;
}

BENCHMARK(sse_float4_cross)
{
	TIMER_BEGIN
	{
		v2[i] = v[i].Cross3(v2[i]);
	}
	TIMER_END;
}

BENCHMARK(sse_avx_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_mat4x4_mul_ps_avx((__m256*)&m[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_mat4x4_mul_ps_avx((__m256*)&m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

BENCHMARK(sse_avx_2_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_mat4x4_mul_ps_avx_2((__m256*)&m[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_mat4x4_mul_ps_avx_2((__m256*)&m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

#endif // ~AVX

#ifdef MATH_SSE41

BENCHMARK(sse41_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_mat4x4_mul_ps_sse41(m[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_mat4x4_mul_ps_sse41(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

#endif

#ifdef MATH_SSE3

BENCHMARK(sse3_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_mat4x4_mul_ps_sse3(m[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_mat4x4_mul_ps_sse3(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

#endif

#ifdef MATH_SSE

BENCHMARK(sse1_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_mat4x4_mul_ps_sse1(m[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_mat4x4_mul_ps_sse1(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

BENCHMARK(sse1_colmajor_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_colmajor_mat4x4_mul_ps_sse1(tm[i].row, v[i]);
	}
	TIMER_END;

	float4x4 *m = MatrixArray();
	float4 res = _mm_colmajor_mat4x4_mul_ps_sse1(tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

BENCHMARK(sse1_colmajor_mat_vec_mul_2)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_colmajor_mat4x4_mul_ps_sse1_2(tm[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_colmajor_mat4x4_mul_ps_sse1_2(tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

#ifdef MATH_AVX

BENCHMARK(sse_colmajor_mat_vec_mul_avx)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_colmajor_mat4x4_mul_ps_avx((__m256*)tm[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_colmajor_mat4x4_mul_ps_avx((__m256*)tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

BENCHMARK(sse_colmajor_mat_vec_mul_avx_2)
{
	TIMER_BEGIN
	{
		v2[i] = _mm_colmajor_mat4x4_mul_ps_avx_2((__m256*)tm[i].row, v[i]);
	}
	TIMER_END;

	float4 res = _mm_colmajor_mat4x4_mul_ps_avx_2((__m256*)tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
}

#endif

BENCHMARK(sse_mat_mat_mul)
{
	TIMER_BEGIN
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
	TIMER_END;

	float4x4 res;
	_mm_mat4x4_mul_ps_dpps(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
}

BENCHMARK(sse_mat_mat_mul_2)
{
	TIMER_BEGIN
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps_2(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
	TIMER_END;

	float4x4 res;
	_mm_mat4x4_mul_ps_dpps_2(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
}

BENCHMARK(sse_mat_mat_mul_3)
{
	TIMER_BEGIN
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps_3(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
	TIMER_END;

	float4x4 res;
	_mm_mat4x4_mul_ps_dpps_3(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
}

BENCHMARK(sse_mat_mat_mul_ps)
{
	TIMER_BEGIN
	{
		float4x4 res;
		_mm_mat4x4_mul_ps(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
	TIMER_END;

	float4x4 res;
	_mm_mat4x4_mul_ps(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
}

BENCHMARK(sse_mat_mat_mul_ps_2)
{
	TIMER_BEGIN
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_2(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
	TIMER_END;

	float4x4 res;
	_mm_mat4x4_mul_ps_2(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
}

#endif

BENCHMARK(sse_scalar_mat_vec_mul)
{
	TIMER_BEGIN
	{
		v2[i] = m[i] * v[i];
	}
	TIMER_END;
}

BENCHMARK(sse_scalar_mat_mat_mul)
{
	TIMER_BEGIN
	{
		m2[i] = m[0] * m[i];
	}
	TIMER_END;
}

BENCHMARK(sse_float3_LengthSq)
{
	TIMER_BEGIN
	{
		f[i] = v[i].Float3Part().LengthSq();
	}
	TIMER_END;
}

BENCHMARK(sse_float3_Length)
{
	TIMER_BEGIN
	{
		f[i] = v[i].Float3Part().Length();
	}
	TIMER_END;
}

BENCHMARK(sse_float3_Normalize)
{
	TIMER_BEGIN
	{
		v[i].Float3Part().Normalize();
	}
	TIMER_END;
}

BENCHMARK(sse_float4_Normalize3)
{
	TIMER_BEGIN
	{
		v[i].Normalize3();
	}
	TIMER_END;
}

BENCHMARK(sse_float4_Normalize4)
{
	TIMER_BEGIN
	{
		v[i].Normalize4();
	}
	TIMER_END;
}

#ifdef MATH_SSE
BENCHMARK(sse_float4_Normalize4_Fast_SSE)
{
	TIMER_BEGIN
	{
		v[i].Normalize4_Fast_SSE();
	}
	TIMER_END;
}
#endif
