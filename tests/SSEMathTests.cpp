#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

#include "Math/SSEMath.h"

const int N = 1000;

// In debug mode, we test the correctness of the SSE code. In release mode, we benchmark performance.
#ifdef _DEBUG
#define TEST_SSE_CORRECTNESS
#endif

#ifdef MATH_SSE41

float4x4 *MatrixArray()
{
	static float4x4 arr[N];
	static bool initialized = false;
	if (!initialized)
	{
		for(int i = 0; i < N; ++i)
			arr[i] = float4x4::RandomGeneral(rng, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4x4 *MatrixArray2()
{
	static float4x4 arr[N];
	static bool initialized = false;
	if (!initialized)
	{
		for(int i = 0; i < N; ++i)
			arr[i] = float4x4::RandomGeneral(rng, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

float4x4 *TransposedMatrixArray()
{
	static float4x4 arr[N];
	static bool initialized = false;
	if (!initialized)
	{
		float4x4 *m = MatrixArray();
		for(int i = 0; i < N; ++i)
			arr[i] = m[i].Transposed();
		initialized = true;
	}
	return arr;
}

float4 *VectorArray()
{
	static float4 arr[N];
	static bool initialized = false;
	if (!initialized)
	{
		for(int i = 0; i < N; ++i)
			arr[i] = float4::RandomGeneral(rng, -10.f, 10.f);
		initialized = true;
	}
	return arr;
}

RANDOMIZED_TEST(sse41_mat_vec_mul)
{
	float4x4 *m = MatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = _mm_mat4x4_mul_ps_sse41(m[i].row, v[i]);
		accum += res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4 res = _mm_mat4x4_mul_ps_sse41(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
#endif
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse3_mat_vec_mul)
{
	float4x4 *m = MatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = _mm_mat4x4_mul_ps_sse3(m[i].row, v[i]);
		accum += res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4 res = _mm_mat4x4_mul_ps_sse3(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
#endif
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse1_mat_vec_mul)
{
	float4x4 *m = MatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = _mm_mat4x4_mul_ps_sse1(m[i].row, v[i]);
		accum += res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4 res = _mm_mat4x4_mul_ps_sse1(m[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
#endif
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse1_colmajor_mat_vec_mul)
{
	float4x4 *tm = TransposedMatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = _mm_colmajor_mat4x4_mul_ps_sse1(tm[i].row, v[i]);
		accum += res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 *m = MatrixArray();
	float4 res = _mm_colmajor_mat4x4_mul_ps_sse1(tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
#endif
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse1_colmajor_mat_vec_mul_2)
{
	float4x4 *tm = TransposedMatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = _mm_colmajor_mat4x4_mul_ps_sse1_2(tm[i].row, v[i]);
		accum += res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 *m = MatrixArray();
	float4 res = _mm_colmajor_mat4x4_mul_ps_sse1_2(tm[0].row, v[0]);
	float4 res2 = m[0]*v[0];
	assert(res.Equals(res2));
#endif
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse_scalar_mat_vec_mul)
{
	float4x4 *m = MatrixArray();
	float4 *v = VectorArray();

	float4 accum = float4::zero;
	for(int i = 0; i < N; ++i)
	{
		float4 res = m[i] * v[i];
		accum += res;
	}
	if (*accum.ptr() == 0) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", accum.ToString().c_str());
}

RANDOMIZED_TEST(sse_mat_mat_mul)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 res;
	_mm_mat4x4_mul_ps_dpps(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
#endif
	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

RANDOMIZED_TEST(sse_mat_mat_mul_2)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps_2(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 res;
	_mm_mat4x4_mul_ps_dpps_2(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
#endif
	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

RANDOMIZED_TEST(sse_mat_mat_mul_3)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_dpps_3(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 res;
	_mm_mat4x4_mul_ps_dpps_3(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
#endif
	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

RANDOMIZED_TEST(sse_mat_mat_mul_ps)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
	{
		float4x4 res;
		_mm_mat4x4_mul_ps(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 res;
	_mm_mat4x4_mul_ps(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
#endif
	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

RANDOMIZED_TEST(sse_mat_mat_mul_ps_2)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
	{
		float4x4 res;
		_mm_mat4x4_mul_ps_2(res.row, m[0].row, m[i].row);
		m2[i] = res;
	}
#ifdef TEST_SSE_CORRECTNESS
	float4x4 res;
	_mm_mat4x4_mul_ps_2(res.row, m[0].row, m[1].row);
	float4x4 res2 = m[0]*m[1];
	assert(res.Equals(res2));
#endif
	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

RANDOMIZED_TEST(sse_scalar_mat_mat_mul)
{
	float4x4 *m = MatrixArray();
	float4x4 *m2 = MatrixArray2();

	for(int i = 0; i < N; ++i)
		m2[i] = m[0] * m[i];

	if (*(char*)m2 == 0xFF && *((char*)m2+1) == 0xEE) // Random condition to confuse compiler not to do dead code elimination.
		LOGI("%s", m2[0].ToString().c_str());
}

#endif