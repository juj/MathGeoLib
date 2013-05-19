#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/SSEMath.h"
#include "../src/Math/float4_sse.h"
#include "../src/Math/float4x4_sse.h"
#include "../src/Math/float4x4_neon.h"

using namespace TestData;

TEST(Float3x4ScaleRow)
{
	float3x4 m2;
	m2.Set(-1,-1,-1,-1,
	        2, 2, 2, 2,
	        4, 4, 4, 4);

	float3x4 m = float3x4::nan;
	m.Set(m2); // Test float3x4::Set() to properly copy data.

	m.ScaleRow(0, -8.f);
	m.ScaleRow(1, 4.f);
	m.ScaleRow(2, 2.f);
	m -= float3x4(8,8,8,8,8,8,8,8,8,8,8,8);
	assert(m.IsSymmetric());
	assert(m.IsSkewSymmetric());
	assert((m+float3x4::identity).IsIdentity());
}

TEST(Float3x4SetRow)
{
	float3x4 m;
	m.SetRow(0, 1,2,3,4);
	m.SetRow(1, float4(5,6,7,8));
	float v[4] = {9,10,11,12};
	m.SetRow(2, v);

	assert(m.Equals(float3x4(1,2,3,4, 5,6,7,8, 9,10,11,12)));
}

TEST(Float3x4SwapRows)
{
	float3x4 v;
	v.SetIdentity();
	v.SwapRows(0,1);
	v.SwapRows(1,2);
	v.SwapRows(1,0);

	float3x4 v3 = v;
	float3x4 v4 = float3x4::nan;

	float3x4 v2;
	v2.SetRotatePart(float3x3(0,0,1, 0,1,0, 1,0,0));
	v2.SetTranslatePart(0,0,0);
	assert(v.Equals(v2));

	v4 = v2;
	assert(v3.Equals(v4));
}

Line RandomLineContainingPoint(const float3 &pt);

RANDOMIZED_TEST(Float3x4TransformFloat4)
{
	Line l = RandomLineContainingPoint(float3::zero);
	l.pos = float3::zero;
	assert(l.dir.IsNormalized());
	float4 pt = float4(l.GetPoint(rng.Float(-3.f, 3.f)), 1.f);
	float3x4 rot = Quat::RandomRotation(rng).ToFloat3x4();
	float4 newDir = rot.Transform(float4(l.dir,0.f));
	assert(newDir.w == 0.f);
	l.dir = newDir.xyz();
	assert(l.dir.IsNormalized(1e-1f));
	l.dir.Normalize();
	pt = rot.Transform(pt);
	assert(pt.w == 1.f);
	float d = l.Distance(pt.xyz());
	assert(EqualAbs(d, 0.f));
	assert(l.Contains(pt.xyz()));
}

RANDOMIZED_TEST(Float3x4TransformPosDir)
{
	Line l = RandomLineContainingPoint(float3::zero);
	l.pos = float3::zero;
	assert(l.dir.IsNormalized());
	float3 pt = l.GetPoint(rng.Float(-3.f, 3.f));
	float3x4 rot = Quat::RandomRotation(rng).ToFloat3x4();
	l.dir = rot.TransformDir(l.dir);
	assert(l.dir.IsNormalized(1e-1f));
	l.dir.Normalize();
	pt = rot.TransformPos(pt);
	float d = l.Distance(pt.xyz());
	assert(EqualAbs(d, 0.f));
	assert(l.Contains(pt.xyz()));
}

RANDOMIZED_TEST(Float3x4TransformPosDirXyz)
{
	Line l = RandomLineContainingPoint(float3::zero);
	l.pos = float3::zero;
	assert(l.dir.IsNormalized());
	float3 pt = l.GetPoint(rng.Float(-3.f, 3.f));
	float3x4 rot = Quat::RandomRotation(rng).ToFloat3x4();
	l.dir = rot.TransformDir(l.dir.x, l.dir.y, l.dir.z);
	assert(l.dir.IsNormalized(1e-1f));
	l.dir.Normalize();
	pt = rot.TransformPos(pt.x, pt.y, pt.z);
	float d = l.Distance(pt.xyz());
	assert(EqualAbs(d, 0.f));
	assert(l.Contains(pt.xyz()));
}

RANDOMIZED_TEST(Float3x4MulFloat3x4)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = float3x4::RandomGeneral(rng, -10.f, 10.f);

	float3x4 m3 = m * m2;
	float3x4 m4;
	for(int i = 0; i < 4; ++i)
	{
		float4 v = float4(m2.Col(i), i < 3 ? 0.f : 1.f);
		v = m * v;
		m4.SetCol(i, v.xyz());
	}
	assert(m3.Equals(m4));
}

RANDOMIZED_TEST(Float3x4MulScalar)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float scalar = rng.Float(-10.f, 10.f);
	float3x4 m2 = m * scalar;
	float3x4 m3 = m;
	m3 *= scalar;
	float3x4 m4;
	for(int i = 0; i < 12; ++i)
		m4.ptr()[i] = m.ptr()[i] * scalar;

	assert(m2.Equals(m4));
	assert(m3.Equals(m4));
}


RANDOMIZED_TEST(Float3x4DivScalar)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float scalar = rng.Float(-10.f, -2.f);
	float3x4 m2 = m / scalar;
	float3x4 m3 = m;
	m3 /= scalar;
	float3x4 m4;
	for(int i = 0; i < 12; ++i)
		m4.ptr()[i] = m.ptr()[i] / scalar;

	assert(m2.Equals(m4));
	assert(m3.Equals(m4));
}

RANDOMIZED_TEST(Float3x4AddFloat3x4)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m3 = m + m2;
	float3x4 m5 = m;
	m5 += m2;
	float3x4 m4;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			m4.At(y,x) = m.At(y,x) + m2.At(y,x);

	assert(m3.Equals(m4));
	assert(m5.Equals(m4));
}

TEST(Float3x3AddUnary)
{
	float3x3 m(1,2,3, 4,5,6, 7,8,9);
	float3x3 m2 = +m;
	assert(m.Equals(m2));
}

TEST(Float3x4AddUnary)
{
	float3x4 m(1,2,3,4, 5,6,7,8, 9,10,11,12);
	float3x4 m2 = +m;
	assert(m.Equals(m2));
}

TEST(Float4x4AddUnary)
{
	float4x4 m(1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16);
	float4x4 m2 = +m;
	assert(m.Equals(m2));
}

///\todo Create and move to QuatTests.cpp
TEST(QuatAddUnary)
{
	Quat q(1,2,3,4);
	Quat q2 = +q;
	assert(q.Equals(q2));
}

RANDOMIZED_TEST(Float3x4SubFloat3x4)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m3 = m - m2;
	float3x4 m4;
	float3x4 m5 = m;
	m5 -= m2;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			m4.At(y,x) = m.At(y,x) - m2.At(y,x);

	assert(m3.Equals(m4));
	assert(m5.Equals(m4));
}

RANDOMIZED_TEST(Float3x4Neg)
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = -m;
	float3x4 m3;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			m3.At(y,x) = -m.At(y,x);

	assert(m2.Equals(m3));
}

RANDOMIZED_TEST(Float3x3SolveAxb)
{
	float3x3 A = float3x3::RandomGeneral(rng, -10.f, 10.f);
	bool mayFail = EqualAbs(A.Determinant(), 0.f, 1e-2f);

	float3 b = float3::RandomBox(rng, float3::FromScalar(-10.f), float3::FromScalar(10.f));

	float3 x;
	bool success = A.SolveAxb(b, x);
	assert(success || mayFail);
	if (success)
	{
		float3 b2 = A*x;
		assert(b2.Equals(b, 1e-1f));
	}
}

RANDOMIZED_TEST(Float3x3Inverse)
{
	float3x3 A = float3x3::RandomGeneral(rng, -10.f, 10.f);
	bool mayFail = EqualAbs(A.Determinant(), 0.f, 1e-2f);

	float3x3 A2 = A;
	bool success = A2.Inverse();
	assert(success || mayFail);
	if (success)
	{
		float3x3 id = A * A2;
		float3x3 id2 = A2 * A;
		assert(id.Equals(float3x3::identity, 0.3f));
		assert(id2.Equals(float3x3::identity, 0.3f));
	}
}

RANDOMIZED_TEST(Float3x3InverseFast)
{
	float3x3 A = float3x3::RandomGeneral(rng, -10.f, 10.f);
	bool mayFail = EqualAbs(A.Determinant(), 0.f, 1e-2f);

	float3x3 A2 = A;
	bool success = A2.InverseFast();
	assert(success || mayFail);
	if (success)
	{
		float3x3 id = A * A2;
		float3x3 id2 = A2 * A;
		assert(id.Equals(float3x3::identity, 0.3f));
		assert(id2.Equals(float3x3::identity, 0.3f));
	}
}

RANDOMIZED_TEST(Float4x4Ctor)
{
	float3x3 m = float3x3::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2(m);
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 3; ++x)
			assert(EqualAbs(m.At(y,x), m2.At(y,x)));
	assert(EqualAbs(m2[0][3], 0.f));
	assert(EqualAbs(m2[1][3], 0.f));
	assert(EqualAbs(m2[2][3], 0.f));

	assert(EqualAbs(m2[3][0], 0.f));
	assert(EqualAbs(m2[3][1], 0.f));
	assert(EqualAbs(m2[3][2], 0.f));
	assert(EqualAbs(m2[3][3], 1.f));

	float3x4 m3 = float3x4::RandomGeneral(rng, -10.f, 10.f);
	m2 = float4x4(m3);
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			assert(EqualAbs(m3.At(y,x), m2.At(y,x)));
	assert(EqualAbs(m2[3][0], 0.f));
	assert(EqualAbs(m2[3][1], 0.f));
	assert(EqualAbs(m2[3][2], 0.f));
	assert(EqualAbs(m2[3][3], 1.f));
}

TEST(Float4x4SetRow)
{
	float4x4 m;
	m.SetRow(0, 1,2,3,4);
	m.SetRow(1, float4(5,6,7,8));
	m.SetRow(2, 9,10,11,12);
	m.SetRow(3, 13,14,15,16);

	float4x4 m3(1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16);
	float4x4 m2;
	m2.Set(1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16);
	assert(m.Equals(m2));
	assert(m.Equals(m3));
}

RANDOMIZED_TEST(Float4x4Set3x4Part)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = m;
	float4x4 m4;
	m4 = m2;
	assert(m4.Equals(m2));
	float3x4 m3 = float3x4::RandomGeneral(rng, -10.f, 10.f);
	m2.Set3x4Part(m3);
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			assert(EqualAbs(m2[y][x], m3.At(y,x)));

	assert(EqualAbs(m2[3][0], m[3][0]));
	assert(EqualAbs(m2[3][1], m[3][1]));
	assert(EqualAbs(m2[3][2], m[3][2]));
	assert(EqualAbs(m2[3][3], m[3][3]));
}

TEST(Float4x4SwapRows)
{
	float4x4 m(1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16);
	float4x4 m2(13,14,15,16, 9,10,11,12, 5,6,7,8, 1,2,3,4);
	m.SwapRows(0,3);
	m.SwapRows(1,2);
	assert(m.Equals(m2));
}

RANDOMIZED_TEST(Float4x4AssignFloat3x4)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = float3x4::RandomGeneral(rng, -10.f, 10.f);

	m = m2;

	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			assert(EqualAbs(m[y][x], m2.At(y,x)));

	assert(EqualAbs(m[3][0], 0.f));
	assert(EqualAbs(m[3][1], 0.f));
	assert(EqualAbs(m[3][2], 0.f));
	assert(EqualAbs(m[3][3], 1.f));

}

TEST(Float4x4CtorCols)
{
	float4x4 m(float4(1,2,3,4), float4(5,6,7,8), float4(9,10,11,12), float4(13,14,15,16));
	float4x4 m2(1,5,9,13, 2,6,10,14, 3,7,11,15, 4,8,12,16);
	assert(m.Equals(m2));
}

RANDOMIZED_TEST(Float4x4CtorFromQuat)
{
	Quat q = Quat::RandomRotation(rng);
	float4x4 m(q);

	float3 v = float3(-1, 5, 20.f);
	float3 v1 = q * v;
	float3 v2 = m.TransformPos(v);
	assert(v1.Equals(v2));
}

RANDOMIZED_TEST(Float4x4CtorFromQuatTrans)
{
	float3 t = float3::RandomBox(rng, float3(-SCALE, -SCALE, -SCALE), float3(SCALE, SCALE, SCALE));
	Quat q = Quat::RandomRotation(rng);
	float4x4 m(q, t);

	float3 v = float3(-1, 5, 20.f);
	float3 v1 = q * v + t;
	float3 v2 = m.TransformPos(v);
	assert(v1.Equals(v2));
}

RANDOMIZED_TEST(Float4x4Translate)
{
	float3 t = float3::RandomBox(rng, float3(-SCALE, -SCALE, -SCALE), float3(SCALE, SCALE, SCALE));
	float3 t2 = float3::RandomBox(rng, float3(-SCALE, -SCALE, -SCALE), float3(SCALE, SCALE, SCALE));
	float4x4 m = float4x4::Translate(t);
	float4x4 m2 = float4x4::Translate(t.x, t.y, t.z);

	float3 v = t + t2;
	float3 v1 = m.TransformPos(t2);
	float3 v2 = m2.TransformPos(t2);
	assert(v1.Equals(v2));
	assert(v.Equals(v1));
}

TEST(Float4x4Scale)
{
	float4x4 m = float4x4::Scale(2,4,6);
	float4x4 m2(2,0,0,0, 0,4,0,0, 0,0,6,0, 0,0,0,1);
	assert(m.Equals(m2));
}

BENCHMARK(Float3x4Inverse, "float3x4::Inverse")
{
	m[i].Float3x4Part().Inverse();
}
BENCHMARK_END;

BENCHMARK(Float4x4Inverse, "float4x4::Inverse")
{
	m[i].Inverse();
}
BENCHMARK_END;

#ifdef MATH_SSE
BENCHMARK(inverse_ps, "test against Float4x4Inverse")
{
	mat4x4_inverse(m[i].row, m[i].row);
}
BENCHMARK_END;
#endif

BENCHMARK(float4x4_InverseOrthogonalUniformScale, "float4x4::InverseOrthogonalUniformScale")
{
	m[i] = ogm[i];
	m[i].InverseOrthogonalUniformScale();
}
BENCHMARK_END;

BENCHMARK(float4x4_InverseOrthonormal, "float4x4::InverseOrthonormal")
{
	m[i] = om[i];
	m[i].InverseOrthonormal();
}
BENCHMARK_END;

BENCHMARK(float3x4_InverseOrthonormal, "float3x4::InverseOrthonormal")
{
	m[i].Float3x4Part() = om[i].Float3x4Part();
	m[i].Float3x4Part().InverseOrthonormal();
}
BENCHMARK_END;

#ifdef MATH_SSE
RANDOMIZED_TEST(mat_inverse_orthonormal_correctness)
{
	float4x4 m = float4x4(Quat::RandomRotation(rng), float3::RandomDir(rng));
	float4x4 m2 = m;
	float4x4 m3 = m;
	m2.InverseOrthonormal();
	mat3x4_inverse_orthonormal(m3.row, m3.row);
	assert(m2.Equals(m3));
}

BENCHMARK(mat3x4_inverse_orthonormal, "test against float3x4_InverseOrthonormal")
{
	mat3x4_inverse_orthonormal(m[i].row, m[i].row);
}
BENCHMARK_END;
#endif

float RelError(const float4x4 &m1, const float4x4 &m2)
{
	float relError = 0.f;
	for(int y = 0; y < 4; ++y)
		for(int x = 0; x < 4; ++x)
			relError = Max(relError, m1[y][x], m2[y][x]);
	return relError;
}

float AbsError(const float4x4 &m1, const float4x4 &m2)
{
	float absError = 0.f;
	for(int y = 0; y < 4; ++y)
		for(int x = 0; x < 4; ++x)
			absError = Max(absError, Abs(m1[y][x] - m2[y][x]));
	return absError;
}

#ifdef MATH_SSE

UNIQUE_TEST(mat_inverse_correctness)
{
	float maxRelError = 0.f;
	float maxAbsError = 0.f;

	for(int k = 0; k < 2; ++k)
	{
		for(int i = 0; i < 10000; ++i)
		{
			float4x4 m;
			if (k == 0)
				m = float3x4::RandomRotation(rng);
			else
				m = float4x4::RandomGeneral(rng, -1.f, 1.f);

			if (m.IsInvertible())
			{
				float4x4 m2 = m.Inverted();
				float4x4 m3 = m;
				mat4x4_inverse(m3.row, m3.row);
				maxRelError = Max(maxRelError, RelError(m2, m3));
				maxAbsError = Max(maxAbsError, AbsError(m2, m3));
			}
		}
		if (k == 0)
			LOGI("mat4x4_inverse max. relative error with rotation matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
		else
			LOGI("mat4x4_inverse max. relative error with general [0,1] matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
	}
}

UNIQUE_TEST(mat_determinant_correctness)
{
	float maxRelError = 0.f;
	float maxAbsError = 0.f;

	for(int k = 0; k < 2; ++k)
	{
		for(int i = 0; i < 10000; ++i)
		{
			float4x4 m;
			if (k == 0)
				m = float3x4::RandomRotation(rng);
			else
				m = float4x4::RandomGeneral(rng, -1.f, 1.f);

			float d = m.Determinant4();
			float d2 = mat4x4_determinant(m.row);
			maxRelError = Max(maxRelError, RelativeError(d, d2));
			maxAbsError = Max(maxAbsError, Abs(d - d2));
		}
		if (k == 0)
			LOGI("mat4x4_determinant max. relative error with rotation matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
		else
			LOGI("mat4x4_determinant max. relative error with general [0,1] matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
	}
}

UNIQUE_TEST(mat_determinant3_correctness)
{
	float maxRelError = 0.f;
	float maxAbsError = 0.f;

	for(int k = 0; k < 2; ++k)
	{
		for(int i = 0; i < 10000; ++i)
		{
			float4x4 m;
			if (k == 0)
				m = float3x4::RandomRotation(rng);
			else
				m = float4x4::RandomGeneral(rng, -1.f, 1.f);

			float d = m.Determinant3();
			float d2 = mat3x4_determinant(m.row);
			maxRelError = Max(maxRelError, RelativeError(d, d2));
			maxAbsError = Max(maxAbsError, Abs(d - d2));
		}
		if (k == 0)
			LOGI("mat3x4_determinant max. relative error with rotation matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
		else
			LOGI("mat3x4_determinant max. relative error with general [0,1] matrices: %f, Max abs error: %f", maxRelError, maxAbsError);
	}
}

#endif

BENCHMARK(float4x4_Determinant3, "float4x4::Determinant3")
{
	f[i] = m[i].Determinant3();
}
BENCHMARK_END;

BENCHMARK(float4x4_Determinant4, "float4x4::Determinant4")
{
	f[i] = m[i].Determinant4();
}
BENCHMARK_END;

UNIQUE_TEST(float4x4_Determinant_Correctness)
{
	float4x4 m(1,0,0,0, 2,2,0,0, 3,3,3,0, 4,4,4,4);
	asserteq(m.Determinant3(), 6.f);
	asserteq(m.Determinant4(), 24.f);
	asserteq(m.Float3x3Part().Determinant(), 6.f);
	asserteq(m.Float3x4Part().Determinant(), 6.f);
}

#ifdef MATH_SSE
BENCHMARK(mat4x4_determinant, "test against float4x4_Determinant4")
{
	f[i] = mat4x4_determinant(m[i].row);
}
BENCHMARK_END;

BENCHMARK(mat3x4_determinant, "test against float3x4_Determinant")
{
	f[i] = mat3x4_determinant(m[i].row);
}
BENCHMARK_END;
#endif

BENCHMARK(float4x4_op_mul, "float4x4 * float4x4")
{
	m2[i] = m[i] * m2[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(mat4x4_mul_mat4x4, "test against float4x4_op_mul")
{
	mat4x4_mul_mat4x4(m2[i].row, m[i].row, m2[i].row);
}
BENCHMARK_END;

#ifdef ANDROID
BENCHMARK(mat4x4_mul_mat4x4_asm, "test against float4x4_op_mul")
{
	mat4x4_mul_mat4x4_asm(m2[i].row, m[i].row, m2[i].row);
}
BENCHMARK_END;
#endif

RANDOMIZED_TEST(mat4x4_mul_mat4x4)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m3;

	mat4x4_mul_mat4x4(m3.row, m.row, m2.row);
	float4x4 correct = m*m2;
	assert(m3.Equals(correct));
}

#ifdef ANDROID
RANDOMIZED_TEST(mat4x4_mul_mat4x4_asm)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m2 = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 m3;

	mat4x4_mul_mat4x4_asm(m3.row, m.row, m2.row);
	float4x4 correct = m*m2;
	assert(m3.Equals(correct));
}
#endif

BENCHMARK(float4x4_Transposed, "float4x4::Transposed")
{
	m2[i] = m[i].Transposed();
}
BENCHMARK_END;

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
BENCHMARK(mat4x4_transpose, "test against float4x4_Transposed")
{
	mat4x4_transpose(m2[i].row, m[i].row);
}
BENCHMARK_END;
#endif

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
RANDOMIZED_TEST(mat4x4_transpose)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4x4 correct = m.Transposed();
	float4x4 m2;
	mat4x4_transpose(m2.row, m.row);
	mat4x4_transpose(m.row, m.row);
	assert(m.Equals(correct));
	assert(m2.Equals(correct));
}
#endif

BENCHMARK(float4x4_mul_float4, "float4x4 * float4")
{
	v2[i] = m[i] * v[i];
}
BENCHMARK_END;

BENCHMARK(float4_mul_float4x4, "float4 * float4x4")
{
	v2[i] = v[i] * m[i];
}
BENCHMARK_END;

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
BENCHMARK(mat4x4_mul_vec4, "test against float4x4_mul_float4")
{
	v2[i] = mat4x4_mul_vec4(m[i].row, v[i].v);
}
BENCHMARK_END;
#endif

BENCHMARK(vec4_mul_mat4x4, "test against float4_mul_float4x4")
{
	v2[i] = vec4_mul_mat4x4(v[i].v, m[i].row);
}
BENCHMARK_END;

#if !defined(ANDROID) ///\bug Android GCC 4.6.6 gives internal compiler error!
RANDOMIZED_TEST(mat4x4_mul_vec4)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 correct = m*v;
	float4 v2 = mat4x4_mul_vec4(m.row, v.v);
	assert(v2.Equals(correct));
}
#endif

RANDOMIZED_TEST(vec4_mul_mat4x4)
{
	float4x4 m = float4x4::RandomGeneral(rng, -10.f, 10.f);
	float4 v = float4::RandomGeneral(rng, -10.f, 10.f);
	float4 correct = v*m;
	float4 v2 = vec4_mul_mat4x4(v.v, m.row);
	assert(v2.Equals(correct));
}
#endif
