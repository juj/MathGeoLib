#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

TEST(Float3x4ScaleRow)
{
	float3x4 m2;
	m2.Set(-1,-1,-1,-1,
	      2,2,2,2,
	      4,4,4,4);

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
