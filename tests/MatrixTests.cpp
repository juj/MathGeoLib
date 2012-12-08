#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

extern LCG rng;

void TestFloat3x4ScaleRow()
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

void TestFloat3x4SetRow()
{
	float3x4 m;
	m.SetRow(0, 1,2,3,4);
	m.SetRow(1, float4(5,6,7,8));
	float v[4] = {9,10,11,12};
	m.SetRow(2, v);

	assert(m.Equals(float3x4(1,2,3,4, 5,6,7,8, 9,10,11,12)));
}

void TestFloat3x4SwapRows()
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

void TestFloat3x4TransformFloat4()
{
	Line l = RandomLineContainingPoint(float3::zero);
	l.pos = float3::zero;
	assert(l.dir.IsNormalized());
	float4 pt = float4(l.GetPoint(rng.Float(-3.f, 3.f)), 1.f);
	float3x4 rot = Quat::RandomRotation(rng).ToFloat3x4();
	float4 newDir = rot.Transform(float4(l.dir,0.f));
	assert(newDir.w == 0.f);
	l.dir = newDir.xyz();
	l.dir.Normalize();
	pt = rot.Transform(pt);
	assert(pt.w == 1.f);
	float d = l.Distance(pt.xyz());
	assert(EqualAbs(d, 0.f));
	assert(l.Contains(pt.xyz()));
}

void TestFloat3x4MulFloat3x4()
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

void TestFloat3x4MulScalar()
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


void TestFloat3x4DivScalar()
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

void TestFloat3x4AddFloat3x4()
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

void TestFloat3x4SubFloat3x4()
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

void TestFloat3x4Neg()
{
	float3x4 m = float3x4::RandomGeneral(rng, -10.f, 10.f);
	float3x4 m2 = -m;
	float3x4 m3;
	for(int y = 0; y < 3; ++y)
		for(int x = 0; x < 4; ++x)
			m3.At(y,x) = -m.At(y,x);

	assert(m2.Equals(m3));
}

void TestFloat3x3SolveAxb()
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

void TestFloat3x3Inverse()
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

void TestFloat3x3InverseFast()
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

void AddMatrixTests()
{
	AddTest("float3x4::ScaleRow", TestFloat3x4ScaleRow, false);
	AddTest("float3x4::SetRow", TestFloat3x4SetRow, false);
	AddTest("float3x4::SwapRows", TestFloat3x4SwapRows, false);
	AddTest("float3x4::Transform", TestFloat3x4TransformFloat4);
	AddTest("float3x4::operator*(float3x4)", TestFloat3x4MulFloat3x4);
	
	AddTest("float3x4::operator*(scalar)", TestFloat3x4MulScalar);
	AddTest("float3x4::operator/(scalar)", TestFloat3x4DivScalar);
	AddTest("float3x4::operator+(float3x4)", TestFloat3x4AddFloat3x4);
	AddTest("float3x4::operator-(float3x4)", TestFloat3x4SubFloat3x4);
	AddTest("float3x4::operator-()", TestFloat3x4Neg);

	AddTest("float3x3::SolveAxb", TestFloat3x3SolveAxb);
	AddTest("float3x3::Inverse", TestFloat3x3InverseFast);
	AddTest("float3x3::InverseFast", TestFloat3x3InverseFast);
	
}
