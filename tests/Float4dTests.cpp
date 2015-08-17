#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/float4d.h"
#include "../src/Math/float4_sse.h"
#include "../src/Math/float4_neon.h"

using namespace TestData;

MATH_IGNORE_UNUSED_VARS_WARNING

TEST(Float4dDot)
{
	float4d f(-1.f, 2.f, 3.f, -4.f);
	float4d f2(2.f, -1.f, 0.f, 4.f);
	assert(EqualAbs((float)f.Dot(f2), -20.f));
}

TEST(Float4dOpAdd)
{
	float4d f = float4d(1,2,3,4);
	float4d f2 = float4d(-5.f, -6.f, -7.f, -8.f);
	float4d f3 = f + f2;
	assert(f3.Equals(float4d(-4.f, -4.f, -4.f, -4.f)));
}

TEST(Float4dOpSub)
{
	float4d f = float4d(1,2,3,4);
	float4d f2 = float4d(-5.f, -6.f, -7.f, -8.f);
	float4d f3 = f - f2;
	assert(f3.Equals(float4d(6.f, 8.f, 10.f, 12.f)));
}

TEST(Float4dOpMul)
{
	float4d f = float4d(1,2,3,4);
	float scalar = -2.f;

	float4d f2 = scalar * f;
	
	assert(f2.Equals(float4d(-2.f, -4.f, -6.f, -8.f)));
}

TEST(Float4dCross)
{
	float4d f(-1.f, 2.f, 3.f, -4.f);
	float4d f2(2.f, -1.f, 0.f, 4.f);

	float4d f3 = f.Cross(f2);
	assert(f3.Equals(float4d(3.f, 6.f, -3.f, 0.f)));

	float4d z = float4d::unitX.Cross(float4d::unitY);
	float4d y = float4d::unitZ.Cross(float4d::unitX);
	float4d x = float4d::unitY.Cross(float4d::unitZ);
	assert(x.Equals(float4d(1,0,0,0)));
	assert(y.Equals(float4d(0,1,0,0)));
	assert(z.Equals(float4d(0,0,1,0)));
}

RANDOMIZED_TEST(Float4dCross_random)
{
	rng = LCG(1);
	float4 f1 = float4::RandomDir(rng, 10.f);
	float4 f2 = float4::RandomDir(rng, 10.f);
	float4d d1 = f1;
	float4d d2 = f2;

	float4d d3 = d1.Cross(d2);
	float4 f3 = f1.Cross(f2);
	assert(float4d(f3).Equals(d3));
}

TEST(Float4dDistance4Sq)
{
	float4d f(1.f, 2.f, 3.f, 4.f);
	float4d f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs((float)f.Distance4Sq(f2), 120.f));
}

TEST(Float4dDistance3Sq)
{
	float4d f(1.f, 2.f, 3.f, 4.f);
	float4d f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs((float)f.Distance3Sq(f2), 56.f));
}

TEST(Float4dNormalize4)
{
	float4d f(-1.f, 2.f, 3.f, 4.f);
	double oldLength = f.Normalize();
	assert2(EqualAbs((float)oldLength, Sqrt(30.f)), (float)oldLength, Sqrt(30.f));
	assert2(EqualAbs((float)f.x, -1.f / Sqrt(30.f)), (float)f.x, -1.f / Sqrt(30.f));
	assert2(EqualAbs((float)f.y, 2.f / Sqrt(30.f)), (float)f.y, 2.f / Sqrt(30.f));
	assert2(EqualAbs((float)f.z, 3.f / Sqrt(30.f)), (float)f.z, 3.f / Sqrt(30.f));
	assert2(EqualAbs((float)f.w, 4.f / Sqrt(30.f)), (float)f.w, 4.f / Sqrt(30.f));

	float4d f2(0,0,0, 0.f);
	oldLength = f2.Normalize();
	MARK_UNUSED(oldLength);
	assert(oldLength == 0.f);

  assert(EqualAbs((float)float4d(-1.f, 2.f, 3.f, 4.f).Normalize(), (float)float4d(-1.f, 2.f, 3.f, 4.f).Normalize4()));
}

TEST(Float4dNormalize3)
{
	float4d f(-1.f, 2.f, 3.f, 4.f);
	double oldLength = f.Normalize3();
	assert2(EqualAbs((float)oldLength, Sqrt(14.f)), (float)oldLength, Sqrt(14.f));
	assert2(EqualAbs((float)f.x, -1.f / Sqrt(14.f)), (float)f.x, -1.f / Sqrt(14.f));
	assert2(EqualAbs((float)f.y, 2.f / Sqrt(14.f)), (float)f.y, 2.f / Sqrt(14.f));
	assert2(EqualAbs((float)f.z, 3.f / Sqrt(14.f)), (float)f.z, 3.f / Sqrt(14.f));
	assert2(EqualAbs((float)f.w, 4.f), (float)f.w, 4.f);

	float4d f2(0,0,0, 0.f);
	oldLength = f2.Normalize3();
	MARK_UNUSED(oldLength);
	assert(oldLength == 0.f);
}

TEST(Float4dOpNeg)
{
	float4d f = float4d(1,2,3,4);
	float4d f3 = -f;
	assert(f3.Equals(float4d(-1.f, -2.f, -3.f, -4.f)));
}

TEST(Float4dToFloat4)
{
	float4d d = float4d(1,2,3,4);
	float4 f = d.ToFloat4();
	assert1(f.x == 1.f, f.x);
	assert1(f.y == 2.f, f.y);
	assert1(f.z == 3.f, f.z);
	assert1(f.w == 4.f, f.w);
}
