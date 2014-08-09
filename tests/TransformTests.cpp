#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"

MATH_IGNORE_UNUSED_VARS_WARNING

TEST(TranslateOp)
{
	float3x4 tm = float3x4::Translate(1,2,3);
	float3x4 tm2;
	tm2 = float3x4::Translate(1,2,3);
	assert(tm.Equals(tm2));
	assert(tm.TranslatePart().Equals(1,2,3));
	assert(tm.Float3x3Part().IsIdentity());

	float4x4 tm3 = float4x4::Translate(1,2,3);
	float4x4 tm4;
	tm4 = float4x4::Translate(1,2,3);
	assert(tm4.Equals(tm3));
	assert(tm3.TranslatePart().Equals(1,2,3));
	assert(tm3.Float3x3Part().IsIdentity());
	assert(tm3.Row(3).Equals(0,0,0,1));
}

RANDOMIZED_TEST(TranslateOpMul)
{
	TranslateOp t = float3x4::Translate(float3::RandomBox(rng, 0.f, 100.f));
	float4x4 tm = t.ToFloat4x4();
	assert(tm.TranslatePart().Equals(t.offset.xyz()));
	float4x4 m = float4x4::RandomGeneral(rng, -100.f, 100.f);
	m.SetRow(3, 0.f, 0.f, 0.f, 1.f);

	float4x4 m1 = t * m;
	float4x4 m2 = tm * m;
	assert2(m1.Equals(m2), m1, m2);

	m1 = m * t;
	m2 = m * tm;
	assert2(m1.Equals(m2), m1, m2);
}
