#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"

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
