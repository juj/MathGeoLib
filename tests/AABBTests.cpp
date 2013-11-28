#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

AABB RandomAABBContainingPoint(const float3 &pt, float maxSideLength);

RANDOMIZED_TEST(AABBTransformAsAABB)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 trans = float3x4::Translate(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)));
	float3x4 scale = float3x4::UniformScale(rng.Float(0.001f, 100.f));
	float3x4 m = trans*rot*scale;
	OBB o = a.Transform(m);
	AABB x = a;
	x.TransformAsAABB(m);
	AABB y = o.MinimalEnclosingAABB();
	assert(x.Equals(y));
}
