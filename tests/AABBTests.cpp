#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"

MATH_IGNORE_UNUSED_VARS_WARNING

AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength);

MATH_BEGIN_NAMESPACE

using namespace TestData;

RANDOMIZED_TEST(AABBTransformAsAABB)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 trans = float3x4::Translate(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)));
	float3x4 scale = float3x4::UniformScale(rng.Float(0.001f, 100.f));
	float3x4 m = trans*rot*scale;
	OBB o = a.Transform(m);
	AABB x = a;
	x.TransformAsAABB(m);
	AABB y = o.MinimalEnclosingAABB();
	assert(x.Equals(y));
}

BENCHMARK(AABBTransformOBBToAABB_BM, "AABB::Transform to OBB and convert back to AABB")
{
	aabb[i] = aabb[i].Transform(om[i]).MinimalEnclosingAABB();
}
BENCHMARK_END

BENCHMARK(AABBTransformAsAABB_BM, "AABB::TransformAsAABB")
{
	aabb[i].TransformAsAABB(om[i]);
}
BENCHMARK_END

#ifdef MATH_SSE
RANDOMIZED_TEST(AABBTransformAsAABB_SIMD)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 trans = float3x4::Translate(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)));
	float3x4 scale = float3x4::UniformScale(rng.Float(0.001f, 100.f));
	float3x4 m = trans*rot*scale;
	OBB o = a.Transform(m);
	AABB x = a;
	AABBTransformAsAABB_SIMD(x, m);
	AABB y = o.MinimalEnclosingAABB();
	assert2(x.Equals(y, 1e-2f), x.SerializeToCodeString(), y.SerializeToCodeString());
}

BENCHMARK(AABBTransformAsAABB_SIMD_BM, "AABB::TransformAsAABB SIMD")
{
	AABBTransformAsAABB_SIMD(aabb[i], om[i]);
}
BENCHMARK_END
#endif

UNIQUE_TEST(AABBIsDegenerate)
{
	AABB a(vec::nan, vec::nan);
	assert(a.IsDegenerate());
	assert(!a.IsFinite());

	a = AABB(vec::zero, vec::nan);
	assert(a.IsDegenerate());
	assert(!a.IsFinite());

	a = AABB(vec::zero, vec::one);
	assert(!a.IsDegenerate());
	assert(a.IsFinite());

	a = AABB(vec::zero, vec::inf);
	assert(!a.IsDegenerate());
	assert(!a.IsFinite());

	a = AABB(vec::zero, vec::zero);
	assert(a.IsDegenerate());
	assert(a.IsFinite());

	a = AABB(vec::zero, -vec::zero);
	assert(a.IsDegenerate());
	assert(a.IsFinite());

	a = AABB(vec::zero, -vec::one);
	assert(a.IsDegenerate());
	assert(a.IsFinite());

	a = AABB(vec::zero, -vec::inf);
	assert(a.IsDegenerate());
	assert(!a.IsFinite());

	a = AABB(vec::inf, -vec::inf);
	assert(a.IsDegenerate());
	assert(!a.IsFinite());

	a = AABB(-vec::inf, vec::inf);
	assert(!a.IsDegenerate());
	assert(!a.IsFinite());
}

UNIQUE_TEST(OBBIsDegenerate)
{
	OBB o = AABB(vec::nan, vec::nan);
	assert(o.IsDegenerate());
	assert(!o.IsFinite());

	o = OBB(AABB(vec::zero, vec::nan));
	assert(o.IsDegenerate());
	assert(!o.IsFinite());

	o = OBB(AABB(vec::zero, vec::one));
	assert(!o.IsDegenerate());
	assert(o.IsFinite());

	o = OBB(AABB(vec::zero, vec::inf));
	assert(!o.IsDegenerate());
	assert(!o.IsFinite());

	o = OBB(AABB(vec::zero, vec::zero));
	assert(o.IsDegenerate());
	assert(o.IsFinite());

	o = OBB(AABB(vec::zero, -vec::zero));
	assert(o.IsDegenerate());
	assert(o.IsFinite());

	o = OBB(AABB(vec::zero, -vec::one));
	assert(o.IsDegenerate());
	assert(o.IsFinite());

	o = OBB(AABB(vec::zero, -vec::inf));
	assert(o.IsDegenerate());
	assert(!o.IsFinite());

	o = OBB(AABB(vec::inf, -vec::inf));
	assert(o.IsDegenerate());
	assert(!o.IsFinite());

	o = OBB(AABB(-vec::inf, vec::inf));
	assert(!o.IsDegenerate());
	assert(!o.IsFinite());
}

MATH_END_NAMESPACE
