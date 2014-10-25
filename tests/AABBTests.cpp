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

BENCHMARK(AABBContains_positive, "AABB::Contains(point) positive")
{
	uf[i] = aabb[i].Contains(aabb[i].minPoint) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(AABBContains_negative, "AABB::Contains(point) negative")
{
	uf[i] = aabb[i].Contains(POINT_VEC_SCALAR(1e10f)) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(AABBContains_unpredictable, "AABB::Contains(point) unpredictable")
{
	uf[i] = aabb[i].Contains((i%2 == 0) ? POINT_VEC_SCALAR(1e10f) : aabb[i].minPoint) ? 1.f : 0.f;
}
BENCHMARK_END

UNIQUE_TEST(AABBContains)
{
	AABB a(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10));
	assert(a.Contains(POINT_VEC(0,0,0)));
	assert(a.Contains(POINT_VEC(10,10,10)));
	assert(a.Contains(POINT_VEC(1,2,3)));
	assert(!a.Contains(POINT_VEC(-1,2,3)));
	assert(!a.Contains(POINT_VEC(1,-2,3)));
	assert(!a.Contains(POINT_VEC(1,2,-3)));
	assert(!a.Contains(POINT_VEC(11,2,3)));
	assert(!a.Contains(POINT_VEC(1,12,3)));
	assert(!a.Contains(POINT_VEC(1,2,13)));
	assert(!a.Contains(POINT_VEC(-1,-2,-3)));
	assert(!a.Contains(POINT_VEC(11,12,13)));
}

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

#ifdef MATH_SIMD
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
