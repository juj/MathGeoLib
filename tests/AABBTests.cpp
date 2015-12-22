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
	uf[i] = aabb[i].Contains((i%2 == 0) ? vec(POINT_VEC_SCALAR(1e10f)) : aabb[i].minPoint) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(AABBContainsAABB_positive, "AABB::Contains(AABB) positive")
{
	dummyResultInt += aabb[i].Contains(aabb[i]) ? 1 : 0;
}
BENCHMARK_END

BENCHMARK(AABBContainsAABB_random, "AABB::Contains(AABB) random")
{
	dummyResultInt += aabb[i].Contains(aabb[i+1]) ? 1 : 0;
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

UNIQUE_TEST(AABBContainsAABB)
{
	AABB a(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10));
	assert(a.Contains(a));
	assert(a.Contains(AABB(POINT_VEC(5,5,5), POINT_VEC(6,6,6))));
	assert(a.Contains(AABB(POINT_VEC(5,5,5), POINT_VEC(10,6,6))));
	assert(!a.Contains(AABB(POINT_VEC(5,5,5), POINT_VEC(15,15,15))));
	assert(!a.Contains(AABB(POINT_VEC(5,5,5), POINT_VEC(5,15,5))));
	assert(!a.Contains(AABB(POINT_VEC(-5,-5,-5), POINT_VEC(5,5,5))));
	assert(!a.Contains(AABB(POINT_VEC(-5,-5,-5), POINT_VEC(0,0,0))));
}

UNIQUE_TEST(AABBContainsLineSegment)
{
	AABB a(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10));
	assert(a.Contains(LineSegment(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10))));
	assert(a.Contains(LineSegment(POINT_VEC(10,10,10), POINT_VEC(0, 0, 0))));
	assert(a.Contains(LineSegment(POINT_VEC(5,5,5), POINT_VEC(6,6,6))));
	assert(a.Contains(LineSegment(POINT_VEC(5,5,5), POINT_VEC(10,6,6))));
	assert(!a.Contains(LineSegment(POINT_VEC(5,5,5), POINT_VEC(15,15,15))));
	assert(!a.Contains(LineSegment(POINT_VEC(5,5,5), POINT_VEC(5,15,5))));
	assert(!a.Contains(LineSegment(POINT_VEC(-5,-5,-5), POINT_VEC(5,5,5))));
	assert(!a.Contains(LineSegment(POINT_VEC(-5,-5,-5), POINT_VEC(0,0,0))));
	assert(!a.Contains(LineSegment(POINT_VEC(15,15,15), POINT_VEC(-15,-15,-15))));
}

UNIQUE_TEST(AABBContainsSphere)
{
	AABB a(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10));
	assert(a.Contains(Sphere(POINT_VEC(0,0,0), 0.f)));
	assert(a.Contains(Sphere(POINT_VEC(5,5,5), 1.f)));
	assert(!a.Contains(Sphere(POINT_VEC(5,5,5), 15.f)));
	assert(!a.Contains(Sphere(POINT_VEC(9,5,5), 2.f)));
	assert(!a.Contains(Sphere(POINT_VEC(1,5,5), 2.f)));
	assert(!a.Contains(Sphere(POINT_VEC(-10,-10,-10), 1000.f)));
}

UNIQUE_TEST(AABBIntersectsAABB)
{
	AABB a(POINT_VEC(0,0,0), POINT_VEC(10, 10, 10));
	AABB b(POINT_VEC(5,0,0), POINT_VEC(15, 10, 10));
	AABB c(POINT_VEC(-5,-5,-5), POINT_VEC(0, 10, 0));
	AABB d(POINT_VEC(20,20,20), POINT_VEC(30, 30, 30));
	AABB e(POINT_VEC(1,1,1), POINT_VEC(9,9,9));
	assert(a.Intersects(a));
	assert(a.Intersects(b));
	assert(!a.Intersects(c));
	assert(!a.Intersects(d));
	assert(a.Intersects(e));
}

BENCHMARK(AABBIntersectsAABB_positive, "AABB::Intersects(AABB) positive")
{
	uf[i] = aabb[i].Intersects(aabb[i]) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(AABBIntersectsAABB_random, "AABB::Intersects(AABB) random")
{
	uf[i] = aabb[i].Intersects(aabb[i+1]) ? 1.f : 0.f;
}
BENCHMARK_END

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

UNIQUE_TEST(AABB_Volume)
{
	AABB a(POINT_VEC_SCALAR(-1.f), POINT_VEC_SCALAR(1.f));
	assert(a.Volume() == 8.f);
}

MATH_END_NAMESPACE
