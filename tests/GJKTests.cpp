#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "ObjectGenerators.h"

MATH_IGNORE_UNUSED_VARS_WARNING

UNIQUE_TEST(TrickyAABBCapsuleNoIntersect)
{
	Capsule a(POINT_VEC(-37.3521881f,-61.0987396f,77.0996475f), POINT_VEC(46.2122498f,-61.2913399f,15.9034805f) ,53.990406f);
	AABB b(POINT_VEC(51.6529007f,-28.0629959f,65.9745636f),POINT_VEC(59.8043518f,-18.9434891f,69.0897827f));
	for(int i = 0; i < 8; ++i)
		LOGI("Distance: %f", a.Distance(b.CornerPoint(i)));
	LOGI("D: %f", b.Distance(a.Centroid()));
	LOGI("D: %f", b.Distance(a.SphereA()));
	LOGI("D: %f", b.Distance(a.SphereB()));
	assert(!a.Intersects(b));
}

UNIQUE_TEST(TrickyGJKSphereSphereIntersect)
{
	Sphere a = Sphere(POINT_VEC(-14.740263f, 8.2647991f, 64.282227f), 7.6029987f);
	Sphere b = Sphere(POINT_VEC(-23.840866f, 9.4233770f, 66.399742f), 1.9519407f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(TrickyGJKSphereSphereIntersect2)
{
	Sphere a = Sphere(POINT_VEC(57.166256f, 99.426201f, 75.735786f), 2.3808355f);
	Sphere b = Sphere(POINT_VEC(54.087727f, 94.719139f, 75.188812f), 3.3114955f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}
/*
UNIQUE_TEST(TrickyGJKCapsuleTriangleNoIntersect)
{
	Capsule c;
	c.l.a = POINT_VEC(-96.444878f, 45.877380f, 29.534300f);
	c.l.b = POINT_VEC(-48.194218f, 47.746323f, 124.26303f);
	c.r = 29.480530f;
	Triangle t;
	t.a = POINT_VEC(-104.49078f, 26.969154f, 132.72052f);
	t.b = POINT_VEC(-80.549988f, 9.9734116f, 7.7765503f);
	t.c = POINT_VEC(65.374435f, 15.144905f, 149.00766f);
	assert(!GJKIntersect(c, t));
	assert(!GJKIntersect(t, c));
}
*/

UNIQUE_TEST(TrickyGJKAABBAABBIntersect)
{
	AABB a(POINT_VEC_SCALAR(-10.f), POINT_VEC_SCALAR(10.f));
	AABB b(POINT_VEC_SCALAR(8.f), POINT_VEC_SCALAR(20.f));
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase)
{
	AABB a(POINT_VEC(37.1478767f,-71.9611969f,-51.1293259f),POINT_VEC(42.8975906f,-67.3180618f,-44.9161682f));
	Sphere b(POINT_VEC(41.9271927f,-71.1957016f,-56.7100677f),7.20756626f);
	assert(GJKIntersect(a, b));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase2)
{
	AABB a(POINT_VEC(-6.17850494f,-1.09283221f,-85.2101898f),POINT_VEC(-3.21846271f,-0.564386666f,-84.2947693f));
	Sphere b(POINT_VEC(-2.29130507f,-1.87549007f,-83.2377472f),2.09292078f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBShereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
	assert(!GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKAABBAABBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	try
	{
		assert(GJKIntersect(a, b));
	} catch(...)
	{
		LOGI("a: %s", a.SerializeToCodeString().c_str());
		LOGI("b: %s", b.SerializeToCodeString().c_str());
		throw;
	}
}

RANDOMIZED_TEST(GJKAABBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
}

/*
UNIQUE_TEST(GJKSphereLineSegmentIntersectCase)
{
	Sphere a(POINT_VEC(52.4970627f,45.4888649f,-7.32828188f),9.61045837f);
	LineSegment b(POINT_VEC(90.447998f,30.0441036f,-46.333149f),POINT_VEC(35.6951218f,38.615181f,4.10816383f));
	assert(GJKIntersect(a, b));
}
*/

RANDOMIZED_TEST(GJKSphereLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	try
	{
		assert(GJKIntersect(a, b));
	} catch(...)
	{
		LOGI("%s", a.SerializeToCodeString().c_str());
		LOGI("%s", b.SerializeToCodeString().c_str());
		throw;
	}
}

RANDOMIZED_TEST(GJKSphereCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBAABBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKFrustumFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKTriangleTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!GJKIntersect(a, b));
}
