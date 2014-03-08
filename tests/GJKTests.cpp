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
	Capsule a(POINT_VEC(-37.3521881,-61.0987396,77.0996475), POINT_VEC(46.2122498,-61.2913399,15.9034805) ,53.990406);
	AABB b(POINT_VEC(51.6529007,-28.0629959,65.9745636),POINT_VEC(59.8043518,-18.9434891,69.0897827));
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
RANDOMIZED_TEST(GJKSphereSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKSphereSphereNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
	assert(!GJKIntersect(b, a));
}

UNIQUE_TEST(TrickyGJKAABBAABBIntersect)
{
	AABB a(POINT_VEC_SCALAR(-10.f), POINT_VEC_SCALAR(10.f));
	AABB b(POINT_VEC_SCALAR(8.f), POINT_VEC_SCALAR(20.f));
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKAABBAABBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKAABBAABBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
	assert(!GJKIntersect(b, a));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase)
{
	AABB a(float4(37.1478767,-71.9611969,-51.1293259,1),float4(42.8975906,-67.3180618,-44.9161682,1));
	Sphere b(float4(41.9271927,-71.1957016,-56.7100677,1),7.20756626);
	assert(GJKIntersect(a, b));
}

UNIQUE_TEST(GJKAABBSphereIntersectCase2)
{
	AABB a(float4(-6.17850494,-1.09283221,-85.2101898,1),float4(-3.21846271,-0.564386666,-84.2947693,1));
	Sphere b(float4(-2.29130507,-1.87549007,-83.2377472,1),2.09292078);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKAABBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
//	LOGI("%s", a.SerializeToCodeString().c_str());
//	LOGI("%s", b.SerializeToCodeString().c_str());
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKAABBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKOBBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKSphereCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(GJKIntersect(a, b));
}

RANDOMIZED_TEST(GJKCapsuleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
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

RANDOMIZED_TEST(GJKAABBCapsuleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
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

RANDOMIZED_TEST(GJKOBBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

RANDOMIZED_TEST(GJKOBBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!GJKIntersect(a, b));
	assert(!GJKIntersect(b, a));
}
