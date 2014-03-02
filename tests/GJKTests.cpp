#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"

Sphere RandomSphereContainingPoint(const vec &pt, float maxRadius);
Sphere RandomSphereInHalfspace(const Plane &plane, float maxRadius);
AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength);
AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength);
OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength);
OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength);

UNIQUE_TEST(TrickyGJKSphereSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));

	Sphere a = Sphere(POINT_VEC(-14.740263f, 8.2647991f, 64.282227f), 7.6029987f);
	Sphere b = Sphere(POINT_VEC(-23.840866f, 9.4233770f, 66.399742f), 1.9519407f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

UNIQUE_TEST(TrickyGJKSphereSphereIntersect2)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));

	Sphere a = Sphere(POINT_VEC(57.166256f, 99.426201f, 75.735786f), 2.3808355f);
	Sphere b = Sphere(POINT_VEC(54.087727f, 94.719139f, 75.188812f), 3.3114955f);
	assert(GJKIntersect(a, b));
	assert(GJKIntersect(b, a));
}

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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
