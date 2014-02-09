#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

Line RandomLineContainingPoint(const vec &pt);
Ray RandomRayContainingPoint(const vec &pt);
LineSegment RandomLineSegmentContainingPoint(const vec &pt);

RANDOMIZED_TEST(LineLineClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2, 1e-2f), d, D2);
	assert2(EqualAbs(D, d2, 1e-2f), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));
}

RANDOMIZED_TEST(LineRayClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2), d, D2);
	assert2(EqualAbs(D, d2), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.pos) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.pos) + 1e-3f);
}

RANDOMIZED_TEST(LineLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2), d, D2);
	assert2(EqualAbs(D, d2), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.b) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.b) + 1e-3f);
}

RANDOMIZED_TEST(RayRayClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Ray a = RandomRayContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2), d, D2);
	assert2(EqualAbs(D, d2), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.pos) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.pos) + 1e-3f);

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointB.Distance(a.pos) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, b.Distance(a.pos) + 1e-3f);
}

RANDOMIZED_TEST(RayLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Ray a = RandomRayContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2), d, D2);
	assert2(EqualAbs(D, d2), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.b) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.b) + 1e-3f);

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointB.Distance(a.pos) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, b.Distance(a.pos) + 1e-3f);
}

RANDOMIZED_TEST(LineSegmentLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	LineSegment a = RandomLineSegmentContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, &d, &d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, &D, &D2);
	assert2(EqualAbs(d, D2), d, D2);
	assert2(EqualAbs(D, d2), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-2f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-3f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-2f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-2f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.b) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.b) + 1e-3f);

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointB.Distance(a.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointB.Distance(a.b) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, b.Distance(a.a) + 1e-3f);
	assertcmp(closestPointA.Distance(closestPointB), <=, b.Distance(a.b) + 1e-3f);
}
