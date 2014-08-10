#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

MATH_IGNORE_UNUSED_VARS_WARNING

Line RandomLineContainingPoint(const vec &pt);
Ray RandomRayContainingPoint(const vec &pt);
LineSegment RandomLineSegmentContainingPoint(const vec &pt);

RANDOMIZED_TEST(ParallelLineLineClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	Line b = a;
	vec displacement = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	b.pos += displacement;
	if (rng.Int()%2) b.dir = -b.dir;
	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	vec perpDistance = displacement - displacement.ProjectTo(a.dir);
	assert2(EqualAbs(closestPointA.Distance(closestPointB), perpDistance.Length()), closestPointA.Distance(closestPointB), perpDistance.Length());
}

RANDOMIZED_TEST(ParallelLineRayClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	Ray b;
	vec displacement = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	b.pos = a.pos + displacement;
	b.dir = a.dir;
	if (rng.Int()%2) b.dir = -b.dir;
	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	vec perpDistance = displacement - displacement.ProjectTo(a.dir);
	assert2(EqualAbs(closestPointA.Distance(closestPointB), perpDistance.Length()), closestPointA.Distance(closestPointB), perpDistance.Length());
}

RANDOMIZED_TEST(ParallelLineLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	LineSegment b;
	vec displacement = vec::RandomBox(rng, DIR_VEC_SCALAR(-SCALE), DIR_VEC_SCALAR(SCALE));
	b.a = a.pos + displacement;
	float len = rng.Float(1e-3f, SCALE) * (rng.Int(0,1) ? 1.f : -1.f);
	b.b = b.a + len * a.dir;

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	vec perpDistance = displacement - displacement.ProjectTo(a.dir);
	assert2(EqualAbs(closestPointA.Distance(closestPointB), perpDistance.Length()), closestPointA.Distance(closestPointB), perpDistance.Length());
}

RANDOMIZED_TEST(ParallelRayRayClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Ray a = RandomRayContainingPoint(pt);
	Ray b = a;
	vec displacement = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	b.pos += displacement;
	if (rng.Int()%2) b.dir = -b.dir;
	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	float cpd = closestPointA.Distance(closestPointB);
	assert(cpd <= displacement.Length()+1e-4f);
	MARK_UNUSED(cpd);

	float t = a.pos.Distance(b.pos);
	assert(cpd <= t+1e-4f);
	MARK_UNUSED(t);
}

RANDOMIZED_TEST(ParallelRayLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Ray a = RandomRayContainingPoint(pt);
	LineSegment b;
	vec displacement = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	b.a = a.pos + displacement;
	vec dir = rng.Float(1e-3f, SCALE) * a.dir;
	if (rng.Int()%2) dir = -dir;
	b.b = b.a + dir;

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	float cpd = closestPointA.Distance(closestPointB);
	assert(cpd <= displacement.Length()+1e-4f);
	MARK_UNUSED(cpd);

	float t1 = a.pos.Distance(b.a);
	float t2 = a.pos.Distance(b.b);
	assert(cpd <= t1+1e-4f);
	assert(cpd <= t2+1e-4f);
	MARK_UNUSED(t1);
	MARK_UNUSED(t2);
}

RANDOMIZED_TEST(ParallelLineSegmentLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	LineSegment a = RandomLineSegmentContainingPoint(pt);
	LineSegment b;
	vec displacement = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	b.a = a.a + displacement;
	if (rng.Int()%2)
	{
		vec dir = (b.a - a.a).ScaledToLength(rng.Float(1e-3f, SCALE));
		if (rng.Int()%2) dir = -dir;
		b.b = b.a + dir;
	}
	else
	{
		b.b = a.b + displacement;
	}
	if (rng.Int()%2)
		std::swap(b.a, b.b);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointB = b.GetPoint(d2);
	float cpd = closestPointA.Distance(closestPointB);
	assert(cpd <= displacement.Length()+1e-4f);
	MARK_UNUSED(cpd);

	float t1 = a.a.Distance(b.a);
	float t2 = a.a.Distance(b.b);
	float t3 = a.b.Distance(b.a);
	float t4 = a.b.Distance(b.b);
	assert(cpd <= t1+1e-4f);
	assert(cpd <= t2+1e-4f);
	assert(cpd <= t3+1e-4f);
	assert(cpd <= t4+1e-4f);
	MARK_UNUSED(t1);
	MARK_UNUSED(t2);
	MARK_UNUSED(t3);
	MARK_UNUSED(t4);
}

RANDOMIZED_TEST(LineLineClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert2(closestPointB.Equals(closestPointB2, 1e-2f), closestPointB.SerializeToCodeString(), closestPointB2.SerializeToCodeString());
	vec closestPointA2 = a.GetPoint(D2);
	assert2(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA.SerializeToCodeString(), closestPointA2.SerializeToCodeString());

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
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointAd = a.GetPoint(d);
	assert3(closestPointA.Equals(closestPointAd, 1e-1f), closestPointA, closestPointAd, closestPointA.Distance(closestPointAd));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
//	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
//	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert(closestPointB.Equals(closestPointB2, 1e-1f));
	vec closestPointA2 = a.GetPoint(D2);
	assert(closestPointA.Equals(closestPointA2, 1e-1f));

	assertcmp(closestPointA.Distance(closestPointB), <=, pt.Distance(pt2) + 1e-2f);
	assert(EqualAbs(a.Distance(b), closestPointA.Distance(closestPointB), 1e-1f));
	assert(EqualAbs(b.Distance(a), closestPointA.Distance(closestPointB), 1e-1f));

	assertcmp(closestPointA.Distance(closestPointB), <=, closestPointA.Distance(b.pos) + 1e-2f);
	assertcmp(closestPointA.Distance(closestPointB), <=, a.Distance(b.pos) + 1e-2f);
}

RANDOMIZED_TEST(LineLineSegmentClosestPoint)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec pt2 = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line a = RandomLineContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt2);

	float d, d2;
	vec closestPointA = a.ClosestPoint(b, d, d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
//	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
//	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert2(closestPointB.Equals(closestPointB2, 1e-2f), closestPointB.SerializeToCodeString(), closestPointB2.SerializeToCodeString());
	vec closestPointA2 = a.GetPoint(D2);
	assert2(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA.SerializeToCodeString(), closestPointA2.SerializeToCodeString());

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
	vec closestPointA = a.ClosestPoint(b, d, d2);
	vec closestPointA2 = a.GetPoint(d);
	assert3(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA, closestPointA2, closestPointA.Distance(closestPointA2));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
//	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
//	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert2(closestPointB.Equals(closestPointB2, 1e-2f), closestPointB.SerializeToCodeString(), closestPointB2.SerializeToCodeString());
	closestPointA2 = a.GetPoint(D2);
	assert2(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA.SerializeToCodeString(), closestPointA2.SerializeToCodeString());

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
	vec closestPointA = a.ClosestPoint(b, d, d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
//	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
//	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert2(closestPointB.Equals(closestPointB2, 1e-2f), closestPointB.SerializeToCodeString(), closestPointB2.SerializeToCodeString());
	vec closestPointA2 = a.GetPoint(D2);
	assert2(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA.SerializeToCodeString(), closestPointA2.SerializeToCodeString());

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
	vec closestPointA = a.ClosestPoint(b, d, d2);
	assert3(closestPointA.Equals(a.GetPoint(d), 1e-2f), closestPointA, a.GetPoint(d), closestPointA.Distance(a.GetPoint(d)));
	vec closestPointB = b.GetPoint(d2);
	float D, D2;
	vec closestPointB2 = b.ClosestPoint(a, D, D2);
//	assert2(EqualAbs(d, D2, 1e-2f) || EqualRel(d, D2, 1e-2f), d, D2);
//	assert2(EqualAbs(D, d2, 1e-2f) || EqualRel(D, d2, 1e-2f), D, d2);
	assert2(closestPointB.Equals(closestPointB2, 1e-2f), closestPointB.SerializeToCodeString(), closestPointB2.SerializeToCodeString());
	vec closestPointA2 = a.GetPoint(D2);
	assert2(closestPointA.Equals(closestPointA2, 1e-2f), closestPointA.SerializeToCodeString(), closestPointA2.SerializeToCodeString());

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
