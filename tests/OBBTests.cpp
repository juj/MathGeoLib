#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

MATH_BEGIN_NAMESPACE

using namespace TestData;

UNIQUE_TEST(OBB_ClosestPoint_Point)
{
	vec pt = POINT_VEC_SCALAR(0.f);
	OBB o(pt, DIR_VEC(1.f, 1.f, 1.f), vec::unitX, vec::unitY, vec::unitZ);
	assert(o.ClosestPoint(pt).Equals(pt));
	assert(o.ClosestPoint(POINT_VEC(5.f, 0.f, 0.f)).Equals(POINT_VEC(1.f, 0.f, 0.f)));
	assert(o.ClosestPoint(POINT_VEC(5.f, 5.f, 5.f)).Equals(POINT_VEC(1.f, 1.f, 1.f)));
	assert(o.ClosestPoint(POINT_VEC(-5.f, -5.f, -5.f)).Equals(POINT_VEC(-1.f, -1.f, -1.f)));
}

BENCHMARK(OBBIntersectsOBB_Random, "OBB::Intersects(OBB) Random")
{
	if (obb[i].Intersects(obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_Positive, "OBB::Intersects(OBB) Positive")
{
	if (obb[i].Intersects(obb[i]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_GJK, "OBB::Intersects(OBB)_GJK")
{
	if (GJKIntersect(obb[i], obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBIntersectsOBB_SAT, "OBB::Intersects(OBB)_SAT")
{
	if (SATIntersect(obb[i], obb[i+1]))
		++dummyResultInt;
}
BENCHMARK_END

BENCHMARK(OBBContains, "OBB::Contains(point)")
{
	uf[i] = obb[i].Contains(ve[i]) ? 1.f : 0.f;
}
BENCHMARK_END

BENCHMARK(OBBClosestPoint, "OBB::ClosestPoint(point)")
{
	dummyResultVec += obb[i].ClosestPoint(ve[i]);
}
BENCHMARK_END

BENCHMARK(OBBOBBIntersectionCentroid, "Centroid of OBB-OBB intersection")
{
	// Best: 43.610 usecs / 74139.1 ticks, Avg: 45.376 usecs, Worst: 52.899 usecs
	const OBB &a = obb[i];
	OBB b = obb[i+1];
	b.pos = a.pos; // Make sure the two OBBs intersect.
	PBVolume<12> intersection = a.ToPBVolume().SetIntersection(b.ToPBVolume());
	Polyhedron Volume = intersection.ToPolyhedron();
	ve[i] = Volume.ConvexCentroid();
}
BENCHMARK_END

RANDOMIZED_TEST(OBB_OptimalEnclosingOBB)
{
	// Generate some points.
#ifdef _DEBUG
	const int n = 6; // This test is very slow, so run debug builds with fewer points.
#else
	const int n = 10;
#endif
	vec points[n];
	for(int i = 0; i < n; ++i)
		points[i] = vec::RandomBox(rng, -100.f, 100.f);

	// Compute the minimal OBB that encloses those points.
	OBB o = OBB::OptimalEnclosingOBB(points, n);

#ifdef MATH_VEC_IS_FLOAT4
	assert(EqualAbs(o.pos.w, 1.f));
	assert(EqualAbs(o.axis[0].w, 0.f));
	assert(EqualAbs(o.axis[1].w, 0.f));
	assert(EqualAbs(o.axis[2].w, 0.f));
	assert(EqualAbs(o.r.w, 0.f));
#endif
	// Test that it does actually enclose the given points.
	for(int i = 0; i < n; ++i)
		assert1(o.Distance(points[i]) < 1e-3f, o.Distance(points[i]));
		//assert2(o.Contains(points[i]), points[i], o.Distance(points[i]));
}

MATH_END_NAMESPACE
