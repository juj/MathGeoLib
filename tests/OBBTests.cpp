#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"

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

MATH_END_NAMESPACE
