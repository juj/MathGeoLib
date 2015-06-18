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

UNIQUE_TEST(OBB_OptimalEnclosingOBB_Case)
{
	const int n = 6;
	vec points[n];
	points[0] = POINT_VEC(-23.211870193481445f,-6.64400577545166f,-56.42197036743164f);
	points[1] = POINT_VEC(-28.85224723815918f,-4.590736389160156f,-50.14798355102539f);
	points[2] = POINT_VEC(-17.786113739013672f,-60.76505661010742f,26.743661880493164f);
	points[3] = POINT_VEC(-20.125133514404297f,42.60211944580078f,-9.701617240905762f);
	points[4] = POINT_VEC(-2.903409719467163f,-5.581843376159668f,16.836111068725586f);
	points[5] = POINT_VEC(54.23094940185547f,-24.333148956298828f,-18.875534057617188f);
/*
	OBB knownTightOBB(POINT_VEC(-7.546141624450684f,-16.55905532836914f,-15.989764213562012f),
		DIR_VEC(35.57231521606445f,41.67085647583008f,29.720752716064453f),
		DIR_VEC(0.6724355816841125f,0.734805166721344f,0.0888344794511795f),
		DIR_VEC(0.6183832883834839f,-0.6236984133720398f,0.4781237840652466f),
		DIR_VEC(0.40673381090164185f,-0.2665737271308899f,-0.8737883567810059f));
		*/
	OBB knownTightOBB(POINT_VEC(-7.546105861663818f,-16.559236526489258f,-15.989850997924805f),
		DIR_VEC(35.572265625f,41.67076110839844f,29.720821380615234f),
		DIR_VEC(0.6724331974983215f,0.7348074316978455f,0.08883479982614517f),
		DIR_VEC(0.6183837652206421f,-0.6236954927444458f,0.47812697291374207f),
		DIR_VEC(0.40673714876174927f,-0.2665744721889496f,-0.8737865686416626f));

	OBB minOBB = OBB::OptimalEnclosingOBB(points, n);
	OBB fastOBB = OBB::BruteEnclosingOBB(points, n);

	for(int i = 0; i < n; ++i)
		assert1(knownTightOBB.Distance(points[i]) < 1e-5f, knownTightOBB.Distance(points[i]));
	LOGI("Tight OBB Volume: %.9g", knownTightOBB.Volume());
	for(int i = 0; i < 6; i += 2)
		LOGI("Tight OBB normal: %s", knownTightOBB.FacePlane(i).normal.ToString().c_str());
//	LOGI("%s", minOBB.SerializeToCodeString().c_str());
	LOGI("Min OBB volume: %.9g", minOBB.Volume());
	LOGI("fast OBB volume: %.9g", fastOBB.Volume());
	Quat q = (Quat)knownTightOBB.LocalToWorld();
	Quat q2 = (Quat)minOBB.LocalToWorld();
	LOGI("Difference in angle: %.9g (%.9g degrees)", q.AngleBetween(q2), RadToDeg(q.AngleBetween(q2)));
//	LOGI("%s", fastOBB.SerializeToCodeString().c_str());

	assert(minOBB.Volume() <= knownTightOBB.Volume());
	assert(fastOBB.Volume() <= knownTightOBB.Volume()*1.005 /* fastOBB is only an approximation, so allow small 0.5% wiggle room. */);
	assert(minOBB.Volume() <= fastOBB.Volume());
}

MATH_END_NAMESPACE
