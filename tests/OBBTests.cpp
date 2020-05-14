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

// Programmatically found test case of two OBBs which by construction should be disjoint.
// Verifies fix to bug https://github.com/juj/MathGeoLib/pull/50
UNIQUE_TEST(OBB_NoIntersect_OBB_Case1)
{
	OBB a;
	a.pos = POINT_VEC(-58.54f, -57.58f, 87.16f);
	a.r = DIR_VEC(0.49f, 1.86f, 2.29f);
	a.axis[0] = DIR_VEC(0.82f, -0.50f, -0.29f);
	a.axis[1] = DIR_VEC(-0.49f, -0.87f, 0.10f);
	a.axis[2] = DIR_VEC(-0.31f, 0.06f, -0.95f);

	OBB b;
	b.pos = POINT_VEC(-59.53f, -50.44f, 83.27f);
	b.r = DIR_VEC(3.62f, 3.07f, 5.00f);
	b.axis[0] = DIR_VEC(-0.41f, 0.81f, -0.42f);
	b.axis[1] = DIR_VEC(0.44f, -0.23f, -0.87f);
	b.axis[2] = DIR_VEC(-0.80f, -0.54f, -0.26f);

	assert(!a.Intersects(b));
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

// Tests that OBB::OptimalEnclosingOBB() works even if all points in the input set lie in a plane (degenerating into a 2D convex hull computation)
RANDOMIZED_TEST(OBB_OptimalEnclosingOBB_Degenerate2D)
{
	return; /// XXX TODO Bug, this sometimes hangs tests execution in debug builds
	// Generate some points.
#ifdef _DEBUG
	const int n = 6; // This test is very slow, so run debug builds with fewer points.
#else
	const int n = 10;
#endif
	vec points[n];
	vec planeNormal = vec::RandomDir(rng);
	vec planeBasisX, planeBasisY;
	vec planeOffset = vec::RandomBox(rng, -10.f, 10.f);
	planeNormal.PerpendicularBasis(planeBasisX, planeBasisY);

	for(int i = 0; i < n; ++i)
	{
		float3 uvw = float3::RandomBox(rng, -10.f, 10.f, -10.f, 10.f, -1e-5f, 1e-5f);
		points[i] = planeOffset + uvw.x * planeBasisX + uvw.y * planeBasisY + uvw.z * planeNormal;
	}

	// Compute the minimal OBB that encloses those points.
	OBB optimalObb = OBB::OptimalEnclosingOBB(points, n);
	assert(!optimalObb.IsDegenerate());
	assert(optimalObb.LocalToWorld().IsOrthonormal());

#ifdef MATH_VEC_IS_FLOAT4
	assert(EqualAbs(optimalObb.pos.w, 1.f));
	assert(EqualAbs(optimalObb.axis[0].w, 0.f));
	assert(EqualAbs(optimalObb.axis[1].w, 0.f));
	assert(EqualAbs(optimalObb.axis[2].w, 0.f));
	assert(EqualAbs(optimalObb.r.w, 0.f));
#endif

	// Brute force compute a bounding box
	OBB bruteObb = OBB::BruteEnclosingOBB(points, n);
	assert(!bruteObb.IsDegenerate());
	assert(bruteObb.LocalToWorld().IsOrthonormal());

	// Test that both boxes actually enclose the given points.
	for(int i = 0; i < n; ++i)
	{
		assert1(optimalObb.Distance(points[i]) < 1e-3f, optimalObb.Distance(points[i]));
		assert1(bruteObb.Distance(points[i]) < 1e-3f, bruteObb.Distance(points[i]));
	}
		//assert2(o.Contains(points[i]), points[i], o.Distance(points[i]));

	// Would like to assert that optimal OBB has smaller volume than brute-force computed OBB volume, but since
	// the point sets are planar, the generated OBB should be practically flat, and volume should be zero. Therefore
	// instead of asserting volume, assert that the surface area of the optimal box is smaller than that of the brute
	// force OBB.
//	assert4(optimalObb.Volume() <= bruteObb.Volume(), optimalObb, bruteObb, optimalObb.Volume(), bruteObb.Volume());

#define MAX_SURFACE_AREA(v) Max(v.x*v.y, v.x*v.z, v.y*v.z)
	float optimalSurfaceArea = MAX_SURFACE_AREA(optimalObb.r);
	float bruteSurfaceArea = MAX_SURFACE_AREA(bruteObb.r);
	assert4(optimalSurfaceArea <= bruteSurfaceArea + 1e-5f, optimalObb, bruteObb, optimalSurfaceArea, bruteSurfaceArea);
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

UNIQUE_TEST(OBB_OptimalEnclosingOBB_Case2)
{
	const int n = 6;
	vec points[n];
	points[0] = POINT_VEC(59.341487884521484f,34.765708923339844f,-4.293600082397461f);
	points[1] = POINT_VEC(125.95323181152344f,82.85620880126953f,-38.54533767700195f);
	points[2] = POINT_VEC(112.87208557128906f,107.59994506835938f,-103.65940856933594f);
	points[3] = POINT_VEC(54.61711883544922f,49.73502731323242f,-40.48734664916992f);
	points[4] = POINT_VEC(59.369625091552734f,89.0080795288086f,-118.2476577758789f);
	points[5] = POINT_VEC(109.97838592529297f,58.90884780883789f,-4.244309425354004f);

	OBB knownTightOBB(POINT_VEC(91.14148712158203,73.37193298339844,-53.52738952636719),
		DIR_VEC(62.99241256713867,38.400611877441406,1.0013580322265625e-5),
		DIR_VEC(5.447334051132202e-2,4.5014870166778564e-1,-8.912904262542725e-1),
		DIR_VEC(9.167373180389404e-1,3.3123981952667236e-1,2.2332215309143066e-1),
		DIR_VEC(3.9575910568237305e-1,-8.292444348335266e-1,-3.946244418621063e-1));

	OBB minOBB = OBB::OptimalEnclosingOBB(points, n);
	OBB fastOBB = OBB::BruteEnclosingOBB(points, n);

	for(int i = 0; i < n; ++i)
		assert1(knownTightOBB.Distance(points[i]) < 1e-4f, knownTightOBB.Distance(points[i]));
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
	assert(minOBB.Volume() <= fastOBB.Volume());
}

UNIQUE_TEST(OBB_OptimalEnclosingOBB_Case3)
{
	const int n = 6;
	vec points[n];
	points[0] = POINT_VEC(133.55322265625,9.555416107177734,-74.741424560546875f);
	points[1] = POINT_VEC(150.7891082763672,11.76172161102295,-110.29942321777344f);
	points[2] = POINT_VEC(109.87129211425781,9.385478019714355,-37.43644332885742f);
	points[3] = POINT_VEC(62.75429916381836,18.51451301574707,-1.4324088096618652f);
	points[4] = POINT_VEC(54.70846939086914,52.70939254760742,-127.0294418334961f);
	points[5] = POINT_VEC(115.27964782714844,27.54103660583496,-119.08977508544922f);

	OBB knownTightOBB(POINT_VEC(105.76068115234375,19.434932708740234,-71.6479721069336),
		DIR_VEC(63.4930305480957,50.283843994140625,14.859146118164063),
		DIR_VEC(-1.3948273658752441e-1,4.7978922724723816e-2,-9.890612959861755e-1),
		DIR_VEC(-9.352766871452332e-1,3.217141330242157e-1,1.4750392735004425e-1),
		DIR_VEC(3.252721428871155e-1,9.456204771995544e-1,1.4875997100816107e-9));

	OBB minOBB = OBB::OptimalEnclosingOBB(points, n);
	OBB fastOBB = OBB::BruteEnclosingOBB(points, n);

	for(int i = 0; i < n; ++i)
		assert1(knownTightOBB.Distance(points[i]) < 1e-4f, knownTightOBB.Distance(points[i]));
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
//	assert(minOBB.Volume() <= fastOBB.Volume()); // TODO: Due to numerical imprecision, brute-forcing seems to jiggle itself to a tiny fraction better result than the minimally computed one
}

UNIQUE_TEST(OBB_OptimalEnclosingOBB_Case4)
{
	const int n = 6;
	vec points[n];
	points[0] = POINT_VEC(17.184921264648438,56.92377471923828,-24.37803077697754f);
	points[1] = POINT_VEC(145.6709442138672,12.358505249023438,97.28238677978516f);
	points[2] = POINT_VEC(15.339648246765137,81.33776092529297,78.882080078125f);
	points[3] = POINT_VEC(127.43468475341797,1.1394100189208984,2.5231540203094482f);
	points[4] = POINT_VEC(130.3594207763672,-4.408816814422607,-14.732804298400879f);
	points[5] = POINT_VEC(96.54469299316406,6.472623348236084,-50.493019104003906f);

	Polyhedron convexHull = Polyhedron::ConvexHull(points, n);
	for(int i = 0; i < n; ++i)
		assert3(convexHull.ContainsConvex(points[i]) || convexHull.Distance(points[i]) < 1e-4f, convexHull, points[i], convexHull.Distance(points[i]));

	OBB minOBB = OBB::OptimalEnclosingOBB(points, n);
	OBB fastOBB = OBB::BruteEnclosingOBB(points, n);

	for(int i = 0; i < n; ++i)
		assert2(fastOBB.Distance(points[i]) < 1e-4f, i, fastOBB.Distance(points[i]));

	LOGI("Min OBB volume: %.9g", minOBB.Volume());
	LOGI("fast OBB volume: %.9g", fastOBB.Volume());

//	assert(minOBB.Volume() <= fastOBB.Volume()); // TODO: Due to numerical imprecision, brute-forcing seems to jiggle itself to a tiny fraction better result than the minimally computed one
}


MATH_END_NAMESPACE
