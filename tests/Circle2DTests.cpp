#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"
#include "../tests/ObjectGenerators.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

RANDOMIZED_TEST(Circle2D_OptimalEnclosingCircle_Random_3)
{
	Circle2D c = RandomCircle2DContainingPoint(rng, float2::RandomBox(rng, -100.f, 100.f), rng.Float(1.f, 100.f));
	std::vector<float2> pts;
	for (int i = 0; i < 3; ++i)
		pts.push_back(c.RandomPointInside(rng));

	Circle2D optimalEnclosingCircle = Circle2D::OptimalEnclosingCircle(pts[0], pts[1], pts[2]);
	assert2(optimalEnclosingCircle.r <= c.r + 1e-2f, optimalEnclosingCircle.r, c.r);
	for (size_t i = 0; i < pts.size(); ++i)
	{
		if (!optimalEnclosingCircle.Contains(pts[i]))
			for (size_t j = 0; j < pts.size(); ++j)
				LOGI("%d %f %s", (int)j, optimalEnclosingCircle.SignedDistance(pts[i]), pts[j].SerializeToCodeString().c_str());
		assert3(optimalEnclosingCircle.Contains(pts[i], 1e-3f), optimalEnclosingCircle, pts[i], optimalEnclosingCircle.SignedDistance(pts[i]));
	}
}

RANDOMIZED_TEST(Circle2D_OptimalEnclosingCircle_Random_N)
{
	Circle2D c = RandomCircle2DContainingPoint(rng, float2::RandomBox(rng, -100.f, 100.f), rng.Float(1.f, 100.f));
	std::vector<float2> pts;
	for(int i = 0; i < 100; ++i)
		pts.push_back(c.RandomPointInside(rng));

	Circle2D optimalEnclosingCircle = Circle2D::OptimalEnclosingCircle(&pts[0], (int)pts.size());
	assert2(optimalEnclosingCircle.r <= c.r + 1e-2f, optimalEnclosingCircle.r, c.r);
	for (size_t i = 0; i < pts.size(); ++i)
		assert3(optimalEnclosingCircle.Contains(pts[i], 1e-3f), optimalEnclosingCircle, pts[i], optimalEnclosingCircle.SignedDistance(pts[i]));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case1)
{
	float2 a(53.910076f, 109.024796f);
	float2 b(49.150211f, 58.530991f);
	float2 c(-55.001110f, 82.599510f);
	Circle2D circle = Circle2D::OptimalEnclosingCircle(a, b, c);
	assert(EqualAbs(circle.pos.Distance(a), 56.53574f));
	assert(EqualAbs(circle.pos.Distance(b), 56.53574f));
	assert(EqualAbs(circle.pos.Distance(c), 56.53574f));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case2)
{
	float2 pts[] = {
		float2(-18.f,-1.f),
		float2(-4.f,-8.f),
		float2(6.f,-10.f),
		float2(-1.f,7.f),
		float2(-15.f,4.f)
	};

	Circle2D circle = Circle2D::OptimalEnclosingCircle(pts, 5);
	assert1(EqualAbs(circle.r, 12.8316f), circle.r);
	assert(EqualAbs(circle.pos.x, -5.778261184692383f));
	assert(EqualAbs(circle.pos.y, -4.908695220947266f));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case3)
{
	float2 pts[] = {
		float2(-129.f,66.f),
		float2(-125.f,52.f),
		float2(-81.f,53.f),
		float2(-100.f,81.f),
		float2(-106.f,86.f),
		float2(-116.f,86.f)
	};

	Circle2D circle = Circle2D::OptimalEnclosingCircle(pts, 6);

	assert1(EqualAbs(circle.r, 25.2722f), circle.r);
	assert(EqualAbs(circle.pos.x, -103.81974792480469f));
	assert(EqualAbs(circle.pos.y, 63.85783767700195f));

}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case4)
{
	float2 pts[] = {
		float2(105.f,-1.f),
		float2(117.f,-14.f),
		float2(130.f,-21.f),
		float2(171.f,-8.f),
		float2(194.f,59.f),
		float2(117.f,102.f),
		float2(112.f,85.f)
	};

	Circle2D circle = Circle2D::OptimalEnclosingCircle(pts, 7);

	assert1(EqualAbs(circle.r, 62.6996f), circle.r);
	assert(EqualAbs(circle.pos.x, 133.76840209960938f));
	assert(EqualAbs(circle.pos.y, 41.58528137207031f));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case5)
{
	float2 pts[] = {
		float2(0.f,0.f),
		float2(1.f,0.f),
		float2(1.f,0.f)
	};

	Circle2D circle = Circle2D::OptimalEnclosingCircle(pts, 3);
	assert1(EqualAbs(circle.r, 0.5f, 1e-2f), circle.r);
	assert(EqualAbs(circle.pos.x, 0.5f));
	assert(EqualAbs(circle.pos.y, 0.f));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case6)
{
	float2 pts[] = {
		float2(-50.f, 78.f),
		float2(-205.f, 40.f),
		float2(-204.f, 36.f)
	};

	Circle2D circle = Circle2D::OptimalEnclosingCircle(pts[0], pts[1], pts[2]);
	assert(circle.Contains(pts[0]));
	assert(circle.Contains(pts[1]));
	assert(circle.Contains(pts[2]));
	assert1(EqualAbs(circle.r, 79.829f), circle.r);
	assert(EqualAbs(circle.pos.x, -127.111092f));
	assert(EqualAbs(circle.pos.y, 57.3470802f));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case7)
{
	float2 a(-25.927188873291016f, -49.56178283691406f);
	float2 b(-26.206756591796875f, -49.47246170043945f);
	float2 c(-25.87479591369629f, -49.58361053466797f);
	Circle2D circle = Circle2D::OptimalEnclosingCircle(a, b, c);
	assert2(circle.Contains(a), circle.SignedDistance(a), circle);
	assert2(circle.Contains(b), circle.SignedDistance(b), circle);
	assert2(circle.Contains(c), circle.SignedDistance(c), circle);
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case8)
{
	float2 a(41.856231689453125f, -113.56066131591797f);
	float2 b(95.8228759765625f, -74.76484680175781f);
	float2 c(96.95646667480469f, -73.947662353515625f);
	Circle2D circle = Circle2D::OptimalEnclosingCircle(a, b, c);
	assert2(circle.Contains(a), circle.SignedDistance(a), circle);
	assert2(circle.Contains(b), circle.SignedDistance(b), circle);
	assert2(circle.Contains(c), circle.SignedDistance(c), circle);
}

BENCHMARK(Circle2D_OptimalEnclosingCircle3_bench, "Circle2D::OptimalEnclosingCircle(a,b,c)")
{
	ucircle2d[i] = Circle2D::OptimalEnclosingCircle(fl_2[i], fl_2[i + 1], fl_2[i + 2]);
}
BENCHMARK_END

BENCHMARK(Circle2D_OptimalEnclosingCircle_bench, "Circle2D::OptimalEnclosingCircle()")
{
	ucircle2d[i] = Circle2D::OptimalEnclosingCircle(fl_2, testrunner_numItersPerTest);
}
BENCHMARK_END
