#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

Circle2D RandomCircle2DContainingPoint(const float2 &pt, float maxRadius);

RANDOMIZED_TEST(Circle2D_OptimalEnclosingCircle)
{
	Circle2D c = RandomCircle2DContainingPoint(float2::RandomBox(rng, -100.f, 100.f), rng.Float(1.f, 100.f));
	std::vector<float2> pts;
	for(int i = 0; i < 100; ++i)
		pts.push_back(c.RandomPointInside(rng));

	Circle2D optimalEnclosingCircle = Circle2D::OptimalEnclosingCircle(&pts[0], (int)pts.size());
	assert2(optimalEnclosingCircle.r <= c.r + 1e-4f, optimalEnclosingCircle.r, c.r);
	for (size_t i = 0; i < pts.size(); ++i)
		assert3(optimalEnclosingCircle.Contains(pts[i]), optimalEnclosingCircle, pts[i], pts[i].Distance(optimalEnclosingCircle.pos) - optimalEnclosingCircle.r);
	assert2(optimalEnclosingCircle.Contains(c.pos), optimalEnclosingCircle, c.pos);
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
