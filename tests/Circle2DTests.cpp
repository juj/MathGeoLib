#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

Circle2D RandomCircle2DContainingPoint(const float2 &pt, float maxRadius);

RANDOMIZED_TEST(Circle2D_OptimalEnclosingCircle_Random)
{
	Circle2D c = RandomCircle2DContainingPoint(float2::RandomBox(rng, -100.f, 100.f), rng.Float(1.f, 100.f));
	std::vector<float2> pts;
	for(int i = 0; i < 100; ++i)
		pts.push_back(c.RandomPointInside(rng));

	Circle2D optimalEnclosingCircle = Circle2D::OptimalEnclosingCircle(&pts[0], (int)pts.size());
	assert2(optimalEnclosingCircle.r <= c.r + 1e-3f, optimalEnclosingCircle.r, c.r);
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
    assert(EqualAbs(circle.r, 12.8319559f));
    assert(EqualAbs(circle.pos.x, -5.778261184692383f));
    assert(EqualAbs(circle.pos.y, -4.908695220947266f));

    /*
    float3 pts3[5];
    for(int i = 0; i < 5; ++i)
        pts3[i] = float3(pts[i], 0.f);
    Sphere s = Sphere::OptimalEnclosingSphere(pts3, 5);
    LOGI("S: %s", s.SerializeToCodeString().c_str());
    for(int i = 0; i < 5; ++i)
        LOGI("%f", pts3[i].Distance(s.pos));
    */
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
    LOGI("%s", circle.ToString().c_str());
    for(int i = 0; i < 6; ++i)
        LOGI("%f", pts[i].Distance(circle.pos));
    /*
     assert(EqualAbs(circle.r, 25.2716084f));
    assert(EqualAbs(circle.pos.x, -103.81974792480469f));
    assert(EqualAbs(circle.pos.y, 63.85783767700195f));
     */

    float3 pts3[6];
     for(int i = 0; i < 6; ++i)
     pts3[i] = float3(pts[i], 0.f);
     Sphere s = Sphere::OptimalEnclosingSphere(pts3, 6);
     LOGI("S: %s", s.SerializeToCodeString().c_str());
     for(int i = 0; i < 6; ++i)
     LOGI("%f", pts3[i].Distance(s.pos));
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
    LOGI("%s", circle.ToString().c_str());
    for(int i = 0; i < 7; ++i)
        LOGI("%f", pts[i].Distance(circle.pos));

    assert(EqualAbs(circle.r, 62.699028f));
    assert(EqualAbs(circle.pos.x, 133.76840209960938f));
    assert(EqualAbs(circle.pos.y, 41.58528137207031f));

    float3 pts3[7];
    for(int i = 0; i < 7; ++i)
        pts3[i] = float3(pts[i], 0.f);
    Sphere s = Sphere::OptimalEnclosingSphere(pts3, 7);
    LOGI("S: %s", s.SerializeToCodeString().c_str());
    for(int i = 0; i < 7; ++i)
        LOGI("%f", pts3[i].Distance(s.pos));
}

UNIQUE_TEST(Circle2D_OptimalEnclosingCircle_Case5)
{
    float2 pts[] = {
        float2(0.f,0.f),
        float2(1.f,0.f),
        float2(1.f,0.f)
    };

    Circle2D circle = Circle2D::OptimalEnclosingCircle(pts, 3);
    assert(EqualAbs(circle.r, 0.5f, 1e-2f));
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
    assert(EqualAbs(circle.r, 79.8291626f));
    assert(EqualAbs(circle.pos.x, -127.111092f));
    assert(EqualAbs(circle.pos.y, 57.3470802f));
}
