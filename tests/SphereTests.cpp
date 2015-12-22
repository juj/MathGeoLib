#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"

MATH_IGNORE_UNUSED_VARS_WARNING

using namespace TestData;

static Sphere s(POINT_VEC(1.f,2.f,3.f), 5.f);

BENCHMARK(Sphere_RandomPointOnSurface, "Sphere::RandomPointOnSurface")
{
	v[i] = POINT_TO_FLOAT4(s.RandomPointOnSurface(rng));
}
BENCHMARK_END;

UNIQUE_TEST(Sphere_RandomPointOnSurface_small)
{
	Sphere s(POINT_VEC_SCALAR(10.f), 0.f);
	vec v = s.RandomPointOnSurface(rng);
	assert(v.Equals(10.f, 10.f, 10.f, 1.f));

	s = Sphere(POINT_VEC_SCALAR(10.f), 0.0001f);
	v = s.RandomPointOnSurface(rng);
	assert(v.Distance(POINT_VEC_SCALAR(10.f)) <= 0.0002f);
}

// Implement an alternative method for generating a random point on the surface of a Sphere that uses
// a closed formula for uniform geometric distribution instead of rejection sampling, which Sphere::RandomPointOnSurface uses.
// See here: http://mathworld.wolfram.com/SpherePointPicking.html
vec Sphere_RandomPointOnSurface2(const Sphere &sphere, LCG &lcg)
{
	float a = lcg.Float(0.f, 2.f*pi);
	float i = Acos(lcg.Float(-1.f, 1.f));
	vec v;
	v.SetFromSphericalCoordinates(a, i);
	return sphere.pos + v * sphere.r;
}

// Which version is faster? Rejection sampling or closed formula?
// On Macbook Air 2012 model running Win7:
// Benchmark 'Sphere_RandomPointOnSurface': Sphere::RandomPointOnSurface
//    Best: 89.146 nsecs / 151.97 ticks, Avg: 98.525 nsecs, Worst: 163.836 nsecs
// Benchmark 'Sphere_RandomPointOnSurface_ClosedFormula': Sphere_RandomPointOnSurface_ClosedFormula
//    Best: 369.836 nsecs / 629.126 ticks, Avg: 403.007 nsecs, Worst: 535.479 nsecs
BENCHMARK(Sphere_RandomPointOnSurface_ClosedFormula, "Sphere_RandomPointOnSurface_ClosedFormula")
{
	v[i] = POINT_TO_FLOAT4(Sphere_RandomPointOnSurface2(s, rng));
}
BENCHMARK_END;
