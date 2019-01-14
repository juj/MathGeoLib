#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"

using namespace TestData;

BENCHMARK(Triangle_intersects_AABB, "Triangle::Intersects(AABB)")
{
	dummyResultInt += Triangle(ve[i], ve[i+1], ve[i+2]).Intersects(aabb[i]) ? 1 : 0;
}
BENCHMARK_END

UNIQUE_TEST(Triangle_Intersects_AABB_Case)
{
	// Special case: one of the edges of the triangle (0,0,0) -> (1,0,0)
	//               coincides with an edge of the AABB, which will cause
	//               a degenerate zero SAT test axis in Triangle::Intersects() code.
	Triangle triangle(
		POINT_VEC(0.0f, 0.0f, 0.0f),
		POINT_VEC(1.0f, 0.0f, 0.0f),
		POINT_VEC(0.0f, 1.0f, 0.0f));

	AABB aabb(
		POINT_VEC(-2.0f, -2.0f, -2.0f),
		POINT_VEC(2.0f, 2.0f, 2.0f));

	assert(triangle.Intersects(aabb));
}

UNIQUE_TEST(Triangle_Intersects_AABB_Case2)
{
	// If a Triangle and an AABB touch at a common point (here zero (0,0,0)), report
	// it as not intersecting.
	Triangle triangle(
		POINT_VEC(0.0f, 0.0f, 0.0f),
		POINT_VEC(1.0f, 0.0f, 0.0f),
		POINT_VEC(0.0f, 1.0f, 0.0f));

	AABB aabb(
		POINT_VEC(-1.0f, -1.0f, -1.0f),
		POINT_VEC(0.0f, 0.0f, 0.0f));

	assert(!triangle.Intersects(aabb));
}

UNIQUE_TEST(Triangle_Intersects_AABB_Case3)
{
	// If a Triangle and an AABB touch at a common edge, report
	// it as not intersecting.
	Triangle triangle(
		POINT_VEC(0.0f, 0.0f, 0.0f),
		POINT_VEC(1.0f, 0.0f, 0.0f),
		POINT_VEC(0.0f, 1.0f, 0.0f));

	AABB aabb(
		POINT_VEC(-10.0f, -10.0f, -1.0f),
		POINT_VEC(10.0f, 10.0f, 0.0f));

	assert(!triangle.Intersects(aabb));
}
