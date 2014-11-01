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
