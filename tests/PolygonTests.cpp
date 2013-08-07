#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

UNIQUE_TEST(Polygon_collinear_points_Plane)
{
	math::Polygon poly;
	poly.p.push_back(float3(4980, 8600, 13060));
	poly.p.push_back(float3(4849.33301f, 8600, 13060));
	poly.p.push_back(float3(4820, 8600, 13060));
	poly.p.push_back(float3(4820, 8600, 12835));
	poly.p.push_back(float3(4980, 8600, 12835));

	assert(!poly.IsDegenerate());
	assert(!poly.IsNull());
	assert(poly.IsPlanar());
	assert(poly.IsFinite());

	math::Plane plane = poly.PlaneCCW();

	assert(!plane.IsDegenerate());

	for(size_t i = 0; i < poly.p.size(); ++i)
		assert(plane.Contains(poly.p[i]));
}
