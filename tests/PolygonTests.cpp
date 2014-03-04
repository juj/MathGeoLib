#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

UNIQUE_TEST(Polygon_collinear_points_Plane)
{
	math::Polygon poly;
	poly.p.push_back(POINT_VEC(4980, 8600, 13060));
	poly.p.push_back(POINT_VEC(4849.33301f, 8600, 13060));
	poly.p.push_back(POINT_VEC(4820, 8600, 13060));
	poly.p.push_back(POINT_VEC(4820, 8600, 12835));
	poly.p.push_back(POINT_VEC(4980, 8600, 12835));

	assert1(!poly.IsDegenerate(), poly);
	assert1(!poly.IsNull(), poly);
	assert1(poly.IsPlanar(), poly);
	assert1(poly.IsFinite(), poly);

	math::Plane plane = poly.PlaneCCW();

	assert1(!plane.IsDegenerate(), plane);

	for(size_t i = 0; i < poly.p.size(); ++i)
		assert(plane.Contains(poly.p[i]));
}

UNIQUE_TEST(Polygon_IsPlanarCase)
{
	Polygon p;
	p.p.push_back(POINT_VEC(0.001175f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f,       0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f,       0.f, 1.f));
	p.p.push_back(POINT_VEC(0.001175f, 0.f, 1.f));
	assert(p.IsPlanar());
}
