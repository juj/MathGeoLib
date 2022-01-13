#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

MATH_IGNORE_UNUSED_VARS_WARNING

UNIQUE_TEST(Polygon_collinear_points_Plane)
{
	math::Polygon poly;
	poly.p.push_back(POINT_VEC(4980, 8600, 13060));
	poly.p.push_back(POINT_VEC(4849.33301f, 8600, 13060));
	poly.p.push_back(POINT_VEC(4820, 8600, 13060));
	poly.p.push_back(POINT_VEC(4820, 8600, 12835));
	poly.p.push_back(POINT_VEC(4980, 8600, 12835));

	mgl_assert1(!poly.IsDegenerate(), poly);
	mgl_assert1(!poly.IsNull(), poly);
	mgl_assert1(poly.IsPlanar(), poly);
	mgl_assert1(poly.IsFinite(), poly);

	math::Plane plane = poly.PlaneCCW();

	mgl_assert1(!plane.IsDegenerate(), plane);

	for(size_t i = 0; i < poly.p.size(); ++i)
		mgl_assert(plane.Contains(poly.p[i]));
}

UNIQUE_TEST(Polygon_IsPlanarCase)
{
	Polygon p;
	p.p.push_back(POINT_VEC(0.001175f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f,       0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f,       0.f, 1.f));
	p.p.push_back(POINT_VEC(0.001175f, 0.f, 1.f));
	mgl_assert(p.IsPlanar());
}

UNIQUE_TEST(Polygon_IsPlanarCase2)
{
	Polygon p;
	p.p.push_back(POINT_VEC(35.2095566f,148.158905f,13.9513502f));
	p.p.push_back(POINT_VEC(36.0442657f,147.617859f,13.1296129f));
	p.p.push_back(POINT_VEC(185.938354f,-49.2594376f,24.0656204f));
	p.p.push_back(POINT_VEC(-38.1764603f,96.0109634f,244.697083f));
	mgl_assert(p.IsPlanar());
}

UNIQUE_TEST(Polygon_IsPlanarCase3)
{
	Polygon p;
	p.p.push_back(POINT_VEC(-134.367065f,-26.9767799f,5.93324137f));
	p.p.push_back(POINT_VEC(-134.202698f,-27.2182236f,6.43854856f));
	p.p.push_back(POINT_VEC(-23.2372704f,4.65003967f,204.453552f));
	p.p.push_back(POINT_VEC(-130.144333f,161.677658f,-124.184113f));
	mgl_assert(p.IsPlanar());
}

UNIQUE_TEST(Triangle_ContainsPoint)
{
	Triangle t(POINT_VEC(37.8599548f,-7.36342621f,62.7666626f),
		POINT_VEC(112.436401f,-119.294914f,-119.175751f),
		POINT_VEC(99.9088364f,-100.386391f,-88.2388687f));

	vec pt = POINT_VEC(96.5959015f,-95.449379f,-80.2810669f);

	mgl_assert(t.Contains(pt));
}

UNIQUE_TEST(Polygon_Intersects_LineSegment_2D)
{
	Polygon p;
	p.p.push_back(POINT_VEC(0, 0, 0));
	p.p.push_back(POINT_VEC(1, 0, 0));
	p.p.push_back(POINT_VEC(1, 1, 0));
	p.p.push_back(POINT_VEC(0, 1, 0));
	mgl_assert(p.IsPlanar());

	LineSegment contained(POINT_VEC(0.5f, 0.5f, 0), POINT_VEC(0.5f, 0.75f, 0));
	LineSegment intersecting(POINT_VEC(0.5f, 0.5f, 0), POINT_VEC(0.5f, 1.5f, 0));
	LineSegment intersecting2(POINT_VEC(-0.5f, 0.5f, 0), POINT_VEC(1.5f, 0.5f, 0));
	LineSegment noncontained(POINT_VEC(1.5f, 1.5f, 0), POINT_VEC(1.5f, 2.5f, 0));

	mgl_assert(p.Intersects(contained));
	mgl_assert(p.Intersects(intersecting));
	mgl_assert(p.Intersects(intersecting2));
	mgl_assert(!p.Intersects(noncontained));

	mgl_assert(p.Contains(contained));
	mgl_assert(!p.Contains(intersecting));
	mgl_assert(!p.Contains(intersecting2));
	mgl_assert(!p.Contains(noncontained));

	mgl_assert(p.Contains2D(contained));
	mgl_assert(!p.Contains2D(intersecting));
	mgl_assert(!p.Contains2D(intersecting2));
	mgl_assert(!p.Contains2D(noncontained));
}

UNIQUE_TEST(Polygon_Intersects_Polygon_2D)
{
	Polygon p;
	p.p.push_back(POINT_VEC(156.644623f, -3.16135454f, 0));
	p.p.push_back(POINT_VEC(160.721878f, 18.5124626f, 0));
	p.p.push_back(POINT_VEC(169.520157f, -3.80513144f, 0));
	mgl_assert(p.IsPlanar());
	mgl_assert(p.IsSimple());

	Polygon p2;
	p2.p.push_back(POINT_VEC(30.0000019f, 0, 0));
	p2.p.push_back(POINT_VEC(30.2276134f, 4.43845034f, 0));
	p2.p.push_back(POINT_VEC(225.f, -10.f, 0));
	p2.p.push_back(POINT_VEC(225.2276f, -5.56155014f, 0));
	mgl_assert(p2.IsPlanar());
	mgl_assert(!p2.IsSimple());

	// p2 is a self-intersecting polygon. The convex hull of that Polygon would intersect p, but p2 itself does not intersect p.
	mgl_assert(!p.Intersects(p2));
	mgl_assert(!p2.Intersects(p));
	mgl_assert(!p.Contains(p2));
	mgl_assert(!p2.Contains(p));

	// Test that the rewinded version of p2 which has its vertices so that it does not self-intersect, does correctly return
	// intersection with p.
	Polygon p3;
	p3.p.push_back(POINT_VEC(30.0000019f, 0, 0));
	p3.p.push_back(POINT_VEC(30.2276134f, 4.43845034f, 0));
	p3.p.push_back(POINT_VEC(225.2276f, -5.56155014f, 0));
	p3.p.push_back(POINT_VEC(225.f, -10.f, 0));
	mgl_assert(p3.IsPlanar());
	mgl_assert(p3.IsSimple());

	mgl_assert(p3.Intersects(p));
	mgl_assert(p.Intersects(p3));
	mgl_assert(!p3.Contains(p));
	mgl_assert(!p.Contains(p3));
}