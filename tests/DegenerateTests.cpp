#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

TEST(AABB_Degenerate)
{
	AABB a;
	a.SetNegativeInfinity();
	assert(a.IsDegenerate());

	a = AABB(POINT_VEC_SCALAR(0.f), POINT_VEC_SCALAR(1.f));
	assert(!a.IsDegenerate());
}

TEST(OBB_Degenerate)
{
	OBB o;
	o.SetNegativeInfinity();
	assert(o.IsDegenerate());

	o = OBB(AABB(POINT_VEC_SCALAR(0.f), POINT_VEC_SCALAR(1.f)));
	assert1(!o.IsDegenerate(), o);
}

TEST(Capsule_Degenerate)
{
	Capsule c;
	c.SetDegenerate();
	assert(c.IsDegenerate());

	c = Capsule(POINT_VEC_SCALAR(0.f), POINT_VEC_SCALAR(1.f), 1.f);
	assert(!c.IsDegenerate());
}
/* ///\todo Implement.
TEST(Circle_Degenerate)
{
	Circle c;
	c.SetDegenerate();
	assert(c.IsDegenerate());

	c = Circle(float3::zero, float3::unitX, 1.f);
	assert(!c.IsDegenerate());
}

TEST(Frustum_Degenerate)
{
	Frustum f;
	f.SetDegenerate();
	assert(f.IsDegenerate());

	f.type = OrthographicFrustum;
	f.pos = float3::zero;
	f.front = float3::unitX;
	f.up = float3::unitY;
	f.nearPlaneDistance = 0.1f;
	f.farPlaneDistance = 1.f;
	f.orthographicWidth = 1.f;
	f.orthographicHeight = 1.f;
	assert(!f.IsDegenerate());
}

TEST(Line_Degenerate)
{
	Line l;
	l.SetDegenerate();
	assert(l.IsDegenerate());

	l = Line(float3::zero, float3::unitX);
	assert(!l.IsDegenerate());
}

TEST(LineSegment_Degenerate)
{
	LineSegment l;
	l.SetDegenerate();
	assert(l.IsDegenerate());

	l = LineSegment(float3::zero, float3::unitX);
	assert(!l.IsDegenerate());
}

TEST(Plane_Degenerate)
{
	Plane p;
	p.SetDegenerate();
	assert(p.IsDegenerate());

	p = Plane(float3::unitX, 1.f);
	assert(!p.IsDegenerate());
}

TEST(Polygon_Degenerate)
{
	Polygon p;
	p.SetDegenerate();
	assert(p.IsDegenerate());

	p = Triangle(float3::unitX, float3::unitY, float3::unitZ).ToPolygon();
	assert(!p.IsDegenerate());
}

TEST(Polyhedron_Degenerate)
{
	Polyhedron p;
	p.SetDegenerate();
	assert(p.IsDegenerate());

	p = Polyhedron::Tetrahedron();
	assert(!p.IsDegenerate());
}

TEST(Ray_Degenerate)
{
	Ray r;
	r.SetDegenerate();
	assert(r.IsDegenerate());

	r = Ray(float3::zero, float3::unitX);
	assert(!r.IsDegenerate());
}
*/
TEST(Sphere_Degenerate)
{
	Sphere s;
	s.SetDegenerate();
	assert(s.IsDegenerate());

	s = Sphere(POINT_VEC_SCALAR(0.f), POINT_VEC(float3::unitX));
	assert(!s.IsDegenerate());
}
/* ///\todo Implement.
TEST(Triangle_Degenerate)
{
	Triangle t;
	t.SetDegenerate();
	assert(t.IsDegenerate());

	t = Triangle(float3::unitX, float3::unitY, float3::unitZ);
	assert(!t.IsDegenerate());
}
*/

///\todo Geometric objects with any non-finite elements should be checked to be degenerate.