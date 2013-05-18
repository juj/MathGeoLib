#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength)
{
	float w = rng.Float(0, maxSideLength);
	float h = rng.Float(0, maxSideLength);
	float d = rng.Float(0, maxSideLength);

	AABB a(float3(0,0,0), float3(w,h,d));

	a.Translate(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)));

	float3 aabbExtremePoint = a.ExtremePoint(-plane.normal);
	float distance = plane.Distance(aabbExtremePoint);
	a.Translate((distance + GUARDBAND) * plane.normal);

	assert(!a.IsDegenerate());
	assert(a.IsFinite());
	assert(!a.Intersects(plane));
//	assert(a.SignedDistance(plane) > 0.f);
	aabbExtremePoint = a.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(aabbExtremePoint) > 0.f);
	assert(plane.SignedDistance(a) > 0.f);
	return a;
}

OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength)
{
	AABB a = RandomAABBInHalfspace(plane, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 tm = float3x4::Translate(a.CenterPoint()) * rot * float3x4::Translate(-a.CenterPoint());
	OBB o = a.Transform(tm);

	float3 obbExtremePoint = o.ExtremePoint(-plane.normal);
	float distance = plane.Distance(obbExtremePoint);
	o.Translate((distance + GUARDBAND) * plane.normal);

	assert(!o.IsDegenerate());
	assert(o.IsFinite());
	assert(!o.Intersects(plane));
//	assert(o.SignedDistance(plane) > 0.f);
	obbExtremePoint = o.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(obbExtremePoint) > 0.f);
	assert(plane.SignedDistance(o) > 0.f);

	return o;
}

Sphere RandomSphereInHalfspace(const Plane &plane, float maxRadius)
{
	Sphere s(float3::zero, rng.Float(0.001f, maxRadius));
	s.Translate(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)));

	float3 extremePoint = s.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	s.Translate((distance + GUARDBAND) * plane.normal);

	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(!s.Intersects(plane));
//	assert(s.SignedDistance(plane) > 0.f);
	extremePoint = s.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(s) > 0.f);
	return s;
}

Frustum RandomFrustumInHalfspace(const Plane &plane)
{
	Frustum f;
	if (rng.Int(0,1))
	{
		f.type = OrthographicFrustum;
		f.orthographicWidth = rng.Float(0.001f, SCALE);
		f.orthographicHeight = rng.Float(0.001f, SCALE);
	}
	else
	{
		f.type = PerspectiveFrustum;
		// Really random Frustum could have fov as ]0, pi[, but limit
		// to much narrower fovs to not cause the corner vertices
		// shoot too far when farPlaneDistance is very large.
		f.horizontalFov = rng.Float(0.001f, 3.f*pi/4.f);
		f.verticalFov = rng.Float(0.001f, 3.f*pi/4.f);
	}
	f.nearPlaneDistance = rng.Float(0.1f, SCALE);
	f.farPlaneDistance = f.nearPlaneDistance + rng.Float(0.1f, SCALE);
	f.pos = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	f.front = float3::RandomDir(rng);
	f.up = f.front.RandomPerpendicular(rng);

//	assert(!f.IsDegenerate());

	float3 extremePoint = f.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	f.Translate((distance + GUARDBAND) * plane.normal);

	assert(f.IsFinite());
//	assert(!f.IsDegenerate());
	assert(!f.Intersects(plane));
//	assert(s.SignedDistance(plane) > 0.f);
	extremePoint = f.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(f) > 0.f);

	return f;
}

Line RandomLineInHalfspace(const Plane &plane)
{
	float3 linePos = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	if (plane.SignedDistance(linePos) < 0.f)
		linePos = plane.Mirror(linePos);
	linePos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(linePos) >= 1e-3f);

	float3 dir = plane.normal.RandomPerpendicular(rng);
	Line l(linePos, dir);
	assert(l.IsFinite());
	assert(!plane.Intersects(l));
	assert(plane.SignedDistance(l) > 0.f);
	return l;
}

Ray RandomRayInHalfspace(const Plane &plane)
{
	if (rng.Int(0, 10) == 0)
		return RandomLineInHalfspace(plane).ToRay();

	float3 rayPos = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	if (plane.SignedDistance(rayPos) < 0.f)
		rayPos = plane.Mirror(rayPos);
	rayPos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(rayPos) >= 1e-3f);

	float3 dir = float3::RandomDir(rng);
	if (dir.Dot(plane.normal) < 0.f)
		dir = -dir;
	Ray r(rayPos, dir);
	assert(r.IsFinite());
	assert(plane.SignedDistance(r.GetPoint(SCALE*10.f)) > 0.f);
	assert(!plane.Intersects(r));
	assert(plane.SignedDistance(r) > 0.f);
	return r;
}

LineSegment RandomLineSegmentInHalfspace(const Plane &plane)
{
	LineSegment ls = RandomRayInHalfspace(plane).ToLineSegment(0.f, rng.Float(0.f, SCALE));
	assert(ls.IsFinite());
	assert(plane.SignedDistance(ls) > 0.f);

	return ls;
}

Capsule RandomCapsuleInHalfspace(const Plane &plane)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	float3 dir = float3::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	float r = rng.Float(0.001f, SCALE);
	Capsule c(pt + a*dir, pt - b*dir, r);
	assert(c.IsFinite());

	float3 extremePoint = c.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	c.Translate((distance + GUARDBAND) * plane.normal);

	assert(c.IsFinite());
//	assert(!c.IsDegenerate());
	assert(!c.Intersects(plane));
//	assert(c.SignedDistance(plane) > 0.f);
	extremePoint = c.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(c) > 0.f);

	return c;
}

Plane RandomPlaneInHalfspace(Plane &plane)
{
	Plane p2;
	p2.normal = plane.normal;
	p2.d = rng.Float(plane.d + 1e-2f, plane.d + 1e-2f + SCALE);
	assert(!p2.IsDegenerate());
	return p2;
}

Triangle RandomTriangleInHalfspace(const Plane &plane)
{
	float3 a = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	float3 b = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	float3 c = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle t(a,b,c);

	assert(t.IsFinite());
	assert(!t.IsDegenerate());

	float3 extremePoint = t.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	t.Translate((distance + GUARDBAND) * plane.normal);

	assert(t.IsFinite());
	assert(!t.IsDegenerate());
	assert(!t.Intersects(plane));
//	assert(t.SignedDistance(plane) > 0.f);
	extremePoint = t.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(t) > 0.f);

	return t;
}

Polyhedron RandomPolyhedronInHalfspace(const Plane &plane)
{
	Polyhedron p;
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));

	switch(rng.Int(0,7))
	{
	case 0: p = RandomAABBInHalfspace(plane, SCALE).ToPolyhedron();
	case 1: p = RandomOBBInHalfspace(plane, SCALE).ToPolyhedron();
	case 2: p = RandomFrustumInHalfspace(plane).ToPolyhedron();
	case 3: p = Polyhedron::Tetrahedron(pt, SCALE); break;
	case 4: p = Polyhedron::Octahedron(pt, SCALE); break;
	case 5: p = Polyhedron::Hexahedron(pt, SCALE); break;
	case 6: p = Polyhedron::Icosahedron(pt, SCALE); break;
	default: p = Polyhedron::Dodecahedron(pt, SCALE); break;
	}

//	assert(p.IsFinite());
//	assert(!p.IsDegenerate());

	float3 extremePoint = p.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	p.Translate((distance + GUARDBAND) * plane.normal);

//	assert(p.IsFinite());
//	assert(!p.IsDegenerate());
	assert(!p.Intersects(plane));
//	assert(p.SignedDistance(plane) > 0.f);
	extremePoint = p.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(p) > 0.f);
	
	return p;
}

Polygon RandomPolygonInHalfspace(const Plane &plane)
{
	Polyhedron p = RandomPolyhedronInHalfspace(plane);
	Polygon poly = p.FacePolygon(rng.Int(0, p.NumFaces()-1));

	assert(!poly.IsDegenerate());
	assert(!poly.IsNull());
	assert(poly.IsPlanar());
	assert(poly.IsFinite());
	assert(!poly.Intersects(plane));
	float3 extremePoint = poly.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(poly) > 0.f);

	return poly;
}

RANDOMIZED_TEST(AABBAABBNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBOBBNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	if (a.Intersects(b))
	{
		LOGI("AABB: %s", a.ToString().c_str());
		LOGI("Line: %s", b.ToString().c_str());
	}
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBSphereNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBCapsuleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBFrustumNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(OBBOBBNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBSphereNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
////	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBCapsuleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBFrustumNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
///	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(SphereSphereNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereCapsuleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereFrustumNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(FrustumLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumCapsuleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumFrustumNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(CapsuleLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleCapsuleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
///	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
///	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(PolyhedronLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Distance(a.ClosestPoint(b)) < 1e-3f);
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polyhedron.
//	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPolyhedronNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

#ifndef _DEBUG
RANDOMIZED_TEST(PolyhedronPolyhedronIntersectionPerformance)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);

	for(size_t i = 0; i < 10; ++i)
	{
		globalPokedData += a.Intersects(b) ? 1 : 0;
		globalPokedData += b.Intersects(a) ? 1 : 0;
	}
}
#endif

RANDOMIZED_TEST(PolyhedronPolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}



RANDOMIZED_TEST(PolygonLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Distance(a.ClosestPoint(b)) < 1e-3f);
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polygon.
//	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPolygonNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
///	assert(b.Contains(b.ClosestPoint(a)));
}



RANDOMIZED_TEST(TriangleLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
//	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
//	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TrianglePlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleTriangleNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
	assert(!a.Contains(b.ClosestPoint(a)));
	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(PlaneLineNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert(!a.Intersects(b));
///	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneRayNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneLineSegmentNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlanePlaneNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(RayTriangleMeshNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	TriangleMesh tm;
	tm.Set(a);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	float d = tm.IntersectRay(b);
	assert(d == FLOAT_INF);
}

RANDOMIZED_TEST(RayKdTreeNoIntersect)
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	KdTree<Triangle> t;
	std::vector<Triangle> tris = a.Triangulate();
	if (!tris.empty())
		t.AddObjects(&tris[0], tris.size());
	t.Build();
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	TriangleKdTreeRayQueryNearestHitVisitor result;
	t.RayQuery(b, result);
	assert(result.rayT == FLOAT_INF);
	assert(result.triangleIndex == KdTree<Triangle>::BUCKET_SENTINEL);
	assert(!result.pos.IsFinite());
	assert(!result.barycentricUV.IsFinite());
}
