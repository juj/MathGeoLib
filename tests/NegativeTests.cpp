#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

#define SCALE 1e2f

#define GUARDBAND 1e-2f

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

void TestAABBAABBNoIntersect()
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

void TestAABBOBBNoIntersect()
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

void TestAABBLineNoIntersect()
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

void TestAABBRayNoIntersect()
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

void TestAABBLineSegmentNoIntersect()
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

void TestAABBPlaneNoIntersect()
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

void TestAABBSphereNoIntersect()
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

void TestAABBCapsuleNoIntersect()
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

void TestAABBTriangleNoIntersect()
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

void TestAABBFrustumNoIntersect()
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

void TestAABBPolyhedronNoIntersect()
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

void TestAABBPolygonNoIntersect()
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




void TestOBBOBBNoIntersect()
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

void TestOBBLineNoIntersect()
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

void TestOBBRayNoIntersect()
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

void TestOBBLineSegmentNoIntersect()
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

void TestOBBPlaneNoIntersect()
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

void TestOBBSphereNoIntersect()
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

void TestOBBCapsuleNoIntersect()
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

void TestOBBTriangleNoIntersect()
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

void TestOBBFrustumNoIntersect()
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

void TestOBBPolyhedronNoIntersect()
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

void TestOBBPolygonNoIntersect()
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





void TestSphereSphereNoIntersect()
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

void TestSphereLineNoIntersect()
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

void TestSphereRayNoIntersect()
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

void TestSphereLineSegmentNoIntersect()
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

void TestSpherePlaneNoIntersect()
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

void TestSphereCapsuleNoIntersect()
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

void TestSphereTriangleNoIntersect()
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

void TestSphereFrustumNoIntersect()
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

void TestSpherePolyhedronNoIntersect()
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

void TestSpherePolygonNoIntersect()
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




void TestFrustumLineNoIntersect()
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

void TestFrustumRayNoIntersect()
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

void TestFrustumLineSegmentNoIntersect()
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

void TestFrustumPlaneNoIntersect()
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

void TestFrustumCapsuleNoIntersect()
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

void TestFrustumTriangleNoIntersect()
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

void TestFrustumFrustumNoIntersect()
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

void TestFrustumPolyhedronNoIntersect()
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

void TestFrustumPolygonNoIntersect()
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




void TestCapsuleLineNoIntersect()
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

void TestCapsuleRayNoIntersect()
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

void TestCapsuleLineSegmentNoIntersect()
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

void TestCapsulePlaneNoIntersect()
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

void TestCapsuleCapsuleNoIntersect()
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

void TestCapsuleTriangleNoIntersect()
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

void TestCapsulePolyhedronNoIntersect()
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

void TestCapsulePolygonNoIntersect()
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





void TestPolyhedronLineNoIntersect()
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

void TestPolyhedronRayNoIntersect()
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

void TestPolyhedronLineSegmentNoIntersect()
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
// TODO BROKEN	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

void TestPolyhedronPlaneNoIntersect()
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

void TestPolyhedronTriangleNoIntersect()
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

void TestPolyhedronPolyhedronNoIntersect()
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
void TestPolyhedronPolyhedronIntersectionPerformance()
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

void TestPolyhedronPolygonNoIntersect()
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



void TestPolygonLineNoIntersect()
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

void TestPolygonRayNoIntersect()
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

void TestPolygonLineSegmentNoIntersect()
{
	Plane p(float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE)), float3::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
// TODO BROKEN	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

void TestPolygonPlaneNoIntersect()
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

void TestPolygonTriangleNoIntersect()
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

void TestPolygonPolygonNoIntersect()
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



void TestTriangleLineNoIntersect()
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

void TestTriangleRayNoIntersect()
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

void TestTriangleLineSegmentNoIntersect()
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

void TestTrianglePlaneNoIntersect()
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

void TestTriangleTriangleNoIntersect()
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




void TestPlaneLineNoIntersect()
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

void TestPlaneRayNoIntersect()
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

void TestPlaneLineSegmentNoIntersect()
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

void TestPlanePlaneNoIntersect()
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

void AddNegativeIntersectionTests()
{
	AddRandomizedTest("AABB-Line negative intersection", TestAABBLineNoIntersect);
	AddRandomizedTest("AABB-Ray negative intersection", TestAABBRayNoIntersect);
	AddRandomizedTest("AABB-LineSegment negative intersection", TestAABBLineSegmentNoIntersect);
	AddRandomizedTest("AABB-AABB negative intersection", TestAABBAABBNoIntersect);
	AddRandomizedTest("AABB-OBB negative intersection", TestAABBOBBNoIntersect);
	AddRandomizedTest("AABB-Sphere negative intersection", TestAABBSphereNoIntersect);
	AddRandomizedTest("AABB-Plane negative intersection", TestAABBPlaneNoIntersect);
	AddRandomizedTest("AABB-Triangle negative intersection", TestAABBTriangleNoIntersect);
	AddRandomizedTest("AABB-Capsule negative intersection", TestAABBCapsuleNoIntersect);
	AddRandomizedTest("AABB-Frustum negative intersection", TestAABBFrustumNoIntersect);
	AddRandomizedTest("AABB-Polygon negative intersection", TestAABBPolygonNoIntersect);
	AddRandomizedTest("AABB-Polyhedron negative intersection", TestAABBPolyhedronNoIntersect);

	AddRandomizedTest("OBB-Line negative intersection", TestOBBLineNoIntersect);
	AddRandomizedTest("OBB-Ray negative intersection", TestOBBRayNoIntersect);
	AddRandomizedTest("OBB-LineSegment negative intersection", TestOBBLineSegmentNoIntersect);
	AddRandomizedTest("OBB-OBB negative intersection", TestOBBOBBNoIntersect);
	AddRandomizedTest("OBB-Plane negative intersection", TestOBBPlaneNoIntersect);
	AddRandomizedTest("OBB-Sphere negative intersection", TestOBBSphereNoIntersect);
	AddRandomizedTest("OBB-Triangle negative intersection", TestOBBTriangleNoIntersect);
	AddRandomizedTest("OBB-Capsule negative intersection", TestOBBCapsuleNoIntersect);
	AddRandomizedTest("OBB-Frustum negative intersection", TestOBBFrustumNoIntersect);
	AddRandomizedTest("OBB-Polygon negative intersection", TestOBBPolygonNoIntersect);
	AddRandomizedTest("OBB-Polyhedron negative intersection", TestOBBPolyhedronNoIntersect);

	AddRandomizedTest("Sphere-Line negative intersection", TestSphereLineNoIntersect);
	AddRandomizedTest("Sphere-Ray negative intersection", TestSphereRayNoIntersect);
	AddRandomizedTest("Sphere-LineSegment negative intersection", TestSphereLineSegmentNoIntersect);
	AddRandomizedTest("Sphere-Plane negative intersection", TestSpherePlaneNoIntersect);
	AddRandomizedTest("Sphere-Sphere negative intersection", TestSphereSphereNoIntersect);
	AddRandomizedTest("Sphere-Triangle negative intersection", TestSphereTriangleNoIntersect);
	AddRandomizedTest("Sphere-Capsule negative intersection", TestSphereCapsuleNoIntersect);
	AddRandomizedTest("Sphere-Frustum negative intersection", TestSphereFrustumNoIntersect);
	AddRandomizedTest("Sphere-Polygon negative intersection", TestSpherePolygonNoIntersect);
	AddRandomizedTest("Sphere-Polyhedron negative intersection", TestSpherePolyhedronNoIntersect);

	AddRandomizedTest("Frustum-Line negative intersection", TestFrustumLineNoIntersect);
	AddRandomizedTest("Frustum-Ray negative intersection", TestFrustumRayNoIntersect);
	AddRandomizedTest("Frustum-LineSegment negative intersection", TestFrustumLineSegmentNoIntersect);
	AddRandomizedTest("Frustum-Plane negative intersection", TestFrustumPlaneNoIntersect);
	AddRandomizedTest("Frustum-Triangle negative intersection", TestFrustumTriangleNoIntersect);
	AddRandomizedTest("Frustum-Capsule negative intersection", TestFrustumCapsuleNoIntersect);
	AddRandomizedTest("Frustum-Frustum negative intersection", TestFrustumFrustumNoIntersect);
	AddRandomizedTest("Frustum-Polygon negative intersection", TestFrustumPolygonNoIntersect);
	AddRandomizedTest("Frustum-Polyhedron negative intersection", TestFrustumPolyhedronNoIntersect);

	AddRandomizedTest("Capsule-Line negative intersection", TestCapsuleLineNoIntersect);
	AddRandomizedTest("Capsule-Ray negative intersection", TestCapsuleRayNoIntersect);
	AddRandomizedTest("Capsule-LineSegment negative intersection", TestCapsuleLineSegmentNoIntersect);
	AddRandomizedTest("Capsule-Plane negative intersection", TestCapsulePlaneNoIntersect);
	AddRandomizedTest("Capsule-Triangle negative intersection", TestCapsuleTriangleNoIntersect);
	AddRandomizedTest("Capsule-Capsule negative intersection", TestCapsuleCapsuleNoIntersect);
	AddRandomizedTest("Capsule-Polygon negative intersection", TestCapsulePolygonNoIntersect);
	AddRandomizedTest("Capsule-Polyhedron negative intersection", TestCapsulePolyhedronNoIntersect);

	AddRandomizedTest("Polyhedron-Line negative intersection", TestPolyhedronLineNoIntersect);
	AddRandomizedTest("Polyhedron-Ray negative intersection", TestPolyhedronRayNoIntersect);
	AddRandomizedTest("Polyhedron-LineSegment negative intersection", TestPolyhedronLineSegmentNoIntersect);
	AddRandomizedTest("Polyhedron-Plane negative intersection", TestPolyhedronPlaneNoIntersect);
	AddRandomizedTest("Polyhedron-Triangle negative intersection", TestPolyhedronTriangleNoIntersect);
	AddRandomizedTest("Polyhedron-Polygon negative intersection", TestPolyhedronPolygonNoIntersect);
	AddRandomizedTest("Polyhedron-Polyhedron negative intersection", TestPolyhedronPolyhedronNoIntersect);
#ifndef _DEBUG
	AddRandomizedTest("Polyhedron-Polyhedron negative intersection performance", TestPolyhedronPolyhedronIntersectionPerformance);
#endif
	
	AddRandomizedTest("Polygon-Line negative intersection", TestPolygonLineNoIntersect);
	AddRandomizedTest("Polygon-Ray negative intersection", TestPolygonRayNoIntersect);
	AddRandomizedTest("Polygon-LineSegment negative intersection", TestPolygonLineSegmentNoIntersect);
	AddRandomizedTest("Polygon-Plane negative intersection", TestPolygonPlaneNoIntersect);
	AddRandomizedTest("Polygon-Triangle negative intersection", TestPolygonTriangleNoIntersect);
	AddRandomizedTest("Polygon-Polygon negative intersection", TestPolygonPolygonNoIntersect);

	AddRandomizedTest("Triangle-Line negative intersection", TestTriangleLineNoIntersect);
	AddRandomizedTest("Triangle-Ray negative intersection", TestTriangleRayNoIntersect);
	AddRandomizedTest("Triangle-LineSegment negative intersection", TestTriangleLineSegmentNoIntersect);
	AddRandomizedTest("Triangle-Plane negative intersection", TestTrianglePlaneNoIntersect);
	AddRandomizedTest("Triangle-Triangle negative intersection", TestTriangleTriangleNoIntersect);

	AddRandomizedTest("Plane-Line negative intersection", TestPlaneLineNoIntersect);
	AddRandomizedTest("Plane-Ray negative intersection", TestPlaneRayNoIntersect);
	AddRandomizedTest("Plane-LineSegment negative intersection", TestPlaneLineSegmentNoIntersect);
	AddRandomizedTest("Plane-Plane negative intersection", TestPlanePlaneNoIntersect);
}
