#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

extern LCG rng;

#define SCALE 1e2f

AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength)
{
	float w = rng.Float(0, maxSideLength);
	float h = rng.Float(0, maxSideLength);
	float d = rng.Float(0, maxSideLength);

	AABB a(float3(0,0,0), float3(w,h,d));

	float3 extremePoint = a.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	a.Translate((distance + 1e-3f) * plane.normal);

	assert(!a.IsDegenerate());
	assert(a.IsFinite());
	assert(!a.Intersects(plane));
//	assert(a.SignedDistance(plane) > 0.f);
	extremePoint = a.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	return a;
}

OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength)
{
	AABB a = RandomAABBInHalfspace(plane, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 tm = float3x4::Translate(a.CenterPoint()) * rot * float3x4::Translate(-a.CenterPoint());
	OBB o = a.Transform(tm);

	float3 extremePoint = o.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	o.Translate((distance + 1e-3f) * plane.normal);

	assert(!o.IsDegenerate());
	assert(o.IsFinite());
	assert(!o.Intersects(plane));
//	assert(o.SignedDistance(plane) > 0.f);
	extremePoint = o.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);

	return o;
}

Sphere RandomSphereInHalfspace(const Plane &plane, float maxRadius)
{
	Sphere s(float3::zero, rng.Float(0.001f, maxRadius));
	s.pos += float3::RandomSphere(rng, float3::zero, s.r);

	float3 extremePoint = s.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	s.Translate((distance + 1e-3f) * plane.normal);

	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(!s.Intersects(plane));
//	assert(s.SignedDistance(plane) > 0.f);
	extremePoint = s.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	return s;
}
#if 0
Frustum RandomFrustumInHalfspace(const float3 &pt)
{
	Frustum f;
	if (rng.Int(0,1))
	{
		f.type = PerspectiveFrustum;
		f.orthographicWidth = rng.Float(0.001f, SCALE);
		f.orthographicHeight = rng.Float(0.001f, SCALE);
	}
	else
	{
		f.type = OrthographicFrustum;
		f.horizontalFov = rng.Float(0.001f, pi-0.001f);
		f.verticalFov = rng.Float(0.001f, pi-0.001f);
	}
	f.nearPlaneDistance = rng.Float(0.1f, SCALE);
	f.farPlaneDistance = f.nearPlaneDistance + rng.Float(0.1f, SCALE);
	f.pos = float3::zero;
	f.front = float3::RandomDir(rng);
	f.up = f.front.RandomPerpendicular(rng);

	float3 pt2 = f.UniformRandomPointInside(rng);
	f.pos += pt - pt2;

	assert(f.IsFinite());
//	assert(!f.IsDegenerate());
	assert(f.Contains(pt));
	return f;
}

Line RandomLineInHalfspace(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Line l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, SCALE));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Ray RandomRayInHalfspace(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Ray l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, 0));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

LineSegment RandomLineSegmentInHalfspace(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	LineSegment l(pt + a*dir, pt - b*dir);
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Capsule RandomCapsuleInHalfspace(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	float r = rng.Float(0.001f, SCALE);
	Capsule c(pt + a*dir, pt - b*dir, r);
	float3 d = float3::RandomSphere(rng, float3::zero, c.r);
	c.l.a += d;
	c.l.b += d;
	assert(c.IsFinite());
	assert(c.Contains(pt));

	return c;
}

Plane RandomPlaneInHalfspace(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Plane p(pt, dir);
	assert(!p.IsDegenerate());
	return p;
}

Triangle RandomTriangleInHalfspace(const float3 &pt)
{
	Plane p = RandomPlaneInHalfspace(pt);
	float3 a = pt;
	float3 b = p.Point(rng.Float(-SCALE, SCALE), rng.Float(-SCALE, SCALE));
	float3 c = p.Point(rng.Float(-SCALE, SCALE), rng.Float(-SCALE, SCALE));
	Triangle t(a,b,c);
	assert(t.Contains(pt));
	/*
	float3 d = t.RandomPointInside(rng);
	float3 br = t.BarycentricUVW(d);
	assert(t.Contains(d));
	d = d - t.a;
	t.a -= d;
	t.b -= d;
	t.c -= d;
	*/
	assert(t.IsFinite());
	assert(!t.IsDegenerate());
	assert(t.Contains(pt));
	return t;
}

Polyhedron RandomPolyhedronInHalfspace(const float3 &pt)
{
	switch(rng.Int(0,7))
	{
	case 0: return RandomAABBInHalfspace(pt, SCALE).ToPolyhedron();
	case 1: return RandomOBBInHalfspace(pt, SCALE).ToPolyhedron();
	case 2: return RandomFrustumInHalfspace(pt).ToPolyhedron();
	case 3: return Polyhedron::Tetrahedron(pt, SCALE); break;
	case 4: return Polyhedron::Octahedron(pt, SCALE); break;
	case 5: return Polyhedron::Hexahedron(pt, SCALE); break;
	case 6: return Polyhedron::Icosahedron(pt, SCALE); break;
	default: return Polyhedron::Dodecahedron(pt, SCALE); break;
	}

//	assert(t.IsFinite());
//	assert(!t.IsDegenerate());
//	assert(t.Contains(pt));
}

Polygon RandomPolygonInHalfspace(const float3 &pt)
{
	Polyhedron p = RandomPolyhedronInHalfspace(pt);
	Polygon poly = p.FacePolygon(rng.Int(0, p.NumFaces()-1));

	float3 pt2 = poly.FastRandomPointInside(rng);
	assert(poly.Contains(pt2));
	poly.Translate(pt - pt2);

	assert(!poly.IsDegenerate());
	assert(!poly.IsNull());
	assert(poly.IsPlanar());
	assert(poly.IsFinite());
	assert(poly.Contains(pt));

	return poly;
}
#endif

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

#if 0
void TestAABBLineNoIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Sphere b = RandomSphereInHalfspace(pt, SCALE);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Capsule b = RandomCapsuleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Frustum b = RandomFrustumInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBInHalfspace(pt, 10.f);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	OBB b = RandomOBBInHalfspace(pt, 10.f);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Sphere b = RandomSphereInHalfspace(pt, SCALE);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Capsule b = RandomCapsuleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Frustum b = RandomFrustumInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBInHalfspace(pt, 10.f);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Sphere b = RandomSphereInHalfspace(pt, 10.f);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Capsule b = RandomCapsuleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Frustum b = RandomFrustumInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereInHalfspace(pt, 10.f);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Capsule b = RandomCapsuleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Frustum b = RandomFrustumInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumInHalfspace(pt);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Capsule b = RandomCapsuleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleInHalfspace(pt);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

void TestPolyhedronPlaneNoIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Polyhedron b = RandomPolyhedronInHalfspace(pt);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

void TestPolyhedronPolygonNoIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronInHalfspace(pt);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

void TestPolygonPlaneNoIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonInHalfspace(pt);
	Polygon b = RandomPolygonInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleInHalfspace(pt);
	Triangle b = RandomTriangleInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneInHalfspace(pt);
	Line b = RandomLineInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneInHalfspace(pt);
	Ray b = RandomRayInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneInHalfspace(pt);
	LineSegment b = RandomLineSegmentInHalfspace(pt);
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
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneInHalfspace(pt);
	Plane b = RandomPlaneInHalfspace(pt);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

#endif

void AddNegativeIntersectionTests()
{
//	AddTest("AABB-Line negative intersection", TestAABBLineNoIntersect);
//	AddTest("AABB-Ray negative intersection", TestAABBRayNoIntersect);
//	AddTest("AABB-LineSegment negative intersection", TestAABBLineSegmentNoIntersect);
	AddTest("AABB-AABB negative intersection", TestAABBAABBNoIntersect);
	AddTest("AABB-OBB negative intersection", TestAABBOBBNoIntersect);/*
	AddTest("AABB-Plane negative intersection", TestAABBPlaneNoIntersect);
	AddTest("AABB-Sphere negative intersection", TestAABBSphereNoIntersect);
	AddTest("AABB-Triangle negative intersection", TestAABBTriangleNoIntersect);
	AddTest("AABB-Capsule negative intersection", TestAABBCapsuleNoIntersect);
	AddTest("AABB-Frustum negative intersection", TestAABBFrustumNoIntersect);
	AddTest("AABB-Polygon negative intersection", TestAABBPolygonNoIntersect);
	AddTest("AABB-Polyhedron negative intersection", TestAABBPolyhedronNoIntersect);

	AddTest("OBB-Line negative intersection", TestOBBLineNoIntersect);
	AddTest("OBB-Ray negative intersection", TestOBBRayNoIntersect);
	AddTest("OBB-LineSegment negative intersection", TestOBBLineSegmentNoIntersect);
	AddTest("OBB-OBB negative intersection", TestOBBOBBNoIntersect);
	AddTest("OBB-Plane negative intersection", TestOBBPlaneNoIntersect);
	AddTest("OBB-Sphere negative intersection", TestOBBSphereNoIntersect);
	AddTest("OBB-Triangle negative intersection", TestOBBTriangleNoIntersect);
	AddTest("OBB-Capsule negative intersection", TestOBBCapsuleNoIntersect);
	AddTest("OBB-Frustum negative intersection", TestOBBFrustumNoIntersect);
	AddTest("OBB-Polygon negative intersection", TestOBBPolygonNoIntersect);
	AddTest("OBB-Polyhedron negative intersection", TestOBBPolyhedronNoIntersect);

	AddTest("Sphere-Line negative intersection", TestSphereLineNoIntersect);
	AddTest("Sphere-Ray negative intersection", TestSphereRayNoIntersect);
	AddTest("Sphere-LineSegment negative intersection", TestSphereLineSegmentNoIntersect);
	AddTest("Sphere-Plane negative intersection", TestSpherePlaneNoIntersect);
	AddTest("Sphere-Sphere negative intersection", TestSphereSphereNoIntersect);
	AddTest("Sphere-Triangle negative intersection", TestSphereTriangleNoIntersect);
	AddTest("Sphere-Capsule negative intersection", TestSphereCapsuleNoIntersect);
	AddTest("Sphere-Frustum negative intersection", TestSphereFrustumNoIntersect);
	AddTest("Sphere-Polygon negative intersection", TestSpherePolygonNoIntersect);
	AddTest("Sphere-Polyhedron negative intersection", TestSpherePolyhedronNoIntersect);

	AddTest("Frustum-Line negative intersection", TestFrustumLineNoIntersect);
	AddTest("Frustum-Ray negative intersection", TestFrustumRayNoIntersect);
	AddTest("Frustum-LineSegment negative intersection", TestFrustumLineSegmentNoIntersect);
	AddTest("Frustum-Plane negative intersection", TestFrustumPlaneNoIntersect);
	AddTest("Frustum-Triangle negative intersection", TestFrustumTriangleNoIntersect);
	AddTest("Frustum-Capsule negative intersection", TestFrustumCapsuleNoIntersect);
	AddTest("Frustum-Frustum negative intersection", TestFrustumFrustumNoIntersect);
	AddTest("Frustum-Polygon negative intersection", TestFrustumPolygonNoIntersect);
	AddTest("Frustum-Polyhedron negative intersection", TestFrustumPolyhedronNoIntersect);

	AddTest("Capsule-Line negative intersection", TestCapsuleLineNoIntersect);
	AddTest("Capsule-Ray negative intersection", TestCapsuleRayNoIntersect);
	AddTest("Capsule-LineSegment negative intersection", TestCapsuleLineSegmentNoIntersect);
	AddTest("Capsule-Plane negative intersection", TestCapsulePlaneNoIntersect);
	AddTest("Capsule-Triangle negative intersection", TestCapsuleTriangleNoIntersect);
	AddTest("Capsule-Capsule negative intersection", TestCapsuleCapsuleNoIntersect);
	AddTest("Capsule-Polygon negative intersection", TestCapsulePolygonNoIntersect);
	AddTest("Capsule-Polyhedron negative intersection", TestCapsulePolyhedronNoIntersect);

	AddTest("Polyhedron-Line negative intersection", TestPolyhedronLineNoIntersect);
	AddTest("Polyhedron-Ray negative intersection", TestPolyhedronRayNoIntersect);
	AddTest("Polyhedron-LineSegment negative intersection", TestPolyhedronLineSegmentNoIntersect);
	AddTest("Polyhedron-Plane negative intersection", TestPolyhedronPlaneNoIntersect);
	AddTest("Polyhedron-Triangle negative intersection", TestPolyhedronTriangleNoIntersect);
	AddTest("Polyhedron-Polygon negative intersection", TestPolyhedronPolygonNoIntersect);
	AddTest("Polyhedron-Polyhedron negative intersection", TestPolyhedronPolyhedronNoIntersect);

	AddTest("Polygon-Line negative intersection", TestPolygonLineNoIntersect);
	AddTest("Polygon-Ray negative intersection", TestPolygonRayNoIntersect);
	AddTest("Polygon-LineSegment negative intersection", TestPolygonLineSegmentNoIntersect);
	AddTest("Polygon-Plane negative intersection", TestPolygonPlaneNoIntersect);
	AddTest("Polygon-Triangle negative intersection", TestPolygonTriangleNoIntersect);
	AddTest("Polygon-Polygon negative intersection", TestPolygonPolygonNoIntersect);

	AddTest("Triangle-Line negative intersection", TestTriangleLineNoIntersect);
	AddTest("Triangle-Ray negative intersection", TestTriangleRayNoIntersect);
	AddTest("Triangle-LineSegment negative intersection", TestTriangleLineSegmentNoIntersect);
	AddTest("Triangle-Plane negative intersection", TestTrianglePlaneNoIntersect);
	AddTest("Triangle-Triangle negative intersection", TestTriangleTriangleNoIntersect);

	AddTest("Plane-Line negative intersection", TestPlaneLineNoIntersect);
	AddTest("Plane-Ray negative intersection", TestPlaneRayNoIntersect);
	AddTest("Plane-LineSegment negative intersection", TestPlaneLineSegmentNoIntersect);
	AddTest("Plane-Plane negative intersection", TestPlanePlaneNoIntersect);
	*/
}
