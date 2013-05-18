#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

AABB RandomAABBContainingPoint(const float3 &pt, float maxSideLength)
{
	float w = rng.Float(1e-2f, maxSideLength);
	float h = rng.Float(1e-2f, maxSideLength);
	float d = rng.Float(1e-2f, maxSideLength);

	AABB a(float3(0,0,0), float3(w,h,d));
	w = rng.Float(1e-3f, w-1e-3f);
	h = rng.Float(1e-3f, h-1e-3f);
	d = rng.Float(1e-3f, d-1e-3f);
	a.Translate(pt - float3(w,h,d));
	assert(!a.IsDegenerate());
	assert(a.IsFinite());
	assert(a.Contains(pt));
	return a;
}

OBB RandomOBBContainingPoint(const float3 &pt, float maxSideLength)
{
	AABB a = RandomAABBContainingPoint(pt, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 tm = float3x4::Translate(pt) * rot * float3x4::Translate(-pt);
	OBB o = a.Transform(tm);
	assert(!o.IsDegenerate());
	assert(o.IsFinite());
	assert(o.Contains(pt));
	return o;
}

Sphere RandomSphereContainingPoint(const float3 &pt, float maxRadius)
{
	Sphere s(pt, rng.Float(1.f, maxRadius));
	s.pos += float3::RandomSphere(rng, float3::zero, Max(0.f, s.r - 1e-2f));
	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(s.Contains(pt));
	return s;
}

Frustum RandomFrustumContainingPoint(const float3 &pt)
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

Line RandomLineContainingPoint(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Line l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, SCALE));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Ray RandomRayContainingPoint(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Ray l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, 0));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

LineSegment RandomLineSegmentContainingPoint(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	LineSegment l(pt + a*dir, pt - b*dir);
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Capsule RandomCapsuleContainingPoint(const float3 &pt)
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

Plane RandomPlaneContainingPoint(const float3 &pt)
{
	float3 dir = float3::RandomDir(rng);
	Plane p(pt, dir);
	assert(!p.IsDegenerate());
	return p;
}

Triangle RandomTriangleContainingPoint(const float3 &pt)
{
	Plane p = RandomPlaneContainingPoint(pt);
	float3 a = pt;
	float3 b = p.Point(rng.Float(-SCALE, SCALE), rng.Float(-SCALE, SCALE));
	float3 c = p.Point(rng.Float(-SCALE, SCALE), rng.Float(-SCALE, SCALE));
	Triangle t(a,b,c);
	assert(t.Contains(pt));

	float3 d = t.RandomPointInside(rng);
	assert(t.Contains(d));
	d = d - t.a;
	t.a -= d;
	t.b -= d;
	t.c -= d;

	assert(t.IsFinite());
	assert(!t.IsDegenerate());
	assert(t.Contains(pt));
	return t;
}

Polyhedron RandomPolyhedronContainingPoint(const float3 &pt)
{
	switch(rng.Int(0,7))
	{
	case 0: return RandomAABBContainingPoint(pt, SCALE).ToPolyhedron();
	case 1: return RandomOBBContainingPoint(pt, SCALE).ToPolyhedron();
	case 2: return RandomFrustumContainingPoint(pt).ToPolyhedron();
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

Polygon RandomPolygonContainingPoint(const float3 &pt)
{
	Polyhedron p = RandomPolyhedronContainingPoint(pt);
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

RANDOMIZED_TEST(AABBAABBIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBOBBIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBSphereIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBCapsuleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBFrustumIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(OBBOBBIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBSphereIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
////	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBCapsuleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBFrustumIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
///	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(SphereSphereIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereCapsuleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereFrustumIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(FrustumLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumCapsuleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumFrustumIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Frustum b = RandomFrustumContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Frustum a = RandomFrustumContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(CapsuleLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleCapsuleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
///	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
///	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(PolyhedronLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Distance(a.ClosestPoint(b)) < 1e-3f);
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polyhedron.
//	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPolyhedronIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}



RANDOMIZED_TEST(PolygonLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPolygonIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
///	assert(b.Contains(b.ClosestPoint(a)));
}



RANDOMIZED_TEST(TriangleLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
//	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
	assert(a.Contains(b.ClosestPoint(a)));
	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
//	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TrianglePlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleTriangleIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
	assert(a.Contains(b.ClosestPoint(a)));
	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(PlaneLineIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
///	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneRayIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneLineSegmentIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));
	assert(a.Distance(b) == 0.f);
	assert(b.Distance(a) == 0.f);
	assert(a.Contains(a.ClosestPoint(b)));
	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlanePlaneIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	if (a.normal.Equals(b.normal))
		a.d = b.d; // Avoid floating-point imprecision issues in the test: if plane normals are equal, make sure the planes are parallel.
	if (a.normal.Equals(-b.normal))
		a.d = -b.d;
	assert(a.Intersects(b));
	assert(b.Intersects(a));
//	assert(a.Distance(b) == 0.f);
//	assert(b.Distance(a) == 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(b.Contains(a.ClosestPoint(b)));
//	assert(a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(RayTriangleMeshIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	TriangleMesh tm;
	tm.Set(a);
	Ray b = RandomRayContainingPoint(pt);
	float d = tm.IntersectRay(b);
	assert(d >= 0.f);
	assert(IsFinite(d));
}

RANDOMIZED_TEST(RayKdTreeIntersect)
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	KdTree<Triangle> t;
	std::vector<Triangle> tris = a.Triangulate();
	if (!tris.empty())
		t.AddObjects(&tris[0], tris.size());
	t.Build();
	Ray b = RandomRayContainingPoint(pt);
	TriangleKdTreeRayQueryNearestHitVisitor result;
	t.RayQuery(b, result);
	assert(result.rayT >= 0.f);
	assert(result.rayT < FLOAT_INF);
	assert(result.triangleIndex != KdTree<Triangle>::BUCKET_SENTINEL);
	assert(result.pos.IsFinite());
	assert(result.barycentricUV.IsFinite());
}

TEST(PolygonContains2D)
{
	float xmin = 0.f, xmax = 10.f, ymin = 0.f, ymax = 10.f, z = 2.f;

	float3 point = float3((xmax-xmin)/2,(ymax-ymin)/2,z);
	Polygon pol;
	pol.p.push_back(float3(xmin,ymin,z));
	pol.p.push_back(float3(xmax,ymin,z));
	pol.p.push_back(float3(xmax,ymax,z));
	pol.p.push_back(float3(xmin,ymax,z));

	assert(pol.Contains(point));
}
