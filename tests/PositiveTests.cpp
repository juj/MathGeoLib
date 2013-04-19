#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
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

RANDOMIZED_TEST(TestAABBAABBIntersect)
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

RANDOMIZED_TEST(TestAABBOBBIntersect)
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

RANDOMIZED_TEST(TestAABBLineIntersect)
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

RANDOMIZED_TEST(TestAABBRayIntersect)
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

RANDOMIZED_TEST(TestAABBLineSegmentIntersect)
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

RANDOMIZED_TEST(TestAABBPlaneIntersect)
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

RANDOMIZED_TEST(TestAABBSphereIntersect)
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

RANDOMIZED_TEST(TestAABBCapsuleIntersect)
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

RANDOMIZED_TEST(TestAABBTriangleIntersect)
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

RANDOMIZED_TEST(TestAABBFrustumIntersect)
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

RANDOMIZED_TEST(TestAABBPolyhedronIntersect)
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

RANDOMIZED_TEST(TestAABBPolygonIntersect)
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




RANDOMIZED_TEST(TestOBBOBBIntersect)
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

RANDOMIZED_TEST(TestOBBLineIntersect)
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

RANDOMIZED_TEST(TestOBBRayIntersect)
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

RANDOMIZED_TEST(TestOBBLineSegmentIntersect)
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

RANDOMIZED_TEST(TestOBBPlaneIntersect)
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

RANDOMIZED_TEST(TestOBBSphereIntersect)
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

RANDOMIZED_TEST(TestOBBCapsuleIntersect)
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

RANDOMIZED_TEST(TestOBBTriangleIntersect)
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

RANDOMIZED_TEST(TestOBBFrustumIntersect)
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

RANDOMIZED_TEST(TestOBBPolyhedronIntersect)
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

RANDOMIZED_TEST(TestOBBPolygonIntersect)
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





RANDOMIZED_TEST(TestSphereSphereIntersect)
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

RANDOMIZED_TEST(TestSphereLineIntersect)
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

RANDOMIZED_TEST(TestSphereRayIntersect)
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

RANDOMIZED_TEST(TestSphereLineSegmentIntersect)
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

RANDOMIZED_TEST(TestSpherePlaneIntersect)
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

RANDOMIZED_TEST(TestSphereCapsuleIntersect)
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

RANDOMIZED_TEST(TestSphereTriangleIntersect)
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

RANDOMIZED_TEST(TestSphereFrustumIntersect)
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

RANDOMIZED_TEST(TestSpherePolyhedronIntersect)
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

RANDOMIZED_TEST(TestSpherePolygonIntersect)
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




RANDOMIZED_TEST(TestFrustumLineIntersect)
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

RANDOMIZED_TEST(TestFrustumRayIntersect)
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

RANDOMIZED_TEST(TestFrustumLineSegmentIntersect)
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

RANDOMIZED_TEST(TestFrustumPlaneIntersect)
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

RANDOMIZED_TEST(TestFrustumCapsuleIntersect)
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

RANDOMIZED_TEST(TestFrustumTriangleIntersect)
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

RANDOMIZED_TEST(TestFrustumFrustumIntersect)
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

RANDOMIZED_TEST(TestFrustumPolyhedronIntersect)
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

RANDOMIZED_TEST(TestFrustumPolygonIntersect)
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




RANDOMIZED_TEST(TestCapsuleLineIntersect)
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

RANDOMIZED_TEST(TestCapsuleRayIntersect)
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

RANDOMIZED_TEST(TestCapsuleLineSegmentIntersect)
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

RANDOMIZED_TEST(TestCapsulePlaneIntersect)
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

RANDOMIZED_TEST(TestCapsuleCapsuleIntersect)
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

RANDOMIZED_TEST(TestCapsuleTriangleIntersect)
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

RANDOMIZED_TEST(TestCapsulePolyhedronIntersect)
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

RANDOMIZED_TEST(TestCapsulePolygonIntersect)
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





RANDOMIZED_TEST(TestPolyhedronLineIntersect)
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

RANDOMIZED_TEST(TestPolyhedronRayIntersect)
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

RANDOMIZED_TEST(TestPolyhedronLineSegmentIntersect)
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

RANDOMIZED_TEST(TestPolyhedronPlaneIntersect)
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

RANDOMIZED_TEST(TestPolyhedronTriangleIntersect)
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

RANDOMIZED_TEST(TestPolyhedronPolyhedronIntersect)
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

RANDOMIZED_TEST(TestPolyhedronPolygonIntersect)
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



RANDOMIZED_TEST(TestPolygonLineIntersect)
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

RANDOMIZED_TEST(TestPolygonRayIntersect)
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

RANDOMIZED_TEST(TestPolygonLineSegmentIntersect)
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

RANDOMIZED_TEST(TestPolygonPlaneIntersect)
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

RANDOMIZED_TEST(TestPolygonTriangleIntersect)
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

RANDOMIZED_TEST(TestPolygonPolygonIntersect)
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



RANDOMIZED_TEST(TestTriangleLineIntersect)
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

RANDOMIZED_TEST(TestTriangleRayIntersect)
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

RANDOMIZED_TEST(TestTriangleLineSegmentIntersect)
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

RANDOMIZED_TEST(TestTrianglePlaneIntersect)
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

RANDOMIZED_TEST(TestTriangleTriangleIntersect)
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




RANDOMIZED_TEST(TestPlaneLineIntersect)
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

RANDOMIZED_TEST(TestPlaneRayIntersect)
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

RANDOMIZED_TEST(TestPlaneLineSegmentIntersect)
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

RANDOMIZED_TEST(TestPlanePlaneIntersect)
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

TEST(TestPolygonContains2D)
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
