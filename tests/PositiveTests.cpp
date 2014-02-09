#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength)
{
	float w = rng.Float(1e-2f, maxSideLength);
	float h = rng.Float(1e-2f, maxSideLength);
	float d = rng.Float(1e-2f, maxSideLength);

	AABB a(POINT_VEC(0, 0, 0), POINT_VEC(w, h, d));
	w = rng.Float(1e-3f, w-1e-3f);
	h = rng.Float(1e-3f, h-1e-3f);
	d = rng.Float(1e-3f, d-1e-3f);
	a.Translate(pt - POINT_VEC(w, h, d));
	assert(!a.IsDegenerate());
	assert(a.IsFinite());
	assert(a.Contains(pt));
	return a;
}

OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength)
{
	AABB a = RandomAABBContainingPoint(pt, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	float3x4 tm = float3x4::Translate(pt) * rot * float3x4::Translate(-pt);
	OBB o = a.Transform(tm);
	assert1(!o.IsDegenerate(), o);
	assert(o.IsFinite());
	assert(o.Contains(pt));
	return o;
}

Sphere RandomSphereContainingPoint(const vec &pt, float maxRadius)
{
	Sphere s(pt, rng.Float(1.f, maxRadius));
	s.pos += vec::RandomSphere(rng, POINT_VEC_SCALAR(0.f), Max(0.f, s.r - 1e-2f));
	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(s.Contains(pt));
	return s;
}

Frustum RandomFrustumContainingPoint(const vec &pt)
{
	Frustum f;
	f.handedness = (rng.Int(0,1) == 1) ? FrustumRightHanded : FrustumLeftHanded;
	f.projectiveSpace = (rng.Int(0,1) == 1) ? FrustumSpaceD3D : FrustumSpaceGL;

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
	f.pos = POINT_VEC_SCALAR(0.f);
	f.front = vec::RandomDir(rng);
	f.up = f.front.RandomPerpendicular(rng);

	vec pt2 = f.UniformRandomPointInside(rng);
	f.pos += pt - pt2;

	assert(f.IsFinite());
//	assert(!f.IsDegenerate());
	assert(f.Contains(pt));
	return f;
}

Line RandomLineContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	Line l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, SCALE));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Ray RandomRayContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	Ray l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, 0));
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

LineSegment RandomLineSegmentContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	LineSegment l(pt + a*dir, pt - b*dir);
	assert(l.IsFinite());
	assert(l.Contains(pt));
	return l;
}

Capsule RandomCapsuleContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	float r = rng.Float(0.001f, SCALE);
	Capsule c(pt + a*dir, pt - b*dir, r);
	vec d = vec::RandomSphere(rng, vec::zero, c.r);
	c.l.a += d;
	c.l.b += d;
	assert(c.IsFinite());
	assert(c.Contains(pt));

	return c;
}

Plane RandomPlaneContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	Plane p(pt, dir);
	assert(!p.IsDegenerate());
	return p;
}

Triangle RandomTriangleContainingPoint(const vec &pt)
{
	Plane p = RandomPlaneContainingPoint(pt);
	vec a = pt;
	float f1 = rng.Float(-SCALE, SCALE);
	float f2 = rng.Float(-SCALE, SCALE);
	float f3 = rng.Float(-SCALE, SCALE);
	float f4 = rng.Float(-SCALE, SCALE);
	vec b = p.Point(f1, f2);
	vec c = p.Point(f3, f4);
	Triangle t(a,b,c);
	assert(t.Contains(pt));

	vec d = t.RandomPointInside(rng);
	assert(t.Contains(d));
	d = d - t.a;
	t.a -= d;
	t.b -= d;
	t.c -= d;

	assert1(t.IsFinite(), t);
	assert1(!t.IsDegenerate(), t);
	assert3(t.Contains(pt), t, pt, t.Distance(pt));
	return t;
}

Polyhedron RandomPolyhedronContainingPoint(const vec &pt)
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

//	assert1(t.IsFinite(), t);
//	assert1(!t.IsDegenerate(), t);
//	assert3(t.Contains(pt), t, pt, t.Distance(pt));
}

Polygon RandomPolygonContainingPoint(const vec &pt)
{
	Polyhedron p = RandomPolyhedronContainingPoint(pt);
	Polygon poly = p.FacePolygon(rng.Int(0, p.NumFaces()-1));

	vec pt2 = poly.FastRandomPointInside(rng);
	assert3(poly.Contains(pt2), poly, pt2, poly.Distance(pt2));
	poly.Translate(pt - pt2);

	assert1(!poly.IsDegenerate(), poly);
	assert1(!poly.IsNull(), poly);
	assert1(poly.IsPlanar(), poly);
	assert1(poly.IsFinite(), poly);
	assert3(poly.Contains(pt), poly, pt, poly.Distance(pt));

	return poly;
}

RANDOMIZED_TEST(AABBAABBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
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
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	KdTree<Triangle> t;
	std::vector<Triangle> tris = a.Triangulate();
	if (!tris.empty())
		t.AddObjects(&tris[0], (int)tris.size());
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

	vec point = POINT_VEC((xmax-xmin)/2,(ymax-ymin)/2,z);
	Polygon pol;
	pol.p.push_back(POINT_VEC(xmin, ymin, z));
	pol.p.push_back(POINT_VEC(xmax, ymin, z));
	pol.p.push_back(POINT_VEC(xmax, ymax, z));
	pol.p.push_back(POINT_VEC(xmin, ymax, z));

	assert(pol.Contains(point));
}
