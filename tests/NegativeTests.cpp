#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength)
{
	float w = rng.Float(1e-3f, maxSideLength);
	float h = rng.Float(1e-3f, maxSideLength);
	float d = rng.Float(1e-3f, maxSideLength);

	AABB a(DIR_VEC(0, 0, 0), DIR_VEC(w, h, d)); // Store with w == 0, the translate below will set the origin and w=1.

	vec origin = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	a.Translate(origin);

	vec aabbExtremePoint = a.ExtremePoint(-plane.normal);
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

	vec obbExtremePoint = o.ExtremePoint(-plane.normal);
	float distance = plane.Distance(obbExtremePoint);
	o.Translate((distance + GUARDBAND) * plane.normal);

	assert1(!o.IsDegenerate(), o);
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
	Sphere s(vec::zero, rng.Float(0.001f, maxRadius));
	s.Translate(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)));

	vec extremePoint = s.ExtremePoint(-plane.normal);
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
		f.SetOrthographic(rng.Float(0.001f, SCALE), rng.Float(0.001f, SCALE));
	}
	else
	{
		// Really random Frustum could have fov as ]0, pi[, but limit
		// to much narrower fovs to not cause the corner vertices
		// shoot too far when farPlaneDistance is very large.
		f.SetPerspective(rng.Float(0.001f, 3.f*pi / 4.f), rng.Float(0.001f, 3.f*pi / 4.f));
	}
	float nearPlaneDistance = rng.Float(0.1f, SCALE);
	f.SetViewPlaneDistances(nearPlaneDistance, nearPlaneDistance + rng.Float(0.1f, SCALE));
	vec front = vec::RandomDir(rng);
	f.SetFrame(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)),
		front,
		front.RandomPerpendicular(rng));

	f.SetKind((rng.Int(0, 1) == 1) ? FrustumSpaceD3D : FrustumSpaceGL, (rng.Int(0, 1) == 1) ? FrustumRightHanded : FrustumLeftHanded);

//	assert(!f.IsDegenerate());

	vec extremePoint = f.ExtremePoint(-plane.normal);
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
	vec linePos = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	if (plane.SignedDistance(linePos) < 0.f)
		linePos = plane.Mirror(linePos);
	linePos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(linePos) >= 1e-3f);

	vec dir = plane.normal.RandomPerpendicular(rng);
	Line l(linePos, dir);
	assert(l.IsFinite());
	assert2(!plane.Intersects(l), plane.SerializeToCodeString(), l.SerializeToCodeString());
	assert1(plane.SignedDistance(l) > 0.f, plane.SignedDistance(l));
	return l;
}

Ray RandomRayInHalfspace(const Plane &plane)
{
	if (rng.Int(0, 10) == 0)
		return RandomLineInHalfspace(plane).ToRay();

	vec rayPos = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	if (plane.SignedDistance(rayPos) < 0.f)
		rayPos = plane.Mirror(rayPos);
	rayPos += plane.normal * 1e-2f;
	assert(plane.SignedDistance(rayPos) >= 1e-3f);

	vec dir = vec::RandomDir(rng);
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
	float f = rng.Float(0.f, SCALE);
	LineSegment ls = RandomRayInHalfspace(plane).ToLineSegment(0.f, f);
	assert(ls.IsFinite());
	assert(plane.SignedDistance(ls) > 0.f);

	return ls;
}

Capsule RandomCapsuleInHalfspace(const Plane &plane)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec dir = vec::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	float r = rng.Float(0.001f, SCALE);
	Capsule c(pt + a*dir, pt - b*dir, r);
	assert(c.IsFinite());

	vec extremePoint = c.ExtremePoint(-plane.normal);
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
	vec a = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec b = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec c = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle t(a,b,c);

	assert1(t.IsFinite(), t);
	assert1(!t.IsDegenerate(), t);

	vec extremePoint = t.ExtremePoint(-plane.normal);
	float distance = plane.Distance(extremePoint);
	t.Translate((distance + GUARDBAND) * plane.normal);

	assert1(t.IsFinite(), t);
	assert1(!t.IsDegenerate(), t);
	assert2(!t.Intersects(plane), t, plane);
//	assert(t.SignedDistance(plane) > 0.f);
	extremePoint = t.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(t) > 0.f);

	return t;
}

Polyhedron RandomPolyhedronInHalfspace(const Plane &plane)
{
	Polyhedron p;
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));

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

	vec extremePoint = p.ExtremePoint(-plane.normal);
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

	assert1(!poly.IsDegenerate(), poly);
	assert1(!poly.IsNull(), poly);
	assert1(poly.IsPlanar(), poly);
	assert1(poly.IsFinite(), poly);
	assert2(!poly.Intersects(plane), poly, plane);
	vec extremePoint = poly.ExtremePoint(-plane.normal);
	assert(plane.SignedDistance(extremePoint) > 0.f);
	assert(plane.SignedDistance(poly) > 0.f);

	return poly;
}

RANDOMIZED_TEST(AABBAABBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));

	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	AABB b = RandomAABBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBOBBNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(AABBLineNoIntersectCase)
{
	Plane p(DIR_VEC(-0.72379446f,0.652315021f,-0.224959269f),27.2405319f);
	Line l(POINT_VEC(-16.0996895f,22.3687477f,-5.59284782f),DIR_VEC(-0.616314948f,-0.757768512f,-0.214342773f));
	assert(!p.Intersects(l));
}

RANDOMIZED_TEST(AABBLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	if (a.Intersects(b))
	{
		LOGI("AABB: %s", a.ToString().c_str());
		LOGI("Line: %s", b.ToString().c_str());
	}
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolyhedronNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(AABBPolygonNoIntersectCase)
{
	AABB a(POINT_VEC(-1.07812309f,50.0289841f,-42.3423996f),POINT_VEC(4.804286f,50.5800514f,-42.3384552f));
	Polygon b;
	b.p.push_back(POINT_VEC(5.48830986f,-108.176254f,84.0496674f));
	b.p.push_back(POINT_VEC(5.48830986f,-69.97966f,84.0496674f));
	b.p.push_back(POINT_VEC(24.5866089f,-58.1762543f,114.95137f));
	b.p.push_back(POINT_VEC(36.3900108f,-89.0779572f,134.049667f));
	b.p.push_back(POINT_VEC(24.5866089f,-119.97966f,114.95137f));
	assert(!a.Intersects(b));
}

RANDOMIZED_TEST(AABBPolygonNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a.SerializeToCodeString(), b.SerializeToString());
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

extern int xxxxx;

BENCHMARK(OBBOBBNoIntersect, "OBB-OBB No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!a.Intersects(b))
		++xxxxx;
	if (!b.Intersects(a))
		++xxxxx;
}
BENCHMARK_END;

BENCHMARK(OBBOBBNoIntersect_SAT, "OBB-OBB SAT No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!SATIntersect(a, b))
		++xxxxx;
	if (!SATIntersect(b, a))
		++xxxxx;
}
BENCHMARK_END;

BENCHMARK(OBBOBBNoIntersect_GJK, "OBB-OBB GJK No Intersection")
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	OBB b = RandomOBBInHalfspace(p, 10.f);
	if (!GJKIntersect(a, b))
		++xxxxx;
	if (!GJKIntersect(b, a))
		++xxxxx;
}
BENCHMARK_END;

RANDOMIZED_TEST(OBBLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, SCALE);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolyhedronNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	OBB a = RandomOBBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Sphere b = RandomSphereInHalfspace(p, 10.f);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumFrustumNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

extern int xxxxx;

BENCHMARK(BM_FrustumFrustumNoIntersect, "Frustum-Frustum No Intersection")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
		Frustum a = RandomFrustumInHalfspace(p);
		p.ReverseNormal();
		Frustum b = RandomFrustumInHalfspace(p);

		if (!a.Intersects(b))
			++xxxxx;
		if (!b.Intersects(a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_FrustumFrustumNoIntersect_SAT, "Frustum-Frustum SAT No Intersection")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);

	if (!SATIntersect(a, b))
		++xxxxx;
	if (!SATIntersect(b, a))
		++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_FrustumFrustumNoIntersect_GJK, "Frustum-Frustum GJK No Intersection")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
		Frustum a = RandomFrustumInHalfspace(p);
		p.ReverseNormal();
		Frustum b = RandomFrustumInHalfspace(p);

		if (!GJKIntersect(a, b))
			++xxxxx;
		if (!GJKIntersect(b, a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

RANDOMIZED_TEST(FrustumPolyhedronNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Frustum a = RandomFrustumInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}


UNIQUE_TEST(PlaneLineNoIntersectCase)
{
	Plane p(DIR_VEC(-0.746312618f,0.586626351f,-0.31446299f),-35.7190437f);
	Line l(POINT_VEC(45.1519928f,46.7459641f,92.9752197f),DIR_VEC(0.631202042f,0.773685277f,-0.0547275133f));
	assert(!p.Intersects(l));
}

UNIQUE_TEST(PlaneLineNoIntersectCase2)
{
	Plane p(DIR_VEC(0.344275832f,-0.882686555f,0.31990397f),-56.7400818f);
	Line l(POINT_VEC(36.2179184f,88.9618607f,29.178812f),DIR_VEC(0.775070965f,0.459497392f,0.433736295f));
	assert(!p.Intersects(l));
}

RANDOMIZED_TEST(CapsuleLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Capsule b = RandomCapsuleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Capsule a = RandomCapsuleInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Distance(a.ClosestPoint(b)) < 1e-3f, a, b, a.ClosestPoint(b), a.Distance(a.ClosestPoint(b)));
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polyhedron.
//	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Polyhedron b = RandomPolyhedronInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Distance(a.ClosestPoint(b)) < 1e-3f, a, b, a.ClosestPoint(b), a.Distance(a.ClosestPoint(b)));
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polygon.
//	assert(a.Contains(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polygon a = RandomPolygonInHalfspace(p);
	p.ReverseNormal();
	Polygon b = RandomPolygonInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
//	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
//	assert(!b.Intersects(a));


//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TrianglePlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TrickyTriangleTriangleSATNoIntersect)
{
	Triangle a, b;
	a.a = POINT_VEC(-19.234608f, 175.11060f, -65.702095f);
	a.b = POINT_VEC(-41.996265f, 86.449524f, 61.047047f);
	a.c = POINT_VEC(-40.166401f, 83.222511f, -7.1153078f);
	b.a = POINT_VEC(-23.087940f, 13.051629f, -21.682280f);
	b.b = POINT_VEC(-90.890396f, -61.371635f, -44.296501f);
	b.c = POINT_VEC(85.991585f, 38.734276f, 29.707987f);
	assert(!a.Intersects(b));
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

}

RANDOMIZED_TEST(TriangleTriangleNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Triangle a = RandomTriangleInHalfspace(p);
	p.ReverseNormal();
	Triangle b = RandomTriangleInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));


	assert(!SATIntersect(a, b));

//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
	assert(!a.Contains(b.ClosestPoint(a)));
	assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(PlaneLineNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Line b = RandomLineInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
///	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
//	assert(a.Contains(a.ClosestPoint(b)));
//	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TrickyPlaneRayClosestPoint)
{
	Plane p(DIR_VEC(0.50561136f,-0.753886223f,0.419538707f),83.6198273f);
	Ray r(POINT_VEC(-34.2622185f,97.5630875f,14.6553354f),DIR_VEC(-0.433765054f,-0.636341155f,-0.637901187f));
	vec cp = p.ClosestPoint(r);
	assert(p.Contains(cp));
}

RANDOMIZED_TEST(PlaneRayNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
//	assert(a.Distance(b) > 0.f);
//	assert(b.Distance(a) > 0.f);
	assert3(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString());
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneLineSegmentNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	LineSegment b = RandomLineSegmentInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));
	assert(a.Distance(b) > 0.f);
	assert(b.Distance(a) > 0.f);
	assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	assert(!b.Contains(a.ClosestPoint(b)));
//	assert(!a.Contains(b.ClosestPoint(a)));
//	assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlanePlaneNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Plane a = RandomPlaneInHalfspace(p);
	p.ReverseNormal();
	Plane b = RandomPlaneInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
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
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	TriangleMesh tm;
	tm.SetConvex(a);
	p.ReverseNormal();
	Ray b = RandomRayInHalfspace(p);
	float d = tm.IntersectRay(b);
	assert(d == FLOAT_INF);
	MARK_UNUSED(d);
}

RANDOMIZED_TEST(RayKdTreeNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Polyhedron a = RandomPolyhedronInHalfspace(p);
	KdTree<Triangle> t;
	TriangleArray tris = a.TriangulateConvex();
	if (!tris.empty())
		t.AddObjects((Triangle*)&tris[0], (int)tris.size());
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

// https://github.com/juj/MathGeoLib/issues/55
UNIQUE_TEST(AABB_Capsule_NoIntersect_Case)
{
	vec minPoint = POINT_VEC(438.420929f, 805.586670f, 493.709167f);
	vec maxPoint = POINT_VEC(443.420929f, 810.586670f, 498.709167f);
	AABB aabb(minPoint, maxPoint);
	
	vec a = POINT_VEC(479.665222f,  -30.f, 509.737244f);
	vec b = POINT_VEC(479.665222f, 1530.f, 509.737244f);
	
	Capsule cylinder(a, b, 37.6882935f);
	
	assert(!cylinder.Intersects(aabb));
}

// https://github.com/juj/MathGeoLib/issues/55
UNIQUE_TEST(AABB_Capsule_NoIntersect_Case_2)
{
	// Slightly simplified numbers from above
	vec minPoint = POINT_VEC(438.f,        0.f, 493.f);
	vec maxPoint = POINT_VEC(443.420929f, 10.f, 499.f);
	AABB aabb(minPoint, maxPoint);
	
	vec a = POINT_VEC(479.665222f,  0.f, 509.737244f);
	vec b = POINT_VEC(479.665222f, 10.f, 509.737244f);
	
	Capsule cylinder(a, b, 37.6882935f);
	
	assert(!cylinder.Intersects(aabb));
}

// https://github.com/juj/MathGeoLib/issues/56
UNIQUE_TEST(AABB_Capsule_NoIntersect_Case_3)
{
	vec minPoint = POINT_VEC(1549.69849f, 639.203125f, 1115.08582f);
	vec maxPoint = POINT_VEC(1554.69849f, 644.203125f, 1120.08582f);
	AABB aabb(minPoint, maxPoint);

	vec a = POINT_VEC(1072.35388f, 1130.68018f, -477.690887f);
	vec b = POINT_VEC(1699.54614f, 490.913147f, 1637.29395f);

	Capsule cylinder(a, b, 4.36077833f);

	// TODO: The Cylinder and AABB should not intersect, but the GJK algorithm
	// does not produce the right result.
	assert(!cylinder.Intersects(aabb));
}
