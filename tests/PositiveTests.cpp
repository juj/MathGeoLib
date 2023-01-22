#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

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
	mgl_assert(!a.IsDegenerate());
	mgl_assert(a.IsFinite());
	mgl_assert(a.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(a.minPoint.w, 1.f);
	mgl_asserteq(a.maxPoint.w, 1.f);
#endif
	return a;
}

OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength)
{
	float w = rng.Float(1e-2f, maxSideLength);
	float h = rng.Float(1e-2f, maxSideLength);
	float d = rng.Float(1e-2f, maxSideLength);
	float3x4 rot = float3x4::RandomRotation(rng);
	OBB o;
	o.pos = pt;
	o.axis[0] = DIR_VEC(rot.Col(0));
	o.axis[1] = DIR_VEC(rot.Col(1));
	o.axis[2] = DIR_VEC(rot.Col(2));
	mgl_assume2(o.axis[0].IsNormalized(), o.axis[0], o.axis[0].LengthSq());
	mgl_assume2(o.axis[1].IsNormalized(), o.axis[1], o.axis[1].LengthSq());
	mgl_assume2(o.axis[2].IsNormalized(), o.axis[2], o.axis[2].LengthSq());
	mgl_assume(vec::AreOrthogonal(o.axis[0], o.axis[1], o.axis[2]));
//	assume(vec::AreOrthonormal(o.axis[0], o.axis[1], o.axis[2]));
	o.r = DIR_VEC(w, h, d);
	const float epsilon = 1e-4f;
	o.pos += rng.Float(-w+epsilon, w-epsilon) * o.axis[0];
	o.pos += rng.Float(-h+epsilon, h-epsilon) * o.axis[1];
	o.pos += rng.Float(-d+epsilon, d-epsilon) * o.axis[2];
	mgl_assert1(!o.IsDegenerate(), o);
	mgl_assert(o.IsFinite());
	mgl_assert(o.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(o.pos.w, 1.f);
	mgl_asserteq(o.r.w, 0.f);
	mgl_asserteq(o.axis[0].w, 0.f);
	mgl_asserteq(o.axis[1].w, 0.f);
	mgl_asserteq(o.axis[2].w, 0.f);
#endif
	return o;
}

Sphere RandomSphereContainingPoint(const vec &pt, float maxRadius)
{
	Sphere s(pt, rng.Float(1.f, maxRadius));
	s.pos += vec::RandomDir(rng, Max(0.f, s.r - 1e-2f));
	mgl_assert(s.IsFinite());
	mgl_assert(!s.IsDegenerate());
	mgl_assert(s.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(s.pos.w, 1.f);
#endif
	return s;
}

Circle2D RandomCircle2DContainingPoint(LCG &lcg, const float2 &pt, float maxRadius)
{
	Circle2D c(pt, lcg.Float(0.1f, Max(0.1f, maxRadius)));
	c.pos += float2::RandomDir(lcg, Max(0.f, c.r - 1e-2f));
#if 0
    // Generate easy test cases
    c.pos.x = (float)(int)c.pos.x;
    c.pos.y = (float)(int)c.pos.y;
    if (!c.Contains(pt))
        return RandomCircle2DContainingPoint(pt, maxRadius);
#endif
	mgl_assert(c.IsFinite());
	mgl_assert(!c.IsDegenerate());
	mgl_assert(c.Contains(pt));
	return c;
}

Frustum RandomFrustumContainingPoint(LCG &lcg, const vec &pt)
{
	Frustum f;
	f.SetKind((lcg.Int(0, 1) == 1) ? FrustumSpaceD3D : FrustumSpaceGL, (lcg.Int(0, 1) == 1) ? FrustumRightHanded : FrustumLeftHanded);

	if (lcg.Int(0,1))
	{
		f.SetOrthographic(lcg.Float(0.1f, SCALE), lcg.Float(0.1f, SCALE));
	}
	else
	{
		// Really random Frustum could have fov as ]0, pi[, but limit
		// to much narrower fovs to not cause the corner vertices
		// shoot too far when farPlaneDistance is very large.
		f.SetPerspective(lcg.Float(0.1f, 3.f*pi / 4.f), lcg.Float(0.1f, 3.f*pi / 4.f));
	}
	float nearPlaneDistance = lcg.Float(0.1f, SCALE);
	f.SetViewPlaneDistances(nearPlaneDistance, nearPlaneDistance + lcg.Float(0.1f, SCALE));
	vec front = vec::RandomDir(lcg);
	f.SetFrame(POINT_VEC_SCALAR(0.f),
		front,
		front.RandomPerpendicular(lcg));

	vec pt2 = f.UniformRandomPointInside(lcg);
	f.SetPos(f.Pos() + pt - pt2);

	mgl_assert(f.IsFinite());
//	mgl_assert(!f.IsDegenerate());
	mgl_assert(f.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(f.Pos().w, 1.f);
	mgl_asserteq(f.Front().w, 0.f);
	mgl_asserteq(f.Up().w, 0.f);
#endif
	return f;
}

Line RandomLineContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	Line l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, SCALE));
	mgl_assert(l.IsFinite());
	mgl_assert(l.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(l.pos.w, 1.f);
	mgl_asserteq(l.dir.w, 0.f);
#endif
	return l;
}

Ray RandomRayContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	Ray l(pt, dir);
	l.pos = l.GetPoint(rng.Float(-SCALE, 0));
	mgl_assert(l.IsFinite());
	mgl_assert(l.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(l.pos.w, 1.f);
	mgl_asserteq(l.dir.w, 0.f);
#endif
	return l;
}

LineSegment RandomLineSegmentContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	float a = rng.Float(0, SCALE);
	float b = rng.Float(0, SCALE);
	LineSegment l(pt + a*dir, pt - b*dir);
	mgl_assert(l.IsFinite());
	mgl_assert(l.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(l.a.w, 1.f);
	mgl_asserteq(l.b.w, 1.f);
#endif
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
	mgl_assert(c.IsFinite());
	mgl_assert(c.Contains(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(c.l.a.w, 1.f);
	mgl_asserteq(c.l.b.w, 1.f);
#endif

	return c;
}

Plane RandomPlaneContainingPoint(const vec &pt)
{
	vec dir = vec::RandomDir(rng);
	mgl_assume2(dir.IsNormalized(), dir.SerializeToCodeString(), dir.Length());
	Plane p(pt, dir);
	mgl_assert(!p.IsDegenerate());
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(p.normal.w, 0.f);
#endif
	return p;
}

Triangle RandomTriangleContainingPoint(const vec &pt)
{
	Triangle t;
	t.a = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	t.b = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	t.c = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	vec p = t.RandomPointInside(rng);
	t.a -= p;
	t.b -= p;
	t.c -= p;
	t.a += pt;
	t.b += pt;
	t.c += pt;

	mgl_assert1(t.IsFinite(), t);
	mgl_assert1(!t.IsDegenerate(), t);
	mgl_assert3(t.Contains(pt), t.SerializeToCodeString(), pt.SerializeToCodeString(), t.Distance(pt));
#ifdef MATH_AUTOMATIC_SSE
	mgl_asserteq(t.a.w, 1.f);
	mgl_asserteq(t.b.w, 1.f);
	mgl_asserteq(t.c.w, 1.f);
#endif
	return t;
}

Polyhedron RandomPolyhedronContainingPoint(const vec &pt)
{
	Polyhedron p;
	switch(rng.Int(0,7))
	{
	case 0: p = RandomAABBContainingPoint(pt, SCALE).ToPolyhedron(); break;
	case 1: p = RandomOBBContainingPoint(pt, SCALE).ToPolyhedron(); break;
	case 2: p = RandomFrustumContainingPoint(rng, pt).ToPolyhedron(); break;
	case 3: p = Polyhedron::Tetrahedron(pt, SCALE); break;
	case 4: p = Polyhedron::Octahedron(pt, SCALE); break;
	case 5: p = Polyhedron::Hexahedron(pt, SCALE); break;
	case 6: p = Polyhedron::Icosahedron(pt, SCALE); break;
	default: p = Polyhedron::Dodecahedron(pt, SCALE); break;
	}
#ifdef MATH_AUTOMATIC_SSE
	for (int i = 0; i < p.NumVertices(); ++i)
		mgl_asserteq(p.Vertex(i).w, 1.f);
#endif
	return p;
//	mgl_assert1(t.IsFinite(), t);
//	mgl_assert1(!t.IsDegenerate(), t);
//	mgl_assert3(t.Contains(pt), t.SerializeToCodeString(), pt.SerializeToCodeString(), t.Distance(pt));
}

UNIQUE_TEST(Polygon_Contains_PointCase)
{
	Polygon p;

	p.p.push_back(POINT_VEC(1.f, 0.f, 1.f));
	p.p.push_back(POINT_VEC(1.f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f, 0.f, 0.f));
	p.p.push_back(POINT_VEC(0.f, 0.f, 1.f));
	vec pt = POINT_VEC(0.5f, 0.f, 0.0007f);

	mgl_assert(p.Contains(pt));
}

Polygon RandomPolygonContainingPoint(const vec &pt)
{
	Polyhedron p = RandomPolyhedronContainingPoint(pt);
	Polygon poly = p.FacePolygon(rng.Int(0, p.NumFaces()-1));

	vec pt2 = poly.FastRandomPointInside(rng);
	mgl_assert3(poly.Contains(pt2), poly.SerializeToString(), pt2.SerializeToString(), poly.Distance(pt2));
	poly.Translate(pt - pt2);

	mgl_assert1(!poly.IsDegenerate(), poly);
	mgl_assert1(!poly.IsNull(), poly);
	mgl_assert1(poly.IsPlanar(), poly);
	mgl_assert1(poly.IsFinite(), poly);
	mgl_assert3(poly.Contains(pt), poly, pt, poly.Distance(pt));
#ifdef MATH_AUTOMATIC_SSE
	for (int i = 0; i < poly.NumVertices(); ++i)
		mgl_asserteq(poly.Vertex(i).w, 1.f);
#endif

	return poly;
}

RANDOMIZED_TEST(AABBAABBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	mgl_assert2(b.Intersects(a), b.SerializeToCodeString(), a.SerializeToCodeString());
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TrickyAABBLineSegmentIntersectGJK)
{
	AABB a;
	a.minPoint = POINT_VEC(-60.895836f, 18.743414f, -17.829493f);
	a.maxPoint = POINT_VEC(-60.294441f, 23.510536f, -10.694467f);
	LineSegment b;
	b.a = POINT_VEC(-61.331539f, 16.955204f, -18.561975f);
	b.b = POINT_VEC(-53.097103f, 40.628937f, 30.422394f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


}

RANDOMIZED_TEST(AABBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);


//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(AABBPolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(AABBPolygonIntersectCase)
{
	AABB a(POINT_VEC(-6.86657524f,-87.7668533f,-40.2900276f),
		POINT_VEC(2.77308559f,-79.9921722f,-34.0750961f));
	Polygon b;
	b.p.push_back(POINT_VEC(-43.6267548f,-19.4667053f,-35.8499298f));
	b.p.push_back(POINT_VEC(56.3732452f,-19.4667053f,-35.8499298f));
	b.p.push_back(POINT_VEC(56.3732452f,-119.466698f,-35.8499298f));
	b.p.push_back(POINT_VEC(-43.6267548f,-119.466698f,-35.8499298f));
	mgl_assert(a.Intersects(b));
}

UNIQUE_TEST(AABBPolygonIntersectCase2)
{
	AABB a(POINT_VEC(-23.4495525f,-25.5456314f,82.2886734f),
		POINT_VEC(-14.9134054f,-22.428545f,87.317482f));
	Polygon b;
	b.p.push_back(POINT_VEC(74.7439194f,-20.989212f,181.39743f));
	b.p.push_back(POINT_VEC(-25.2560806f,-20.989212f,81.3974304f));
	b.p.push_back(POINT_VEC(74.7439194f,-120.989212f,81.3974304f));
	mgl_assert(a.Intersects(b));
}

RANDOMIZED_TEST(AABBPolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToString());
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(OBBOBBIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

int xxxxx = 0;

BENCHMARK(BM_OBBOBBIntersect, "OBB-OBB Intersects")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
		OBB a = RandomOBBContainingPoint(pt, 10.f);
		OBB b = RandomOBBContainingPoint(pt, 10.f);
		if (a.Intersects(b))
			++xxxxx;
		if (b.Intersects(a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_OBBOBBIntersect_SAT, "OBB-OBB SAT Intersection")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
		OBB a = RandomOBBContainingPoint(pt, 10.f);
		OBB b = RandomOBBContainingPoint(pt, 10.f);
		if (SATIntersect(a, b))
			++xxxxx;
		if (SATIntersect(b, a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_OBBOBBIntersect_GJK, "OBB-OBB GJK Intersection")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
		OBB a = RandomOBBContainingPoint(pt, 10.f);
		OBB b = RandomOBBContainingPoint(pt, 10.f);
		if (GJKIntersect(a, b))
			++xxxxx;
		if (GJKIntersect(b, a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

RANDOMIZED_TEST(OBBLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
////	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(OBBPolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	OBB a = RandomOBBContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
///	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(SphereSphereIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, 10.f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SphereFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(SpherePolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(FrustumLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumFrustumIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

BENCHMARK(BM_FrustumFrustumIntersect, "Frustum-Frustum Intersects")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
		Frustum a = RandomFrustumContainingPoint(rng, pt);
		Frustum b = RandomFrustumContainingPoint(rng, pt);
		if (a.Intersects(b))
			++xxxxx;
		if (b.Intersects(a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_FrustumFrustumIntersect_SAT, "Frustum-Frustum SAT")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Frustum b = RandomFrustumContainingPoint(rng, pt);
	if (SATIntersect(a, b))
		++xxxxx;
	if (SATIntersect(b, a))
		++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

BENCHMARK(BM_FrustumFrustumIntersect_GJK, "Frustum-Frustum GJK")
{
#ifdef FAIL_USING_EXCEPTIONS
	try { // Ignore failures in this benchmark.
#endif
		vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
		Frustum a = RandomFrustumContainingPoint(rng, pt);
		Frustum b = RandomFrustumContainingPoint(rng, pt);
		if (GJKIntersect(a, b))
			++xxxxx;
		if (GJKIntersect(b, a))
			++xxxxx;
#ifdef FAIL_USING_EXCEPTIONS
	} catch(...) {};
#endif
}
BENCHMARK_END;

RANDOMIZED_TEST(FrustumPolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(FrustumPolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum a = RandomFrustumContainingPoint(rng, pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(CapsuleLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleCapsuleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Capsule b = RandomCapsuleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsuleTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
///	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
///	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(CapsulePolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule a = RandomCapsuleContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}





RANDOMIZED_TEST(PolyhedronLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Distance(a.ClosestPoint(b)) < 1e-3f, a, b, a.ClosestPoint(b), a.Distance(a.ClosestPoint(b)));
//	TODO: The following is problematic due to numerical
//	stability issues at the surface of the Polyhedron.
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPolyhedronIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Polyhedron b = RandomPolyhedronContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolyhedronPolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(PolygonLineIntersectCase)
{
	Polygon p;
	p.p.push_back(POINT_VEC(-19.0257339f,93.0070877f,20.232048f));
	p.p.push_back(POINT_VEC(-19.0257339f,131.203674f,20.232048f));
	p.p.push_back(POINT_VEC(0.0725631714f,143.00708f,51.1337509f));
	p.p.push_back(POINT_VEC(11.875967f,112.105385f,70.232048f));
	p.p.push_back(POINT_VEC(0.0725631714f,81.2036819f,51.1337509f));

	Line l(POINT_VEC(-51.2448387f,66.6799698f,-31.887619f),DIR_VEC(-0.494226635f,-0.341286302f,-0.799539685f));

	mgl_assert(p.Intersects(l));
}

RANDOMIZED_TEST(PolygonLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToString(), b.SerializeToCodeString());
	mgl_assert2(b.Intersects(a), b.SerializeToCodeString(), a.SerializeToString());
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Contains(a.ClosestPoint(b)), a, b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	mgl_assert3(b.Contains(a.ClosestPoint(b)), b.SerializeToCodeString(), a.SerializeToString(), b.Distance(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonPlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PolygonTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(PolygonPolygonIntersectCase)
{
	for(int i = 0; i < 2; ++i)
	{
		Triangle t1, t2;
		switch(i)
		{
		case 0:
			t1 = Triangle(POINT_VEC(0,0,0),
					POINT_VEC(1,0,0),
					POINT_VEC(0,1,0));
			t2 = Triangle(POINT_VEC(0.5f,   0, 1e-6f),
					POINT_VEC(1.5f,   0, 1e-6f),
					POINT_VEC(   0, 1.f, 1e-6f));
			break;
		case 1:
			t1 = Triangle(POINT_VEC(-18.8999062f,7.59376526f,25.2815018f),
				POINT_VEC(12.0017948f,26.6920624f,75.2815018f),
				POINT_VEC(-49.801609f,26.6920624f,75.2815018f));

			t2 = Triangle(POINT_VEC(-44.9369545f,11.4193268f,35.2969398f),
				POINT_VEC(-14.0352535f,30.5176239f,85.296936f),
				POINT_VEC(-75.8386536f,30.5176239f,85.296936f));
			break;
		}

		mgl_assert(t1.Intersects(t1));
		mgl_assert(t2.Intersects(t2));
		mgl_assert(t1.Intersects(t2));

		Polygon a = t1.ToPolygon();
		mgl_assert(a.Intersects(a));
		Polygon b = t2.ToPolygon();

		mgl_assert(a.Intersects(b));
	}
}

UNIQUE_TEST(PolygonPolygonIntersectCase2)
{
	Polygon a;
	a.p.push_back(POINT_VEC(40.6926041f,-36.1174965f, 0.f));
	a.p.push_back(POINT_VEC(40.6926041f,-9.93014526f, 0.f));
	a.p.push_back(POINT_VEC(43.8726807f,-9.93014526f, 0.f));
	a.p.push_back(POINT_VEC(43.8726807f,-36.1174965f, 0.f));

	Polygon b;
	b.p.push_back(POINT_VEC(70.9185791f,-46.9780273f, 0.f));
	b.p.push_back(POINT_VEC(70.9185791f, 53.0219727f, 0.f));
	b.p.push_back(POINT_VEC(-29.0814209f,53.0219727f, 0.f));
	b.p.push_back(POINT_VEC(-29.0814209f,-46.9780273f, 0.f));

	mgl_assert(a.Intersects(b));
}

UNIQUE_TEST(PolygonContainsPointCase)
{
	Polygon a;
	a.p.push_back(POINT_VEC(-27.6082363f,-17.8272648f,116.150414f));
	a.p.push_back(POINT_VEC(15.0997639f,-67.2276688f,12.971736f));
	a.p.push_back(POINT_VEC(15.062994f,-67.2823105f,12.9826784f));
	a.p.push_back(POINT_VEC(-27.6450062f,-17.8819065f,116.161354f));

	vec pt = POINT_VEC(12.1201611f,-63.8624725f,20.105011f);
	mgl_assert(a.Contains(pt, 1e-2f));
}

RANDOMIZED_TEST(PolygonPolygonIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon a = RandomPolygonContainingPoint(pt);
	Polygon b = RandomPolygonContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToString(), b.SerializeToString());
	mgl_assert2(b.Intersects(a), b.SerializeToString(), a.SerializeToString());
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
///	mgl_assert(b.Contains(b.ClosestPoint(a)));
}



RANDOMIZED_TEST(TriangleLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
//	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	mgl_assert(b.Contains(a.ClosestPoint(b)));
	mgl_assert(a.Contains(b.ClosestPoint(a)));
	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(TriangleLineSegmentIntersectCase)
{
	Triangle a(POINT_VEC(-45.7166939f,-104.675713f,17.1150723f),POINT_VEC(-20.9888325f,-89.1524963f,-31.5042286f),POINT_VEC(-1.45244789f,-76.914505f,-69.9231262f));
	LineSegment b(POINT_VEC(-19.0950012f,-134.222748f,-33.7456589f),POINT_VEC(-52.5003471f,-49.3652039f,28.5405655f));

	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	vec cp = a.ClosestPoint(b);
	mgl_assert(a.Contains(cp));
//	mgl_assert(b.Contains(cp));
}

RANDOMIZED_TEST(TriangleLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert(a.Intersects(b));
//	mgl_assert(b.Intersects(a));


//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TrianglePlaneIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Plane b = RandomPlaneContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(TriangleTriangleIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle a = RandomTriangleContainingPoint(pt);
	Triangle b = RandomTriangleContainingPoint(pt);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));


	mgl_assert(SATIntersect(a, b));

//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	mgl_assert(b.Contains(a.ClosestPoint(b)));
	mgl_assert(a.Contains(b.ClosestPoint(a)));
	mgl_assert(b.Contains(b.ClosestPoint(a)));
}




RANDOMIZED_TEST(PlaneLineIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	Line b = RandomLineContainingPoint(pt);
	mgl_assert(a.Intersects(b));
///	mgl_assert(b.Intersects(a));
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(PlaneRayIntersectCase)
{
	Plane p(DIR_VEC(-0.25385046f,-0.518036366f,-0.816822112f),91.5489655f);
	Ray r(POINT_VEC(-70.5785141f,-19.6609783f,-77.6785507f),DIR_VEC(0.916250288f,0.141897082f,-0.374634057f));
	mgl_assert(p.Intersects(r));
}

RANDOMIZED_TEST(PlaneRayIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	Ray b = RandomRayContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	mgl_assert2(b.Intersects(a), a.SerializeToCodeString(), b.SerializeToCodeString());
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert2(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString());
	mgl_assert2(b.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString());
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(PlaneLineSegmentIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Plane a = RandomPlaneContainingPoint(pt);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	mgl_assert2(b.Intersects(a), b.SerializeToCodeString(), a.SerializeToCodeString());
	mgl_assert(a.Distance(b) == 0.f);
	mgl_assert(b.Distance(a) == 0.f);
	mgl_assert4(a.Contains(a.ClosestPoint(b)), a.SerializeToCodeString(), b.SerializeToCodeString(), a.ClosestPoint(b).SerializeToCodeString(), a.Distance(a.ClosestPoint(b)));
	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

UNIQUE_TEST(PlanePlaneIntersectCase)
{
	Plane a(DIR_VEC(-9.31284958e-005f,0.896122217f,-0.44380734f).Normalized(),-63.5531387f);
	Plane b(DIR_VEC(0.0797545761f,-0.9964259f,0.0185146127f).Normalized(),45.0416794f);
	mgl_assert(a.Intersects(b));
	mgl_assert(b.Intersects(a));
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
	mgl_assert2(a.Intersects(b), a.SerializeToCodeString(), b.SerializeToCodeString());
	mgl_assert2(b.Intersects(a), b.SerializeToCodeString(), a.SerializeToCodeString());
//	mgl_assert(a.Distance(b) == 0.f);
//	mgl_assert(b.Distance(a) == 0.f);
//	mgl_assert(a.Contains(a.ClosestPoint(b)));
//	mgl_assert(b.Contains(a.ClosestPoint(b)));
//	mgl_assert(a.Contains(b.ClosestPoint(a)));
//	mgl_assert(b.Contains(b.ClosestPoint(a)));
}

RANDOMIZED_TEST(RayTriangleMeshIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	TriangleMesh tm;
	tm.SetConvex(a);
	Ray b = RandomRayContainingPoint(pt);
	float d = tm.IntersectRay(b);
	MARK_UNUSED(d);
	mgl_assert(d >= 0.f);
	mgl_assert(IsFinite(d));
}

RANDOMIZED_TEST(RayKdTreeIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron a = RandomPolyhedronContainingPoint(pt);
	KdTree<Triangle> t;
	TriangleArray tris = a.TriangulateConvex();
	if (!tris.empty())
		t.AddObjects((Triangle*)&tris[0], (int)tris.size());
	t.Build();
	Ray b = RandomRayContainingPoint(pt);
	TriangleKdTreeRayQueryNearestHitVisitor result;
	t.RayQuery(b, result);
	mgl_assert(result.rayT >= 0.f);
	mgl_assert(result.rayT < FLOAT_INF);
	mgl_assert(result.triangleIndex != KdTree<Triangle_storage>::BUCKET_SENTINEL);
	mgl_assert(result.pos.IsFinite());
	mgl_assert(result.barycentricUV.IsFinite());
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

	mgl_assert(pol.Contains(point));
}

#if 0
UNIQUE_TEST(PolygonContainsPointCase2)
{
	Polygon p;
	p.p.push_back(POINT_VEC(0,0,0));
	p.p.push_back(POINT_VEC(2.f,0,0));
	p.p.push_back(POINT_VEC(2.f,0.0646286f,0));
	p.p.push_back(POINT_VEC(0,0.0646286f,0));

	vec pt = POINT_VEC(1.f,0.0645294f,0);

	mgl_assert(p.Contains(pt));
}
#endif
