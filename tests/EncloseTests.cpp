#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "ObjectGenerators.h"

vec RandomPointNearOrigin(float maxDistanceFromOrigin)
{
	return Sphere(POINT_VEC_SCALAR(0.f), maxDistanceFromOrigin).RandomPointInside(rng);
}

AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength);
AABB RandomAABBNearOrigin(float maxDistanceFromOrigin, float maxSideLength)
{
	return RandomAABBContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin), maxSideLength);
}

OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength);
OBB RandomOBBNearOrigin(float maxDistanceFromOrigin, float maxSideLength)
{
	return RandomOBBContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin), maxSideLength);
}

LineSegment RandomLineSegmentNearOrigin(float maxDistanceFromOrigin)
{
	return LineSegment(RandomPointNearOrigin(maxDistanceFromOrigin), RandomPointNearOrigin(maxDistanceFromOrigin));
}

Sphere RandomSphereContainingPoint(const vec &pt, float maxRadius);
Sphere RandomSphereNearOrigin(float maxDistanceFromOrigin, float maxRadius)
{
	return RandomSphereContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin), maxRadius);
}

Capsule RandomCapsuleContainingPoint(const vec &pt);
Capsule RandomCapsuleNearOrigin(float maxDistanceFromOrigin)
{
	return RandomCapsuleContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin));
}

Triangle RandomTriangleNearOrigin(float maxDistanceFromOrigin)
{
	return Triangle(RandomPointNearOrigin(maxDistanceFromOrigin), RandomPointNearOrigin(maxDistanceFromOrigin), RandomPointNearOrigin(maxDistanceFromOrigin));
}

Frustum RandomFrustumNearOrigin(float maxDistanceFromOrigin)
{
	return RandomFrustumContainingPoint(rng, RandomPointNearOrigin(maxDistanceFromOrigin));
}

Polygon RandomPolygonContainingPoint(const vec &pt);
Polygon RandomPolygonNearOrigin(float maxDistanceFromOrigin)
{
	return RandomPolygonContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin));
}

Polyhedron RandomPolyhedronContainingPoint(const vec &pt);
Polyhedron RandomPolyhedronNearOrigin(float maxDistanceFromOrigin)
{
	return RandomPolyhedronContainingPoint(RandomPointNearOrigin(maxDistanceFromOrigin));
}

#define DISTSCALE 1e2f
#define SIZESCALE 1e1f

RANDOMIZED_TEST(AABB_Enclose_point)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	vec pt = RandomPointNearOrigin(DISTSCALE);
	aabb.Enclose(pt);

	assert(aabb.Contains(pt));
}

/** @bug Improve numerical stability of the test with epsilon and enable this test. 
RANDOMIZED_TEST(OBB_Enclose_point)
{
	OBB obb = RandomOBBNearOrigin(DISTSCALE, SIZESCALE);
	vec pt = RandomPointNearOrigin(DISTSCALE);
	obb.Enclose(pt);

	assert(obb.Contains(pt));
} */

RANDOMIZED_TEST(AABB_Enclose_LineSegment)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	LineSegment ls = RandomLineSegmentNearOrigin(DISTSCALE);
	aabb.Enclose(ls);

	assert(aabb.Contains(ls));
}

RANDOMIZED_TEST(AABB_Enclose_AABB)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	AABB aabb2 = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	aabb.Enclose(aabb2);

	assert(aabb.Contains(aabb2));
}

/** @bug Improve numerical stability of the test with epsilon and enable this test. 
RANDOMIZED_TEST(AABB_Enclose_OBB)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	OBB obb = RandomOBBNearOrigin(DISTSCALE, SIZESCALE);
	aabb.Enclose(obb);

	assert(aabb.Contains(obb));
} */

RANDOMIZED_TEST(AABB_Enclose_Sphere)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	aabb.Enclose(s);

	assert(aabb.Contains(s));
}

RANDOMIZED_TEST(AABB_Enclose_Triangle)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Triangle t = RandomTriangleNearOrigin(DISTSCALE);
	aabb.Enclose(t);

	assert(aabb.Contains(t));
}

RANDOMIZED_TEST(AABB_Enclose_Capsule)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Capsule c = RandomCapsuleNearOrigin(DISTSCALE);
	aabb.Enclose(c);

	assert(aabb.Contains(c));
}

RANDOMIZED_TEST(AABB_Enclose_Frustum)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Frustum f = RandomFrustumNearOrigin(DISTSCALE);
	aabb.Enclose(f);

	assert(aabb.Contains(f));
}

RANDOMIZED_TEST(AABB_Enclose_Polygon)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Polygon p = RandomPolygonNearOrigin(DISTSCALE);
	aabb.Enclose(p);

	assert2(aabb.Contains(p), aabb, p);
}

RANDOMIZED_TEST(AABB_Enclose_Polyhedron)
{
	AABB aabb = RandomAABBNearOrigin(DISTSCALE, SIZESCALE);
	Polyhedron p = RandomPolyhedronNearOrigin(DISTSCALE);
	aabb.Enclose(p);

	assert2(aabb.Contains(p), aabb, p);
}

RANDOMIZED_TEST(Sphere_Enclose_point)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	vec pt = RandomPointNearOrigin(DISTSCALE);
	s.Enclose(pt);

	assert3(s.Contains(pt), s, pt, s.Distance(pt));
}

RANDOMIZED_TEST(Sphere_Enclose_LineSegment)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	LineSegment ls = RandomLineSegmentNearOrigin(DISTSCALE);
	s.Enclose(ls);

	assert(s.Contains(ls));
}

RANDOMIZED_TEST(Sphere_Enclose_AABB)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Sphere aabb = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	s.Enclose(aabb);

	assert(s.Contains(aabb));
}

RANDOMIZED_TEST(Sphere_Enclose_OBB)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	OBB obb = RandomOBBNearOrigin(DISTSCALE, SIZESCALE);
	s.Enclose(obb);

	assert(s.Contains(obb));
}

RANDOMIZED_TEST(Sphere_Enclose_Sphere)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Sphere s2 = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	s.Enclose(s2);

	assert(s.Contains(s2));
}

RANDOMIZED_TEST(Sphere_Enclose_Triangle)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Triangle t = RandomTriangleNearOrigin(DISTSCALE);
	s.Enclose(t);

	assert(s.Contains(t));
}

RANDOMIZED_TEST(Sphere_Enclose_Capsule)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Capsule c = RandomCapsuleNearOrigin(DISTSCALE);
	s.Enclose(c);

	assert2(s.Contains(c), s.SerializeToCodeString(), c.SerializeToCodeString());
}

RANDOMIZED_TEST(Sphere_Enclose_Frustum)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Frustum f = RandomFrustumNearOrigin(DISTSCALE);
	s.Enclose(f);

	assert2(s.Contains(f), s, f);
}

RANDOMIZED_TEST(Sphere_Enclose_Polygon)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Polygon p = RandomPolygonNearOrigin(DISTSCALE);
	s.Enclose(p);

	assert2(s.Contains(p), s, p);
}

RANDOMIZED_TEST(Sphere_Enclose_Polyhedron)
{
	Sphere s = RandomSphereNearOrigin(DISTSCALE, SIZESCALE);
	Polyhedron p = RandomPolyhedronNearOrigin(DISTSCALE);
	s.Enclose(p);

	assert2(s.Contains(p), s, p);
}
