#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"

LCG rng(Clock::Tick());

#define SCALE 1e2f

AABB RandomAABBContainingPoint(const float3 &pt, float maxSideLength)
{
	float w = rng.Float(0, maxSideLength);
	float h = rng.Float(0, maxSideLength);
	float d = rng.Float(0, maxSideLength);

	AABB a(float3(0,0,0), float3(w,h,d));
	w = rng.Float(0, w);
	h = rng.Float(0, h);
	d = rng.Float(0, d);
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
	Sphere s(pt, rng.Float(0.001f, maxRadius));
	s.pos += float3::RandomSphere(rng, float3::zero, s.r);
	assert(s.IsFinite());
	assert(!s.IsDegenerate());
	assert(s.Contains(pt));
	return s;
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

void TestAABBAABBIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	AABB b = RandomAABBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
//	assert(a.Distance(b) == 0.f);
}

void TestAABBOBBIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	pt = float3::zero;
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	OBB b = RandomOBBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
}

void TestAABBLineIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Line b = RandomLineContainingPoint(pt);
	assert(a.Intersects(b));
}

void TestAABBRayIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Ray b = RandomRayContainingPoint(pt);
	assert(a.Intersects(b));
}

void TestAABBLineSegmentIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	LineSegment b = RandomLineSegmentContainingPoint(pt);
	assert(a.Intersects(b));
}

void TestAABBPlaneIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Plane b = RandomPlaneContainingPoint(pt);
	assert(a.Intersects(b));
}

void TestAABBSphereIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Sphere b = RandomSphereContainingPoint(pt, SCALE);
	assert(a.Intersects(b));
}

void TestAABBCapsuleIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Capsule b = RandomCapsuleContainingPoint(pt);
	assert(a.Intersects(b));
}
void TestAABBTriangleIntersect()
{
	float3 pt = float3::RandomBox(rng, -float3(SCALE,SCALE,SCALE), float3(SCALE,SCALE,SCALE));
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	Triangle b = RandomTriangleContainingPoint(pt);
	assert(a.Intersects(b));
}

typedef void (*TestFunctionPtr)();
struct Test
{
	std::string name;
	std::string description;
	TestFunctionPtr function;
};

std::vector<Test> tests;

void AddTest(std::string name, TestFunctionPtr function, std::string description = "")
{
	Test t;
	t.name = name;
	t.description = description;
	t.function = function;
	tests.push_back(t);
}

void RunTests(int numTimes)
{
	for(size_t i = 0; i < tests.size(); ++i)
	{
		printf("Testing '%s': ", tests[i].name.c_str());
		int numFails = 0;
		int numPasses = 0;
		try
		{
			for(int j = 0; j < numTimes; ++j)
			{
				tests[i].function();
				++numPasses;
			}
		}
		catch(const std::exception &e)
		{
			printf("FAILED: '%s' (%d passes)\n", e.what(), numPasses);
			++numFails;
		}

		if (numFails == 0)
			printf("ok (%d passes)\n", numPasses);
	}
}

int main()
{
	AddTest("AABB-AABB positive intersection", TestAABBAABBIntersect);
	AddTest("AABB-Line positive intersection", TestAABBLineIntersect);
	AddTest("AABB-Ray positive intersection", TestAABBRayIntersect);
	AddTest("AABB-LineSegment positive intersection", TestAABBLineSegmentIntersect);
	AddTest("AABB-OBB positive intersection", TestAABBOBBIntersect);
	AddTest("AABB-Plane positive intersection", TestAABBPlaneIntersect);
	AddTest("AABB-Sphere positive intersection", TestAABBSphereIntersect);
	AddTest("AABB-Triangle positive intersection", TestAABBTriangleIntersect);
	AddTest("AABB-Capsule positive intersection", TestAABBCapsuleIntersect);

	RunTests(10000);
}
