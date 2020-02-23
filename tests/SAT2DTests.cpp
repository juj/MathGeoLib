#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Algorithm/GJK.h"
#include "../src/Algorithm/SAT.h"

MATH_IGNORE_UNUSED_VARS_WARNING

MATH_BEGIN_NAMESPACE

using namespace TestData;

std::vector<float2> GenerateRandomPolygonContainingPt(LCG &rng, int numVertices, const float2 &pt, float boxRadius)
{
	std::vector<float2> vertices;
	float2 center = float2::zero;
	for(int i = 0; i < numVertices; ++i)
	{
		vertices.push_back(float2::RandomBox(rng, -boxRadius, boxRadius));
		center += vertices.back();
	}
	center /= vertices.size();
	for(int i = 0; i < numVertices; ++i)
	{
		vertices[i] += pt - center;
	}

	vertices.resize(float2_ConvexHullInPlace(&vertices[0], vertices.size()));
	return vertices;
}

RANDOMIZED_TEST(Poly2D_Poly2D_Intersect)
{
	std::vector<float2> p1 = GenerateRandomPolygonContainingPt(rng, rng.Int(3, 10), float2::zero, 10.f);
	std::vector<float2> p2 = GenerateRandomPolygonContainingPt(rng, rng.Int(3, 10), float2::zero, 10.f);
	assert(SATCollide2D(&p1[0], (int)p1.size(), &p2[0], (int)p2.size()));
}

BENCHMARK(Poly2D_Poly2D_Intersect, "Poly2D-Poly2D SAT positive collision")
{
	uf[i] = SATCollide2D(&poly2DsContainingZero[i][0], poly2DsContainingZero[i].size(),
						 &poly2DsContainingZero[i+1][0], poly2DsContainingZero[i+1].size()) ? 1.0 : 0.0;
}
BENCHMARK_END

RANDOMIZED_TEST(Poly2D_Poly2D_NoIntersect)
{
	std::vector<float2> p1 = GenerateRandomPolygonContainingPt(rng, rng.Int(3, 10), float2::zero, 10.f);
	std::vector<float2> p2 = GenerateRandomPolygonContainingPt(rng, rng.Int(3, 10), float2::RandomDir(rng) * 50.f, 10.f);
	assert(!SATCollide2D(&p1[0], (int)p1.size(), &p2[0], (int)p2.size()));
}

BENCHMARK(Poly2D_Poly2D_NoIntersect, "Poly2D-Poly2D SAT no collision")
{
	uf[i] = SATCollide2D(&poly2DsContainingZero[i][0], poly2DsContainingZero[i].size(),
						 &poly2DsAwayFromZero[i][0], poly2DsAwayFromZero[i].size()) ? 1.0 : 0.0;
}
BENCHMARK_END

MATH_END_NAMESPACE
