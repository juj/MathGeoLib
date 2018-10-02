#include "SAT.h"

MATH_BEGIN_NAMESPACE

void ProjectionInterval2D(const float2 &normal, const float2 *pts, int numPts, float &minE, float &maxE)
{
	assert(numPts > 0 && pts);
	minE = maxE = pts[0].Dot(normal);
	for(int i = 1; i < numPts; ++i)
	{
		float d = pts[i].Dot(normal);
		minE = Min(minE, d);
		maxE = Max(maxE, d);
	}
}

bool SATCollide2D(const float2 *a, int numA, const float2 *b, int numB)
{
	assert(numA > 0 && a);
	assert(numB > 0 && b);
	float2 normal;
	float min1, max1, min2, max2;

	int prev = numA-1;
	for(int i = 0; i < numA; ++i)
	{
		normal = (a[i] - a[prev]).Perp();
		ProjectionInterval2D(normal, a, numA, min1, max1);
		ProjectionInterval2D(normal, b, numB, min2, max2);
		if (max1 <= min2 || max2 <= min1) return false;
		prev = i;
	}

	prev = numB-1;
	for(int i = 0; i < numB; ++i)
	{
		normal = (b[i] - b[prev]).Perp();
		ProjectionInterval2D(normal, a, numA, min1, max1);
		ProjectionInterval2D(normal, b, numB, min2, max2);
		if (max1 <= min2 || max1 <= min2) return false;
		prev = i;
	}

	return true;
}

MATH_END_NAMESPACE
