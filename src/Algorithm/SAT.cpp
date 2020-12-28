#include "SAT.h"
#include "../Math/float2.inl"

MATH_BEGIN_NAMESPACE

bool HasCcwWindingOrder(const float2 *poly, int numVertices)
{
	int prev2 = numVertices - 2;
	int prev1 = numVertices - 1;
	for(int i = 0; i < numVertices; ++i)
	{
		if (PerpDot2D(poly[prev2], poly[prev1], poly[i]) < -1e-2f)
			return false;
		prev2 = prev1;
		prev1 = i;
	}
	return true;
}

bool SATCollide2D(const float2 *a, int numA, const float2 *b, int numB)
{
	assert(numA > 0 && a);
	assert(numB > 0 && b);
	assert(HasCcwWindingOrder(a, numA));
	assert(HasCcwWindingOrder(b, numB));

	float2 edge;

	int prev = numA-1;
	for(int i = 0; i < numA; ++i)
	{
		edge = a[i] - a[prev];

		// N.b. for improved numerical stability, could do
		/*  float minDistance = (b[0] - a[i]).PerpDot(edge);
		 	for(int j = 1; j < numB; ++j)
		 	minDistance = Min(minDistance, (b[j] - a[i]).PerpDot(edge));
		 	if (minDistance > 0)
		 	return false;
		   but uncertain how much that will improve. */

		float maxThis = a[i].PerpDot(edge);
		float minOther = b[0].PerpDot(edge);
		for(int j = 1; j < numB; ++j)
			minOther = Min(minOther, b[j].PerpDot(edge));
		if (minOther > maxThis)
			return false;
		prev = i;
	}

	prev = numB-1;
	for(int i = 0; i < numB; ++i)
	{
		edge = b[i] - b[prev];
		float maxThis = b[i].PerpDot(edge);
		float minOther = a[0].PerpDot(edge);
		for(int j = 1; j < numA; ++j)
			minOther = Min(minOther, a[j].PerpDot(edge));
		if (minOther > maxThis)
			return false;
		prev = i;
	}

	return true;
}

float SATCollide2D_CollisionPoint(const float2 *a, int numA, const float2 *b, int numB,
                                 float2 &outCollisionPoint, float2 &outCollisionNormal)
{
	assert(numA > 0 && a);
	assert(numB > 0 && b);
	assert(HasCcwWindingOrder(a, numA));
	assert(HasCcwWindingOrder(b, numB));

	float2 edge;

	float minimumPenetrationDistance = -FLOAT_INF;

	int prev = numA-1;
	for(int i = 0; i < numA; ++i)
	{
		edge = (a[i] - a[prev]).Normalized(); // TODO: Allow caching these normalized direction vectors?
		float maxThis = a[i].PerpDot(edge);
		float minOther = b[0].PerpDot(edge);
		int minPenetrationIndex = 0;
		for(int j = 1; j < numB; ++j)
		{
			float penetrationDistance = b[j].PerpDot(edge);
			if (penetrationDistance < minOther)
			{
				minOther = penetrationDistance;
				minPenetrationIndex = j;
			}
		}
		float dist = minOther - maxThis;

		if (dist > 0.f)
			return dist;

		if (dist > minimumPenetrationDistance)
		{
			minimumPenetrationDistance = dist;
			outCollisionNormal = edge.Perp();
			outCollisionPoint = b[minPenetrationIndex];
		}

		prev = i;
	}

	prev = numB-1;
	for(int i = 0; i < numB; ++i)
	{
		edge = (b[i] - b[prev]).Normalized(); // TODO: Allow caching these normalized direction vectors?

		float maxThis = b[i].PerpDot(edge);
		float minOther = a[0].PerpDot(edge);
		int minPenetrationIndex = 0;

		for(int j = 1; j < numA; ++j)
		{
			float penetrationDistance = a[j].PerpDot(edge);
			if (penetrationDistance < minOther)
			{
				minOther = penetrationDistance;
				minPenetrationIndex = j;
			}
		}

		float dist = minOther - maxThis;

		if (dist > 0.f)
			return dist;

		if (dist > minimumPenetrationDistance)
		{
			minimumPenetrationDistance = dist;
			outCollisionNormal = -edge.Perp();
			outCollisionPoint = a[minPenetrationIndex];
		}

		prev = i;
	}

	return minimumPenetrationDistance;
}

MATH_END_NAMESPACE
