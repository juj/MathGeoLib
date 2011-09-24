
#include "Circle.h"
#include "Geometry/Plane.h"
#include "Math/MathFunc.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Line.h"

Circle::Circle(const float3 &center, const float3 &n, float radius)
:pos(center),
normal(n),
r(radius)
{
}

float3 Circle::BasisU() const
{
	return normal.Perpendicular();
}

float3 Circle::BasisV() const
{
	return normal.AnotherPerpendicular();
}

float3 Circle::GetPoint(float angleRadians) const
{
	return pos + r * (cos(angleRadians) * BasisU() + sin(angleRadians) * BasisV());
}

float3 Circle::GetPoint(float angleRadians, float d) const
{
	return pos + r * d * (cos(angleRadians) * BasisU() + sin(angleRadians) * BasisV());
}

Plane Circle::ContainingPlane() const
{
	return Plane(pos, normal);
}

bool Circle::EdgeContains(const float3 &point, float maxDistance) const
{
	return DistanceToEdge(point) <= maxDistance;
}
/*
bool Circle::DiscContains(const float3 &point, float maxDistance) const
{
	return DistanceToDisc(point) <= maxDistance;
}

bool Circle::DiscContains(const LineSegment &lineSegment, float maxDistance = 1e-6f) const
{
}

*/
float Circle::DistanceToEdge(const float3 &point) const
{
	return ClosestPointToEdge(point).Distance(point);
}

float Circle::DistanceToEdge(const Ray &ray, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(ray, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(ray.GetPoint(t));
}

float Circle::DistanceToEdge(const LineSegment &lineSegment, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(lineSegment, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(lineSegment.GetPoint(t));
}

float Circle::DistanceToEdge(const Line &line, float *d, float3 *closestPoint) const
{
	float t;
	float3 cp = ClosestPointToEdge(line, &t);
	if (closestPoint)
		*closestPoint = cp;
	if (d)
		*d = t;
	return cp.Distance(line.GetPoint(t));
}
/*
float Circle::DistanceToDisc(const float3 &point) const
{
}

float Circle::DistanceToDisc(const Ray &ray, float *d, float3 *closestPoint) const
{
}

float Circle::DistanceToDisc(const LineSegment &lineSegment, float *d, float3 *closestPoint) const
{
}

float Circle::DistanceToDisc(const Line &line, float *d, float3 *closestPoint) const
{
}
*/
float3 Circle::ClosestPointToEdge(const float3 &point) const
{
	float3 pointOnPlane = ContainingPlane().Project(point);
	float3 diff = pointOnPlane - pos;
	if (diff.IsZero())
		return GetPoint(0); // The point is in the center of the circle, all points are equally close.
	return pos + diff.ScaledToLength(r);
}

float3 Circle::ClosestPointToEdge(const Ray &ray, float *d) const
{
	assume(false && "Not implemented!");
	return float3();
}

float3 Circle::ClosestPointToEdge(const LineSegment &lineSegment, float *d) const
{
	assume(false && "Not implemented!");
	return float3();
}

float3 Circle::ClosestPointToEdge(const Line &line, float *d) const
{
	assume(false && "Not implemented!");
	return float3();
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Circle::ToString() const
{
    char str[256];
    sprintf(str, "Circle(pos:(%.2f, %.2f, %.2f) normal:(%.2f, %.2f, %.2f), r:%.2f)", 
        pos.x, pos.y, pos.z, normal.x, normal.y, normal.z, r);
    return str;
}
#endif
