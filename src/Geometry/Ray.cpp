/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/

#include "Geometry/AABB.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/LineSegment.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Geometry/OBB.h"
#include "Geometry/Plane.h"
#include "Math/Quat.h"
#include "Geometry/Sphere.h"
#include "Geometry/Capsule.h"
#include "Geometry/Triangle.h"
#include "Math/MathFunc.h"

Ray::Ray(const float3 &pos_, const float3 &dir_)
:pos(pos_), dir(dir_)
{
    assume(dir.IsNormalized());
}

Ray::Ray(const Line &line)
:pos(line.pos), dir(line.dir)
{
    assume(dir.IsNormalized());
}

Ray::Ray(const LineSegment &lineSegment)
:pos(lineSegment.a), dir(lineSegment.Dir())
{
}

/// Returns a point on this line.
float3 Ray::GetPoint(float d) const
{
    assert(dir.IsNormalized());
    return pos + d * dir;
}

/// Applies a transformation to this line.
void Ray::Transform(const float3x3 &transform)
{
    pos = transform.Transform(pos);
    dir = transform.Transform(dir);
}

void Ray::Transform(const float3x4 &transform)
{
    pos = transform.TransformPos(pos);
    dir = transform.TransformDir(dir);
}

void Ray::Transform(const float4x4 &transform)
{
    pos = transform.TransformPos(pos);
    dir = transform.TransformDir(dir);
}

void Ray::Transform(const Quat &transform)
{
    pos = transform.Transform(pos);
    dir = transform.Transform(dir);
}

bool Ray::Contains(const float3 &point, float distanceThreshold) const
{
    return ClosestPoint(point).DistanceSq(point) <= distanceThreshold;
}

bool Ray::Contains(const LineSegment &lineSegment, float distanceThreshold) const
{
    return Contains(lineSegment.a, distanceThreshold) && Contains(lineSegment.b, distanceThreshold);
}

bool Ray::Equals(const Ray &rhs, float epsilon) const
{
    return pos.Equals(rhs.pos, epsilon) && dir.Equals(rhs.dir, epsilon);
}

/// Returns the distance of the given point to this line.
/// @param d [out] This element will receive the distance along this line that specifies the closest point on this line to the given point.
float Ray::Distance(const float3 &point, float *d) const
{
    return ClosestPoint(point, d).Distance(point);
}

float Ray::Distance(const float3 &point) const
{
    return Distance(point, 0);
}

/// Returns the distance of the given ray to this line.
/// @param d [out] Receives the distance along this line that specifies the closest point on this line to the given point.
/// @param d2 [out] Receives the distance along the other line that specifies the closest point on that line to this line.
float Ray::Distance(const Ray &other, float *d, float *d2) const
{
    float u2;
    float3 c = ClosestPoint(other, d, &u2);
    if (d2) *d2 = u2;
    return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const Ray &ray) const
{
    return Distance(ray, 0, 0);
}

float Ray::Distance(const Line &other, float *d, float *d2) const
{
    float u2;
    float3 c = ClosestPoint(other, d, &u2);
    if (d2) *d2 = u2;
    return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const Line &line) const
{
    return Distance(line, 0, 0);
}

float Ray::Distance(const LineSegment &other, float *d, float *d2) const
{
    float u2;
    float3 c = ClosestPoint(other, d, &u2);
    if (d2) *d2 = u2;
    return c.Distance(other.GetPoint(u2));
}

float Ray::Distance(const LineSegment &lineSegment) const
{
    return Distance(lineSegment, 0, 0);
}

float Ray::Distance(const Sphere &sphere) const
{
    return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Ray::Distance(const Capsule &capsule) const
{
    return Max(0.f, Distance(capsule.l) - capsule.r);
}

float3 Ray::ClosestPoint(const float3 &targetPoint, float *d) const
{
    float u = Max(0.f, Dot(targetPoint - pos, dir));
    if (d)
        *d = u;
    return GetPoint(u);
}

float3 Ray::ClosestPoint(const Ray &other, float *d, float *d2) const
{
    ///\todo Properly cap d2.
    return LineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Ray::ClosestPoint(const Line &other, float *d, float *d2) const
{
    return LineLine(pos, pos + dir, other.pos, other.pos + other.dir, d, d2);
}

float3 Ray::ClosestPoint(const LineSegment &other, float *d, float *d2) const
{
    ///\todo Properly cap d2.
    return LineLine(pos, pos + dir, other.a, other.b, d, d2);
}

bool Ray::Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const
{
    return triangle.Intersects(*this, d, intersectionPoint);
}

bool Ray::Intersects(const Triangle &triangle) const
{
    return triangle.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const Plane &plane, float *d) const
{
    return plane.Intersects(*this, d);
}

bool Ray::Intersects(const Plane &plane) const
{
    return plane.Intersects(*this, 0);
}

bool Ray::Intersects(const Sphere &sphere, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
    return sphere.Intersects(*this, intersectionPoint, intersectionNormal, d);
}

bool Ray::Intersects(const Sphere &sphere) const
{
    return sphere.Intersects(*this, 0, 0, 0);
}

bool Ray::Intersects(const AABB &aabb, float *dNear, float *dFar) const
{
    return aabb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const AABB &aabb) const
{
    return aabb.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const OBB &obb, float *dNear, float *dFar) const
{
    return obb.Intersects(*this, dNear, dFar);
}

bool Ray::Intersects(const OBB &obb) const
{
    return obb.Intersects(*this, 0, 0);
}

bool Ray::Intersects(const Capsule &capsule) const
{
    return capsule.Intersects(*this);
}
/*
bool Ray::Intersects(const Polygon &polygon) const
{
    return polygon.Intersects(*this);
}
*/
Line Ray::ToLine() const
{
    return Line(pos, dir);
}

LineSegment Ray::ToLineSegment(float d) const
{
    return LineSegment(pos, GetPoint(d));
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Ray::ToString() const
{
    char str[256];
    sprintf(str, "Ray(Pos:(%.2f, %.2f, %.2f) Dir:(%.2f, %.2f, %.2f))", pos.x, pos.y, pos.z, dir.x, dir.y, dir.z);
    return str;
}
#endif

Ray operator *(const float3x3 &transform, const Ray &ray)
{
    return Ray(transform * ray.pos, transform * ray.dir);
}

Ray operator *(const float3x4 &transform, const Ray &ray)
{
    return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir));
}

Ray operator *(const float4x4 &transform, const Ray &ray)
{
    return Ray(transform.MulPos(ray.pos), transform.MulDir(ray.dir));
}

Ray operator *(const Quat &transform, const Ray &ray)
{
    return Ray(transform * ray.pos, transform * ray.dir);
}
