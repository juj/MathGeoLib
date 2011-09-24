/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/


#include "Math/MathFunc.h"
#include "Geometry/AABB.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Ray.h"
#include "Geometry/Line.h"
#include "Geometry/Plane.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Geometry/OBB.h"
#include "Math/Quat.h"
#include "Geometry/Sphere.h"
#include "Geometry/Triangle.h"

LineSegment::LineSegment(const float3 &a_, const float3 &b_)
:a(a_), b(b_)
{
}

LineSegment::LineSegment(const Ray &ray, float d)
:a(ray.pos), b(ray.GetPoint(d))
{
}

LineSegment::LineSegment(const Line &line, float d)
:a(line.pos), b(line.GetPoint(d))
{
}

/// Returns a point on the line segment.
float3 LineSegment::GetPoint(float d) const
{
    return (1.f - d) * a + d * b;
}

float3 LineSegment::CenterPoint() const
{
    return (a + b) * 0.5f;
}

void LineSegment::Reverse()
{
    Swap(a, b);
}

float3 LineSegment::Dir() const
{
    return (b - a).Normalized();
}

/// Applies a transformation to this line.
void LineSegment::Transform(const float3x3 &transform)
{
    a = transform * a;
    b = transform * b;
}

void LineSegment::Transform(const float3x4 &transform)
{
    a = transform.MulPos(a);
    b = transform.MulPos(b);
}

void LineSegment::Transform(const float4x4 &transform)
{
    a = transform.MulPos(a);
    b = transform.MulPos(b);
}

void LineSegment::Transform(const Quat &transform)
{
    a = transform * a;
    b = transform * b;
}

float LineSegment::Length() const
{
    return a.Distance(b);
}

float LineSegment::LengthSq() const
{
    return a.DistanceSq(b);
}

bool LineSegment::IsFinite() const
{
    return a.IsFinite() && b.IsFinite();
}

bool LineSegment::Contains(const float3 &point, float distanceThreshold) const
{
    return ClosestPoint(point).DistanceSq(point) <= distanceThreshold;
}

float3 LineSegment::ClosestPoint(const float3 &point, float *d) const
{
    float3 dir = b - a;
    float u = Clamp01(Dot(point - a, dir) / dir.LengthSq());
    if (d)
        *d = u;
    return a + *d * dir;
}

float3 LineSegment::ClosestPoint(const Ray &other, float *d, float *d2) const
{
    float u, u2;
    LineLine(a, b, other.pos, other.pos + other.dir, &u, &u2);
    u = Clamp01(u); // This is a line segment - cap both ends.
    if (d)
        *d = u;
    u2 = Max(0.f, u2); // The other primitive is a ray - cap negative side.
    if (d2)
        *d2 = u2;
    return GetPoint(u);
}

float3 LineSegment::ClosestPoint(const Line &other, float *d, float *d2) const
{
    float u, u2;
    LineLine(a, b, other.pos, other.pos + other.dir, &u, &u2);
    u = Clamp01(u); // This is a line segment - cap both ends.
    if (d)
        *d = u;
    if (d2)
        *d2 = u2;
    return GetPoint(u);
}

float3 LineSegment::ClosestPoint(const LineSegment &other, float *d, float *d2) const
{
    float u, u2;
    LineLine(a, b, other.a, other.b, &u, &u2);
    u = Clamp01(u); // This is a line segment - cap both ends.
    if (d)
        *d = u;
    u2 = Clamp01(u2); // The other primitive is a line segment as well - cap both ends.
    if (d2)
        *d2 = u2;
    return GetPoint(u);
}

float LineSegment::Distance(const float3 &point, float *d) const
{
    ///\todo This function could be slightly optimized.
    /// See Christer Ericson's Real-Time Collision Detection, p.130.
    float3 closestPoint = ClosestPoint(point, d);
    return closestPoint.Distance(point);
}

float LineSegment::Distance(const Ray &other, float *d, float *d2) const
{
    float u, u2;
    ClosestPoint(other, &u, &u2);
    if (d)
        *d = u;
    if (d2)
        *d2 = u2;
    return GetPoint(u).Distance(other.GetPoint(u2));
}

float LineSegment::Distance(const Line &other, float *d, float *d2) const
{
    float u, u2;
    ClosestPoint(other, &u, &u2);
    if (d)
        *d = u;
    if (d2)
        *d2 = u2;
    return GetPoint(u).Distance(other.GetPoint(u2));
}

float LineSegment::Distance(const LineSegment &other, float *d, float *d2) const
{
    float u, u2;
    ClosestPoint(other, &u, &u2);
    if (d)
        *d = u;
    if (d2)
        *d2 = u2;
    return GetPoint(u).Distance(other.GetPoint(u2));
}

float LineSegment::Distance(const Plane &other) const
{
    float aDist = other.SignedDistance(a);
    float bDist = other.SignedDistance(b);
    if (aDist * bDist < 0.f)
        return 0.f; // There was an intersection, so the distance is zero.
    return Min(Abs(aDist), Abs(bDist));
}

float LineSegment::Distance(const Sphere &other) const
{
    return Max(0.f, Distance(other.pos) - other.r);
}

bool LineSegment::Intersects(const Plane &plane) const
{
    float d = plane.SignedDistance(a);
    float d2 = plane.SignedDistance(b);
    return d * d2 <= 0.f;
}

bool LineSegment::Intersects(const Triangle &triangle, float *d, float3 *intersectionPoint) const
{
    return triangle.Intersects(*this, d, intersectionPoint);
}

bool LineSegment::Intersects(const Sphere &s, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
    return s.Intersects(*this, intersectionPoint, intersectionNormal, d);
}

bool LineSegment::Intersects(const AABB &aabb, float *dNear, float *dFar) const
{
    return aabb.Intersects(*this, dNear, dFar);
}

bool LineSegment::Intersects(const OBB &obb, float *dNear, float *dFar) const
{
    return obb.Intersects(*this, dNear, dFar);
}

Ray LineSegment::ToRay() const
{
    return Ray(a, Dir());
}

Line LineSegment::ToLine() const
{
    return Line(a, Dir());
}

LineSegment operator *(const float3x3 &transform, const LineSegment &l)
{
    return LineSegment(transform * l.a, transform * l.b);
}

LineSegment operator *(const float3x4 &transform, const LineSegment &l)
{
    return LineSegment(transform.MulPos(l.a), transform.MulPos(l.b));
}

LineSegment operator *(const float4x4 &transform, const LineSegment &l)
{
    return LineSegment(transform.MulPos(l.a), transform.MulPos(l.b));
}

LineSegment operator *(const Quat &transform, const LineSegment &l)
{
    return LineSegment(transform * l.a, transform * l.b);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string LineSegment::ToString() const
{
    char str[256];
    sprintf(str, "LineSegment(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f))", 
        a.x, a.y, a.z, b.x, b.y, b.z);
    return str;
}
#endif
