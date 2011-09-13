#include "Math/MathConstants.h"
#include "Math/MathFunc.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Math/Quat.h"
#include "Geometry/AABB.h"
#include "Geometry/OBB.h"
#include "Geometry/Capsule.h"
#include "Geometry/Sphere.h"
#include "Geometry/Circle.h"
#include "assume.h"

Capsule::Capsule(const LineSegment &endPoints, float radius)
:l(endPoints), r(radius)
{
}

Capsule::Capsule(const float3 &bottomPoint, const float3 &topPoint, float radius)
:l(bottomPoint, topPoint), r(radius)
{
}

void Capsule::SetFrom(const Sphere &s)
{
    l = LineSegment(s.pos, s.pos);
    r = s.r;
}

float Capsule::LineLength() const
{
    return l.Length();
}

float Capsule::Diameter() const
{
    return 2.f * r;
}

float3 Capsule::Bottom() const
{
    return l.a - UpDirection() * r;
}

float3 Capsule::Center() const
{
    return l.CenterPoint();
}

float3 Capsule::Top() const
{
    return l.b + UpDirection() * r;
}

float3 Capsule::UpDirection() const
{
    float3 d = l.b - l.a;
    d.Normalize(); // Will always result in a normalized vector, even if l.a == l.b.
    return d;
}

float Capsule::Height() const
{
    return LineLength() + Diameter();
}

float Capsule::Volume() const
{
    return pi * r * r * LineLength() + 4.f * pi * r * r * r / 3.f;
}

float Capsule::SurfaceArea() const
{
    return 2.f * pi * r * LineLength() + 4.f * pi * r * r;
}

Circle Capsule::CrossSection(float l) const
{
    ///\todo Implement!
    assume(false && "Not implemented!");
    return Circle();
}

LineSegment Capsule::HeightLineSegment() const
{
    return LineSegment(Bottom(), Top());
}

bool Capsule::IsFinite() const
{
    return l.IsFinite() && isfinite(r);
}

float3 Capsule::PointInside(float l, float a, float d) const
{
    ///\todo Implement!
    assume(false && "Not implemented!");
    return float3();
}

float3 Capsule::UniformPointPerhapsInside(float l, float x, float y) const
{
    return EnclosingOBB().PointInside(l, x, y);
}

AABB Capsule::EnclosingAABB() const
{
    AABB aabb(Min(l.a, l.b) - float3(r, r, r), Max(l.a, l.b) + float3(r, r, r));
    return aabb;
}

OBB Capsule::EnclosingOBB() const
{
    OBB obb;
    obb.axis[0] = UpDirection();
    obb.axis[1] = obb.axis[0].Perpendicular();
    obb.axis[2] = obb.axis[0].AnotherPerpendicular();
    obb.pos = Center();
    obb.r[0] = Height() * 0.5f;
    obb.r[1] = r;
    obb.r[2] = r;
    return obb;
}

float3 Capsule::RandomPointInside(LCG &rng) const
{
    assume(IsFinite());

    OBB obb = EnclosingOBB();
    for(int i = 0; i < 1000; ++i)
    {
        float3 pt = obb.RandomPointInside(rng);
        if (Contains(pt))
            return pt;
    }
    assume(false && "Warning: Capsule::RandomPointInside ran out of iterations to perform!");
    return Center(); // Just return some point that is known to be inside.
}

float3 Capsule::RandomPointOnSurface(LCG &rng) const
{
    ///\todo Implement!
    assume(false && "Not implemented!");
    return float3();
}

void Capsule::Translate(const float3 &offset)
{
    l.a += offset;
    l.b += offset;
}

void Capsule::Scale(const float3 &centerPoint, float scaleFactor)
{
    float3x4 tm = float3x4::Scale(float3::FromScalar(scaleFactor), centerPoint);
    l.Transform(tm);
    r *= scaleFactor;
}

void Capsule::Transform(const float3x3 &transform)
{
    assume(transform.HasUniformScale());
    l.Transform(transform);
    r *= transform.Col(0).Length(); // Scale the radius.
}

void Capsule::Transform(const float3x4 &transform)
{
    assume(transform.HasUniformScale());
    l.Transform(transform);
    r *= transform.Col(0).Length(); // Scale the radius.
}

void Capsule::Transform(const float4x4 &transform)
{
    assume(transform.HasUniformScale());
    l.Transform(transform);
    r *= transform.Col3(0).Length(); // Scale the radius.
}

void Capsule::Transform(const Quat &transform)
{
    l.Transform(transform);
}

float3 Capsule::ClosestPoint(const float3 &targetPoint) const
{
    ///\todo Implement!
    assume(false && "Not implemented!");
    return float3();
}

float Capsule::Distance(const float3 &point) const
{
    ///\todo Implement!
    assume(false && "Not implemented!");
    return 0;
}

bool Capsule::Contains(const float3 &point) const
{
    return l.Distance(point) <= r;
}

bool Capsule::Contains(const LineSegment &lineSegment) const
{
    return Contains(lineSegment.a) && Contains(lineSegment.b);
}
/*
bool Capsule::Intersects(const Ray &ray, float *dNear, float *dFar) const
{
}

bool Capsule::Intersects(const Line &line, float *dNear, float *dFar) const
{
}

bool Capsule::Intersects(const LineSegment &lineSegment, float *dNear, float *dFar) const
{
}

bool Capsule::Intersects(const Plane &plane) const
{
}

bool Capsule::Intersects(const AABB &aabb) const
{
}

bool Capsule::Intersects(const OBB &obb) const
{
}

bool Capsule::Intersects(const Sphere &sphere) const
{
}

bool Capsule::Intersects(const Triangle &triangle) const
{
}
*/

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Capsule::ToString() const
{
    char str[256];
    sprintf(str, "Capsule(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f), r:%.2f)", l.a.x, l.a.y, l.a.z, l.b.x, l.b.y, l.b.z, r);
    return str;
}
#endif
