/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/

#include "Math/MathFunc.h"
#include "Math/float2.h"
#include "Math/float3.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Math/Quat.h"
#include "Geometry/Frustum.h"
#include "Geometry/Triangle.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Line.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Ray.h"
#include "Geometry/Sphere.h"
#include "Geometry/AABB.h"
#include "Geometry/OBB.h"

MATH_BEGIN_NAMESPACE

Triangle::Triangle(const float3 &a_, const float3 &b_, const float3 &c_)
:a(a_), b(b_), c(c_)
{
}

float3 Triangle::Barycentric(const float3 &point) const
{
    /// @note An alternate mechanism to compute the barycentric is given in Christer Ericson's
    /// Real-Time Collision Detection, pp. 51-52, which might be slightly faster.
    float3 v0 = b - a;
    float3 v1 = c - a;
    float3 v2 = point - a;
    float d00 = Dot(v0, v0);
    float d01 = Dot(v0, v1);
    float d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0);
    float d21 = Dot(v2, v1);
    float denom = 1.f / (d00 * d11 - d01 * d01);
    float v = (d11 * d20 - d01 * d21) * denom;
    float w = (d00 * d21 - d01 * d20) * denom;
    float u = 1.0f - v - w;
    return float3(u, v, w);
}

bool Triangle::BarycentricInsideTriangle(const float3 &barycentric)
{
    return barycentric.x >= 0.f && barycentric.y >= 0.f && barycentric.z >= 0.f &&
        EqualAbs(barycentric.x + barycentric.y + barycentric.z, 1.f);
}

float3 Triangle::Point(float u, float v, float w) const
{
    return u * a + v * b + w * c;
}

float3 Triangle::Point(const float3 &b) const
{
    return Point(b.x, b.y, b.z);
}

float Triangle::Area() const
{
    return 0.5f * Cross(b-a, c-a).Length();
}

LineSegment Triangle::Edge(int i) const
{
    assume(0 <= i);
    assume(i <= 2);
    if (i == 0)
        return LineSegment(a, b);
    if (i == 1)
        return LineSegment(b, c);
    return LineSegment(c, a);
}

float3 Triangle::Vertex(int i) const
{
    assume(0 <= i);
    assume(i <= 2);
    if (i == 0)
        return a;
    if (i == 1)
        return b;
    return c;
}

Plane Triangle::PlaneCCW() const
{
    return Plane(a, b, c);
}

Plane Triangle::PlaneCW() const
{
    return Plane(a, c, b);
}

float3 Triangle::NormalCCW() const
{
    return UnnormalizedNormalCCW().Normalized();
}

float3 Triangle::NormalCW() const
{
    return UnnormalizedNormalCW().Normalized();
}

float3 Triangle::UnnormalizedNormalCCW() const
{
    return Cross(b-a, c-a);
}

float3 Triangle::UnnormalizedNormalCW() const
{
    return Cross(c-a, b-a);
}

Polygon Triangle::ToPolygon() const
{
    Polygon p;
    p.p.push_back(a);
    p.p.push_back(b);
    p.p.push_back(c);
    return p;
}

Polyhedron Triangle::ToPolyhedron() const
{
    return ToPolygon().ToPolyhedron();
}

float Triangle::Area2D(const float2 &p1, const float2 &p2, const float2 &p3)
{
    return (p1.x - p2.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p2.y);
}

float Triangle::SignedArea(const float3 &pt, const float3 &a, const float3 &b, const float3 &c)
{
    return Dot(Cross(b-pt, c-pt), Cross(b-a, c-a).Normalized());
}

bool Triangle::IsDegenerate(float epsilon) const
{
    return IsDegenerate(a, b, c);
}

bool Triangle::IsDegenerate(const float3 &a, const float3 &b, const float3 &c, float epsilon)
{
    return a.Equals(b, epsilon) || a.Equals(c, epsilon) || b.Equals(c, epsilon);
}

bool Triangle::Contains(const float3 &point, float triangleThickness) const
{
    if (PlaneCCW().Distance(point) > triangleThickness) // The winding order of the triangle plane does not matter.s
        return false; ///\todo This test is omitted in Real-Time Collision Detection. p. 25. A bug in the book?

    float3 br = Barycentric(point);
    return br.y >= 0.f && br.z >= 0.f && (br.y + br.z) <= 1.f;
}

bool Triangle::Contains(const LineSegment &lineSegment, float triangleThickness) const
{
    return Contains(lineSegment.a, triangleThickness) && Contains(lineSegment.b, triangleThickness);
}

bool Triangle::Contains(const Triangle &triangle, float triangleThickness) const
{
    return Contains(triangle.a, triangleThickness) && Contains(triangle.b, triangleThickness)
      && Contains(triangle.c, triangleThickness);
}

/*
bool Triangle::Contains(const Polygon &polygon, float triangleThickness) const
{
    if (polygon.points.size() == 0)
        return false;
    for(int i = 0; i < polygon.points.size(); ++i)
        if (!Contains(polygon.points[i], triangleThickness))
            return false;
    return true;
}
*/
float Triangle::Distance(const float3 &point) const
{
    return ClosestPoint(point).Distance(point);
}

float Triangle::Distance(const Sphere &sphere) const
{
    return Max(0.f, Distance(sphere.pos) - sphere.r);
}

/** Calculates the intersection between a ray and a triangle. The facing is not accounted for, so
	rays are reported to intersect triangles that are both front and backfacing.
	According to "T. Möller, B. Trumbore. Fast, Minimum Storage Ray/Triangle Intersection. 2005."
	http://jgt.akpeters.com/papers/MollerTrumbore97/
	@param ray The ray to test.
	@param v0 Vertex 0 of the triangle.
	@param v1 Vertex 1 of the triangle.
	@param v2 Vertex 2 of the triangle.
	@param u [out] The barycentric u coordinate is returned here if an intersection occurred.
	@param v [out] The barycentric v coordinate is returned here if an intersection occurred.
	@param t [out] The signed distance from ray origin to ray intersection position will be returned here. (if intersection occurred)
	@return True if an intersection occurred. If no intersection, then u,v and t will contain undefined values. */
bool IntersectLineTri(const float3 &linePos, const float3 &lineDir,
		const float3 &v0, const float3 &v1, const float3 &v2,
		float &u, float &v, float &t)
{
	float3 vE1, vE2;
	float3 vT, vP, vQ;

	const float epsilon = 1e-6f;

	// Edge vectors
	vE1 = v1 - v0;
	vE2 = v2 - v0;

	// begin calculating determinant - also used to calculate U parameter
	vP = Cross(lineDir, vE2);

	// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
	const float det = Dot(vE1, vP);

	// If determinant is near zero, ray lies in plane of triangle.
	if (fabs(det) <= epsilon)
		return false;
	const float recipDet = 1.f / det;

	// Calculate distance from v0 to ray origin
	vT = linePos - v0;

	// Output barycentric u
	u = Dot(vT, vP) * recipDet;
	if (u < 0.f || u > 1.f)
		return false; // Barycentric U is outside the triangle - early out.

	// Prepare to test V parameter
	vQ = Cross(vT, vE1);

	// Output barycentric v
	v = Dot(lineDir, vQ) * recipDet;
	if (v < 0.f || u + v > 1.f) // Barycentric V or the combination of U and V are outside the triangle - no intersection.
		return false;

	// Barycentric u and v are in limits, the ray intersects the triangle. 
	
	// Output signed distance from ray to triangle.
	t = Dot(vE2, vQ) * recipDet;
    return true;
//	return (det < 0.f) ? IntersectBackface : IntersectFrontface;
}

bool Triangle::Intersects(const LineSegment &l, float *d, float3 *intersectionPoint) const
{
    float u, v, t;
    bool success = IntersectLineTri(l.a, l.Dir(), a, b, c, u, v, t);
    if (!success)
        return false;
    float length = l.LengthSq();
    if (t < 0.f || t*t >= length)
        return false;
    length = sqrtf(length);
    if (d)
    {
        float len = t / length;
        *d = len;
        if (intersectionPoint)
            *intersectionPoint = l.GetPoint(len);
    }
    else if (intersectionPoint)
        *intersectionPoint = l.GetPoint(t / length);
    return true;
}

bool Triangle::Intersects(const Line &l, float *d, float3 *intersectionPoint) const
{
    float u, v, t;
    bool success = IntersectLineTri(l.pos, l.dir, a, b, c, u, v, t);
    if (!success)
        return false;
    if (d)
        *d = t;
    if (intersectionPoint)
        *intersectionPoint = l.GetPoint(t);
    return success;
}

bool Triangle::Intersects(const Ray &r, float *d, float3 *intersectionPoint) const
{
    float u, v, t;
    bool success = IntersectLineTri(r.pos, r.dir, a, b, c, u, v, t);
    if (!success || t <= 0.f)
        return false;
    if (d)
        *d = t;
    if (intersectionPoint)
        *intersectionPoint = r.GetPoint(t);
    return success;
}

bool Triangle::Intersects(const Plane &plane) const
{
    return plane.Intersects(*this);
}

/// See Christer Ericson's Real-Time Collision Detection, p.167.
bool Triangle::Intersects(const Sphere &sphere, float3 *closestPointOnTriangle) const
{
    float3 pt = ClosestPoint(sphere.pos);

    if (closestPointOnTriangle)
        *closestPointOnTriangle = pt;

    return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

static void FindIntersectingLineSegments(const Triangle &t, float da, float db, float dc, LineSegment &l1, LineSegment &l2)
{
    if (da*db > 0.f)
    {
        l1 = LineSegment(t.a, t.c);
        l2 = LineSegment(t.b, t.c);
    }
    else if (db*dc > 0.f)
    {
        l1 = LineSegment(t.a, t.b);
        l1 = LineSegment(t.a, t.c);
    }
    else
    {
        l1 = LineSegment(t.a, t.b);
        l1 = LineSegment(t.b, t.c);
    }
}

/// Implementation based on pseudo-code from Tomas Möller's 
/// "A Fast Triangle-Triangle Intersection Test". http://jgt.akpeters.com/papers/Moller97/
/// See also Christer Ericson's Real-Time Collision Detection, p. 172.
bool Triangle::Intersects(const Triangle &t2, LineSegment *outLine) const
{
    // Is the triangle t2 completely on one side of the plane of this triangle?
    Plane p1 = this->PlaneCCW();
    float t2da = p1.SignedDistance(t2.a);
    float t2db = p1.SignedDistance(t2.b);
    float t2dc = p1.SignedDistance(t2.c);
    if (t2da*t2db > 0.f && t2da*t2dc > 0.f)
        return false;
    // Is this triangle completely on one side of the plane of the triangle t2?
    Plane p2 = t2.PlaneCCW();
    float t1da = p2.SignedDistance(this->a);
    float t1db = p2.SignedDistance(this->b);
    float t1dc = p2.SignedDistance(this->c);
    if (t1da*t1db > 0.f && t1da*t1dc > 0.f)
        return false;

    // Find the intersection line of the two planes. 
    Line l;
    bool success = p1.Intersects(p2, &l);
    assume(success); // We already determined the two triangles have intersecting planes, so this should always succeed.
    if (!success)
        return false;

    // Find the two line segments of both triangles which straddle the intersection line.
    LineSegment l1a, l1b;
    LineSegment l2a, l2b;
    FindIntersectingLineSegments(*this, t1da, t1db, t1dc, l1a, l1b);
    FindIntersectingLineSegments(t2, t2da, t2db, t2dc, l2a, l2b);

    // Find the projection intervals on the intersection line.
    float d1a, d1b, d2a, d2b;
    l.Distance(l1a, &d1a);
    l.Distance(l1b, &d1b);
    l.Distance(l2a, &d2a);
    l.Distance(l2b, &d2b);
    if (d1a > d1b)
        Swap(d1a, d1b);
    if (d2a > d2b)
        Swap(d2a, d2b);
    float rStart = Max(d1a, d2a);
    float rEnd = Min(d1b, d2b);
    if (rStart <= rEnd)
    {
        if (outLine)
            *outLine = LineSegment(l.GetPoint(rStart), l.GetPoint(rEnd));
        return true;
    }
    return false;
}

bool RangesOverlap(float start1, float end1, float start2, float end2)
{
    return end1 >= start2 && end2 >= start1;
}

/// Implementation based on the pseudo-code in Christer Ericson's Real-Time Collision Detection, pp. 169-172.
bool Triangle::Intersects(const AABB &aabb) const
{
    ///\todo This test can be greatly optimized by manually unrolling loops, trivial math and by avoiding 
    /// unnecessary copying.
    float t1, t2, a1, a2;
    const float3 e[3] = { float3(1,0,0), float3(0,1,0), float3(0,0,1) };

    for(int i = 0; i < 3; ++i)
    {
        ProjectToAxis(e[i], t1, t2);
        aabb.ProjectToAxis(e[i], a1, a2);
        if (!RangesOverlap(t1, t2, a1, a2))
            return false;
    }

    float3 n = UnnormalizedNormalCCW();
    ProjectToAxis(n, t1, t2);
    aabb.ProjectToAxis(n, a1, a2);
    if (!RangesOverlap(t1, t2, a1, a2))
        return false;

    const float3 t[3] = { b-a, c-a, c-b };

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        {
            float3 axis = Cross(e[i], t[j]);
            float len = axis.LengthSq();
            if (len <= 1e-4f)
                continue; // Ignore tests on degenerate axes.

            ProjectToAxis(axis, t1, t2);
            aabb.ProjectToAxis(axis, a1, a2);
            if (!RangesOverlap(t1, t2, a1, a2))
                return false;
        }

    // No separating axis exists, the AABB and triangle intersect.
    return true;
}

bool Triangle::Intersects(const OBB &obb) const
{
    return obb.Intersects(*this);
}

bool Triangle::Intersects(const Polygon &polygon) const
{
    return polygon.Intersects(*this);
}

bool Triangle::Intersects(const Frustum &frustum) const
{
    return frustum.Intersects(*this);
}

bool Triangle::Intersects(const Polyhedron &polyhedron) const
{
    return polyhedron.Intersects(*this);
}

void Triangle::ProjectToAxis(const float3 &axis, float &dMin, float &dMax) const
{
    dMin = dMax = Dot(axis, a);
    float t = Dot(axis, b);
    dMin = Min(t, dMin);
    dMax = Max(t, dMax);
    t = Dot(axis, c);
    dMin = Min(t, dMin);
    dMax = Max(t, dMax);
}

/// Code from Christer Ericson's Real-Time Collision Detection, pp. 141-142.
float3 Triangle::ClosestPoint(const float3 &p) const
{
    // Check if P is in vertex region outside A.
    float3 ab = b - a;
    float3 ac = c - a;
    float3 ap = p - a;
    float d1 = Dot(ab, ap);
    float d2 = Dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f)
        return a; // Barycentric coordinates are (1,0,0).

    // Check if P is in vertex region outside B.
    float3 bp = p - b;
    float d3 = Dot(ab, bp);
    float d4 = Dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3)
        return b; // Barycentric coordinates are (0,1,0).

    // Check if P is in edge region of AB, and if so, return the projection of P onto AB.
    float vc = d1*d4 - d3*d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        float v = d1 / (d1 - d3);
        return a + v * ab; // The barycentric coordinates are (1-v, v, 0).
    }

    // Check if P is in vertex region outside C.
    float3 cp = p - c;
    float d5 = Dot(ab, cp);
    float d6 = Dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6)
        return c; // The barycentric coordinates are (0,0,1).

    // Check if P is in edge region of AC, and if so, return the projection of P onto AC.
    float vb = d5*d2 - d1*d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        float w = d2 / (d2 - d6);
        return a + w * ac; // The barycentric coordinates are (1-w, 0, w).
    }

    // Check if P is in edge region of BC, and if so, return the projection of P onto BC.
    float va = d3*d6 - d5*d4;
    if (va <= 0.f && d4 - d3 >= 0.f && d5 - d6 >= 0.f)
    {
        float w = (d4 - d3) / (d4 - d3 + d5 - d6);
        return b + w * (c - b); // The barycentric coordinates are (0, 1-w, w).
    }

    // P must be inside the face region. Compute the closest point through its barycentric coordinates (u,v,w).
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

float3 Triangle::ClosestPoint(const LineSegment &line, float3 *otherPt) const
{
    float3 intersectionPoint;
    bool success = Intersects(line, 0, &intersectionPoint);
    if (success)
        return intersectionPoint;

    Plane p = PlaneCCW();
    float d1 = p.Distance(line.a);
    float d2 = p.Distance(line.b);
    bool aProjectsInsideTriangle = BarycentricInsideTriangle(line.a);
    bool bProjectsInsideTriangle = BarycentricInsideTriangle(line.b);

    if (aProjectsInsideTriangle && bProjectsInsideTriangle)
    {
        // We tested above for intersection, so cannot intersect now.
        if (d1 <= d2)
        {
            if (otherPt)
                *otherPt = line.a;
            return p.Project(line.a);
        }
        else
        {
            if (otherPt)
                *otherPt = line.b;
            return p.Project(line.b);
        }
    }
    LineSegment ab(a, b);
    LineSegment ac(a, c);
    LineSegment bc(b, c);

    float tab, tac, tbc;
    float tab2, tac2, tbc2;

    float dab = ab.Distance(line, &tab, &tab2);
    float dac = ac.Distance(line, &tac, &tac2);
    float dbc = bc.Distance(line, &tbc, &tbc2);

    if (dab <= dac && dab <= dbc && dab <= d1 && dab <= d2)
    {
        if (otherPt)
            *otherPt = line.GetPoint(tab2);
        return ab.GetPoint(tab);
    }
    else if (dac <= dbc && dac <= d1 && dac <= d2)
    {
        if (otherPt)
            *otherPt = line.GetPoint(tac2);
        return ab.GetPoint(tac);
    }
    else if (dbc <= d1 && dbc <= d2)
    {
        if (otherPt)
            *otherPt = line.GetPoint(tbc2);
        return ab.GetPoint(tbc);
    }
    else if (d1 <= d2)
    {
        if (otherPt)
            *otherPt = line.a;
        return p.Project(line.a);
    }
    else
    {
        if (otherPt)
            *otherPt = line.b;
        return p.Project(line.b);
    }
}

/// Implemented based on pseudo-code from Christer Ericson's Real-Time Collision Detection, pp. 155-156.
float3 Triangle::ClosestPoint(const Triangle &other, float3 *otherPt) const
{
    // First detect if the two triangles are intersecting.
    LineSegment l;
    bool success = this->Intersects(other, &l);
    if (success)
    {
        float3 cp = l.CenterPoint();
        if (otherPt)
            *otherPt = cp;
        return cp;
    }

    float3 closestThis = this->ClosestPoint(other.a);
    float3 closestOther = other.a;
    float closestDSq = closestThis.DistanceSq(closestOther);

    float3 pt = this->ClosestPoint(other.b);
    float dSq = pt.DistanceSq(other.b);
    if (dSq < closestDSq) closestThis = pt, closestOther = other.b, closestDSq = dSq;

    pt = this->ClosestPoint(other.c);
    dSq = pt.DistanceSq(other.c);
    if (dSq < closestDSq) closestThis = pt, closestOther = other.c, closestDSq = dSq;

    LineSegment l1[3] = { LineSegment(a,b), LineSegment(a,c), LineSegment(b,c) };
    LineSegment l2[3] = { LineSegment(other.a,other.b), LineSegment(other.a,other.c), LineSegment(other.b,other.c) };
    float d, d2;
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        {
            float dist = l1[i].Distance(l2[j], &d, &d2);
            if (dist*dist < closestDSq)
            {
                closestThis = l1[i].GetPoint(d);
                closestOther = l2[j].GetPoint(d2);
                closestDSq = dist*dist;
            }
        }

    if (otherPt)
        *otherPt = closestOther;
    return closestThis;
}

Triangle operator *(const float3x3 &transform, const Triangle &t)
{
    return Triangle(transform*t.a, transform*t.b, transform*t.c);
}

Triangle operator *(const float3x4 &transform, const Triangle &t)
{
    return Triangle(transform.MulPos(t.a), transform.MulPos(t.b), transform.MulPos(t.c));
}

Triangle operator *(const float4x4 &transform, const Triangle &t)
{
    return Triangle(transform.MulPos(t.a), transform.MulPos(t.b), transform.MulPos(t.c));
}

Triangle operator *(const Quat &transform, const Triangle &t)
{
    return Triangle(transform*t.a, transform*t.b, transform*t.c);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Triangle::ToString() const
{
    char str[256];
    sprintf(str, "Triangle(a:(%.2f, %.2f, %.2f) b:(%.2f, %.2f, %.2f) c:(%.2f, %.2f, %.2f))", 
        a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
    return str;
}
#endif

MATH_END_NAMESPACE
