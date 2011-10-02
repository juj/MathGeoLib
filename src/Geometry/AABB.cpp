/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#define _WINDOWS_

#include "Math/MathFunc.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#endif
#include "Geometry/AABB.h"
#include "Geometry/Frustum.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Algorithm/Random/LCG.h"
#include "Geometry/OBB.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Sphere.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4.h"
#include "Math/float4x4.h"
#include "Math/Quat.h"
#include "Geometry/Triangle.h"

AABB::AABB(const float3 &minPoint_, const float3 &maxPoint_)
:minPoint(minPoint_), maxPoint(maxPoint_)
{
}

AABB::AABB(const OBB &obb)
{
    SetFrom(obb);
}

AABB::AABB(const Sphere &s)
{
    SetFrom(s);
}

void AABB::SetNegativeInfinity()
{
    minPoint.SetFromScalar(FLOAT_INF);
    maxPoint.SetFromScalar(-FLOAT_INF);
}

void AABB::SetCenter(const float3 &center, const float3 &halfSize)
{
    minPoint = center - halfSize;
    maxPoint = center + halfSize;
}

void AABB::SetFrom(const OBB &obb)
{
    float3 halfSize = Abs(obb.axis[0]*obb.r[0]) + Abs(obb.axis[1]*obb.r[1]) + Abs(obb.axis[2]*obb.r[2]);
    SetCenter(obb.pos, halfSize);
}
 
//    bool AABB::SetFrom(const Polyhedron &polyhedron)
//{
//}

void AABB::SetFrom(const Sphere &s)
{
    minPoint = s.pos - float3(s.r, s.r, s.r);
    maxPoint = s.pos + float3(s.r, s.r, s.r);
}

void AABB::SetFrom(const float3 *pointArray, int numPoints)
{
    SetNegativeInfinity();
    for(int i = 0; i < numPoints; ++i)
        Enclose(pointArray[i]);
}

Polyhedron AABB::ToPolyhedron() const
{
    // Note for maintainer: This function is an exact copy of OBB:ToPolyhedron() and Frustum::ToPolyhedron().
    Polyhedron p;
    // Populate the corners of this AABB.
    // The will be in the order 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++.
    for(int i = 0; i < 8; ++i)
        p.v.push_back(CornerPoint(i));

    // Generate the 6 faces of this AABB.
    const int faces[6][4] = 
    { 
        { 0, 1, 3, 2 }, // X-
        { 4, 6, 7, 5 }, // X+
        { 0, 4, 5, 1 }, // Y-
        { 7, 6, 2, 3 }, // Y+
        { 0, 2, 6, 4 }, // Z-
        { 1, 5, 7, 3 }, // Z+
    };

    for(int f = 0; f < 6; ++f)
    {
        Polyhedron::Face face;
        for(int v = 0; v < 4; ++v)
            face.v.push_back(faces[f][v]);
        p.f.push_back(face);
    }

    return p;
}

OBB AABB::ToOBB() const
{
    return OBB(*this);
}

Sphere AABB::MinimalEnclosingSphere() const
{
    return Sphere(CenterPoint(), Size().Length()/2.f);
}

Sphere AABB::MaximalContainedSphere() const
{
    float3 size = Size();
    return Sphere(CenterPoint(), Min(Min(size.x, size.y), size.z));
}

bool AABB::IsFinite() const
{
    return minPoint.IsFinite() && maxPoint.IsFinite();
}

bool AABB::IsDegenerate() const
{
    return minPoint.x > maxPoint.x || minPoint.y > maxPoint.y || minPoint.z > maxPoint.z;
}

float3 AABB::CenterPoint() const
{
    return (minPoint + maxPoint) / 2.f;
}

float3 AABB::PointInside(float x, float y, float z) const
{
    assume(0.f <= x && x <= 1.f);
    assume(0.f <= y && y <= 1.f);
    assume(0.f <= z && z <= 1.f);

    float3 d = maxPoint - minPoint;
    return minPoint + float3(d.x*x, d.y*y, d.z*z);
}

LineSegment AABB::Edge(int edgeIndex) const
{
    assume(0 <= edgeIndex && edgeIndex <= 11);
    switch(edgeIndex)
    {
        default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
        case 0: return LineSegment(CornerPoint(0), CornerPoint(1));
        case 1: return LineSegment(CornerPoint(0), CornerPoint(2));
        case 2: return LineSegment(CornerPoint(0), CornerPoint(4));
        case 3: return LineSegment(CornerPoint(1), CornerPoint(3));
        case 4: return LineSegment(CornerPoint(1), CornerPoint(5));
        case 5: return LineSegment(CornerPoint(2), CornerPoint(3));
        case 6: return LineSegment(CornerPoint(2), CornerPoint(6));
        case 7: return LineSegment(CornerPoint(3), CornerPoint(7));
        case 8: return LineSegment(CornerPoint(4), CornerPoint(5));
        case 9: return LineSegment(CornerPoint(4), CornerPoint(6));
        case 10: return LineSegment(CornerPoint(5), CornerPoint(7));
        case 11: return LineSegment(CornerPoint(6), CornerPoint(7));
    }
}

float3 AABB::CornerPoint(int cornerIndex) const
{
    assume(0 <= cornerIndex && cornerIndex <= 7);
    switch(cornerIndex)
    {
        default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
        case 0: return float3(minPoint.x, minPoint.y, minPoint.z);
        case 1: return float3(minPoint.x, minPoint.y, maxPoint.z);
        case 2: return float3(minPoint.x, maxPoint.y, minPoint.z);
        case 3: return float3(minPoint.x, maxPoint.y, maxPoint.z);
        case 4: return float3(maxPoint.x, minPoint.y, minPoint.z);
        case 5: return float3(maxPoint.x, minPoint.y, maxPoint.z);
        case 6: return float3(maxPoint.x, maxPoint.y, minPoint.z);
        case 7: return float3(maxPoint.x, maxPoint.y, maxPoint.z);
    }
}

float3 AABB::PointOnEdge(int edgeIndex, float u) const
{
    assume(0 <= edgeIndex && edgeIndex <= 11);
    assume(0 <= u && u <= 1.f);

    float3 d = maxPoint - minPoint;
    switch(edgeIndex)
    {
    default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
    case 0: return float3(minPoint.x, minPoint.y, minPoint.z + u * d.z);
    case 1: return float3(minPoint.x, maxPoint.y, minPoint.z + u * d.z);
    case 2: return float3(maxPoint.x, minPoint.y, minPoint.z + u * d.z);
    case 3: return float3(maxPoint.x, maxPoint.y, minPoint.z + u * d.z);

    case 4: return float3(minPoint.x, minPoint.y + u * d.y, minPoint.z);
    case 5: return float3(maxPoint.x, minPoint.y + u * d.y, minPoint.z);
    case 6: return float3(minPoint.x, minPoint.y + u * d.y, maxPoint.z);
    case 7: return float3(maxPoint.x, minPoint.y + u * d.y, maxPoint.z);

    case 8: return float3(minPoint.x + u * d.x, minPoint.y, minPoint.z);
    case 9: return float3(minPoint.x + u * d.x, minPoint.y, maxPoint.z);
    case 10: return float3(minPoint.x + u * d.x, maxPoint.y, minPoint.z);
    case 11: return float3(minPoint.x + u * d.x, maxPoint.y, maxPoint.z);
    }
}

float3 AABB::FaceCenterPoint(int faceIndex) const
{
    assume(0 <= faceIndex && faceIndex <= 5);

    float3 center = (minPoint + maxPoint) / 2.f;
    switch(faceIndex)
    {
    default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
    case 0: return float3(minPoint.x, center.y, center.z);
    case 1: return float3(maxPoint.x, center.y, center.z);
    case 2: return float3(center.x, minPoint.y, center.z);
    case 3: return float3(center.x, maxPoint.y, center.z);
    case 4: return float3(center.x, center.y, minPoint.z);
    case 5: return float3(center.x, center.y, maxPoint.z);
    }
}

float3 AABB::FacePoint(int faceIndex, float u, float v) const
{
    assume(0 <= faceIndex && faceIndex <= 5);
    assume(0 <= u && u <= 1.f);
    assume(0 <= v && v <= 1.f);

    float3 d = maxPoint - minPoint;
    switch(faceIndex)
    {
    default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
    case 0: return float3(minPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
    case 1: return float3(maxPoint.x, minPoint.y + u * d.y, minPoint.z + v * d.z);
    case 2: return float3(minPoint.x + u * d.x, minPoint.y, minPoint.z + v * d.z);
    case 3: return float3(minPoint.x + u * d.x, maxPoint.y, minPoint.z + v * d.z);
    case 4: return float3(minPoint.x + u * d.x, minPoint.y + v * d.y, minPoint.z);
    case 5: return float3(minPoint.x + u * d.x, minPoint.y + v * d.y, maxPoint.z);
    }
}

Plane AABB::FacePlane(int faceIndex) const
{
    assume(0 <= faceIndex && faceIndex <= 5);
    switch(faceIndex)
    {
    default: // For release builds where assume() is disabled, return always the first option if out-of-bounds.
    case 0: return Plane(FaceCenterPoint(0), float3(-1,0,0));
    case 1: return Plane(FaceCenterPoint(1), float3( 1,0,0));
    case 2: return Plane(FaceCenterPoint(2), float3(0,-1,0));
    case 3: return Plane(FaceCenterPoint(3), float3(0, 1,0));
    case 4: return Plane(FaceCenterPoint(4), float3(0,0,-1));
    case 5: return Plane(FaceCenterPoint(5), float3(0,0, 1));
    }
}

void AABB::GetCornerPoints(float3 *outPointArray) const
{
    assert(outPointArray);
    for(int i = 0; i < 8; ++i)
        outPointArray[i] = CornerPoint(i);
}

void AABB::GetFacePlanes(Plane *outPlaneArray) const
{
    assert(outPlaneArray);
    for(int i = 0; i < 6; ++i)
        outPlaneArray[i] = FacePlane(i);
}

AABB AABB::MinimalEnclosingAABB(const float3 *pointArray, int numPoints)
{
    AABB aabb;
    aabb.SetFrom(pointArray, numPoints);
    return aabb;
}

void AABB::ExtremePointsAlongAABB(const float3 *pts, int numPoints, int &minx, int &maxx, int &miny, int &maxy, int &minz, int &maxz)
{
    assert(pts || numPoints == 0);
    minx = maxx = miny = maxy = minz = maxz = 0;
    for(int i = 1; i < numPoints; ++i)
    {
        if (pts[i].x < pts[minx].x) minx = i;
        if (pts[i].x > pts[maxx].x) maxx = i;
        if (pts[i].y < pts[miny].y) miny = i;
        if (pts[i].y > pts[maxy].y) maxy = i;
        if (pts[i].z < pts[minz].z) minz = i;
        if (pts[i].z > pts[maxz].z) maxz = i;
    }
}

AABB AABB::FromCenterAndSize(const float3 &aabbCenterPos, const float3 &aabbSize)
{
    float3 halfSize = aabbSize * 0.5f;
    return AABB(aabbCenterPos - halfSize, aabbCenterPos + halfSize);
}

float3 AABB::Size() const
{
    return maxPoint - minPoint;
}

float3 AABB::HalfSize() const
{
    return Size() / 2.f;
}

float AABB::Volume() const
{
    return Size().ProductOfElements();
}

    /// Returns the surface area of the faces of this AABB.
float AABB::SurfaceArea() const
{
    float3 size = Size();
    return 2.f * (size.x*size.y + size.x*size.z + size.y*size.z);
}

float3 AABB::RandomPointInside(LCG &rng) const
{
    return PointInside(rng.Float(), rng.Float(), rng.Float());
}

float3 AABB::RandomPointOnSurface(LCG &rng) const
{
    return FacePoint(rng.Int(0, 5), rng.Float(), rng.Float());
}

float3 AABB::RandomPointOnEdge(LCG &rng) const
{
    return PointOnEdge(rng.Int(0, 11), rng.Float());
}

float3 AABB::RandomCornerPoint(LCG &rng) const
{
    return CornerPoint(rng.Int(0, 7));
}

void AABB::Translate(const float3 &offset)
{
    minPoint += offset;
    maxPoint += offset;
}

void AABB::Scale(const float3 &centerPoint, float scaleFactor)
{
    return Scale(centerPoint, float3(scaleFactor, scaleFactor, scaleFactor));
}

void AABB::Scale(const float3 &centerPoint, const float3 &scaleFactor)
{
    float3x4 transform = float3x4::Scale(scaleFactor, centerPoint);
    minPoint = transform.MulPos(minPoint);
    maxPoint = transform.MulPos(maxPoint);
}

/// See Christer Ericson's Real-time Collision Detection, p. 87, or 
/// James Arvo's "Transforming Axis-aligned Bounding Boxes" in Graphics Gems 1, pp. 548-550.
/// http://www.graphicsgems.org/
template<typename Matrix>
void AABBTransformAsAABB(AABB &aabb, Matrix &m)
{
    float3 newCenter = m.MulPos(aabb.CenterPoint());

    float3 newDir;
    float3 h = aabb.HalfSize();
    // The following is equal to taking the absolute value of the whole matrix m.
    newDir.x = ABSDOT3(m[0], h);
    newDir.y = ABSDOT3(m[1], h);
    newDir.z = ABSDOT3(m[2], h);
    aabb.minPoint = newCenter - newDir;
    aabb.maxPoint = newCenter + newDir;
}

void AABB::TransformAsAABB(const float3x3 &transform)
{
    assume(transform.IsColOrthogonal());
    assume(transform.HasUniformScale());

    AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float3x4 &transform)
{
    assume(transform.IsColOrthogonal());
    assume(transform.HasUniformScale());

    AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const float4x4 &transform)
{
    assume(transform.IsColOrthogonal3());
    assume(transform.HasUniformScale());
    assume(transform.Row(3).Equals(0,0,0,1));

    AABBTransformAsAABB(*this, transform);
}

void AABB::TransformAsAABB(const Quat &transform)
{
    float3 newCenter = transform.Transform(CenterPoint());
    float3 newDir = Abs((transform.Transform(Size()) / 2.f));
    minPoint = newCenter - newDir;
    maxPoint = newCenter + newDir;
}

OBB AABB::Transform(const float3x3 &transform) const
{
    OBB obb;
    obb.SetFrom(*this, transform);
    return obb;
}

OBB AABB::Transform(const float3x4 &transform) const
{
    OBB obb;
    obb.SetFrom(*this, transform);
    return obb;
}

OBB AABB::Transform(const float4x4 &transform) const
{
    OBB obb;
    obb.SetFrom(*this, transform);
    return obb;
}

OBB AABB::Transform(const Quat &transform) const
{
    OBB obb;
    obb.SetFrom(*this, transform);
    return obb;
}

float3 AABB::ClosestPoint(const float3 &targetPoint) const
{
    return targetPoint.Clamp(minPoint, maxPoint);
}

float AABB::Distance(const float3 &point) const
{
    ///\todo This function could be slightly optimized. See Christer Ericson's
    /// Real-Time Collision Detection, p.131.
    return ClosestPoint(point).Distance(point);
}

float AABB::Distance(const Sphere &sphere) const
{
    return Max(0.f, Distance(sphere.pos) - sphere.r);
}

/*
float AABB::Distance(const Ray &ray, float3 *outClosestPoint, float *outClosestDistance) const
float Distance(const Line &line, float3 *outClosestPoint, float *outClosestdistance) const;
float Distance(const LineSegment &lineSegment, float3 *outClosestPoint, float *outClosestDistance) const;
float Distance(const AABB &aabb, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const OBB &obb, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Plane &plane, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Sphere &sphere, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Ellipsoid &ellipsoid, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Triangle &triangle, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Cylinder &cylinder, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Capsule &capsule, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Torus &torus, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
float Distance(const Frustum &frustum, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Polygon &polygon, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
//    float Distance(const Polyhedron &polyhedron, float3 *outClosestPoint, float3 *outClosestPointOther) const; ///< [noscript]
*/
bool AABB::Contains(const float3 &point) const
{
    return minPoint.x <= point.x && point.x <= maxPoint.x &&
           minPoint.y <= point.y && point.y <= maxPoint.y &&
           minPoint.z <= point.z && point.z <= maxPoint.z;
}

bool AABB::Contains(const LineSegment &lineSegment) const
{
    return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool AABB::Contains(const AABB &aabb) const
{
    return Contains(aabb.minPoint) && Contains(aabb.maxPoint);
}

bool AABB::Contains(const OBB &obb) const
{
    return Contains(obb.MinimalEnclosingAABB());
}

bool AABB::Contains(const Sphere &sphere) const
{
    ///\todo Optimize.
    return Contains(sphere.MinimalEnclosingAABB());
}

bool AABB::Contains(const Triangle &triangle) const
{
    return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}
/*
bool AABB::Contains(const Polygon &polygon) const
{
    return Contains(polygon.MinimalEnclosingAABB());
}
*/
bool AABB::Contains(const Frustum &frustum) const
{
    return Contains(frustum.MinimalEnclosingAABB());
}

bool AABB::Contains(const Polyhedron &polyhedron) const
{
    return Contains(polyhedron.MinimalEnclosingAABB());
}

/** Computes the intersection of a ray and a AABB.
	Based on "T. Kay, J. Kajiya. Ray Tracing Complex Scenes. SIGGRAPH 1986 vol 20, number 4. pp. 269-"
	http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
	@param r The ray to test. The ray direction vector must be normalized!
	@param aabb The aabb to test.
	@param tNear [out] If intersection occurs, the signed distance from ray origin to the ray entry point in aabb
		is returned here.
	@param tFar [out] If intersection occurs, the signed distance from ray origin to the ray exit point in aabb
		is returned here.
	@return True if an intersection occurs, false otherwise. */
bool IntersectRayAABB(const float3 &rayPos, const float3 &rayDir, const AABB &aabb, float &tNear, float &tFar)
{
    assume(rayDir.IsNormalized());
    tNear = -FLOAT_INF;
    tFar = FLOAT_INF;

	for(int i = 0; i < 3; ++i) // loop for each AABB plane (X,Y,Z)
	{
		if (EqualAbs(rayDir[i], 0.f)) // ray is parallel to plane in question
            if (rayPos[i] < aabb.minPoint[i] || rayPos[i] > aabb.maxPoint[i]) // early-out if the ray can't possibly enter the box.
				return false;

        // intersection distances to plane.
        float recipDir = 1.f / rayDir[i];
        float t1 = (aabb.minPoint[i] - rayPos[i]) * recipDir;
		float t2 = (aabb.maxPoint[i] - rayPos[i]) * recipDir;

		if (t1 > t2) Swap(t1, t2); // swap so that t1 is the distance to nearer of the two planes.
		if (t1 > tNear) tNear = t1; // tNear tracks distance to intersect the AABB.
		if (t2 < tFar)
			tFar = t2; // tFar tracks the distance to exit the AABB.
		if (tNear > tFar) // Box is missed since we "exit" before entering it.
			return false;
		if (tFar < 0) // Box is behind the ray.
			return false;
	}
	return true;
}

/// Computes the intersection of a line and a AABB. For reference, see IntersectRayAABB.
bool IntersectLineAABB(const float3 &linePos, const float3 &lineDir, const AABB &aabb, float &tNear, float &tFar)
{
    tNear = -FLOAT_INF;
    tFar = FLOAT_INF;

	for(int i = 0; i < 3; ++i) // loop for each AABB plane (X,Y,Z)
	{
		if (EqualAbs(lineDir[i], 0.f)) // ray is parallel to plane in question
			if (linePos[i] < aabb.minPoint[i] || linePos[i] > aabb.maxPoint[i]) // early-out if the ray can't possibly enter the box.
				return false;

        // intersection distances to plane.
        float recipDir = 1.f / lineDir[i];
        float t1 = (aabb.minPoint[i] - linePos[i]) * recipDir;
		float t2 = (aabb.maxPoint[i] - linePos[i]) * recipDir;

		if (t1 > t2) Swap(t1, t2); // swap so that t1 is the distance to nearer of the two planes.
		if (t1 > tNear) tNear = t1; // tNear tracks distance to intersect the AABB.
		if (t2 < tFar)
			tFar = t2; // tFar tracks the distance to exit the AABB.
		if (tNear > tFar) // Box is missed since we "exit" before entering it.
			return false;
	}
	return true;
}

bool AABB::Intersects(const Ray &ray, float *dNear, float *dFar) const
{
    float tNear, tFar;
    bool success = IntersectRayAABB(ray.pos, ray.dir, *this, tNear, tFar);
    if (dNear)
        *dNear = tNear;
    if (dFar)
        *dFar = tFar;
    return success;
}

bool AABB::Intersects(const Line &line, float *dNear, float *dFar) const
{
    float tNear, tFar;
    bool success = IntersectLineAABB(line.pos, line.dir, *this, tNear, tFar);
    if (dNear)
        *dNear = tNear;
    if (dFar)
        *dFar = tFar;
    return success;
}

bool AABB::Intersects(const LineSegment &lineSegment, float *dNear, float *dFar) const
{
    float tNear, tFar;
    bool success = IntersectLineAABB(lineSegment.a, lineSegment.Dir(), *this, tNear, tFar);
    success = (tNear >= 0.f && tNear*tNear <= lineSegment.LengthSq());
    if (dNear)
        *dNear = tNear / lineSegment.Length();
    if (dFar)
        *dFar = tFar / lineSegment.Length();
    return success;
}

bool AABB::Intersects(const Plane &plane) const
{
    return plane.Intersects(*this);
}

bool AABB::Intersects(const AABB &aabb) const
{
    // If any of the cardinal X,Y,Z axes is a separating axis, then
    // there is no intersection.
    return !(minPoint.x >= aabb.maxPoint.x ||
             minPoint.y >= aabb.maxPoint.y ||
             minPoint.z >= aabb.maxPoint.z ||
             aabb.minPoint.x >= maxPoint.x ||
             aabb.minPoint.y >= maxPoint.y ||
             aabb.minPoint.z >= maxPoint.z);
}

bool AABB::Intersects(const OBB &obb) const
{
    return obb.Intersects(*this);
}

/// See Christer Ericson's Real-Time Collision Detection, p. 165.
bool AABB::Intersects(const Sphere &sphere, float3 *closestPointOnAABB) const
{
    // Find the point on this AABB closest to the sphere center.
    float3 pt = ClosestPoint(sphere.pos);

    // If that point is inside sphere, the AABB and sphere intersect.
    if (closestPointOnAABB)
        *closestPointOnAABB = pt;

    return pt.DistanceSq(sphere.pos) <= sphere.r * sphere.r;
}

bool AABB::Intersects(const Triangle &triangle) const
{
    return triangle.Intersects(*this);
}

#ifndef _WINDOWS_
bool AABB::Intersects(const Polygon &polygon) const
{
    return ToPolyhedron().Intersects(polygon);
}
#endif

bool AABB::Intersects(const Frustum &frustum) const
{
    return frustum.Intersects(*this);
}

bool AABB::Intersects(const Polyhedron &polyhedron) const
{
    return polyhedron.Intersects(*this);
}

void AABB::ProjectToAxis(const float3 &axis, float &dMin, float &dMax) const
{
    float3 c = CenterPoint();
    float3 e = HalfDiagonal();

    // Compute the projection interval radius of the AABB onto L(t) = aabb.center + t * plane.normal;
    float r = e[0]*Abs(axis[0]) + e[1]*Abs(axis[1]) + e[2]*Abs(axis[2]);
    // Compute the distance of the box center from plane.
    float s = Dot(axis, c);
    dMin = s - r;
    dMax = s + r;
    if (dMin > dMax)
        Swap(dMin, dMax);
}

/*
HitInfo Intersect(const Ray &ray, float *outDistance) const; ///< [noscript]
HitInfo Intersect(const Ray &ray, float maxDistance, float *outDistance) const; ///< [noscript]
HitInfo Intersect(const Line &line, float *outDistance) const; ///< [noscript]
HitInfo Intersect(const LineSegment &lineSegment, float *outDistance) const; ///< [noscript]
HitInfo Intersect(const AABB &aabb) const; ///< [noscript]
HitInfo Intersect(const OBB &obb) const; ///< [noscript]
HitInfo Intersect(const Plane &plane) const; ///< [noscript]
HitInfo Intersect(const Sphere &sphere) const; ///< [noscript]
HitInfo Intersect(const Ellipsoid &ellipsoid) const; ///< [noscript]
HitInfo Intersect(const Triangle &triangle) const; ///< [noscript]
HitInfo Intersect(const Cylinder &cylinder) const; ///< [noscript]
//    HitInfo Intersect(const Capsule &capsule) const; ///< [noscript]
//    HitInfo Intersect(const Torus &torus) const; ///< [noscript]
HitInfo Intersect(const Frustum &frustum) const; ///< [noscript]
//    HitInfo Intersect(const Polygon &polygon) const; ///< [noscript]
//    HitInfo Intersect(const Polyhedron &polyhedron) const; ///< [noscript]
*/
void AABB::Enclose(const float3 &point)
{
    minPoint = Min(minPoint, point);
    maxPoint = Max(maxPoint, point);
}

void AABB::Enclose(const LineSegment &lineSegment)
{
    Enclose(lineSegment.a);
    Enclose(lineSegment.b);
}

void AABB::Enclose(const AABB &aabb)
{
    Enclose(aabb.minPoint);
    Enclose(aabb.maxPoint);
}

void AABB::Enclose(const OBB &obb)
{
    for(int i = 0; i < 8; ++i)
        Enclose(obb.CornerPoint(i));
}

/*
void AABB::Enclose(const OBB &obb)
{
}
*/
void AABB::Enclose(const Sphere &sphere)
{
    Enclose(sphere.pos - float3(sphere.r,sphere.r,sphere.r));
    Enclose(sphere.pos + float3(sphere.r,sphere.r,sphere.r));
}
/*
void Enclose(const Ellipsoid &ellipsoid);
void Enclose(const Triangle &triangle);
void Enclose(const Cylinder &cylinder);
void Enclose(const Frustum &frustum);
*/
void AABB::Enclose(const float3 *pointArray, int numPoints)
{
    for(int i = 0; i < numPoints; ++i)
        Enclose(pointArray[i]);
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string AABB::ToString() const
{
    char str[256];
    sprintf(str, "AABB(Min:(%.2f, %.2f, %.2f) Max:(%.2f, %.2f, %.2f))", minPoint.x, minPoint.y, minPoint.z, maxPoint.x, maxPoint.y, maxPoint.z);
    return str;
}
#endif

AABB AABB::Intersection(const AABB &aabb) const
{
    return AABB(Max(minPoint, aabb.minPoint), Min(maxPoint, aabb.maxPoint));
}

//    Polyhedron Intersection(const OBB &obb) const;

//    Polyhedron Intersection(const Polyhedron &polyhedron) const;
