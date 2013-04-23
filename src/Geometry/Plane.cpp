/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file Plane.cpp
	@author Jukka Jylänki
	@brief Implementation for the Plane geometry object. */
#include "Plane.h"
#include "../Math/MathFunc.h"
#include "../Math/Polynomial.h"
#include "AABB.h"
#include "Circle.h"
#include "Line.h"
#include "OBB.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Ray.h"
#include "Capsule.h"
#include "Sphere.h"
#include "Triangle.h"
#include "LineSegment.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/Quat.h"
#include "Frustum.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <iostream>
#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

Plane::Plane(const float3 &normal_, float d_)
:normal(normal_), d(d_)
{
	assume(normal.IsNormalized());
}

Plane::Plane(const float3 &v1, const float3 &v2, const float3 &v3)
{
	Set(v1, v2, v3);
}

Plane::Plane(const float3 &point, const float3 &normal_)
{
	Set(point, normal_);
}

Plane::Plane(const Ray &ray, const float3 &normal)
{
	float3 perpNormal = normal - normal.ProjectToNorm(ray.dir);
	Set(ray.pos, perpNormal.Normalized());
}

Plane::Plane(const Line &line, const float3 &normal)
{
	float3 perpNormal = normal - normal.ProjectToNorm(line.dir);
	Set(line.pos, perpNormal.Normalized());
}

Plane::Plane(const LineSegment &lineSegment, const float3 &normal)
{
	float3 perpNormal = normal - normal.ProjectTo(lineSegment.b - lineSegment.a);
	Set(lineSegment.a, perpNormal.Normalized());
}

bool Plane::IsDegenerate() const
{
	return !normal.IsFinite() || normal.IsZero() || !IsFinite(d);
}

void Plane::Set(const float3 &v1, const float3 &v2, const float3 &v3)
{
	normal = ((v2-v1).Cross(v3-v1)).Normalized();
	d = Dot(v1, normal);

#ifdef MATH_ASSERT_CORRECTNESS
	float d2 = Dot(v2, normal);
	float d3 = Dot(v3, normal);
	mathassert(EqualAbs(d, d2, 1e-2f));
	mathassert(EqualAbs(d, d3, 1e-2f));
#endif
}

void Plane::Set(const float3 &point, const float3 &normal_)
{
	normal = normal_;
	assume(normal.IsNormalized());
	d = Dot(point, normal);

	mathassert(EqualAbs(SignedDistance(point), 0.f, 0.01f));
	mathassert(EqualAbs(SignedDistance(point + normal_), 1.f, 0.01f));
}

void Plane::ReverseNormal()
{
	normal = -normal;
	d = -d;
}

float3 Plane::PointOnPlane() const
{
	return normal * d;
}

float3 Plane::Point(float u, float v) const
{
	return PointOnPlane() + u * normal.Perpendicular() + v * normal.AnotherPerpendicular();
}

float3 Plane::Point(float u, float v, const float3 &referenceOrigin) const
{
	return Project(referenceOrigin) + u * normal.Perpendicular() + v * normal.AnotherPerpendicular();
}

void Plane::Translate(const float3 &offset)
{
	d -= Dot(normal, offset);
}

void Plane::Transform(const float3x3 &transform)
{
	float3x3 it = transform.InverseTransposed(); ///@todo Could optimize the inverse here by assuming orthogonality or orthonormality.
	normal = it * normal;
}

/// For Plane-float3x4 transform code, see Eric Lengyel's Mathematics for 3D Game Programming And Computer Graphics 2nd ed., p.110, chapter 4.2.3. [groupSyntax]
void Plane::Transform(const float3x4 &transform)
{
	///@todo Could optimize this function by switching to plane convention ax+by+cz+d=0 instead of ax+by+cz=d.
	float3x3 r = transform.Float3x3Part();
	bool success = r.Inverse(); ///@todo Can optimize the inverse here by assuming orthogonality or orthonormality.
	assume(success);
	MARK_UNUSED(success);
	d = d + Dot(normal, r * transform.TranslatePart());
	normal = normal * r;
}

void Plane::Transform(const float4x4 &transform)
{
	assume(transform.Row(3).Equals(float4(0,0,0,1)));
	Transform(transform.Float3x4Part());
}

void Plane::Transform(const Quat &transform)
{
	float3x3 r = transform.ToFloat3x3();
	Transform(r);
}

bool Plane::IsInPositiveDirection(const float3 &directionVector) const
{
	return normal.Dot(directionVector) >= 0.f;
}

bool Plane::IsOnPositiveSide(const float3 &point) const
{
	return SignedDistance(point) >= 0.f;
}

int Plane::ExamineSide(const Triangle &triangle) const
{
	float a = SignedDistance(triangle.a);
	float b = SignedDistance(triangle.b);
	float c = SignedDistance(triangle.c);
	const float epsilon = 1e-4f; // Allow a small epsilon amount for tests for floating point inaccuracies.
	if (a >= -epsilon && b >= -epsilon && c >= -epsilon)
		return 1;
	if (a <= epsilon && b <= epsilon && c <= epsilon)
		return -1;
	return 0;
}

bool Plane::AreOnSameSide(const float3 &p1, const float3 &p2) const
{
	return SignedDistance(p1) * SignedDistance(p2) >= 0.f;
}

float Plane::Distance(const float3 &point) const
{
	return Abs(SignedDistance(point));
}

float Plane::Distance(const LineSegment &lineSegment) const
{
	return lineSegment.Distance(*this);
}

float Plane::Distance(const Sphere &sphere) const
{
	return Max(0.f, Distance(sphere.pos) - sphere.r);
}

float Plane::Distance(const Capsule &capsule) const
{
	return Max(0.f, Distance(capsule.l) - capsule.r);
}

float Plane::SignedDistance(const float3 &point) const
{
	return normal.Dot(point) - d;
}

template<typename T>
float Plane_SignedDistance(const Plane &plane, const T &object)
{
	float pMin, pMax;
	assume(plane.normal.IsNormalized());
	object.ProjectToAxis(plane.normal, pMin, pMax);
	pMin -= plane.d;
	pMax -= plane.d;
	if (pMin * pMax <= 0.f)
		return 0.f;
	return Abs(pMin) < Abs(pMax) ? pMin : pMax;
}

float Plane::SignedDistance(const AABB &aabb) const { return Plane_SignedDistance(*this, aabb); }
float Plane::SignedDistance(const OBB &obb) const { return Plane_SignedDistance(*this, obb); }
float Plane::SignedDistance(const Capsule &capsule) const { return Plane_SignedDistance(*this, capsule); }
//float Plane::SignedDistance(const Circle &circle) const { return Plane_SignedDistance(*this, circle); }
float Plane::SignedDistance(const Frustum &frustum) const { return Plane_SignedDistance(*this, frustum); }
float Plane::SignedDistance(const Line &line) const { return Plane_SignedDistance(*this, line); }
float Plane::SignedDistance(const LineSegment &lineSegment) const { return Plane_SignedDistance(*this, lineSegment); }
float Plane::SignedDistance(const Ray &ray) const { return Plane_SignedDistance(*this, ray); }
//float Plane::SignedDistance(const Plane &plane) const { return Plane_SignedDistance(*this, plane); }
float Plane::SignedDistance(const Polygon &polygon) const { return Plane_SignedDistance(*this, polygon); }
float Plane::SignedDistance(const Polyhedron &polyhedron) const { return Plane_SignedDistance(*this, polyhedron); }
float Plane::SignedDistance(const Sphere &sphere) const { return Plane_SignedDistance(*this, sphere); }
float Plane::SignedDistance(const Triangle &triangle) const { return Plane_SignedDistance(*this, triangle); }

float3x4 Plane::OrthoProjection() const
{
	return float3x4::OrthographicProjection(*this);
}

#if 0
float3x4 Plane::ObliqueProjection(const float3 & /*obliqueProjectionDir*/) const
{
#ifdef _MSC_VER
#pragma WARNING(Plane::ObliqueProjection not implemented!)
#else
#warning Plane::ObliqueProjection not implemented!
#endif
	assume(false && "Plane::ObliqueProjection not implemented!"); /// @todo Implement.
	return float3x4();
}
#endif

float3x4 Plane::MirrorMatrix() const
{
	return float3x4::Mirror(*this);
}

float3 Plane::Mirror(const float3 &point) const
{
#ifdef MATH_ASSERT_CORRECTNESS
	float signedDistance = SignedDistance(point);
#endif
	assume(normal.IsNormalized());
	float3 reflected = point - 2.f * (Dot(point, normal) - d) * normal;
	mathassert(EqualAbs(signedDistance, -SignedDistance(reflected)));
	mathassert(reflected.Equals(MirrorMatrix().MulPos(point)));
	return reflected;
}

float3 Plane::Refract(const float3 &vec, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const
{
	return float3(vec).Refract(normal, negativeSideRefractionIndex, positiveSideRefractionIndex);
}

float3 Plane::Project(const float3 &point) const
{
	float3 projected = point - (Dot(normal, point) - d) * normal;
	mathassert(projected.Equals(OrthoProjection().MulPos(point)));
	return projected;
}

LineSegment Plane::Project(const LineSegment &lineSegment) const
{
	return LineSegment(Project(lineSegment.a), Project(lineSegment.b));
}

Line Plane::Project(const Line &line, bool *nonDegenerate) const
{
	Line l;
	l.pos = Project(line.pos);
	l.dir = l.dir - l.dir.ProjectToNorm(normal);
	float len = l.dir.Normalize();
	if (nonDegenerate)
		*nonDegenerate = (len > 0.f);
	return l;
}

Ray Plane::Project(const Ray &ray, bool *nonDegenerate) const
{
	Ray r;
	r.pos = Project(ray.pos);
	r.dir = r.dir - r.dir.ProjectToNorm(normal);
	float len = r.dir.Normalize();
	if (nonDegenerate)
		*nonDegenerate = (len > 0.f);
	return r;
}

Triangle Plane::Project(const Triangle &triangle) const
{
	Triangle t;
	t.a = Project(triangle.a);
	t.b = Project(triangle.b);
	t.c = Project(triangle.c);
	return t;
}

Polygon Plane::Project(const Polygon &polygon) const
{
	Polygon p;
	for(size_t i = 0; i < polygon.p.size(); ++i)
		p.p.push_back(Project(polygon.p[i]));

	return p;
}

float3 Plane::ClosestPoint(const Ray &ray) const
{
	assume(ray.IsFinite());
	assume(!IsDegenerate());

	float denom = Dot(normal, ray.dir);
	if (EqualAbs(denom, 0.f))
		return Project(ray.pos); // Output t == 0.
	else
	{
		///@todo Output parametric t along the ray as well.
		float t = (d - Dot(normal, ray.pos)) / denom;
		return ray.GetPoint(t);
	}
}

float3 Plane::ClosestPoint(const LineSegment &lineSegment) const
{
	/*
	///@todo Output parametric d as well.
	float d;
	if (lineSegment.Intersects(*this, &d))
		return lineSegment.GetPoint(d);
	else
		if (Distance(lineSegment.a) < Distance(lineSegment.b))
			return Project(lineSegment.a);
		else
			return Project(lineSegment.b);
	*/

	assume(lineSegment.IsFinite());
	assume(!IsDegenerate());

	float aDist = Dot(normal, lineSegment.a);
	float bDist = Dot(normal, lineSegment.b);

	float denom = bDist - aDist;
	if (EqualAbs(denom, 0.f))
		return Project(Abs(aDist) < Abs(bDist) ? lineSegment.a : lineSegment.b); // Project()ing the result here is not strictly necessary,
		                                                                         // but done for numerical stability, so that Plane::Contains()
		                                                                         // will return true for the returned point.
	else
	{
		///@todo Output parametric t along the ray as well.
		float t = (d - Dot(normal, lineSegment.a)) / (bDist - aDist);
		t = Clamp01(t);
		// Project()ing the result here is necessary only if we clamped, but done for numerical stability, so that Plane::Contains() will
		// return true for the returned point.
		return Project(lineSegment.GetPoint(t));
	}
}

#if 0
float3 Plane::ObliqueProject(const float3 & /*point*/, const float3 & /*obliqueProjectionDir*/) const
{
#ifdef _MSC_VER
#pragma WARNING(Plane::ObliqueProject not implemented!)
#else
#warning Plane::ObliqueProject not implemented!
#endif
	assume(false && "Plane::ObliqueProject not implemented!"); /// @todo Implement.
	return float3();
}
#endif

bool Plane::Contains(const float3 &point, float distanceThreshold) const
{
	return Distance(point) <= distanceThreshold;
}

bool Plane::Contains(const Line &line, float epsilon) const
{
	return Contains(line.pos) && line.dir.IsPerpendicular(normal, epsilon);
}

bool Plane::Contains(const Ray &ray, float epsilon) const
{
	return Contains(ray.pos) && ray.dir.IsPerpendicular(normal, epsilon);
}

bool Plane::Contains(const LineSegment &lineSegment, float epsilon) const
{
	return Contains(lineSegment.a, epsilon) && Contains(lineSegment.b, epsilon);
}

bool Plane::Contains(const Triangle &triangle, float epsilon) const
{
	return Contains(triangle.a, epsilon) && Contains(triangle.b, epsilon) && Contains(triangle.c, epsilon);
}

bool Plane::Contains(const Circle &circle, float epsilon) const
{
	return Contains(circle.pos, epsilon) && (EqualAbs(Abs(Dot(normal, circle.normal)), 1.f) || circle.r <= epsilon);
}

bool Plane::Contains(const Polygon &polygon, float epsilon) const
{
	switch(polygon.NumVertices())
	{
	case 0: assume(false && "Plane::Contains(Polygon) called with a degenerate polygon of 0 vertices!"); return false;
	case 1: return Contains(polygon.Vertex(0), epsilon);
	case 2: return Contains(polygon.Vertex(0), epsilon) && Contains(polygon.Vertex(1), epsilon);
	default:
		return SetEquals(polygon.PlaneCCW(), epsilon);
	  }
}

bool Plane::SetEquals(const Plane &plane, float epsilon) const
{
	return (normal.Equals(plane.normal) && EqualAbs(d, plane.d, epsilon)) ||
		(normal.Equals(-plane.normal) && EqualAbs(-d, plane.d, epsilon));
}

bool Plane::Equals(const Plane &other, float epsilon) const
{
	return IsParallel(other, epsilon) && EqualAbs(d, other.d, epsilon);
}

bool Plane::Intersects(const Plane &plane, Line *outLine) const
{
	float3 perp = normal.Perpendicular(plane.normal);//float3::Perpendicular Cross(normal, plane.normal);

	float3x3 m;
	m.SetRow(0, normal);
	m.SetRow(1, plane.normal);
	m.SetRow(2, perp); // This is arbitrarily chosen, to produce m invertible.
	float3 intersectionPos;
	bool success = m.SolveAxb(float3(d, plane.d, 0.f),intersectionPos);
	if (!success) // Inverse failed, so the planes must be parallel.
	{
		float normalDir = Dot(normal,plane.normal);
		if ((normalDir > 0.f && EqualAbs(d, plane.d)) || (normalDir < 0.f && EqualAbs(d, -plane.d)))
		{
			if (outLine)
				*outLine = Line(normal*d, plane.normal.Perpendicular());
			return true;
		}
		else
			return false;
	}
	if (outLine)
		*outLine = Line(intersectionPos, perp.Normalized());
	return true;
}

bool Plane::Intersects(const Plane &plane, const Plane &plane2, Line *outLine, float3 *outPoint) const
{
	Line dummy;
	if (!outLine)
		outLine = &dummy;

	// First check all planes for parallel pairs.
	if (this->IsParallel(plane) || this->IsParallel(plane2))
	{
		if (EqualAbs(d, plane.d) || EqualAbs(d, plane2.d))
		{
			bool intersect = plane.Intersects(plane2, outLine);
			if (intersect && outPoint)
				*outPoint = outLine->GetPoint(0);
			return intersect;
		}
		else
			return false;
	}
	if (plane.IsParallel(plane2))
	{
		if (EqualAbs(plane.d, plane2.d))
		{
			bool intersect = this->Intersects(plane, outLine);
			if (intersect && outPoint)
				*outPoint = outLine->GetPoint(0);
			return intersect;
		}
		else
			return false;
	}

	// All planes point to different directions.
	float3x3 m;
	m.SetRow(0, normal);
	m.SetRow(1, plane.normal);
	m.SetRow(2, plane2.normal);
	float3 intersectionPos;
	bool success = m.SolveAxb(float3(d, plane.d, plane2.d), intersectionPos);
	if (!success)
		return false;
	if (outPoint)
		*outPoint = intersectionPos;
	return true;
}

bool Plane::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

#if 0
bool Plane::IntersectLinePlane(const float3 &p, const float3 &n, const float3 &a, const float3 &d, float &t)
{
	/* The set of points x lying on a plane is defined by the equation

		(x - p)*n == 0, where p is a point on the plane, and n is the plane normal.

	The set of points x on a line is constructed explicitly by a single parameter t by

		x = a + t*d, where a is a point on the line, and d is the direction vector of the line.

	To solve the intersection of these two objects, substitute the second equation to the first above,
	and we get

		  (a + t*d - p)*n == 0, or
		t*(d*n) + (a-p)*n == 0, or
	                    t == (p-a)*n / (d*n), assuming that d*n != 0.

	If d*n == 0, then the line is parallel to the plane, and either no intersection occurs, or the whole line
	is embedded on the plane, and infinitely many intersections occur. */

	float denom = Dot(d, n);
	if (EqualAbs(denom, 0.f))
	{
		t = 0.f;
		float f = Dot(a-p, n);
		bool b = EqualAbs(Dot(a-p, n), 0.f);
		return EqualAbs(Dot(a-p, n), 0.f); // If (a-p)*n == 0, then then above equation holds for all t, and return true.
	}
	else
	{
		// Compute the distance from the line starting point to the point of intersection.
		t = Dot(p - a, n) / denom;
		return true;
	}
}
#endif

bool Plane::IntersectLinePlane(const float3 &planeNormal, float planeD, const float3 &linePos, const float3 &lineDir, float &t)
{
	/* The set of points x lying on a plane is defined by the equation

		<planeNormal, x> = planeD.

	The set of points x on a line is constructed explicitly by a single parameter t by

		x = linePos + t*lineDir.

	To solve the intersection of these two objects, substitute the second equation to the first above,
	and we get

	                     <planeNormal, linePos + t*lineDir> == planeD, or
	    <planeNormal, linePos> + t * <planeNormal, lineDir> == planeD, or
	                                                      t == (planeD - <planeNormal, linePos>) / <planeNormal, lineDir>,
	
	                                                           assuming that <planeNormal, lineDir> != 0.

	If <planeNormal, lineDir> == 0, then the line is parallel to the plane, and either no intersection occurs, or the whole line
	is embedded on the plane, and infinitely many intersections occur. */

	float denom = Dot(planeNormal, lineDir);
	if (EqualAbs(denom, 0.f))
	{
		t = 0.f;
		return EqualAbs(Dot(planeNormal, linePos), planeD, 1e-2f);
	}
	else
	{
		// Compute the distance from the line starting point to the point of intersection.
		t = (planeD - Dot(planeNormal, linePos)) / denom;
		return true;
	}
}


bool Plane::Intersects(const Ray &ray, float *d) const
{
	float t;
	bool success = IntersectLinePlane(normal, this->d, ray.pos, ray.dir, t);
	if (d)
		*d = t;
	return success && t >= 0.f;
}

bool Plane::Intersects(const Line &line, float *d) const
{
	float t;
	bool intersects = IntersectLinePlane(normal, this->d, line.pos, line.dir, t);
	if (d)
		*d = t;
	return intersects;
}

bool Plane::Intersects(const LineSegment &lineSegment, float *d) const
{
	float t;
	bool success = IntersectLinePlane(normal, this->d, lineSegment.a, lineSegment.Dir(), t);
	const float lineSegmentLength = lineSegment.Length();
	if (d)
		*d = t / lineSegmentLength;
	return success && t >= 0.f && t <= lineSegmentLength;
}

bool Plane::Intersects(const Sphere &sphere) const
{
	return Distance(sphere.pos) <= sphere.r;
}

bool Plane::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

/// The Plane-AABB intersection is implemented according to Christer Ericson's Real-Time Collision Detection, p.164. [groupSyntax]
bool Plane::Intersects(const AABB &aabb) const
{
	float3 c = aabb.CenterPoint();
	float3 e = aabb.HalfDiagonal();

	// Compute the projection interval radius of the AABB onto L(t) = aabb.center + t * plane.normal;
	float r = e[0]*Abs(normal[0]) + e[1]*Abs(normal[1]) + e[2]*Abs(normal[2]);
	// Compute the distance of the box center from plane.
	float s = Dot(normal, c) - d;
	return Abs(s) <= r;
}

bool Plane::Intersects(const OBB &obb) const
{
	return obb.Intersects(*this);
}

bool Plane::Intersects(const Triangle &triangle) const
{
	float a = SignedDistance(triangle.a);
	float b = SignedDistance(triangle.b);
	float c = SignedDistance(triangle.c);
	return (a*b <= 0.f || a*c <= 0.f);
}

bool Plane::Intersects(const Frustum &frustum) const
{
	bool sign = IsOnPositiveSide(frustum.CornerPoint(0));
	for(int i = 1; i < 8; ++i)
		if (sign != IsOnPositiveSide(frustum.CornerPoint(i)))
			return true;
	return false;
}

bool Plane::Intersects(const Polyhedron &polyhedron) const
{
	if (polyhedron.NumVertices() == 0)
		return false;
	bool sign = IsOnPositiveSide(polyhedron.Vertex(0));
	for(int i = 1; i < polyhedron.NumVertices(); ++i)
		if (sign != IsOnPositiveSide(polyhedron.Vertex(i)))
			return true;
	return false;
}

int Plane::Intersects(const Circle &circle, float3 *pt1, float3 *pt2) const
{
	Line line;
	bool planeIntersects = Intersects(circle.ContainingPlane(), &line);
	if (!planeIntersects)
		return false;

	// Offset both line and circle position so the circle origin is at center.
	line.pos -= circle.pos;

	float a = 1.f;
	float b = 2.f * Dot(line.pos, line.dir);
	float c = line.pos.LengthSq() - circle.r * circle.r;
	float r1, r2;
	int numRoots = Polynomial::SolveQuadratic(a, b, c, r1, r2);
	if (numRoots >= 1 && pt1)
		*pt1 = circle.pos + line.GetPoint(r1);
	if (numRoots >= 2 && pt2)
		*pt2 = circle.pos + line.GetPoint(r2);
	return numRoots;
}

int Plane::Intersects(const Circle &circle) const
{
	return Intersects(circle, 0, 0);
}

bool Plane::Clip(float3 &a, float3 &b) const
{
	float t;
	bool intersects = IntersectLinePlane(normal, d, a, b-a, t);
	if (!intersects || t <= 0.f || t >= 1.f)
	{
		if (SignedDistance(a) <= 0.f)
			return false; // Discard the whole line segment, it's completely behind the plane.
		else
			return true; // The whole line segment is in the positive halfspace. Keep all of it.
	}
	float3 pt = a + (b-a) * t; // The intersection point.
	// We are either interested in the line segment [a, pt] or the segment [pt, b]. Which one is in the positive side?
	if (IsOnPositiveSide(a))
		b = pt;
	else
		a = pt;

	return true;
}

bool Plane::Clip(LineSegment &line) const
{
	return Clip(line.a, line.b);
}

int Plane::Clip(const Line &line, Ray &outRay) const
{
	float t;
	bool intersects = IntersectLinePlane(normal, d, line.pos, line.dir, t);
	if (!intersects)
	{
		if (SignedDistance(line.pos) <= 0.f)
			return 0; // Discard the whole line, it's completely behind the plane.
		else
			return 2; // The whole line is in the positive halfspace. Keep all of it.
	}

	outRay.pos = line.pos + line.dir * t; // The intersection point
	if (Dot(line.dir, normal) >= 0.f)
		outRay.dir = line.dir;
	else
		outRay.dir = -line.dir;

	return 1; // Clipping resulted in a ray being generated.
}

int Plane::Clip(const Triangle &triangle, Triangle &t1, Triangle &t2) const
{
	bool side[3];
	side[0] = IsOnPositiveSide(triangle.a);
	side[1] = IsOnPositiveSide(triangle.b);
	side[2] = IsOnPositiveSide(triangle.c);
	int nPos = (side[0] ? 1 : 0) + (side[1] ? 1 : 0) + (side[2] ? 1 : 0);
	if (nPos == 0) // Everything should be clipped?
		return 0;
	// We will output at least one triangle, so copy the input to t1 for processing.
	t1 = triangle;

	if (nPos == 3) // All vertices of the triangle are in positive side?
		return 1;

	if (nPos == 1)
	{
		if (side[1])
		{
			float3 tmp = t1.a;
			t1.a = t1.b;
			t1.b = t1.c;
			t1.c = tmp;
		}
		else if (side[2])
		{
			float3 tmp = t1.a;
			t1.a = t1.c;
			t1.c = t1.b;
			t1.b = tmp;
		}

		// After the above cycling, t1.a is the triangle on the positive side.
		float t;
		Intersects(LineSegment(t1.a, t1.b), &t);
		t1.b = t1.a + (t1.b-t1.a)*t;
		Intersects(LineSegment(t1.a, t1.c), &t);
		t1.c = t1.a + (t1.c-t1.a)*t;
		return 1;
	}
	// Must be nPos == 2.
	if (!side[1])
	{
		float3 tmp = t1.a;
		t1.a = t1.b;
		t1.b = t1.c;
		t1.c = tmp;
	}
	else if (!side[2])
	{
		float3 tmp = t1.a;
		t1.a = t1.c;
		t1.c = t1.b;
		t1.b = tmp;
	}
	// After the above cycling, t1.a is the triangle on the negative side.

	float t, r;
	Intersects(LineSegment(t1.a, t1.b), &t);
	float3 ab = t1.a + (t1.b-t1.a)*t;
	Intersects(LineSegment(t1.a, t1.c), &r);
	float3 ac = t1.a + (t1.c-t1.a)*t;
	t1.a = ab;

	t2.a = t1.c;
	t2.b = ac;
	t2.c = ab;

	return 2;
}

bool Plane::IsParallel(const Plane &plane, float epsilon) const
{
	return normal.Equals(plane.normal, epsilon);
}

bool Plane::PassesThroughOrigin(float epsilon) const
{
	return fabs(d) <= epsilon;
}

float Plane::DihedralAngle(const Plane &plane) const
{
	return Dot(normal, plane.normal);
}

Circle Plane::GenerateCircle(const float3 &circleCenter, float radius) const
{
	return Circle(Project(circleCenter), normal, radius);
}

Plane operator *(const float3x3 &transform, const Plane &plane)
{
	Plane p(plane);
	p.Transform(transform);
	return p;
}

Plane operator *(const float3x4 &transform, const Plane &plane)
{
	Plane p(plane);
	p.Transform(transform);
	return p;
}

Plane operator *(const float4x4 &transform, const Plane &plane)
{
	Plane p(plane);
	p.Transform(transform);
	return p;
}

Plane operator *(const Quat &transform, const Plane &plane)
{
	Plane p(plane);
	p.Transform(transform);
	return p;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Plane::ToString() const
{
	char str[256];
	sprintf(str, "Plane(Normal:(%.2f, %.2f, %.2f) d:%.2f)", normal.x, normal.y, normal.z, d);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Plane &plane)
{
	o << plane.ToString();
	return o;
}

#endif

#ifdef MATH_GRAPHICSENGINE_INTEROP
void Plane::Triangulate(VertexBuffer &vb, float uWidth, float vHeight, const float3 &centerPoint, int numFacesU, int numFacesV, bool ccwIsFrontFacing) const
{
	float3 topLeft = Point(-uWidth*0.5f, -vHeight *0.5f, centerPoint);
	float3 uEdge = (Point(uWidth*0.5f, -vHeight *0.5f, centerPoint) - topLeft) / (float)numFacesU;
	float3 vEdge = (Point(-uWidth*0.5f, vHeight *0.5f, centerPoint) - topLeft) / (float)numFacesV;

	int i = vb.AppendVertices(numFacesU * numFacesV * 6);
	for(int y = 0; y < numFacesV; ++y)
		for(int x = 0; x < numFacesU; ++x)
		{
			float4 tl = float4(topLeft + uEdge * (float)x + vEdge * (float)y, 1.f);
			float4 tr = float4(topLeft + uEdge * (float)(x+1) + vEdge * (float)y, 1.f);
			float4 bl = float4(topLeft + uEdge * (float)x + vEdge * (float)(y+1), 1.f);
			float4 br = float4(topLeft + uEdge * (float)(x+1) + vEdge * (float)(y+1), 1.f);
			int i0 = ccwIsFrontFacing ? i : i+5;
			int i1 = ccwIsFrontFacing ? i+5 : i;
			vb.Set(i0, VDPosition, tl);
			vb.Set(i+1, VDPosition, tr);
			vb.Set(i+2, VDPosition, bl);
			vb.Set(i+3, VDPosition, bl);
			vb.Set(i+4, VDPosition, tr);
			vb.Set(i1, VDPosition, br);

			if (vb.Declaration()->HasType(VDUV))
			{
				float4 uvTL((float)x/numFacesU, (float)y/numFacesV, 0.f, 1.f);
				float4 uvTR((float)(x+1)/numFacesU, (float)y/numFacesV, 0.f, 1.f);
				float4 uvBL((float)x/numFacesU, (float)(y+1)/numFacesV, 0.f, 1.f);
				float4 uvBR((float)(x+1)/numFacesU, (float)(y+1)/numFacesV, 0.f, 1.f);

				vb.Set(i0, VDUV, uvTL);
				vb.Set(i+1, VDUV, uvTR);
				vb.Set(i+2, VDUV, uvBL);
				vb.Set(i+3, VDUV, uvBL);
				vb.Set(i+4, VDUV, uvTR);
				vb.Set(i1, VDUV, uvBR);
			}

			if (vb.Declaration()->HasType(VDNormal))
			{
				for(int k = 0; k < 6; ++k)
					vb.Set(i+k, VDNormal, float4(normal, 0.f));
			}

			i += 6;
		}
}

void Plane::ToLineList(VertexBuffer &vb, float uWidth, float vHeight, const float3 &centerPoint, int numLinesU, int numLinesV) const
{
	float3 topLeft = Point(-uWidth*0.5f, -vHeight *0.5f, centerPoint);
	float3 uEdge = (Point(uWidth*0.5f, -vHeight *0.5f, centerPoint) - topLeft) / (float)numLinesU;
	float3 vEdge = (Point(-uWidth*0.5f, vHeight *0.5f, centerPoint) - topLeft) / (float)numLinesV;

	int i = vb.AppendVertices((numLinesU + numLinesV) * 2);
	for(int y = 0; y < numLinesV; ++y)
	{
		float4 start = float4(topLeft + vEdge * (float)y, 1.f);
		float4 end   = float4(topLeft + uWidth * uEdge + vEdge * (float)y, 1.f);
		vb.Set(i, VDPosition, start);
		vb.Set(i+1, VDPosition, end);
		i += 2;
	}

	for(int x = 0; x < numLinesU; ++x)
	{
		float4 start = float4(topLeft + uEdge * (float)x, 1.f);
		float4 end   = float4(topLeft + vHeight * vEdge + uEdge * (float)x, 1.f);
		vb.Set(i, VDPosition, start);
		vb.Set(i+1, VDPosition, end);
		i += 2;
	}
}

#endif
MATH_END_NAMESPACE
