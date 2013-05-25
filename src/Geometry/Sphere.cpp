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

/** @file Sphere.cpp
	@author Jukka Jylänki
	@brief Implementation for the Sphere geometry object. */
#include "Sphere.h"
#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#include <vector>
#include <iostream>
#else
#include "Container/Array.h"
#endif

#include "../Math/MathLog.h"
#include "../Math/MathFunc.h"
#include "OBB.h"
#include "AABB.h"
#include "Capsule.h"
#include "Frustum.h"
#include "../Algorithm/Random/LCG.h"
#include "LineSegment.h"
#include "Line.h"
#include "Ray.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "Plane.h"
#include "../Math/float2.h"
#include "../Math/float3x3.h"
#include "../Math/float3x4.h"
#include "../Math/float4.h"
#include "../Math/float4x4.h"
#include "../Math/Quat.h"
#include "Triangle.h"

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "VertexBuffer.h"
#endif

MATH_BEGIN_NAMESPACE

Sphere::Sphere(const float3 &center, float radius)
:pos(center), r(radius)
{
}

Sphere::Sphere(const float3 &a, const float3 &b)
{
	*this = FitThroughPoints(a, b);
}

Sphere::Sphere(const float3 &a, const float3 &b, const float3 &c)
{
	*this = FitThroughPoints(a, b, c);
}

Sphere::Sphere(const float3 &a, const float3 &b, const float3 &c, const float3 &d)
{
	*this = FitThroughPoints(a, b, c, d);
}

void Sphere::Translate(const float3 &offset)
{
	pos += offset;
}

void Sphere::Transform(const float3x3 &transform)
{
	assume(transform.HasUniformScale());
	pos = transform * pos;
	r *= transform.Col(0).Length();
}

void Sphere::Transform(const float3x4 &transform)
{
	assume(transform.HasUniformScale());
	pos = transform.MulPos(pos);
	r *= transform.Col(0).Length();
}

void Sphere::Transform(const float4x4 &transform)
{
	assume(transform.HasUniformScale());
	assume(!transform.ContainsProjection());
	pos = transform.MulPos(pos);
	r *= transform.Col3(0).Length();
}

void Sphere::Transform(const Quat &transform)
{
	pos = transform * pos;
}

AABB Sphere::MinimalEnclosingAABB() const
{
	AABB aabb;
	aabb.SetFrom(*this);
	return aabb;
}

AABB Sphere::MaximalContainedAABB() const
{
	AABB aabb;
	static const float recipSqrt3 = RSqrt(3);
	float halfSideLength = r * recipSqrt3;
	aabb.SetFromCenterAndSize(pos, float3(halfSideLength,halfSideLength,halfSideLength));
	return aabb;
}

void Sphere::SetNegativeInfinity()
{
	pos = float3(0,0,0);
	r = -FLOAT_INF;
}

float Sphere::Volume() const
{
	return 4.f * pi * r*r*r / 3.f;
}

float Sphere::SurfaceArea() const
{
	return 4.f * pi * r*r;
}

float3 Sphere::ExtremePoint(const float3 &direction) const
{
	return pos + direction.ScaledToLength(r);
}

void Sphere::ProjectToAxis(const float3 &direction, float &outMin, float &outMax) const
{
	float d = Dot(direction, pos);
	outMin = d - r;
	outMax = d + r;
}

bool Sphere::IsFinite() const
{
	return pos.IsFinite() && MATH_NS::IsFinite(r);
}

bool Sphere::IsDegenerate() const
{
	return r < 0.f;
}

void Sphere::SetDegenerate()
{
	pos = float3::nan;
	r = FLOAT_NAN;
}

bool Sphere::Contains(const float3 &point) const
{
	return pos.DistanceSq(point) <= r*r;
}

bool Sphere::Contains(const float3 &point, float epsilon) const
{
	return pos.DistanceSq(point) <= r*r + epsilon;
}

bool Sphere::Contains(const LineSegment &lineSegment) const
{
	return Contains(lineSegment.a) && Contains(lineSegment.b);
}

bool Sphere::Contains(const Triangle &triangle) const
{
	return Contains(triangle.a) && Contains(triangle.b) && Contains(triangle.c);
}

bool Sphere::Contains(const Polygon &polygon) const
{
	for(int i = 0; i < polygon.NumVertices(); ++i)
		if (!Contains(polygon.Vertex(i)))
			return false;
	return true;
}

bool Sphere::Contains(const AABB &aabb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(aabb.CornerPoint(i)))
			return false;

	return true;
}

bool Sphere::Contains(const OBB &obb) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(obb.CornerPoint(i)))
			return false;

	return true;
}

bool Sphere::Contains(const Frustum &frustum) const
{
	for(int i = 0; i < 8; ++i)
		if (!Contains(frustum.CornerPoint(i)))
			return false;

	return true;
}

bool Sphere::Contains(const Polyhedron &polyhedron) const
{
	assume(polyhedron.IsClosed());
	for(int i = 0; i < polyhedron.NumVertices(); ++i)
		if (!Contains(polyhedron.Vertex(i)))
			return false;

	return true;
}

bool Sphere::Contains(const Sphere &sphere) const
{
	return pos.Distance(sphere.pos) + sphere.r <= r;
}

bool Sphere::Contains(const Capsule &capsule) const
{
	return pos.Distance(capsule.l.a) + capsule.r <= r &&
		pos.Distance(capsule.l.b) + capsule.r <= r;
}

Sphere Sphere::FastEnclosingSphere(const float3 *pts, int numPoints)
{
	Sphere s;
	if (numPoints == 0)
	{
		s.SetNegativeInfinity();
		return s;
	}
	assume(pts || numPoints == 0);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pts)
		return Sphere();
#endif

	// First pass: Pick the cardinal axis (X,Y or Z) which has the two most distant points.
	int minx, maxx, miny, maxy, minz, maxz;
	AABB::ExtremePointsAlongAABB(pts, numPoints, minx, maxx, miny, maxy, minz, maxz);
	float dist2x = pts[minx].DistanceSq(pts[maxx]);
	float dist2y = pts[miny].DistanceSq(pts[maxy]);
	float dist2z = pts[minz].DistanceSq(pts[maxz]);

	int min = minx;
	int max = maxx;
	if (dist2y > dist2x && dist2y > dist2z)
	{
		min = miny;
		max = maxy;
	}
	else if (dist2z > dist2x && dist2z > dist2y)
	{
		min = minz;
		max = maxz;
	}

	// The two points on the longest axis define the initial sphere.
	s.pos = (pts[min] + pts[max]) / 2.f;
	s.r = pts[min].Distance(s.pos);

	// Second pass: Make sure each point lies inside this sphere, expand if necessary.
	for(int i = 0; i < numPoints; ++i)
		s.Enclose(pts[i]);
	return s;
}

/* This implementation was adapted from Christer Ericson's Real-time Collision Detection, pp. 99-100.
	@bug This implementation is broken! */
/*
Sphere WelzlSphere(const float3 *pts, int numPoints, float3 *support, int numSupports)
{
	if (numPoints == 0)
	{
		switch(numSupports)
		{
		default: assert(false);
		case 0: return Sphere();
		case 1: return Sphere(support[0], 0.f);
		case 2: return Sphere(support[0], support[1]);
		case 3: return Sphere(support[0], support[1], support[2]);
		case 4: return Sphere(support[0], support[1], support[2], support[3]);
		}
	}

	// todo The following recursion can easily crash the stack for large inputs.  Convert this to proper form.
	Sphere smallestSphere = WelzlSphere(pts, numPoints - 1, support, numSupports);
	if (smallestSphere.Contains(pts[numPoints-1]))
		return smallestSphere;
	support[numSupports] = pts[numPoints-1];
	return WelzlSphere(pts, numPoints - 1,  support, numSupports + 1);
}
*/

// The epsilon value used for enclosing sphere computations.
static const float epsilon = 1e-4f;

Sphere Sphere::OptimalEnclosingSphere(const float3 *pts, int numPoints)
{
	// If we have only a small number of points, can solve with a specialized function.
	switch(numPoints)
	{
	case 0: return Sphere();
	case 1: return Sphere(pts[0], 0.f); // Sphere around a single point will be degenerate with r = 0.
	case 2: return OptimalEnclosingSphere(pts[0], pts[1]);
	case 3: return OptimalEnclosingSphere(pts[0], pts[1], pts[2]);
	case 4: return OptimalEnclosingSphere(pts[0], pts[1], pts[2], pts[3]);
	default: break;
	}

	// The set of supporting points for the minimal sphere. Even though the minimal enclosing
	// sphere might have 2, 3 or 4 points in its support (sphere surface), always store here
	// indices to exactly four points.
	int sp[4] = { 0, 1, 2, 3 };
	// Due to numerical issues, it can happen that the minimal sphere for four points {a,b,c,d} does not
	// accommodate a fifth point e, but replacing any of the points a-d from the support with the point e
	// does not accommodate the all the five points either.
	// Therefore, keep a set of flags for each support point to avoid going in cycles, where the same
	// set of points are again and again added and removed from the support, causing an infinite loop.
	bool expendable[4] = { true, true, true, true };
	// The so-far constructed minimal sphere.
	Sphere s = OptimalEnclosingSphere(pts[sp[0]], pts[sp[1]], pts[sp[2]], pts[sp[3]]);
	float rSq = s.r * s.r + epsilon;
	for(int i = 4; i < numPoints; ++i)
	{
		if (i == sp[0] || i == sp[1] || i == sp[2] || i == sp[3])
			continue; // Take care not to add the same point twice to the support set.
		// If the next point (pts[i]) does not fit inside the currently computed minimal sphere, compute
		// a new minimal sphere that also contains pts[i].
		if (pts[i].DistanceSq(s.pos) > rSq)
		{
			int redundant;
			s = OptimalEnclosingSphere(pts[sp[0]], pts[sp[1]], pts[sp[2]], pts[sp[3]], pts[i], redundant);
			rSq = s.r*s.r + epsilon;
			// A sphere is uniquely defined by four points, so one of the five points passed in above is
			// now redundant, and can be removed from the support set.
			if (redundant != 4 && (sp[redundant] < i || expendable[redundant]))
			{
				sp[redundant] = i; // Replace the old point with the new one.
				expendable[redundant] = false; // This new one cannot be evicted (until we proceed past it in the input list later)
				// Mark all points in the array before this index "expendable", meaning that they can be removed from the support set.
				if (sp[0] < i) expendable[0] = true;
				if (sp[1] < i) expendable[1] = true;
				if (sp[2] < i) expendable[2] = true;
				if (sp[3] < i) expendable[3] = true;

				// Have to start all over and make sure all old points also lie inside this new sphere,
				// since our guess for the minimal enclosing sphere changed.
				i = 0;
			}
		}
	}

	return s;
}

float Sphere::Distance(const float3 &point) const
{
	return Max(0.f, pos.Distance(point) - r);
}

float Sphere::Distance(const Sphere &sphere) const
{
	return Max(0.f, pos.Distance(sphere.pos) - r - sphere.r);
}

float Sphere::Distance(const Capsule &capsule) const
{
	return capsule.Distance(*this);
}

float Sphere::Distance(const AABB &aabb) const
{
	return aabb.Distance(*this);
}

float Sphere::Distance(const OBB &obb) const
{
	return obb.Distance(*this);
}

float Sphere::Distance(const Plane &plane) const
{
	return plane.Distance(*this);
}

float Sphere::Distance(const Triangle &triangle) const
{
	return triangle.Distance(*this);
}

float Sphere::Distance(const Ray &ray) const
{
	return ray.Distance(*this);
}

float Sphere::Distance(const LineSegment &lineSegment) const
{
	return lineSegment.Distance(*this);
}

float Sphere::Distance(const Line &line) const
{
	return line.Distance(*this);
}

float Sphere::MaxDistance(const float3 &point) const
{
	return point.Distance(pos) + r;
}

float3 Sphere::ClosestPoint(const float3 &point) const
{
	float d = pos.Distance(point);
	float t = (d >= r ? r : d);
	return pos + (point - pos) * (t / d);
}

bool Sphere::Intersects(const Sphere &sphere) const
{
	return (pos - sphere.pos).LengthSq() <= (r + sphere.r) * (r + sphere.r);
}

bool Sphere::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

int Sphere::IntersectLine(const float3 &linePos, const float3 &lineDir, const float3 &sphereCenter,
                          float sphereRadius, float &t1, float &t2)
{
	assume(lineDir.IsNormalized());
	assume(sphereRadius >= 0.f);

	/* A line is represented explicitly by the set { linePos + t * lineDir }, where t is an arbitrary float.
	  A sphere is represented implictly by the set of vectors that satisfy ||v - sphereCenter|| == sphereRadius.
	  To solve which points on the line are also points on the sphere, substitute v <- linePos + t * lineDir
	  to obtain:

	    || linePos + t * lineDir - sphereCenter || == sphereRadius, and squaring both sides we get
	    || linePos + t * lineDir - sphereCenter ||^2 == sphereRadius^2, or rearranging:
	    || (linePos - sphereCenter) + t * lineDir ||^2 == sphereRadius^2. */

	// This equation represents the set of points which lie both on the line and the sphere. There is only one
	// unknown variable, t, for which we solve to get the actual points of intersection.

	// Compute variables from the above equation:
	const float3 a = linePos - sphereCenter;
	const float radSq = sphereRadius * sphereRadius;

	/* so now the equation looks like

	    || a + t * lineDir ||^2 == radSq.

	  Since ||x||^2 == <x,x> (i.e. the square of a vector norm equals the dot product with itself), we get
	
	    <a + t * lineDir, a + t * lineDir> == radSq,
	
	  and using the identity <a+b, a+b> == <a,a> + 2*<a,b> + <b,b> (which holds for dot product when a and b are reals),
	  we have

	    <a,a> + 2 * <a, t * lineDir> + <t * lineDir, t * lineDir> == radSq, or		
	    <a,a> - radSq + 2 * <a, lineDir> * t + <lineDir, lineDir> * t^2 == 0, or

	    C + Bt + At^2 == 0, where

	    C = <a,a> - radSq,
	    B = 2 * <a, lineDir>, and
	    A = <lineDir, lineDir> == 1, since we assumed lineDir is normalized. */

	// Warning! If Dot(a,a) is large (distance between line pos and sphere center) and sphere radius very small,
	// catastrophic cancellation can occur here!
	const float C = Dot(a,a) - radSq;
	const float B = 2.f * Dot(a, lineDir);

	/* The equation A + Bt + Ct^2 == 0 is a second degree equation on t, which is easily solvable using the
	  known formula, and we obtain

	    t = [-B +/- Sqrt(B^2 - 4AC)] / 2A. */

	float D = B*B - 4.f * C; // D = B^2 - 4AC.
	if (D < 0.f) // There is no solution to the square root, so the ray doesn't intersect the sphere.
	{
		// Output a degenerate enter-exit range so that batch processing code may use min of t1's and max of t2's to
		// compute the nearest enter and farthest exit without requiring branching on the return value of this function.
		t1 = FLOAT_INF;
		t2 = -FLOAT_INF;
		return 0;
	}

	if (D < 1e-4f) // The expression inside Sqrt is ~ 0. The line is tangent to the sphere, and we have one solution.
	{
		t1 = t2 = -B * 0.5f;
		return 1;
	}

	// The Sqrt expression is strictly positive, so we get two different solutions for t.
	D = Sqrt(D);
	t1 = (-B - D) * 0.5f;
	t2 = (-B + D) * 0.5f;
	return 2;
}

int Sphere::Intersects(const Ray &ray, float3 *intersectionPoint, float3 *intersectionNormal, float *d, float *d2) const
{
	float t1, t2;
	int numIntersections = IntersectLine(ray.pos, ray.dir, pos, r, t1, t2);

	// If the line of this ray intersected in two places, but the first intersection was "behind" this ray,
	// handle the second point of intersection instead. This case occurs when the origin of the ray is inside
	// the Sphere.
	if (t1 < 0.f && numIntersections == 2)
		t1 = t2;

	if (t1 < 0.f)
		return 0; // The intersection position is on the negative direction of the ray.

	float3 hitPoint = ray.pos + t1 * ray.dir;
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t1;
	if (d2)
		*d2 = t2;

	return numIntersections;
}

int Sphere::Intersects(const Line &line, float3 *intersectionPoint, float3 *intersectionNormal, float *d, float *d2) const
{
	float t1, t2;
	int numIntersections = IntersectLine(line.pos, line.dir, pos, r, t1, t2);
	if (numIntersections == 0)
		return 0;

	float3 hitPoint = line.pos + t1 * line.dir;
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t1;
	if (d2)
		*d2 = t2;

	return numIntersections;
}

int Sphere::Intersects(const LineSegment &l, float3 *intersectionPoint, float3 *intersectionNormal, float *d, float *d2) const
{
	float t1, t2;
	int numIntersections = IntersectLine(l.a, l.Dir(), pos, r, t1, t2);

	if (numIntersections == 0)
		return 0;

	float lineLength = l.Length();
	if (t2 < 0.f || t1 > lineLength)
		return 0;
	float3 hitPoint = l.GetPoint(t1 / lineLength);
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t1 / lineLength;
	if (d2)
		*d2 = t2 / lineLength;

	return true;
}

bool Sphere::Intersects(const Plane &plane) const
{
	return plane.Intersects(*this);
}

bool Sphere::Intersects(const AABB &aabb, float3 *closestPointOnAABB) const
{
	return aabb.Intersects(*this, closestPointOnAABB);
}

bool Sphere::Intersects(const OBB &obb, float3 *closestPointOnOBB) const
{
	return obb.Intersects(*this, closestPointOnOBB);
}

bool Sphere::Intersects(const Triangle &triangle, float3 *closestPointOnTriangle) const
{
	return triangle.Intersects(*this, closestPointOnTriangle);
}

bool Sphere::Intersects(const Polygon &polygon) const
{
	return polygon.Intersects(*this);
}

bool Sphere::Intersects(const Frustum &frustum) const
{
	return frustum.Intersects(*this);
}

bool Sphere::Intersects(const Polyhedron &polyhedron) const
{
	return polyhedron.Intersects(*this);
}

void Sphere::Enclose(const float3 &point, float epsilon)
{
	float3 d = point - pos;
	float dist2 = d.LengthSq() + epsilon;
	if (dist2 > r*r)
	{
		float dist = Sqrt(dist2);
		float newRadius = (r + dist) / 2.f;
		pos += d * (newRadius - r) / dist;
		r = newRadius;
	}
}

void Sphere::Enclose(const AABB &aabb)
{
	///@todo This might not be very optimal at all. Perhaps better to enclose the farthest point first.
	for(int i = 0; i < 8; ++i)
		Enclose(aabb.CornerPoint(i));
}

void Sphere::Enclose(const OBB &obb)
{
	///@todo This might not be very optimal at all. Perhaps better to enclose the farthest point first.
	for(int i = 0; i < 8; ++i)
		Enclose(obb.CornerPoint(i));
}

void Sphere::Enclose(const Sphere &sphere)
{
	// To enclose another sphere into this sphere, we can simply enclose the farthest point
	// of that sphere to this sphere.
	float3 farthestPoint = sphere.pos - pos;
	farthestPoint = sphere.pos + farthestPoint * (sphere.r / farthestPoint.Length());
	Enclose(farthestPoint);
}

void Sphere::Enclose(const LineSegment &lineSegment)
{
	///@todo This might not be very optimal at all. Perhaps better to enclose the farthest point first.
	Enclose(lineSegment.a);
	Enclose(lineSegment.b);
}

void Sphere::Enclose(const float3 *pointArray, int numPoints)
{
	assume(pointArray || numPoints == 0);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!pointArray)
		return;
#endif
	///@todo This might not be very optimal at all. Perhaps better to enclose the farthest point first.
	for(int i = 0; i < numPoints; ++i)
		Enclose(pointArray[i]);
}

void Sphere::Enclose(const Triangle &triangle)
{
	Enclose(triangle.a);
	Enclose(triangle.b);
	Enclose(triangle.c);
}

void Sphere::Enclose(const Polygon &polygon)
{
	Enclose(polygon.VertexArrayPtr(), polygon.NumVertices());
}

void Sphere::Enclose(const Polyhedron &polyhedron)
{
	Enclose(polyhedron.VertexArrayPtr(), polyhedron.NumVertices());
}

void Sphere::Enclose(const Frustum &frustum)
{
	for(int i = 0; i < 8; ++i)
		Enclose(frustum.CornerPoint(i));
}

int Sphere::Triangulate(float3 *outPos, float3 *outNormal, float2 *outUV, int numVertices, bool ccwIsFrontFacing) const
{
	assume(outPos);
	assume(numVertices >= 24 && "At minimum, sphere triangulation will contain at least 8 triangles, which is 24 vertices, but fewer were specified!");
	assume(numVertices % 3 == 0 && "Warning:: The size of output should be divisible by 3 (each triangle takes up 3 vertices!)");

#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPos)
		return 0;
#endif
	assume(this->r > 0.f);

	if (numVertices < 24)
		return 0;

#ifdef MATH_ENABLE_STL_SUPPORT
	std::vector<Triangle> temp;
#else
	Array<Triangle> temp;
#endif
	// Start subdividing from a diamond shape.
	float3 xp(r,0,0);
	float3 xn(-r,0,0);
	float3 yp(0,r,0);
	float3 yn(0,-r,0);
	float3 zp(0,0,r);
	float3 zn(0,0,-r);

	if (ccwIsFrontFacing)
	{
		temp.push_back(Triangle(yp,xp,zp));
		temp.push_back(Triangle(xp,yp,zn));
		temp.push_back(Triangle(yn,zp,xp));
		temp.push_back(Triangle(yn,xp,zn));
		temp.push_back(Triangle(zp,xn,yp));
		temp.push_back(Triangle(yp,xn,zn));
		temp.push_back(Triangle(yn,xn,zp));
		temp.push_back(Triangle(xn,yn,zn));
	}
	else
	{
		temp.push_back(Triangle(yp,zp,xp));
		temp.push_back(Triangle(xp,zn,yp));
		temp.push_back(Triangle(yn,xp,zp));
		temp.push_back(Triangle(yn,zn,xp));
		temp.push_back(Triangle(zp,yp,xn));
		temp.push_back(Triangle(yp,zn,xn));
		temp.push_back(Triangle(yn,zp,xn));
		temp.push_back(Triangle(xn,zn,yn));
	}

	int oldEnd = 0;
	while(((int)temp.size()-oldEnd+3)*3 <= numVertices)
	{
		Triangle cur = temp[oldEnd];
		float3 a = ((cur.a + cur.b) * 0.5f).ScaledToLength(this->r);
		float3 b = ((cur.a + cur.c) * 0.5f).ScaledToLength(this->r);
		float3 c = ((cur.b + cur.c) * 0.5f).ScaledToLength(this->r);

		temp.push_back(Triangle(cur.a, a, b));
		temp.push_back(Triangle(cur.b, c, a));
		temp.push_back(Triangle(cur.c, b, c));
		temp.push_back(Triangle(a, c, b));

		++oldEnd;
	}
	// Check that we really did tessellate as many new triangles as possible.
	assert(((int)temp.size()-oldEnd)*3 <= numVertices && ((int)temp.size()-oldEnd)*3 + 9 > numVertices);

	for(size_t i = oldEnd, j = 0; i < temp.size(); ++i, ++j)
	{
		outPos[3*j] = this->pos + temp[i].a;
		outPos[3*j+1] = this->pos + temp[i].b;
		outPos[3*j+2] = this->pos + temp[i].c;
	}

	if (outNormal)
		for(size_t i = oldEnd, j = 0; i < temp.size(); ++i, ++j)
		{
			outNormal[3*j] = temp[i].a.Normalized();
			outNormal[3*j+1] = temp[i].b.Normalized();
			outNormal[3*j+2] = temp[i].c.Normalized();
		}

	if (outUV)
		for(size_t i = oldEnd, j = 0; i < temp.size(); ++i, ++j)
		{
			outUV[3*j] = float2(atan2(temp[i].a.y, temp[i].a.x) / (2.f * 3.141592654f) + 0.5f, (temp[i].a.z + r) / (2.f * r));
			outUV[3*j+1] = float2(atan2(temp[i].b.y, temp[i].b.x) / (2.f * 3.141592654f) + 0.5f, (temp[i].b.z + r) / (2.f * r));
			outUV[3*j+2] = float2(atan2(temp[i].c.y, temp[i].c.x) / (2.f * 3.141592654f) + 0.5f, (temp[i].c.z + r) / (2.f * r));
		}

	return ((int)temp.size() - oldEnd) * 3;
}

float3 Sphere::RandomPointInside(LCG &lcg)
{
	assume(r > 1e-3f);
	for(int i = 0; i < 1000; ++i)
	{
		float x = lcg.Float(-r, r);
		float y = lcg.Float(-r, r);
		float z = lcg.Float(-r, r);
		if (x*x + y*y + z*z <= r*r)
			return pos + float3(x,y,z);
	}
	assume(false && "Sphere::RandomPointInside failed!");

	// Failed to generate a point inside this sphere. Return the sphere center as fallback.
	return pos;
}

float3 Sphere::RandomPointOnSurface(LCG &lcg)
{
	assume(r > 1e-3f);
	for(int i = 0; i < 1000; ++i)
	{
		float x = lcg.Float(-r, r);
		float y = lcg.Float(-r, r);
		float z = lcg.Float(-r, r);
		float lenSq = x*x + y*y + z*z;
		if (lenSq >= 1e-6f && lenSq <= r*r)
			return pos + r / Sqrt(lenSq) * float3(x,y,z);
	}
	assume(false && "Sphere::RandomPointOnSurface failed!");

	// Failed to generate a point inside this sphere. Return an arbitrary point on the surface as fallback.
	return pos + float3(r, 0, 0);
}

float3 Sphere::RandomPointInside(LCG &lcg, const float3 &center, float radius)
{
	return Sphere(center, radius).RandomPointInside(lcg);
}

float3 Sphere::RandomPointOnSurface(LCG &lcg, const float3 &center, float radius)
{
	return Sphere(center, radius).RandomPointOnSurface(lcg);
}

Sphere Sphere::OptimalEnclosingSphere(const float3 &a, const float3 &b)
{
	Sphere s;
	s.pos = (a + b) / 2.f;
	s.r = (b - s.pos).Length();
	assume(s.pos.IsFinite());
	assume(s.r >= 0.f);

	// Allow floating point inconsistency and expand the radius by a small epsilon so that the containment tests
	// really contain the points (note that the points must be sufficiently near enough to the origin)
	s.r += epsilon;

	mathassert(s.Contains(a));
	mathassert(s.Contains(b));
	return s;
}

/** Computes the (s,t) coordinates of the smallest sphere that passes through three points (0,0,0), ab and ac.
	@param ab The first point to fit the sphere through.
	@param ac The second point to fit the sphere through. The third point is hardcoded to (0,0,0). When fitting a sphere
		through three points a, b and c, pass in b-a as the parameter ab, and c-a as the parameter ac (i.e. translate
		the coordinate system center to lie at a).
	@param s [out] Outputs the s-coordinate of the sphere center (in the 2D barycentric UV convention)
	@param t [out] Outputs the t-coordinate of the sphere center (in the 2D barycentric UV convention) To
		compute the actual point, calculate the expression origin + s*ab + t*ac.
	@note The returned sphere is one that passes through the three points (0,0,0), ab and ac. It is NOT necessarily the
		smallest sphere that encloses these three points!
	@return True if the function succeeded. False on failure. This function fails if the points (0,0,0), ab and ac
		are collinear, in which case there does not exist a sphere that passes through the three given points. */
bool FitSphereThroughPoints(const float3 &ab, const float3 &ac, float &s, float &t)
{
	/* The task is to compute the minimal radius sphere through the three points
	   a, b and c. (Note that this is not necessarily the minimal radius sphere enclosing
	   the points a, b and c!)

	   Denote by p the sphere center position, and r the sphere radius. If the sphere
	   is to run through the points a, b and c, then the center point of the sphere
	   must be equidistant to these points, i.e.

	      || p - a || == || p - b || == || p - c ||,

	   or

	      a^2 - 2ap + p^2 == b^2 - 2bp + p^2 == c^2 - 2cp + p^2.

	   Subtracting pairwise, we get

	      (b-a)p == (b^2 - a^2)/2 and       (1)
	      (c-a)p == (c^2 - a^2)/2.          (2)

	   Additionally, the center point of the sphere must lie on the same plane as the triangle
	   defined by the points a, b and c. Therefore, the point p can be represented as a 2D
	   barycentric coordinates (s,t) as follows:

	      p == a + s*(b-a) + t*(c-a).        (3)

	   Now, without loss of generality, assume that the point a lies at origin (translate the origin
	   of the coordinate system to be centered at the point a), i.e. make the substitutions
	   A = (0,0,0), B = b-a, C = c-a, and we have:and we have:

	      BP == B^2/2,            (1')
	      CP == C^2/2 and         (2')
	       P == s*B + t*C.        (3') */

	const float BB = Dot(ab,ab);
	const float CC = Dot(ac,ac);
	const float BC = Dot(ab,ac);

	/* Substitute (3') into (1') and (2'), to obtain a matrix equation

	   ( B^2  BC  ) * (s) = (B^2 / 2)
	   ( BC   C^2 )   (t)   (C^2 / 2)

	   which equals
	
	   (s) = ( B^2  BC  )^-1  *  (B^2 / 2)
	   (t)   ( BC   C^2 )        (C^2 / 2)

	   	Use the formula for inverting a 2x2 matrix, and we have

	   (s) = 1 / (2 * B^2 * C^2 - (BC)^2) * ( C^2   -BC ) *  (B^2)
	   (t)                                  ( -BC   B^2 )    (C^2)
	*/

	float denom = BB*CC - BC*BC;

	if (EqualAbs(denom, 0.f, 5e-3f))
		return false;

	denom = 0.5f / denom; // == 1 / (2 * B^2 * C^2 - (BC)^2)

	s = (CC * BB - BC * CC) * denom;
	t = (CC * BB - BC * BB) * denom;

	return true;
}

/** Computes the center point coordinates of the sphere that passes through four points (0,0,0), ab, ac and ad.
	@param ab The first point to fit the sphere through.
	@param ac The second point to fit the sphere through.
	@param ad The third point to fit the sphere through. The fourth point is hardcoded to (0,0,0). When fitting a sphere
		through four points a, b c and d, pass in b-a as the parameter ab, and c-a as the parameter ac and d-a as
		the parameter ad (i.e. translate
		the coordinate system center to lie at a).
	@param s [out] Outputs the s-coordinate of the sphere center.
	@param t [out] Outputs the t-coordinate of the sphere center.
	@param u [out] Outputs the u-coordinate of the sphere center. To
		compute the actual point, calculate the expression a + s*ab + t*ac + u*ad.
	@note The returned sphere is one that passes through the four points (0,0,0), ab, ac and ad. It is NOT necessarily
		the smallest sphere that encloses these four points!
	@return True if the function succeeded. False on failure. This function fails if the points (0,0,0), ab, ac and ad
		are coplanar, in which case there does not exist a sphere that passes through the four given points. */
bool FitSphereThroughPoints(const float3 &ab, const float3 &ac, const float3 &ad, float &s, float &t, float &u)
{
	/* The task is to compute the (unique) sphere through the four points
	   a, b c and d. (Note that this is not necessarily the minimal radius sphere enclosing
	   the points a, b, c and d!)

	   Denote by p the sphere center position, and r the sphere radius. If the sphere
	   is to run through the points a, b, c and d, then the center point of the sphere
	   must be equidistant to these points, i.e.

	      || p - a || == || p - b || == || p - c || == || p - d ||,

	   or

	      a^2 - 2ap + p^2 == b^2 - 2bp + p^2 == c^2 - 2cp + p^2 == d^2 - 2dp + p ^2.

	   Subtracting pairwise, we get

	      (b-a)p == (b^2 - a^2)/2,          (1)
	      (c-a)p == (c^2 - a^2)/2 and       (2)
	      (d-a)p == (d^2 - a^2)/2.          (3)

	   Additionally, the center point of the sphere can be represented as a linear combination
	   of the four points a, b, c and d, as follows:

	      p == a + s*(b-a) + t*(c-a) + u*(d-a).        (4)

	   Now, without loss of generality, assume that the point a lies at origin (translate the origin
	   of the coordinate system to be centered at the point a, i.e. make the substitutions
	   A = (0,0,0), B = b-a, C = c-a, D = d-a, and we have:

	      BP == B^2/2,            (1')
	      CP == C^2/2 and         (2')
	      DP == D^2/2 and         (3')
	       P == s*B + t*C + u*D.  (4')

	   Substitute (4') into (1'), (2') and (3'), to obtain a matrix equation

	      ( B^2  BC  BD )   (s)   (B^2 / 2)
	      ( BC   C^2 CD ) * (t) = (C^2 / 2)
	      ( BD   CD  D^2)   (u)   (D^2 / 2)

	   which equals
	
	      (s)   ( B^2  BC  BD )^-1   (B^2 / 2)
	      (t) = ( BC   C^2 CD )    * (C^2 / 2)
	      (u)   ( BD   CD  D^2)      (D^2 / 2)

	   	Then we simply invert the 3x3 matrix and compute the vector (s, t, u). */

	const float BB = Dot(ab, ab);
	const float BC = Dot(ab, ac);
	const float BD = Dot(ab, ad);
	const float CC = Dot(ac, ac);
	const float CD = Dot(ac, ad);
	const float DD = Dot(ad, ad);

	float3x3 m;
	m[0][0] = BB; m[0][1] = BC; m[0][2] = BD;
	m[1][0] = BC; m[1][1] = CC; m[1][2] = CD;
	m[2][0] = BD; m[2][1] = CD; m[2][2] = DD;
	bool success = m.InverseSymmetric();
	if (!success)
		return false;
	float3 v = m * float3(BB * 0.5f, CC * 0.5f, DD * 0.5f);
	s = v.x;
	t = v.y;
	u = v.z;

	return true;
}

/** For reference, see http://realtimecollisiondetection.net/blog/?p=20 . */
Sphere Sphere::OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c)
{
	Sphere sphere;

	float3 ab = b-a;
	float3 ac = c-a;

	float s, t;
	bool areCollinear = ab.Cross(ac).LengthSq() < 1e-4f; // Manually test that we don't try to fit sphere to three collinear points.
	bool success = !areCollinear && FitSphereThroughPoints(ab, ac, s, t);
	if (!success || Abs(s) > 10000.f || Abs(t) > 10000.f) // If s and t are very far from the triangle, do a manual box fitting for numerical stability.
	{
		float3 minPt = Min(a, b, c);
		float3 maxPt = Max(a, b, c);
		sphere.pos = (minPt + maxPt) * 0.5f;
		sphere.r = sphere.pos.Distance(minPt);
	}
	else if (s < 0.f)
	{
		sphere.pos = (a + c) * 0.5f;
		sphere.r = a.Distance(c) * 0.5f;
		sphere.r = Max(sphere.r, b.Distance(sphere.pos)); // For numerical stability, expand the radius of the sphere so it certainly contains the third point.
	}
	else if (t < 0.f)
	{
		sphere.pos = (a + b) * 0.5f;
		sphere.r = a.Distance(b) * 0.5f;
		sphere.r = Max(sphere.r, c.Distance(sphere.pos)); // For numerical stability, expand the radius of the sphere so it certainly contains the third point.
	}
	else if (s+t > 1.f)
	{
		sphere.pos = (b + c) * 0.5f;
		sphere.r = b.Distance(c) * 0.5f;
		sphere.r = Max(sphere.r, a.Distance(sphere.pos)); // For numerical stability, expand the radius of the sphere so it certainly contains the third point.
	}
	else
	{
		const float3 center = s*ab + t*ac;
		sphere.pos = a + center;
		// Mathematically, the following would be correct, but it suffers from floating point inaccuracies,
		// since it only tests distance against one point.
		//sphere.r = center.Length();

		// For robustness, take the radius to be the distance to the farthest point (though the distance are all
		// equal).
		sphere.r = Sqrt(Max(sphere.pos.DistanceSq(a), sphere.pos.DistanceSq(b), sphere.pos.DistanceSq(c)));
	}

	// Allow floating point inconsistency and expand the radius by a small epsilon so that the containment tests
	// really contain the points (note that the points must be sufficiently near enough to the origin)
	sphere.r += 2.f * epsilon; // We test against one epsilon, so expand by two epsilons.

#ifdef MATH_ASSERT_CORRECTNESS
	if (!sphere.Contains(a, epsilon) || !sphere.Contains(b, epsilon) || !sphere.Contains(c, epsilon))
	{
		LOGE("Pos: %s, r: %f", sphere.pos.ToString().c_str(), sphere.r);
		LOGE("A: %s, dist: %f", a.ToString().c_str(), a.Distance(sphere.pos));
		LOGE("B: %s, dist: %f", b.ToString().c_str(), b.Distance(sphere.pos));
		LOGE("C: %s, dist: %f", c.ToString().c_str(), c.Distance(sphere.pos));
		mathassert(false);
	}
#endif
	return sphere;
}

/** For reference, see http://realtimecollisiondetection.net/blog/?p=20 . */
Sphere Sphere::OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c, const float3 &d)
{
	Sphere sphere;

	float s,t,u;
	const float3 ab = b-a;
	const float3 ac = c-a;
	const float3 ad = d-a;
	bool success = FitSphereThroughPoints(ab, ac, ad, s, t, u);
	if (!success || s < 0.f || t < 0.f || u < 0.f || s+t+u > 1.f)
	{
		sphere = OptimalEnclosingSphere(a,b,c);
		if (!sphere.Contains(d))
		{
			sphere = OptimalEnclosingSphere(a,b,d);
			if (!sphere.Contains(c))
			{
				sphere = OptimalEnclosingSphere(a,c,d);
				if (!sphere.Contains(b))
				{
					sphere = OptimalEnclosingSphere(b,c,d);
					sphere.r = Max(sphere.r, a.Distance(sphere.pos) + 1e-3f); // For numerical stability, expand the radius of the sphere so it certainly contains the fourth point.
					assume(sphere.Contains(a));
				}
			}
		}
	}
	/* // Note: Trying to approach the problem like this, like was in the triangle case, is flawed:
	if (s < 0.f)
		sphere = OptimalEnclosingSphere(a, c, d);
	else if (t < 0.f)
		sphere = OptimalEnclosingSphere(a, b, d);
	else if (u < 0.f)
		sphere = OptimalEnclosingSphere(a, b, c);
	else if (s + t + u > 1.f)
		sphere = OptimalEnclosingSphere(b, c, d); */
	else // The fitted sphere is inside the convex hull of the vertices (a,b,c,d), so it must be optimal.
	{
		const float3 center = s*ab + t*ac + u*ad;

		sphere.pos = a + center;
		// Mathematically, the following would be correct, but it suffers from floating point inaccuracies,
		// since it only tests distance against one point.
		//sphere.r = center.Length();

		// For robustness, take the radius to be the distance to the farthest point (though the distance are all
		// equal).
		sphere.r = Sqrt(Max(sphere.pos.DistanceSq(a), sphere.pos.DistanceSq(b), sphere.pos.DistanceSq(c), sphere.pos.DistanceSq(d)));
	}

		// Allow floating point inconsistency and expand the radius by a small epsilon so that the containment tests
		// really contain the points (note that the points must be sufficiently near enough to the origin)
		sphere.r += 2.f*epsilon; // We test against one epsilon, so expand using 2 epsilons.

#ifdef MATH_ASSERT_CORRECTNESS
	if (!sphere.Contains(a, epsilon) || !sphere.Contains(b, epsilon) || !sphere.Contains(c, epsilon) || !sphere.Contains(d, epsilon))
	{
		LOGE("Pos: %s, r: %f", sphere.pos.ToString().c_str(), sphere.r);
		LOGE("A: %s, dist: %f", a.ToString().c_str(), a.Distance(sphere.pos));
		LOGE("B: %s, dist: %f", b.ToString().c_str(), b.Distance(sphere.pos));
		LOGE("C: %s, dist: %f", c.ToString().c_str(), c.Distance(sphere.pos));
		LOGE("D: %s, dist: %f", d.ToString().c_str(), d.Distance(sphere.pos));
		mathassert(false);
	}
#endif

	return sphere;
}

Sphere Sphere::OptimalEnclosingSphere(const float3 &a, const float3 &b, const float3 &c, const float3 &d, const float3 &e,
                                      int &redundantPoint)
{
	Sphere s = OptimalEnclosingSphere(b,c,d,e);
	if (s.Contains(a, epsilon))
	{
		redundantPoint = 0;
		return s;
	}
	s = OptimalEnclosingSphere(a,c,d,e);
	if (s.Contains(b, epsilon))
	{
		redundantPoint = 1;
		return s;
	}
	s = OptimalEnclosingSphere(a,b,d,e);
	if (s.Contains(c, epsilon))
	{
		redundantPoint = 2;
		return s;
	}
	s = OptimalEnclosingSphere(a,b,c,e);
	if (s.Contains(d, epsilon))
	{
		redundantPoint = 3;
		return s;
	}
	s = OptimalEnclosingSphere(a,b,c,d);
	mathassert(s.Contains(e, epsilon));
	redundantPoint = 4;
	return s;
}

/** For reference, see http://realtimecollisiondetection.net/blog/?p=20 . */
Sphere Sphere::FitThroughPoints(const float3 &a, const float3 &b, const float3 &c)
{
	Sphere sphere;

	float3 ab = b-a;
	float3 ac = c-a;

	float s, t;
	bool success = FitSphereThroughPoints(ab, ac, s, t);
	if (!success)
	{
		LOGW("Sphere::FitThroughPoints(a,b,c) failed! The three input points are collinear!");
		sphere.SetDegenerate();
		return sphere;
	}

	const float3 p = s*ab + t*ac;

	// In our translated coordinate space, the origin lies on the sphere, so the distance of p from origin
	// gives the radius of the sphere.
	sphere.r = p.Length();

	// Translate back to original coordinate space.
	sphere.pos = a + p;

	return sphere;
}

/** For reference, see http://realtimecollisiondetection.net/blog/?p=20 . */
Sphere Sphere::FitThroughPoints(const float3 &a, const float3 &b, const float3 &c, const float3 &d)
{
	Sphere sphere;

	float s,t,u;
	const float3 ab = b-a;
	const float3 ac = c-a;
	const float3 ad = d-a;
	bool success = FitSphereThroughPoints(ab, ac, ad, s, t, u);
	if (success)
	{
		const float3 center = s*ab + t*ac + u*ad;
		sphere.r = center.Length();
		sphere.pos = a + center;
	}
	else
	{
		LOGW("Sphere::FitThroughPoints through four points failed! The points lie on the same plane!");
		sphere.SetDegenerate();
	}

	return sphere;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Sphere::ToString() const
{
	char str[256];
	sprintf(str, "Sphere(pos:(%.2f, %.2f, %.2f) r:%.2f)",
		pos.x, pos.y, pos.z, r);
	return str;
}

std::ostream &operator <<(std::ostream &o, const Sphere &sphere)
{
	o << sphere.ToString();
	return o;
}

#endif

Sphere operator *(const float3x3 &transform, const Sphere &sphere)
{
	Sphere s(sphere);
	s.Transform(transform);
	return s;
}

Sphere operator *(const float3x4 &transform, const Sphere &sphere)
{
	Sphere s(sphere);
	s.Transform(transform);
	return s;
}

Sphere operator *(const float4x4 &transform, const Sphere &sphere)
{
	Sphere s(sphere);
	s.Transform(transform);
	return s;
}

Sphere operator *(const Quat &transform, const Sphere &sphere)
{
	Sphere s(sphere);
	s.Transform(transform);
	return s;
}

#ifdef MATH_GRAPHICSENGINE_INTEROP
void Sphere::Triangulate(VertexBuffer &vb, int numVertices, bool ccwIsFrontFacing) const
{
	Array<float3> pos;
	Array<float3> normal;
	Array<float2> uv;
	pos.Resize_pod(numVertices);
	normal.Resize_pod(numVertices);
	uv.Resize_pod(numVertices);
	Triangulate(pos.beginptr(), normal.beginptr(), uv.beginptr(), numVertices, ccwIsFrontFacing);
	int startIndex = vb.AppendVertices(numVertices);
	for(int i = 0; i < (int)pos.size(); ++i)
	{
		vb.Set(startIndex+i, VDPosition, float4(pos[i],1.f));
		if (vb.Declaration()->TypeOffset(VDNormal) >= 0)
			vb.Set(startIndex+i, VDNormal, float4(normal[i],0.f));
		if (vb.Declaration()->TypeOffset(VDUV) >= 0)
			vb.SetFloat2(startIndex+i, VDUV, 0, uv[i]);
	}
}
#endif

MATH_END_NAMESPACE
