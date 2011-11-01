/* Copyright 2011 Jukka Jylänki

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
#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#include <vector>
#else
#include "Container/Array.h"
#endif
#include "Math/MathFunc.h"
#include "Geometry/OBB.h"
#include "Geometry/AABB.h"
#include "Geometry/Capsule.h"
#include "Geometry/Frustum.h"
#include "Algorithm/Random/LCG.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Line.h"
#include "Geometry/Ray.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/Plane.h"
#include "Geometry/Sphere.h"
#include "Math/float2.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4.h"
#include "Math/float4x4.h"
#include "Math/Quat.h"
#include "Geometry/Triangle.h"

MATH_BEGIN_NAMESPACE

Sphere::Sphere(const float3 &center, float radius)
:pos(center), r(radius) 
{
}

Sphere::Sphere(const float3 &pointA, const float3 &pointB)
{
	pos = (pointA + pointB) / 2.f;
	r = (pointB - pos).Length();
	assume(pos.IsFinite());
	assume(r >= 0.f);
}

Sphere::Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC)
{
	// See e.g. http://en.wikipedia.org/wiki/Circumcenter .

	float3 b = pointB - pointA;
	float3 c = pointC - pointA;
	float3 normal = Cross(b, c);
	float denom = 2.f * normal.LengthSq();
	if (EqualAbs(denom, 0.f))
	{
		SetNegativeInfinity();
		return;
	}

#if 0
	{
		// The three points are collinear. Construct a line through two most extremal points.
		float dC = Dot(b,c);

		if (dC < 0.f)
			*this = Sphere(pointB, pointC);
		else
		{
			float dB = Dot(b, b);
			if (dC > dB)
				*this = Sphere(pointA, pointC);
			else
				*this = sphere(pointA, pointB);
		}
		return;
	}
#endif

	pos = (c.LengthSq() * Cross(normal, c) + b.LengthSq() * Cross(b, normal)) / denom;
	r = pos.Length();
	pos += pointA;

/* // An alternate formulation that is probably correct, but the above contains fewer operations.
   // This one contains a matrix inverse operation.
	float3x3 m;
	m.SetRow(0, pointB - pointA);
	m.SetRow(1, pointC - pointA);
	m.SetRow(2, Cross(m.Row(0), m.Row(1)));
	float3 lengths = float3(m.Row(0).LengthSq(), m.Row(1).LengthSq(), 0.f) * 0.5f;

	bool success = m.Inverse();
	if (!success)
	{
		SetNegativeInfinity();
		return;
	}

	pos = m * lengths;
	r = pos.Length();
	pos += pointA;
*/
}

Sphere::Sphere(const float3 &pointA, const float3 &pointB, const float3 &pointC, const float3 &pointD)
{
	float3x3 m;
	m.SetRow(0, pointB - pointA);
	m.SetRow(1, pointC - pointA);
	m.SetRow(2, pointD - pointA);
	float3 lengths = float3(m.Row(0).LengthSq(), m.Row(1).LengthSq(), m.Row(2).LengthSq()) * 0.5f;

	bool success = m.Inverse();
	if (!success)
	{
		SetNegativeInfinity();
		return;
	}

	pos = m * lengths;
	r = pos.Length();
	pos += pointA;
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

bool Sphere::IsFinite() const
{
	return pos.IsFinite() && isfinite(r);
}

bool Sphere::IsDegenerate() const
{
	return r < 0.f;
}

bool Sphere::Contains(const float3 &point) const
{
	return pos.DistanceSq(point) <= r*r;
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

Sphere Sphere::OptimalEnclosingSphere(const float3 *pointArray, int numPoints)
{
	assume(false && "Not implemented!");
	return Sphere();
}

/* This implementation was adapted from Christer Ericson's Real-time Collision Detection, pp. 99-100. */
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
/*
Sphere Sphere::OptimalEnclosingSphere(const float3 *pts, int numPoints)
{
	float3 support[4];
	WelzlSphere(pts, numPoints, &support, 0);
}
*/
/*
Sphere Sphere::ApproximateEnclosingSphere(const float3 *pointArray, int numPoints)

*/
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

float3 Sphere::ClosestPoint(const float3 &point) const
{
	float d = pos.Distance(point);
	float t = (d >= r ? r : d);
	return pos + (point - pos) * (t / d);
}

bool Sphere::Intersects(const Sphere &sphere) const
{
	return (pos - sphere.pos).LengthSq() <= r*r + sphere.r*sphere.r;
}

bool Sphere::Intersects(const Capsule &capsule) const
{
	return capsule.Intersects(*this);
}

bool IntersectLineSphere(const float3 &lPos, const float3 &lDir, const Sphere &s, float &t)
{
	assume(lDir.IsNormalized());

	const float3 dist = lPos - s.pos;
	const float distSq = dist.LengthSq();
	const float radSq = s.r*s.r;

	const float b = 2.f * Dot(lDir, dist);
	const float c = distSq - radSq;
	const float D = b*b - 4.f*c;
	if (D <= 0.f)
		return false;  // The ray doesn't even come near.

	t = (-b - sqrtf(D)) * 0.5f;
	return true;
}

bool Sphere::Intersects(const Ray &r, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
	float t;
	IntersectLineSphere(r.pos, r.dir, *this, t);
	if (t < 0.f)
		return false; // The intersection position is on the negative direction of the ray.

	float3 hitPoint = r.pos + t * r.dir;
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t;
//	return (distSq <= radSq) ? IntersectBackface : IntersectFrontface;
	return true;
}

bool Sphere::Intersects(const Line &l, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
	float t;
	IntersectLineSphere(l.pos, l.dir, *this, t);

	float3 hitPoint = l.pos + t * l.dir;
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t;
//	return (distSq <= radSq) ? IntersectBackface : IntersectFrontface;
	return true;
}

bool Sphere::Intersects(const LineSegment &l, float3 *intersectionPoint, float3 *intersectionNormal, float *d) const
{
	float t;
	IntersectLineSphere(l.a, l.Dir(), *this, t);

	float lineLength = l.Length();
	if (t < 0.f || t > lineLength)
		return false;
	float3 hitPoint = l.GetPoint(t / lineLength);
	if (intersectionPoint)
		*intersectionPoint = hitPoint;
	if (intersectionNormal)
		*intersectionNormal = (hitPoint - pos).Normalized();
	if (d)
		*d = t;
//	return (distSq <= radSq) ? IntersectBackface : IntersectFrontface;
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

void Sphere::Enclose(const float3 &point)
{
	float3 d = point - pos;
	float dist2 = d.LengthSq();
	if (dist2 > r*r)
	{
		float dist = sqrt(dist2);
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

int Sphere::Triangulate(float3 *outPos, float3 *outNormal, float2 *outUV, int numVertices)
{
	assume(outPos);
#ifndef MATH_ENABLE_INSECURE_OPTIMIZATIONS
	if (!outPos)
		return 0;
#endif
	assume(this->r > 0.f);

#ifdef MATH_ENABLE_STL_SUPPORT
	std::vector<Triangle> temp;
#else
	Array<Triangle> temp;
#endif
	// Start subdividing from a tetrahedron.
	float3 xp(r,0,0);
	float3 xn(-r,0,0);
	float3 yp(0,r,0);
	float3 yn(0,-r,0);
	float3 zp(0,0,r);
	float3 zn(0,0,-r);

	temp.push_back(Triangle(yp,xp,zp));
	temp.push_back(Triangle(xp,yp,zn));
	temp.push_back(Triangle(yn,zp,xp));
	temp.push_back(Triangle(yn,xp,zn));
	temp.push_back(Triangle(zp,xn,yp));
	temp.push_back(Triangle(yp,xn,zn));
	temp.push_back(Triangle(yn,xn,zp));
	temp.push_back(Triangle(xn,yn,zn));

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
			return pos + r / sqrt(lenSq) * float3(x,y,z);
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

#ifdef MATH_ENABLE_STL_SUPPORT
std::string Sphere::ToString() const
{
	char str[256];
	sprintf(str, "Sphere(pos:(%.2f, %.2f, %.2f) r:%.2f)", 
		pos.x, pos.y, pos.z, r);
	return str;
}
#endif

MATH_END_NAMESPACE
