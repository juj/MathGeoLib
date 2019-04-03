#pragma once

#include "../src/MathGeoLibFwd.h"

AABB RandomAABBContainingPoint(const vec &pt, float maxSideLength);
OBB RandomOBBContainingPoint(const vec &pt, float maxSideLength);
Sphere RandomSphereContainingPoint(const vec &pt, float maxRadius);
Frustum RandomFrustumContainingPoint(LCG &rng, const vec &pt);
Line RandomLineContainingPoint(const vec &pt);
Ray RandomRayContainingPoint(const vec &pt);
LineSegment RandomLineSegmentContainingPoint(const vec &pt);
Capsule RandomCapsuleContainingPoint(const vec &pt);
Plane RandomPlaneContainingPoint(const vec &pt);
Triangle RandomTriangleContainingPoint(const vec &pt);
Polyhedron RandomPolyhedronContainingPoint(const vec &pt);
Polygon RandomPolygonContainingPoint(const vec &pt);
Circle2D RandomCircle2DContainingPoint(LCG &lcg, const float2 &pt, float maxRadius);

AABB RandomAABBInHalfspace(const Plane &plane, float maxSideLength);
OBB RandomOBBInHalfspace(const Plane &plane, float maxSideLength);
Sphere RandomSphereInHalfspace(const Plane &plane, float maxRadius);
Frustum RandomFrustumInHalfspace(const Plane &plane);
Line RandomLineInHalfspace(const Plane &plane);
Ray RandomRayInHalfspace(const Plane &plane);
LineSegment RandomLineSegmentInHalfspace(const Plane &plane);
Capsule RandomCapsuleInHalfspace(const Plane &plane);
Plane RandomPlaneInHalfspace(Plane &plane);
Triangle RandomTriangleInHalfspace(const Plane &plane);
Polyhedron RandomPolyhedronInHalfspace(const Plane &plane);
Polygon RandomPolygonInHalfspace(const Plane &plane);
