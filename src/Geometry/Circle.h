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

/** @file Circle.h
    @author Jukka Jylänki
	@brief The Circle geometry object. */
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A two-dimensional circle in 3D space.
/** This class represents both a hollow circle (only edge) and a solid circle (disc). */
class Circle
{
public:
    /// The default ctor does not initialize the Circle to any value.
    Circle() {}

	/// Constructs a new Circle with the given parameters.
    Circle(const float3 &center, const float3 &normal, float radius);

	/// The center position of this circle.
    float3 pos;

	/// The normal of the circle specifies the plane in which the circle lies in.
	/// This member is always assumed to be normalized.
    float3 normal;

    /// The radius of the circle.
    float r;

	/// Returns a normalized direction vector to the 'U direction' of the circle.
	/// This vector lies on the plane of this circle.
	float3 BasisU() const;

	/// Returns a normalized direction vector to the 'V direction' of the circle.
	/// This vector lies on the plane of this circle.
	float3 BasisV() const;

    /// Returns a point at the edge of this circle.
	float3 GetPoint(float angleRadians) const;

    /// Returns a point inside this circle.
    /// @param d A value in the range [0,1] to specify the distance from the center of the circle to get.
    /// The range of d is not enforced, so this function can be used to generate points outside the [0, 1] range.
	float3 GetPoint(float angleRadians, float d) const;

    /// Returns the center of mass of this Circle.
    float3 Centroid() const { return pos; }

	/// Returns the plane this circle is contained in.
	Plane ContainingPlane() const;

	/// Returns true if the given point lies on the edge of this circle.
	/// @param maxDistance The epsilon threshold to test the distance against.
    bool EdgeContains(const float3 &point, float maxDistance = 1e-6f) const;

	/// Returns true if the given point lies inside this filled circle.
	/// @param maxDistance The epsilon threshold to test the distance against.
//	bool DiscContains(const float3 &point, float maxDistance = 1e-6f) const;
//	bool DiscContains(const LineSegment &lineSegment, float maxDistance = 1e-6f) const;

	/// Returns the distance of the given object to the edge of this circle.
    float DistanceToEdge(const float3 &point) const;
//    float DistanceToEdge(const Ray &ray, float *d, float3 *closestPoint) const;
//    float DistanceToEdge(const LineSegment &lineSegment, float *d, float3 *closestPoint) const;
//    float DistanceToEdge(const Line &line, float *d, float3 *closestPoint) const;

    /// Returns the distance of the given object to this filled circle.
    float DistanceToDisc(const float3 &point) const;
/*
    float DistanceToDisc(const Ray &ray, float *d, float3 *closestPoint) const;
    float DistanceToDisc(const LineSegment &lineSegment, float *d, float3 *closestPoint) const;
    float DistanceToDisc(const Line &line, float *d, float3 *closestPoint) const;
*/
	/// Returns the closest point on the edge of this circle to the given object.
	float3 ClosestPointToEdge(const float3 &point) const;
//	float3 ClosestPointToEdge(const Ray &ray, float *d) const;
//	float3 ClosestPointToEdge(const LineSegment &lineSegment, float *d) const;
//	float3 ClosestPointToEdge(const Line &line, float *d) const;

    /// Returns the closest point on the disc of this circle to the given object.
	float3 ClosestPointToDisc(const float3 &point) const;

    /// Tests this circle for an intersection against the given plane.
    /// Returns the number of intersection points found for this circle and the given plane.
    int Intersects(const Plane &plane, float3 *pt1, float3 *pt2) const;
    int Intersects(const Plane &plane) const;
    bool IntersectsDisc(const Line &line) const;
    bool IntersectsDisc(const LineSegment &lineSegment) const;
    bool IntersectsDisc(const Ray &ray) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns a human-readable representation of this Circle. Most useful for debugging purposes.
    /** The returned string specifies the center position, normal direction and the radius of this Circle. */
    std::string ToString() const;
#endif
#ifdef MATH_QT_INTEROP
    operator QString() const { return toString(); }
    QString toString() const { return QString::fromStdString(ToString()); }
#endif
};

MATH_END_NAMESPACE

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(Circle)
Q_DECLARE_METATYPE(Circle*)
#endif
