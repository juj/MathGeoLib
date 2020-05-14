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

   /** @file Circle.h
	   @author Jukka Jylänki
	   @brief The Circle geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float2.h"

MATH_BEGIN_NAMESPACE

/// A two-dimensional circle in 2D space.
/** This class represents both a hollow circle (only edge) and a solid circle (disc). */
class Circle2D
{
public:
	/// The center position of this circle.
	float2 pos;

	/// The radius of the circle. [similarOverload: pos]
	/** This parameter must be strictly positive to specify a non-degenerate circle. If zero is specified, this circle
		is considered to be degenerate.
		@see Circle::Circle(). */
	float r;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members pos and r are all undefined after creating a new circle using
		this default constructor. Remember to assign to them before use.
		@see pos, r. */
	Circle2D() {}

	/// Constructs a 2D circle with a given position and radius.
	/** @param radius A value > 0 constructs a circle with positive volume. A value of <= 0 is valid, and constructs a degenerate circle.
		@see pos, r, IsFinite(), IsDegenerate() */
	Circle2D(const float2 &center, float radius);

	/// Tests if this Circle2D is finite.
	/** A circle is <b><i>finite</i></b> if its members pos and r do not contain floating-point NaNs or +/-infs
		in them.
		@return True if the members pos and r both have finite floating-point values.
		@see pos, r, IsDegenerate(), ::IsFinite(), IsInf(), IsNan(), IsFinite(), inf, negInf, nan, float3::nan, float3::inf, SetNegativeInfinity(). */
	bool IsFinite() const;

	/// Returns true if this Circle2D is degenerate.
	/** A circle is <b><i>degenerate</i></b> if it is not finite, or if the radius of the circle is less or equal to 0.
		@see pos, r, IsFinite(), SetNegativeInfinity(). */
	bool IsDegenerate() const;

	/// Tests if the given object is fully contained inside this sphere.
	/** @see Distance(), Intersects(), ClosestPoint().
		@todo Add Sphere::Contains(Circle/Disc). */
	bool Contains(const float2 &point) const;
	bool Contains(const float2 &point, float epsilon) const;

	/// Returns the distance between this sphere and the given object.
	/** This function finds the nearest pair of points on this and the given object, and computes their distance.
		If the two objects intersect, or one object is contained inside the other, the returned distance is zero.
		@see Contains(), Intersects(), ClosestPoint().
		@todo Add Sphere::Distance(Polygon/Circle/Disc/Frustum/Polyhedron). */
	float Distance(const float2 &point) const;
	float SignedDistance(const float2 &point) const;

	/// Computes the minimal bounding circle for three points.
	/** This function computes the smallest volume circle that contains the given three points. The smallest
		enclosing circle may not pass through all the three points that are specified. */
	static Circle2D OptimalEnclosingCircle(const float2 &a, const float2 &b, const float2 &c);

	/// Computes the minimal bounding circle for the given point array.
	/** This function implements Emo Welzl's optimal enclosing circle algorithm.
		See "Smallest enclosing disks (balls and ellipsoids)", Lecture Notes in Computer Science 555 (1991) pp. 359-370.
		The running time is expected linear time, but compared to Ritter's algorithm (the FastEnclosingCircle() function),
		this algorithm is considerably slower.
		@param pointArray An array of points to compute an enclosing circle for. This pointer must not be null.
		@param numPoints The number of elements in the input array pointArray.
		@see FastEnclosingCircle(). */
	static Circle2D OptimalEnclosingCircle(const float2 *pointArray, int numPoints);

	/// Generates a random point inside this sphere.
	/** The points are distributed uniformly.
		This function uses the rejection method to generate a uniform distribution of points inside a sphere. Therefore
		it is assumed that this sphere is not degenerate, i.e. it has a positive radius.
		A fixed number of 1000 tries is performed, after which the sphere center position is returned as a fallback.
		@param lcg A pre-seeded random number generator object that is to be used by this function to generate random values.
		@see class LCG, RandomPointOnSurface(), IsDegenerate().
		@todo Add Sphere::Point(polarYaw, polarPitch, radius). */
	float2 RandomPointInside(LCG &lcg);

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a human-readable representation of this Sphere. Most useful for debugging purposes.
	StringT ToString() const;
#endif
};

MATH_END_NAMESPACE
