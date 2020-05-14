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

/** @file float2.h
	@author Jukka Jylänki
	@brief A 2D (x,y) ordered pair. */
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#include <vector>
#endif

#include "../MathGeoLibFwd.h"
#include "MathConstants.h"
#include "assume.h"

MATH_BEGIN_NAMESPACE

/// A vector of form (x,y).
class float2
{
public:
	enum
	{
		/// Specifies the number of elements in this vector.
		Size = 2
	};
	/// The x component.
	/** A float2 is 8 bytes in size. This element lies in the memory offsets 0-3 of this class. */
	float x;
	/// The y component. [similarOverload: x]
	/** This element is packed to the memory offsets 4-7 of this class. */
	float y;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members x and y are both undefined after creating a new float2 using
		this default constructor. Remember to assign to them before use.
		@see x, y. */
	float2() {}

	/// The float2 copy constructor.
	float2(const float2 &rhs) { x = rhs.x; y = rhs.y; }

	/// Constructs a new float2 with the value (x, y).
	/** @see x, y. */
	float2(float x, float y);

	/// Constructs a new float2 with the value (scalar, scalar).
	/** @see x, y. */
	explicit float2(float scalar);

	/// Constructs this float2 from a C array, to the value (data[0], data[1]).
	/** @param data An array containing two elements for x and y. This pointer may not be null. */
	explicit float2(const float *data);

	/// Casts this float2 to a C array.
	/** This function does not allocate new memory or make a copy of this float2. This function simply
		returns a C pointer view to this data structure. Use ptr()[0] to access the x component of this float2
		and ptr()[1] to access the y component.
		@note Since the returned pointer points to this class, do not dereference the pointer after this
			float2 has been deleted. You should never store a copy of the returned pointer.
		@note This function is provided for compatibility with other APIs which require raw C pointer access
			to vectors. Avoid using this function in general, and instead always use the operator []
			or the At() function to access the elements of this vector by index.
		@return A pointer to the first float element of this class. The data is contiguous in memory.
		@see operator [](), At(). */
	FORCE_INLINE float *ptr() { return &x; }
	FORCE_INLINE const float *ptr() const { return &x; }

	/// Accesses an element of this vector using array notation.
	/** @param index The element to get. Pass in 0 for x and 1 for y.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec[1] = 10.f; would set the y-component of this vector.
		@see ptr(), At(). */
	FORCE_INLINE float &operator [](int index) { return At(index); }
	FORCE_INLINE CONST_WIN32 float operator [](int index) const { return At(index); }

	/// Accesses an element of this vector.
	/** @param index The element to get. Pass in 0 for x and 1 for y.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec.At(1) = 10.f; would set the y-component of this vector.
		@see ptr(), operator [](). */
	FORCE_INLINE CONST_WIN32 float At(int index) const
	{
		assume(index >= 0);
		assume(index < Size);
		return ptr()[index];
	}
	
	FORCE_INLINE float &At(int index)
	{
		assume(index >= 0);
		assume(index < Size);
		return ptr()[index];
	}

	/// Adds two vectors. [indexTitle: operators +,-,*,/]
	/** This function is identical to the member function Add().
		@return float2(x + v.x, y + v.y); */
	float2 operator +(const float2 &v) const;
	/// Performs an unary negation of this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Neg().
		@return float2(-x, -y). */
	float2 operator -() const;
	/// Subtracts the given vector from this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Sub().
		@return float2(x - v.x, y - v.y); */
	float2 operator -(const float2 &v) const;
	/// Multiplies this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Mul().
		@return float2(x * scalar, y * scalar); */
	float2 operator *(float scalar) const;
	/// Divides this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Div().
		@return float2(x / scalar, y / scalar); */
	float2 operator /(float scalar) const;
	/// Unary operator + allows this structure to be used in an expression '+x'.
	float2 operator +() const { return *this; }

	/// Assigns a vector to another.
	/** @return A reference to this. */
	float2 &operator =(const float2 &v);
	/// Adds a vector to this vector, in-place. [indexTitle: operators +=,-=,*=,/=]
	/** @return A reference to this. */
	float2 &operator +=(const float2 &v);
	/// Subtracts a vector from this vector, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float2 &operator -=(const float2 &v);
	/// Multiplies this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float2 &operator *=(float scalar);
	/// Divides this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float2 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
	// In math textbooks, pointwise multiplication of vectors is not defined within a linear space.
	// However, in programming it is often useful for e.g. modulating colors via pointwise multiplication.
	// If you #define MATH_ENABLE_UNCOMMON_OPERATIONS, you'll get these operations upgraded to handy
	// operator * and / notation and can use vec * vec and vec / vec. Otherwise, use the notation
	// vec.Mul(vec) and vec.Div(vec) for pointwise notation. MATH_ENABLE_UNCOMMON_OPERATIONS also enables
	// the operation scalar / vec.
	float2 operator *(const float2 &vector) const { return this->Mul(vector); }
	float2 operator /(const float2 &vector) const { return this->Div(vector); }
	float2 &operator *=(const float2 &vector) { *this = this->Mul(vector); return *this; }
	float2 &operator /=(const float2 &vector) { *this = this->Div(vector); return *this; }
#endif

	/// Adds a vector to this vector. [IndexTitle: Add/Sub/Mul/Div]
	/// @return (x+v.x, y+v.y).
	float2 Add(const float2 &v) const { return *this + v; }

	/// Adds the vector (s,s) to this vector.
	/// @note Mathematically, the addition of a vector and scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x+s, y+s).
	float2 Add(float s) const;

	/// Subtracts a vector from this vector. [similarOverload: Add] [hideIndex]
	/// @return (x-v.x, y-v.y).
	float2 Sub(const float2 &v) const { return *this - v; }

	/// Subtracts the vector (s,s) from this vector. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a vector by a scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x-s, y-s).
	float2 Sub(float s) const;

	/// Subtracts this vector from the vector (s,s). [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s-x, s-y).
	float2 SubLeft(float s) const;

	/// Multiplies this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the multiplication of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x*v.x, y*v.y).
	float2 Mul(const float2 &v) const;

	/// Multiplies this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @return (x*s, y*s).
	float2 Mul(float s) const { return *this * s; }

	/// Divides this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x/v.x, y/v.y).
	float2 Div(const float2 &v) const;

	/// Divides this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @return (x/s, y/s).
	float2 Div(float s) const { return *this / s; }
	
	/// Divides the vector (s,s) by this vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s/x, s/y).
	float2 DivLeft(float s) const;

	/// Performs a 2D swizzled access to this vector. [indexTitle: xx/xy/yx/yy]
	float2 xx() const { return float2(x,x); }
	float2 xy() const { return float2(x,y); } ///< [similarOverload: xx] [hideIndex]
	float2 yx() const { return float2(y,x); } ///< [similarOverload: xx] [hideIndex]
	float2 yy() const { return float2(y,y); } ///< [similarOverload: xx] [hideIndex]

#ifndef MATH_VEC_IS_FLOAT4
	/// Reinterpret-casts this float4 to a vec2d, which is either a float2 if building without SSE/NEON enabled,
	/// or a float4 if building with SSE enabled. (practically projects this 4D vector to 2D x-y part).
	FORCE_INLINE const vec2d &ToVec2D() const { return *this; }
#endif

	/// Performs a swizzled access to this vector.
	/** For example, Swizzled(2,1,0) return float3(z,y,x). Swizzled(2,2,2,2) returns float4(z,z,z,z).
		@param i Chooses the element of this vector to pick for the x value of the returned vector, in the range [0, 2].
		@param j Chooses the element of this vector to pick for the y value of the returned vector, in the range [0, 2].
		@param k Chooses the element of this vector to pick for the z value of the returned vector, in the range [0, 2].
		@param l Chooses the element of this vector to pick for the w value of the returned vector, in the range [0, 2]. */
	float4 Swizzled(int i, int j, int k, int l) const;
	float3 Swizzled(int i, int j, int k) const;
	float2 Swizzled(int i, int j) const;

	/// Generates a new float2 by filling its entries by the given scalar.
	/** @see float2::float2(float scalar), SetFromScalar(). */
	static float2 FromScalar(float scalar);

	/// Fills each entry of this float2 by the given scalar.
	/** @see float2::float2(float scalar), FromScalar(). */
	void SetFromScalar(float scalar);

	/// Sets all elements of this vector.
	/** @see x, y, At().. */
	void Set(float x, float y);

	/// Converts the given vector represented in polar coordinates to an euclidean float2 (x,y) pair.
	/** @param theta The direction (aimed angle, azimuth) of the vector, in radians. The value theta==0 returns a value in the +X direction,
		the value theta=pi/2 corresponds to +Y, theta=pi corresponds to -X and theta=-pi/2 (or 3pi/2) corresponds to -Y.
			This value is typically in the range [-pi, pi] (, or [0, 2pi]).
		@param length The magnitude of the vector. This is usually >= 0, although passing in the zero vector as radius returns (0,0), and passing
			in a negative radius mirrors the coordinate along the origin.
		@see FromPolarCoordinates, ToPolarCoordinates, AimedAngle. */
	void SetFromPolarCoordinates(float theta, float length);
	void SetFromPolarCoordinates(const float2 &polar) { SetFromPolarCoordinates(polar.x, polar.y); }
	static float2 FromPolarCoordinates(float theta, float length);
	static float2 FromPolarCoordinates(const float2 &polar) { return FromPolarCoordinates(polar.x, polar.y); }

	/// Converts this euclidean (x,y) float2 to polar coordinates representation in the form (theta, length).
	/** @note It is valid for the magnitude of this vector to be (very close to) zero, in which case the return value is the zero vector.
		@return A float2 that has the first component (x) representing the aimed angle (azimuth) of this direction vector, in radians,
		and is equal to atan2(this.y, this.x). The x component has a range of ]-pi/2, pi/2]. The second component (y) of the returned vector
		stores the length (radius) of this vector.
		@see SetFromPolarCoordinates, FromPolarCoorindates, AimedAngle. */
	float2 ToPolarCoordinates() const;

	/// Returns the aimed angle direction of this vector, in radians.
	/** The aimed angle of a 2D vector corresponds to the theta part (or azimuth) of the polar coordinate representation of this vector. Essentially,
		describes the direction this vector is pointing at. A vector pointing towards +X returns 0, vector pointing towards +Y returns pi/2, vector
		pointing towards -X returns pi, and a vector pointing towards -Y returns -pi/2 (equal to 3pi/2).
		@note This vector does not need to be normalized for this function to work, but it DOES need to be non-zero (unlike the function ToPolarCoordinates).
		@return The aimed angle in the range ]-pi/2, pi/2].
		@see ToPolarCoordinates, FromPolarCoordinates, SetFromPolarCoordinates. */
	float AimedAngle() const;

	/// Computes the length of this vector.
	/** @return Sqrt(x*x + y*y).
		@see LengthSq(), Distance(), DistanceSq(). */
	float Length() const;

	/// Computes the squared length of this vector.
	/** Calling this function is faster than calling Length(), since this function avoids computing a square root.
		If you only need to compare lengths to each other, but are not interested in the actual length values,
		you can compare by using LengthSq(), instead of Length(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@return x*x + y*y.
		@see LengthSq(), Distance(), DistanceSq(). */
	float LengthSq() const;

	/// Normalizes this float2.
	/** In the case of failure, this vector is set to (1, 0), so calling this function will never result in an
		unnormalized vector.
		@note If this function fails to normalize the vector, no error message is printed, the vector is set to (1,0) and
			an error code 0 is returned. This is different than the behavior of the Normalized() function, which prints an
			error if normalization fails.
		@note This function operates in-place.
		@return The old length of this vector, or 0 if normalization failed.
		@see Normalized(). */
	float Normalize();

	/// Returns a normalized copy of this vector.
	/** @note If the vector is zero and cannot be normalized, the vector (1, 0) is returned, and an error message is printed.
			If you do not want to generate an error message on failure, but want to handle the failure yourself, use the
			Normalize() function instead.
		@see Normalize(). */
	float2 Normalized() const;

	/// Scales this vector so that its new length is as given.
	/** Calling this function is effectively the same as normalizing the vector first and then multiplying by newLength.
		In the case of failure, this vector is set to (newLength, 0), so calling this function will never result in an
		unnormalized vector.
		@note This function operates in-place.
		@return The old length of this vector. If this function returns 0, the scaling failed, and this vector is arbitrarily
			reset to (newLength, 0). In case of failure, no error message is generated. You are expected to handle the failure
			yourself.
		@see ScaledToLength(). */
	float ScaleToLength(float newLength);

	/// Returns a scaled copy of this vector which has its new length as given.
	/** This function assumes the length of this vector is not zero. In the case of failure, an error message is printed,
		and the vector (newLength, 0) is returned.
		@see ScaleToLength(). */
	float2 ScaledToLength(float newLength) const;

	/// Tests if the length of this vector is one, up to the given epsilon.
	/** @see IsZero(), IsFinite(), IsPerpendicular(). */
	bool IsNormalized(float epsilonSq = 1e-5f) const;

	/// Tests if this is the null vector, up to the given epsilon.
	/** @see IsNormalized(), IsFinite(), IsPerpendicular(). */
	bool IsZero(float epsilonSq = 1e-6f) const;

	/// Tests if this vector contains valid finite elements.
	/** @see IsNormalized(), IsZero(), IsPerpendicular(). */
	bool IsFinite() const;

	/// Tests if two vectors are perpendicular to each other.
	/** @see IsNormalized(), IsZero(), IsPerpendicular(), Equals(). */
	bool IsPerpendicular(const float2 &other, float epsilonSq = 1e-5f) const;

	/// Tests if two vectors are equal, up to the given epsilon.
	/** @see IsPerpendicular(). */
	bool Equals(const float2 &other, float epsilon = 1e-3f) const;
	bool Equals(float x, float y, float epsilon = 1e-3f) const;

	/// Compares whether this float2 and the given float2 are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const float2 &other) const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x, y)".
	StringT ToString() const;

	/// Returns "x,y". This is the preferred format for the float2 if it has to be serialized to a string for machine transfer.
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static float2 FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	/// Parses a string that is of form "x,y" or "(x,y)" or "(x;y)" or "x y" to a new float2.
	static float2 FromString(const char *str, const char **outEndStr = 0);

	/// @return x + y.
	float SumOfElements() const;
	/// @return x * y.
	float ProductOfElements() const;
	/// @return (x+y)/2.
	float AverageOfElements() const;
	/// @return Min(x, y).
	/** @see MinElementIndex(). */
	float MinElement() const;
	/// Returns the index that has the smallest value in this vector.
	/** @see MinElement(). */
	int MinElementIndex() const;
	/// @return Max(x, y).
	/** @see MaxElementIndex(). */
	float MaxElement() const;
	/// Returns the index that has the smallest value in this vector.
	/** @see MaxElement(). */
	int MaxElementIndex() const;
	/// Takes the element-wise absolute value of this vector.
	/** @return float2(|x|, |y|).
		@see Neg(). */
	float2 Abs() const;
	/// Returns a copy of this vector with each element negated.
	/** This function returns a new vector where each element x of the original vector is replaced by the value -x.
		@return float2(-x, -y).
		@see Abs(). */
	float2 Neg() const;
	/// Computes the element-wise reciprocal of this vector.
	/** This function returns a new vector where each element x of the original vector is replaced by the value 1/x.
		@return float2(1/x, 1/y). */
	float2 Recip() const;
	/// Returns an element-wise minimum of this and the vector (ceil, ceil, ceil).
	/** Each element that is larger than ceil is replaced by ceil. */
	float2 Min(float ceil) const;
	/// Returns an element-wise minimum of this and the given vector.
	/** Each element that is larger than ceil is replaced by ceil.
		@see Max(), Clamp(). */
	float2 Min(const float2 &ceil) const;
	/// Returns an element-wise maximum of this and the vector (floor, floor, floor).
	/** Each element that is smaller than floor is replaced by floor. */
	float2 Max(float floor) const;
	/// Returns an element-wise maximum of this and the given vector.
	/** Each element that is smaller than floor is replaced by floor.
		@see Min(), Clamp(). */
	float2 Max(const float2 &floor) const;
	/// Returns a vector that has floor <= this[i] <= ceil for each element.
	float2 Clamp(float floor, float ceil) const;
	/// Limits each element of this vector between the corresponding elements in floor and ceil.
	/** @see Min(), Max(), Clamp01(). */
	float2 Clamp(const float2 &floor, const float2 &ceil) const;
	/// Limits each element of this vector in the range [0, 1].
	/** @see Min(), Max(), Clamp(). */
	float2 Clamp01() const;

	/// Computes the distance between this and the given float2.
	/** @see DistanceSq(), Length(), LengthSq(). */
	float Distance(const float2 &point) const;

	/// Computes the squared distance between this and the given point.
	/** Calling this function is faster than calling Distance(), since this function avoids computing a square root.
		If you only need to compare distances to each other, but are not interested in the actual distance values,
		you can compare by using DistanceSq(), instead of Distance(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@see Distance(), Length(), LengthSq(). */
	float DistanceSq(const float2 &point) const;

	/// Computes the dot product of this and the given vector.
	/** The dot product has a geometric interpretation of measuring how close two direction vectors are to pointing
		in the same direction, computing angles between vectors, or the length of a projection of one vector to another.
		@return x*v.x + y*v.y.
		@see AngleBetween(), ProjectTo(), ProjectToNorm(), Perp(), PerpDot(). */
	float Dot(const float2 &v) const;

	/// Returns this vector with the "perp" operator applied to it.
	/** The perp operator rotates a vector 90 degrees ccw (around the "z axis"), i.e.
		for a 2D vector (x,y), this function returns the vector (-y, x).
		@note This function is identical to Rotated90CCW().
		@return (-y, x). The returned vector is perpendicular to this vector.
		@see PerpDot(), Rotated90CCW(). */
	float2 Perp() const;

	/// Computes the perp-dot product of this and the given float2 in the order this^perp (dot) rhs.
	/** @see Dot(), Perp(). */
	float PerpDot(const float2 &rhs) const;

	/// Rotates this vector 90 degrees clock-wise.
	/** This rotation is interpreted in a coordinate system on a plane where +x extends to the right, and +y extends upwards.
		@see Perp(), Rotated90CW(), Rotate90CCW(), Rotated90CCW(). */
	void Rotate90CW();

	/// Returns a vector that is perpendicular to this vector (rotated 90 degrees clock-wise).
	/** @note This function is identical to Perp().
		@see Perp(), Rotate90CW(), Rotate90CCW(), Rotated90CCW(). */
	float2 Rotated90CW() const;

	/// Rotates this vector 90 degrees counterclock-wise .
	/// This is in a coordinate system on a plane where +x extends to the right, and +y extends upwards.
	/** @see Perp(), Rotate90CW(), Rotated90CW(), Rotated90CCW(). */
	void Rotate90CCW();

	/// Returns a vector that is perpendicular to this vector (rotated 90 degrees counter-clock-wise).
	/** @see Perp(), Rotate90CW(), Rotated90CW(), Rotate90CCW(). */
	float2 Rotated90CCW() const;

	/// Returns this vector reflected about a plane with the given normal.
	/** By convention, both this and the reflected vector point away from the plane with the given normal
		@see Refract(). */
	float2 Reflect(const float2 &normal) const;

	/// Refracts this vector about a plane with the given normal.
	/** By convention, the this vector points towards the plane, and the returned vector points away from the plane.
		When the ray is going from a denser material to a lighter one, total internal reflection can occur.
		In this case, this function will just return a reflected vector from a call to Reflect().
		@param normal Specifies the plane normal direction
		@param negativeSideRefractionIndex The refraction index of the material we are exiting.
		@param positiveSideRefractionIndex The refraction index of the material we are entering.
		@see Reflect(). */
	float2 Refract(const float2 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

	/// Projects this vector onto the given unnormalized direction vector.
	/** @param direction The direction vector to project this vector onto. This function will normalize this
			vector, so you can pass in an unnormalized vector.
		@see ProjectToNorm(). */
	float2 ProjectTo(const float2 &direction) const;

	/// Projects this vector onto the given normalized direction vector.
	/** @param direction The vector to project onto. This vector must be normalized.
		@see ProjectTo(). */
	float2 ProjectToNorm(const float2 &direction) const;

	/// Returns the angle between this vector and the specified vector, in radians.
	/** @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
			If you are computing the angle between two normalized vectors, it is better to use AngleBetweenNorm().
		@see AngleBetweenNorm(). */		
	float AngleBetween(const float2 &other) const;

	/// Returns the angle between this vector and the specified normalized vector, in radians.
	/** @param normalizedVector The direction vector to compute the angle against. This vector must be normalized.
		@note This vector must be normalized to call this function.
		@see AngleBetween(). */
	float AngleBetweenNorm(const float2 &normalizedVector) const;

	/// Breaks this vector down into parallel and perpendicular components with respect to the given direction.
	/** @param direction The direction the decomposition is to be computed. This vector must be normalized.
		@param outParallel [out] Receives the part of this vector that is parallel to the given direction vector.
		@param outPerpendicular [out] Receives the part of this vector that is perpendicular to the given direction vector. */
	void Decompose(const float2 &direction, float2 &outParallel, float2 &outPerpendicular) const;

	/// Linearly interpolates between this and the vector b.
	/** @param b The target endpoint to lerp towards to.
		@param t The interpolation weight, in the range [0, 1].
		@return Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
			Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on.
			Lerp(b, t) returns (1-t)*this + t*b. */
	float2 Lerp(const float2 &b, float t) const;
	/// This function is the same as calling a.Lerp(b, t).
	static float2 Lerp(const float2 &a, const float2 &b, float t);

	/// Makes the given vectors linearly independent.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
		The vector a is kept unmodified, and vector b is modified to be perpendicular to a.
		@note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthogonal.
		@see AreOrthogonal(), Orthonormalize(), AreOrthonormal(). */
	static void Orthogonalize(const float2 &a, float2 &b);

	/// Returns true if the given vectors are orthogonal to each other.
	/** @see Orthogonalize(), Orthonormalize(), AreOrthonormal(). */
	static bool AreOrthogonal(const float2 &a, const float2 &b, float epsilon = 1e-3f);

	/// Makes the given vectors linearly independent and normalized in length.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
		The vector a is first normalized, and vector b is modified to be perpendicular to a, and also normalized.
		@note If either of the input vectors is zero, then the resulting set of vectors cannot be made orthonormal.
		@see Orthogonalize(), AreOrthogonal(), AreOrthonormal(). */
	static void Orthonormalize(float2 &a, float2 &b);

	/// Tests if the triangle a->b->c is oriented counter-clockwise.
	/** Returns true if the triangle a->b->c is oriented counter-clockwise, when viewed in the XY-plane
		where x spans to the right and y spans up.
		Another way to think of this is that this function returns true, if the point C lies to the left
		of the directed line AB. */
	static bool OrientedCCW(const float2 &a, const float2 &b, const float2 &c);

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Computes the 2D convex hull of the given point set.
	/* @see ConvexHullInPlace */
	static void ConvexHull(const float2 *pointArray, int numPoints, std::vector<float2> &outConvexHull);
#endif

	/// Computes the 2D convex hull of the given point set, in-place.
	/** This version of the algorithm works in-place, meaning that when the algorithm finishes,
		pointArray will contain the list of the points on the convex hull.
		@note As a convention, the convex hull winds counter-clockwise when graphed in the xy plane where
			+x points to the right and +y points up. That is, walking along the polylist
			intArray[0] -> pointArray[1] -> pointArray[2] -> ... -> pointArray[numPoints-1] -> pointArray[0] performs
			a counter-clockwise tour.
		@param pointArray [in, out] A pointer to an array of numPoints float2 points that represent a point cloud. This
			array will be rewritten to contain the convex hull of the original point set.
		@return The number of points on the convex hull, i.e. the number of elements used in pointArray after the operation.
		@see ConvexHull(). */
	static int ConvexHullInPlace(float2 *pointArray, int numPoints);

	/// Tests whether a 2D convex hull contains the given point.
	/** @param convexHull [in] A pointer to an array of points in the convex hull.
		@param numPointsInConvexHull The number of elements in the array convexHull.
		@param point The target point to test. */
	static bool ConvexHullContains(const float2 *convexHull, int numPointsInConvexHull, const float2 &point);

	/// Computes the minimum-area rectangle that bounds the given point set. [noscript]
	/** Implementation adapted from Christer Ericson's Real-time Collision Detection, p.111.
		@param pointArray [in] A pointer to an array of points to process.
		@param numPoints The number of elements in the array pointed to by pointArray.
		@param center [out] This variable will receive the center point of the rectangle.
		@param uDir [out] This variable will receive a normalized direction vector pointing one of the side directionss of the rectangle.
		@param vDir [out] This variable will receive a normalized direction vector pointing the other side direction of the rectangle.
		@param minU [out] Receives the minimum extent of the processed point set along the u direction.
		@param maxU [out] Receives the maximum extent of the processed point set along the u direction.
		@param minV [out] Receives the minimum extent of the processed point set along the v direction.
		@param maxV [out] Receives the maximum extent of the processed point set along the v direction.
		@note This algorithm runs in O(n^2) time to the number of points in the input.
		@note For best performance, the input point array should contain only the points in the convex hull of the point set. This algorithm
			does not compute the convex hull for you.
		@return The area of the resulting rectangle. */
	static float MinAreaRectInPlace(float2 *pointArray, int numPoints, float2 &center, float2 &uDir, float2 &vDir, float &minU, float &maxU, float &minV, float &maxV);

	/// Generates a direction vector of the given length pointing at a uniformly random direction.
	static float2 RandomDir(LCG &lcg, float length = 1.f);

	/// Returns a random float3 with each entry randomized between the range [minElem, maxElem].
	static MUST_USE_RESULT float2 RandomBox(LCG &lcg, float minElem, float maxElem);

	/// Specifies a compile-time constant float2 with value (0, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 zero;
	/// Specifies a compile-time constant float2 with value (1, 1). [similarOverload: zero]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 one;
	/// Specifies a compile-time constant float2 with value (1, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 unitX;
	/// Specifies a compile-time constant float2 with value (0, 1). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 unitY;
	/// A compile-time constant float2 with value (NaN, NaN).
	/** For this constant, each element has the value of quiet NaN, or Not-A-Number.
		@note Never compare a float2 to this value! Due to how IEEE floats work, "nan == nan" returns false!
			  That is, nothing is equal to NaN, not even NaN itself!
		@note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 nan;
	/// A compile-time constant float2 with value (+infinity, +infinity). [similarOverload: nan]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float2 inf;
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float2 to the given stream.
std::ostream &operator <<(std::ostream &out, const float2 &rhs);
#endif

float2 operator *(float scalar, const float2 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float2 operator /(float scalar, const float2 &rhs) { return float2::FromScalar(scalar) / rhs; }
#endif

inline float Dot(const float2 &a, const float2 &b) { return a.Dot(b); }
inline float2 Abs(const float2 &a) { return a.Abs(); }
inline float Length(const float2 &a) { return a.Length(); }
inline float Distance(const float2 &a, const float2 &b) { return a.Distance(b); }
inline float2 Min(const float2 &a, const float2 &b) { return a.Min(b); }
inline float2 Max(const float2 &a, const float2 &b) { return a.Max(b); }
inline float2 Clamp(const float2 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp(const float2 &a, const float2 &floor, const float2 &ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp01(const float2 &a) { return a.Clamp01(); }
inline float2 Lerp(const float2 &a, const float2 &b, float t) { return a.Lerp(b, t); }

inline float2 Perp2D(const float2 &v) { return v.Perp(); }
float2 Mul2D(const float3x3 &transform, const float2 &v);
float2 MulPos2D(const float3x4 &transform, const float2 &v);
float2 MulDir2D(const float3x4 &transform, const float2 &v);
float2 MulPos2D(const float4x4 &transform, const float2 &v);
float2 MulDir2D(const float4x4 &transform, const float2 &v);

template<typename T>
int float2_ConvexHullInPlace(T *p, int n);

template<typename T>
bool float2_ConvexHullContains(T *hull, int n, const T &point);

MATH_END_NAMESPACE
