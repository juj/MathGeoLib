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

/** @file float3.h
	@author Jukka Jylänki
	@brief A 3D (x,y,z) ordered tuple. */
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#endif

#include "../MathGeoLibFwd.h"

#ifdef MATH_QT_INTEROP
#include <QVector3D>
#endif
#ifdef MATH_OGRE_INTEROP
#include <OgreVector3.h>
#endif
#ifdef MATH_BULLET_INTEROP
#include <LinearMath/btVector3.h>
#endif
#ifdef MATH_URHO3D_INTEROP
#include <Urho3D/Math/Vector3.h>
#endif
#ifdef MATH_IRRKLANG_INTEROP
#include <ik_vec3d.h>
#endif

MATH_BEGIN_NAMESPACE

/// A vector of form (x,y,z).
class float3
{
public:
	enum
	{
		/// Specifies the number of elements in this vector.
		Size = 3
	};
	/// The x component.
	/** A float3 is 12 bytes in size. This element lies in the memory offsets 0-3 of this class. */
	float x;
	/// The y component. [similarOverload: x]
	/** This element is packed to the memory offsets 4-7 of this class. */
	float y;
	/// The z component. [similarOverload: x]
	/** This element is packed to the memory offsets 8-11 of this class. */
	float z;

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members x, y and z are all undefined after creating a new float3 using
		this default constructor. Remember to assign to them before use.
		@see x, y, z. */
	float3() {}

#ifdef MATH_EXPLICIT_COPYCTORS
	/// The float3 copy constructor.
	/** The copy constructor is a standard default copy-ctor, but it is explicitly written to be able to automatically pick up
		this function for script bindings. */
	float3(const float3 &rhs) { x = rhs.x; y = rhs.y; z = rhs.z; }
#endif

	/// Constructs a new float3 with the value (x, y, z).
	/** @see x, y, z. */
	float3(float x, float y, float z);

	/// Constructs a new float3 with the value (scalar, scalar, scalar).
	/** @see x, y, z. */
	explicit float3(float scalar);

	/// Constructs a new float3 with the value (xy.x, xy.y, z).
	float3(const float2 &xy, float z);

	/// Constructs this float3 from a C array, to the value (data[0], data[1], data[2]).
	/** @param data An array containing three elements for x, y and z. This pointer may not be null. */
	explicit float3(const float *data);

	/// Casts this float3 to a C array.
	/** This function does not allocate new memory or make a copy of this float3. This function simply
		returns a C pointer view to this data structure. Use ptr()[0] to access the x component of this float3,
		ptr()[1] to access y, and ptr()[2] to access the z component of this float3.
		@note Since the returned pointer points to this class, do not dereference the pointer after this
			float3 has been deleted. You should never store a copy of the returned pointer.
		@note This function is provided for compatibility with other APIs which require raw C pointer access
			to vectors. Avoid using this function in general, and instead always use the operator [] or
			the At() function to access the elements of this vector by index.
		@return A pointer to the first float element of this class. The data is contiguous in memory.
		@see operator [](), At(). */
	FORCE_INLINE float *ptr() { return &x; }
	FORCE_INLINE const float *ptr() const { return &x; }

	/// Accesses an element of this vector using array notation.
	/** @param index The element to get. Pass in 0 for x, 1 for y and 2 for z.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec[1] = 10.f; would set the y-component of this vector.
		@see ptr(), At(). */
	float &operator [](int index) { return At(index); }
	CONST_WIN32 float operator [](int index) const { return At(index); }

	/// Accesses an element of this vector.
	/** @param index The element to get. Pass in 0 for x, 1 for y, and 2 for z.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec.At(1) = 10.f; would set the y-component of this vector.
		@see ptr(), operator [](). */
	float &At(int index);
	CONST_WIN32 float At(int index) const;

	/// Adds two vectors. [indexTitle: operators +,-,*,/]
	/** This function is identical to the member function Add().
		@return float3(x + v.x, y + v.y, z + v.z); */
	float3 operator +(const float3 &v) const;
	/// Performs an unary negation of this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Neg().
		@return float3(-x, -y, -z). */
	float3 operator -() const;
	/// Subtracts the given vector from this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Sub().
		@return float3(x - v.x, y - v.y, z - v.z); */
	float3 operator -(const float3 &v) const;
	/// Multiplies this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Mul().
		@return float3(x * scalar, y * scalar, z * scalar); */
	float3 operator *(float scalar) const;
	/// Divides this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Div().
		@return float3(x / scalar, y / scalar, z / scalar); */
	float3 operator /(float scalar) const;
	/// Unary operator + allows this structure to be used in an expression '+x'.
	float3 operator +() const { return *this; }

	/// Adds a vector to this vector, in-place. [indexTitle: operators +=,-=,*=,/=]
	/** @return A reference to this. */
	float3 &operator +=(const float3 &v);
	/// Subtracts a vector from this vector, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float3 &operator -=(const float3 &v);
	/// Multiplies this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float3 &operator *=(float scalar);
	/// Divides this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float3 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
	// In math textbooks, pointwise multiplication of vectors is not defined within a linear space.
	// However, in programming it is often useful for e.g. modulating colors via pointwise multiplication.
	// If you #define MATH_ENABLE_UNCOMMON_OPERATIONS, you'll get these operations upgraded to handy
	// operator * and / notation and can use vec * vec and vec / vec. Otherwise, use the notation
	// vec.Mul(vec) and vec.Div(vec) for pointwise notation. MATH_ENABLE_UNCOMMON_OPERATIONS also enables
	// the operation scalar / vec.
	float3 operator *(const float3 &vector) const { return this->Mul(vector); }
	float3 operator /(const float3 &vector) const { return this->Div(vector); }
	float3 &operator *=(const float3 &vector) { *this = this->Mul(vector); return *this; }
	float3 &operator /=(const float3 &vector) { *this = this->Div(vector); return *this; }
#endif

	/// Adds a vector to this vector. [IndexTitle: Add/Sub/Mul/Div]
	/// @return (x+v.x, y+v.y, z+v.z).
	float3 Add(const float3 &v) const { return *this + v; }

	/// Adds the vector (s,s,s) to this vector.
	/// @note Mathematically, the addition of a vector and scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x+s, y+s, z+s).
	float3 Add(float s) const;

	/// Subtracts a vector from this vector. [similarOverload: Add] [hideIndex]
	/// @return (x-v.x, y-v.y, z-v.z).
	float3 Sub(const float3 &v) const { return *this - v; }

	/// Subtracts the vector (s,s,s) from this vector. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a vector by a scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x-s, y-s, z-s).
	float3 Sub(float s) const;

	/// Subtracts this vector from the vector (s,s,s). [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s-x, s-y, s-z).
	float3 SubLeft(float s) const;

	/// Multiplies this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the multiplication of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x*v.x, y*v.y, z*v.z).
	float3 Mul(const float3 &v) const;

	/// Multiplies this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @return (x*s, y*s, z*s).
	float3 Mul(float s) const { return *this * s; }

	/// Divides this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x/v.x, y/v.y, z/v.z).
	float3 Div(const float3 &v) const;

	/// Divides this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @return (x/s, y/s, z/s).
	float3 Div(float s) const { return *this / s; }
	
	/// Divides the vector (s,s,s) by this vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s/x, s/y, s/z).
	float3 DivLeft(float s) const;

	/// Performs a 2D swizzled access to this vector. [indexTitle: xx/xy/xz/..]
	/** The xy(), yz(), etc.. functions return a float2 which is a permuted selection of two elements of this float3.
		The xx(), yy(), zz() functions return a float2 which have the same scalar for both elements. */
	float2 xx() const;
	float2 xy() const; ///< [similarOverload: xx] [hideIndex]
	float2 xz() const; ///< [similarOverload: xx] [hideIndex]
	float2 yx() const; ///< [similarOverload: xx] [hideIndex]
	float2 yy() const; ///< [similarOverload: xx] [hideIndex]
	float2 yz() const; ///< [similarOverload: xx] [hideIndex]
	float2 zx() const; ///< [similarOverload: xx] [hideIndex]
	float2 zy() const; ///< [similarOverload: xx] [hideIndex]
	float2 zz() const; ///< [similarOverload: xx] [hideIndex]

	/// Performs a 3D swizzled access to this vector. [indexTitle: xyz/xzy/yzx/..]
	/** Use these functions to permute or replace elements of this vector with others.
		@return The function abc() returns float3(a,b,c). */
	float3 xxx() const { return float3(x,x,x); }
	float3 xxy() const { return float3(x,x,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 xxz() const { return float3(x,x,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 xyx() const { return float3(x,y,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 xyy() const { return float3(x,y,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 xyz() const { return float3(x,y,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 xzx() const { return float3(x,z,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 xzy() const { return float3(x,z,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 xzz() const { return float3(x,z,z); } ///< [similarOverload: xxx] [hideIndex]

	float3 yxx() const { return float3(y,x,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 yxy() const { return float3(y,x,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 yxz() const { return float3(y,x,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 yyx() const { return float3(y,y,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 yyy() const { return float3(y,y,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 yyz() const { return float3(y,y,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 yzx() const { return float3(y,z,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 yzy() const { return float3(y,z,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 yzz() const { return float3(y,z,z); } ///< [similarOverload: xxx] [hideIndex]

	float3 zxx() const { return float3(z,x,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 zxy() const { return float3(z,x,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 zxz() const { return float3(z,x,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 zyx() const { return float3(z,y,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 zyy() const { return float3(z,y,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 zyz() const { return float3(z,y,z); } ///< [similarOverload: xxx] [hideIndex]
	float3 zzx() const { return float3(z,z,x); } ///< [similarOverload: xxx] [hideIndex]
	float3 zzy() const { return float3(z,z,y); } ///< [similarOverload: xxx] [hideIndex]
	float3 zzz() const { return float3(z,z,z); } ///< [similarOverload: xxx] [hideIndex]

	/// Performs a swizzled access to this vector.
	/** For example, Swizzled(2,1,0) return float3(z,y,x). Swizzled(2,2,2,2) returns float4(z,z,z,z).
		@param i Chooses the element of this vector to pick for the x value of the returned vector, in the range [0, 2].
		@param j Chooses the element of this vector to pick for the y value of the returned vector, in the range [0, 2].
		@param k Chooses the element of this vector to pick for the z value of the returned vector, in the range [0, 2].
		@param l Chooses the element of this vector to pick for the w value of the returned vector, in the range [0, 2]. */
	float4 Swizzled(int i, int j, int k, int l) const;
	float3 Swizzled(int i, int j, int k) const;
	float2 Swizzled(int i, int j) const;

	/// Generates a new float3 by filling its entries by the given scalar.
	/** @see float3::float3(float scalar), SetFromScalar(). */
	static MUST_USE_RESULT float3 FromScalar(float scalar);

	/// Fills each entry of this float3 by the given scalar.
	/** @see float3::float3(float scalar), FromScalar(). */
	void SetFromScalar(float scalar);

	/// Sets all elements of this vector.
	/** @see x, y, z, At(). */
	void Set(float x, float y, float z);

	/// Converts the given vector represented in spherical coordinates to an euclidean float3 (x,y,z) triplet.
	/** @param azimuth The direction, or yaw, of the vector. This function uses the convention that the X-Z plane is
			the 2D horizontal "map" plane, with the vector (0,0,radius) corresponding to the vector in the direction azimuth=0 and inclination=0.
			This value is typically in the range [-pi, pi] (, or [0, 2pi]).
		@param inclination The elevation, or pitch, of the vector. This function uses the convention that the +Y axis
			points towards up, i.e. +Y is the "Zenith direction". This value is typically in the range [-pi/2, pi/2].
		@param radius The magnitude of the vector. This is usually >= 0, although passing in the zero vector as radius returns (0,0,0), and passing
			in a negative radius mirrors the coordinate along the origin.
		@see FromSphericalCoordinates, ToSphericalCoordinates, ToSphericalCoordinatesNormalized. */
	void SetFromSphericalCoordinates(float azimuth, float inclination, float radius);
	void SetFromSphericalCoordinates(const float3 &spherical) { SetFromSphericalCoordinates(spherical.x, spherical.y, spherical.z); }
	static MUST_USE_RESULT float3 FromSphericalCoordinates(float azimuth, float inclination, float radius);
	static MUST_USE_RESULT float3 FromSphericalCoordinates(const float3 &spherical) { return FromSphericalCoordinates(spherical.x, spherical.y, spherical.z); }

	/// Identical to SetFromSphericalCoordinates(azimuth, inclination, radius), except this function sets radius == 1 to generate a normalized
	/// vector on the unit sphere.
	/** @see FromSphericalCoordinates, ToSphericalCoordinates, ToSphericalCoordinatesNormalized. */
	void SetFromSphericalCoordinates(float azimuth, float inclination);
	static MUST_USE_RESULT float3 FromSphericalCoordinates(float azimuth, float inclination);

	/// @return float4(x,y,z,1).
	/** @see x, y, z, class float4, ToDir4(). */
	float4 ToPos4() const;

	/// @return float4(x,y,z,0). [similarOverload: ToPos4]
	/** @see x, y, z, class float4, ToPos4(). */
	float4 ToDir4() const;

	/// Converts this euclidean (x,y,z) float3 to spherical coordinates representation in the form (azimuth, inclination, radius).
	/** @note This corresponds to the matrix operation R_y * R_x * (0,0,radius), where R_y is a rotation about the y-axis by azimuth,
			and R_x is a rotation about the x-axis by inclination.
		@note It is valid for the magnitude of this vector to be (very close to) zero, in which case the return value is the zero vector.
		@see FromSphericalCoordinates, SetFromSphericalCoordinates, ToSphericalCoordinatesNormalized. */
	float3 ToSphericalCoordinates() const;

	/// Converts this normalized euclidean (x,y,z) float3 to spherical coordinates representation in the form (azimuth, inclination)
	/** @note This function requires that this float3 is normalized. This function is identical to ToSphericalCoordinates, but is slightly
			faster in the case this vector is known to be normalized in advance.
		@note This corresponsds to the matrix operation R_y * R_x * (0,0,radius), where R_y is a rotation about the y-axis by azimuth,
			and R_x is a rotation about the x-axis by inclination.
		@see ToSphericalCoordinates, FromSphericalCoordinates, SetFromSphericalCoordinates. */
	float2 ToSphericalCoordinatesNormalized() const;

	/// Computes the length of this vector.
	/** @return Sqrt(x*x + y*y + z*z).
		@see LengthSq(), Distance(), DistanceSq(). */
	float Length() const;

	/// Computes the squared length of this vector.
	/** Calling this function is faster than calling Length(), since this function avoids computing a square root.
		If you only need to compare lengths to each other, but are not interested in the actual length values,
		you can compare by using LengthSq(), instead of Length(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@return x*x + y*y + z*z.
		@see LengthSq(), Distance(), DistanceSq(). */
	float LengthSq() const;

	/// Normalizes this float3.
	/** In the case of failure, this vector is set to (1, 0, 0), so calling this function will never result in an
		unnormalized vector.
		@note If this function fails to normalize the vector, no error message is printed, the vector is set to (1,0,0) and
			an error code 0 is returned. This is different than the behavior of the Normalized() function, which prints an
			error if normalization fails.
		@note This function operates in-place.
		@return The old length of this vector, or 0 if normalization failed.
		@see Normalized(). */
	float Normalize();

	/// Returns a normalized copy of this vector.
	/** @note If the vector is zero and cannot be normalized, the vector (1, 0, 0) is returned, and an error message is printed.
			If you do not want to generate an error message on failure, but want to handle the failure yourself, use the
			Normalize() function instead.
		@see Normalize(). */
	float3 Normalized() const;

	/// Scales this vector so that its new length is as given.
	/** Calling this function is effectively the same as normalizing the vector first and then multiplying by newLength.
		In the case of failure, this vector is set to (newLength, 0, 0), so calling this function will never result in an
		unnormalized vector.
		@note This function operates in-place.
		@return The old length of this vector. If this function returns 0, the scaling failed, and this vector is arbitrarily
			reset to (newLength, 0, 0). In case of failure, no error message is generated. You are expected to handle the failure
			yourself.
		@see ScaledToLength(). */
	float ScaleToLength(float newLength);

	/// Returns a scaled copy of this vector which has its new length as given.
	/** This function assumes the length of this vector is not zero. In the case of failure, an error message is printed,
		and the vector (newLength, 0, 0) is returned.
		@see ScaleToLength(). */
	float3 ScaledToLength(float newLength) const;

	/// Tests if the length of this vector is one, up to the given epsilon.
	/** @see IsZero(), IsFinite(), IsPerpendicular(). */
	bool IsNormalized(float epsilonSq = 1e-5f) const;

	/// Tests if this is the null vector, up to the given epsilon.
	/** @see IsNormalized(), IsFinite(), IsPerpendicular(). */
	bool IsZero(float epsilonSq = 1e-7f) const;

	/// Tests if this vector contains valid finite elements.
	/** @see IsNormalized(), IsZero(), IsPerpendicular(). */
	bool IsFinite() const;

	/// Tests if two vectors are perpendicular to each other.
	/** @see IsNormalized(), IsZero(), IsPerpendicular(), Equals(). */
	bool IsPerpendicular(const float3 &other, float epsilonSq = 1e-5f) const;

	/// Tests if the points p1, p2 and p3 lie on a straight line, up to the given epsilon.
	/** @see AreOrthogonal(), AreOrthonormal(), Line::AreCollinear(). */
	static MUST_USE_RESULT bool AreCollinear(const float3 &p1, const float3 &p2, const float3 &p3, float epsilonSq = 1e-7f);

	/// Tests if two vectors are equal, up to the given epsilon.
	/** @see IsPerpendicular(). */
	bool Equals(const float3 &other, float epsilon = 1e-3f) const;
	bool Equals(float x, float y, float z, float epsilon = 1e-3f) const;

	/// Compares whether this float3 and the given float3 are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const float3 &other) const;

#ifdef MATH_ENABLE_STL_SUPPORT
	/// Returns "(x, y, z)".
	std::string ToString() const;

	/// Returns "x,y,z". This is the preferred format for the float3 if it has to be serialized to a string for machine transfer.
	std::string SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	std::string SerializeToCodeString() const;
#endif

	/// Parses a string that is of form "x,y,z" or "(x,y,z)" or "(x;y;z)" or "x y z" to a new float3.
	static MUST_USE_RESULT float3 FromString(const char *str, const char **outEndStr = 0);
#ifdef MATH_ENABLE_STL_SUPPORT
	static MUST_USE_RESULT float3 FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

	/// @return x + y + z.
	float SumOfElements() const;
	/// @return x * y * z.
	float ProductOfElements() const;
	/// @return (x+y+z)/3.
	float AverageOfElements() const;
	/// @return Min(x, y, z).
	/** @see MinElementIndex(). */
	float MinElement() const;
	/// Returns the index that has the smallest value in this vector.
	/** @see MinElement(). */
	int MinElementIndex() const;
	/// @return Max(x, y, z).
	/** @see MaxElementIndex(). */
	float MaxElement() const;
	/// Returns the index that has the smallest value in this vector.
	/** @see MaxElement(). */
	int MaxElementIndex() const;
	/// Takes the element-wise absolute value of this vector.
	/** @return float3(|x|, |y|, |z|).
		@see Neg(). */
	float3 Abs() const;
	/// Returns a copy of this vector with each element negated.
	/** This function returns a new vector where each element x of the original vector is replaced by the value -x.
		@return float3(-x, -y, -z).
		@see Abs(). */
	float3 Neg() const;
	/// Computes the element-wise reciprocal of this vector.
	/** This function returns a new vector where each element x of the original vector is replaced by the value 1/x.
		@return float3(1/x, 1/y, 1/z). */
	float3 Recip() const;
	/// Returns an element-wise minimum of this and the vector (ceil, ceil, ceil).
	/** Each element that is larger than ceil is replaced by ceil. */
	float3 Min(float ceil) const;
	/// Returns an element-wise minimum of this and the given vector.
	/** Each element that is larger than ceil is replaced by ceil.
		@see Max(), Clamp(). */
	float3 Min(const float3 &ceil) const;
	/// Returns an element-wise maximum of this and the vector (floor, floor, floor).
	/** Each element that is smaller than floor is replaced by floor. */
	float3 Max(float floor) const;
	/// Returns an element-wise maximum of this and the given vector.
	/** Each element that is smaller than floor is replaced by floor.
		@see Min(), Clamp(). */
	float3 Max(const float3 &floor) const;
	/// Returns a vector that has floor <= this[i] <= ceil for each element.
	float3 Clamp(float floor, float ceil) const;
	/// Limits each element of this vector between the corresponding elements in floor and ceil.
	/** @see Min(), Max(), Clamp01(), ClampLength(). */
	float3 Clamp(const float3 &floor, const float3 &ceil) const;
	/// Limits each element of this vector in the range [0, 1].
	/** @see Min(), Max(), Clamp(), ClampLength(). */
	float3 Clamp01() const;

	/// Returns a copy of this vector, with its length scaled down to maxLength.
	/** @see Clamp(). */
	float3 ClampLength(float maxLength) const;
	/// Returns a copy of this vector, with its length scaled between minLength and maxLength.
	/** @see Clamp(). */
	float3 ClampLength(float minLength, float maxLength) const;
				
	/// Computes the distance between this point and the given object.
	/** This function finds the nearest point to this point on the given object, and computes its distance
		to this point.
		If this point lies inside the given object, a distance of 0 is returned.
		@todo Add float3::Distance(Polygon/Circle/Disc/Frustum/Polyhedron).
		@see DistanceSq(), Length(), LengthSq(). */
	float Distance(const float3 &point) const;
	float Distance(const Line &line) const;
	float Distance(const Ray &ray) const;
	float Distance(const LineSegment &lineSegment) const;
	float Distance(const Plane &plane) const;
	float Distance(const Triangle &triangle) const;
	float Distance(const AABB &aabb) const;
	float Distance(const OBB &obb) const;
	float Distance(const Sphere &sphere) const;
	float Distance(const Capsule &capsule) const;

	/// Computes the squared distance between this and the given point.
	/** Calling this function is faster than calling Distance(), since this function avoids computing a square root.
		If you only need to compare distances to each other, but are not interested in the actual distance values,
		you can compare by using DistanceSq(), instead of Distance(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@see Distance(), Length(), LengthSq(). */
	float DistanceSq(const float3 &point) const;

	/// Computes the dot product of this and the given vector.
	/** The dot product has a geometric interpretation of measuring how close two direction vectors are to pointing
		in the same direction, computing angles between vectors, or the length of a projection of one vector to another.
		@return x*v.x + y*v.y + z*v.z.
		@see AngleBetween(), ProjectTo(), ProjectToNorm(), Cross(), OuterProduct(), ScalarTripleProduct(). */
	float Dot(const float3 &v) const;

	/// Computes the cross product of this and the given vector.
	/** Unless this vector and the given vector are linearly dependent, the cross product returns a vector that is perpendicular
		to both vectors.
		@return float3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x).
		@see Dot(), OuterProduct(), Perpendicular(), AnotherPerpendicular(), ScalarTripleProduct(). */
	float3 Cross(const float3 &v) const;

	/// Computes the outer product of this and the given vector.
	/** @see Dot(), Cross(). */
	float3x3 OuterProduct(const float3 &rhs) const;

	/// Computes a new normalized direction vector that is perpendicular to this vector and the specified hint vector.
	/** If this vector points toward the hint vector, the vector hint2 is returned instead.
		@see AnotherPerpendicular(), Cross(). */
	float3 Perpendicular(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;

	/// Returns another vector that is perpendicular to this vector and the vector returned by Perpendicular().
	/** The set (this, Perpendicular(), AnotherPerpendicular()) forms a right-handed normalized 3D basis.
		@see Perpendicular(), Cross(). */
	float3 AnotherPerpendicular(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;

	// Completes this vector to generate a perpendicular basis.
	/** This function computes two new vectors b and c which are both orthogonal to this vector and to each other.
		That is, the set { this, b, c} is an orthogonal set. The vectors b and c that are outputted are also normalized.
		@param outB [out] Receives vector b.
		@param outC [out] Receives vector c.
		@note When calling this function, this vector should not be zero! */
	void PerpendicularBasis(float3 &outB, float3 &outC) const;

	/// Generates a random vector that is perpendicular to this vector.
	/** The distribution is uniformly random. */
	float3 RandomPerpendicular(LCG &rng) const;

	/// Computes the scalar triple product of the given three vectors.
	/** @return [u v w] = (u x v) . w = u . (v x w)
		@see Dot(), Cross(). */
	static MUST_USE_RESULT float ScalarTripleProduct(const float3 &u, const float3 &v, const float3 &w);

	/// Returns this vector reflected about a plane with the given normal.
	/** By convention, both this and the reflected vector point away from the plane with the given normal
		@see Refract(). */
	float3 Reflect(const float3 &normal) const;

	/// Refracts this vector about a plane with the given normal.
	/** By convention, the this vector points towards the plane, and the returned vector points away from the plane.
		When the ray is going from a denser material to a lighter one, total internal reflection can occur.
		In this case, this function will just return a reflected vector from a call to Reflect().
		@param normal Specifies the plane normal direction
		@param negativeSideRefractionIndex The refraction index of the material we are exiting.
		@param positiveSideRefractionIndex The refraction index of the material we are entering.
		@see Reflect(). */
	float3 Refract(const float3 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

	/// Projects this vector onto the given unnormalized direction vector.
	/** @param direction The direction vector to project this vector onto. This function will normalize this
			vector, so you can pass in an unnormalized vector.
		@see ProjectToNorm(). */
	float3 ProjectTo(const float3 &direction) const;

	/// Projects this vector onto the given normalized direction vector.
	/** @param direction The vector to project onto. This vector must be normalized.
		@see ProjectTo(). */
	float3 ProjectToNorm(const float3 &direction) const;

	/// Returns the angle between this vector and the specified vector, in radians.
	/** @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
			If you are computing the angle between two normalized vectors, it is better to use AngleBetweenNorm().
		@see AngleBetweenNorm(). */		
	float AngleBetween(const float3 &other) const;

	/// Returns the angle between this vector and the specified normalized vector, in radians.
	/** @param normalizedVector The direction vector to compute the angle against. This vector must be normalized.
		@note This vector must be normalized to call this function.
		@see AngleBetween(). */
	float AngleBetweenNorm(const float3 &normalizedVector) const;

	/// Breaks this vector down into parallel and perpendicular components with respect to the given direction.
	/** @param direction The direction the decomposition is to be computed. This vector must be normalized.
		@param outParallel [out] Receives the part of this vector that is parallel to the given direction vector.
		@param outPerpendicular [out] Receives the part of this vector that is perpendicular to the given direction vector. */
	void Decompose(const float3 &direction, float3 &outParallel, float3 &outPerpendicular) const;

	/// Linearly interpolates between this and the vector b.
	/** @param b The target endpoint to lerp towards to.
		@param t The interpolation weight, in the range [0, 1].
		@return Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
			Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on.
			Lerp(b, t) returns (1-t)*this + t*b. */
	float3 Lerp(const float3 &b, float t) const;
	/// This function is the same as calling a.Lerp(b, t).
	static MUST_USE_RESULT float3 Lerp(const float3 &a, const float3 &b, float t);

	/// Makes the given vectors linearly independent.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
		The vector a is kept unmodified, and vector b is modified to be perpendicular to a.
		Finally, if specified, the vector c is adjusted to be perpendicular to a and b.
		@note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthogonal.
		@see AreOrthogonal(), Orthonormalize(), AreOrthonormal(). */
	static void Orthogonalize(const float3 &a, float3 &b);
	static void Orthogonalize(const float3 &a, float3 &b, float3 &c);

	/// Returns true if the given vectors are orthogonal to each other.
	/** @see Orthogonalize(), Orthonormalize(), AreOrthonormal(), AreCollinear(). */
	static MUST_USE_RESULT bool AreOrthogonal(const float3 &a, const float3 &b, float epsilon = 1e-3f);
	static MUST_USE_RESULT bool AreOrthogonal(const float3 &a, const float3 &b, const float3 &c, float epsilon = 1e-3f);

	/// Makes the given vectors linearly independent and normalized in length.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
		The vector a is first normalized, and vector b is modified to be perpendicular to a, and also normalized.
		Finally, if specified, the vector c is adjusted to be perpendicular to a and b, and normalized.
		@note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthonormal.
		@see Orthogonalize(), AreOrthogonal(), AreOrthonormal(). */
	static void Orthonormalize(float3 &a, float3 &b);
	static void Orthonormalize(float3 &a, float3 &b, float3 &c);

	/// Returns true if the given vectors are orthogonal to each other and all of length 1.
	/** @see Orthogonalize(), AreOrthogonal(), Orthonormalize(), AreCollinear(). */
	static MUST_USE_RESULT bool AreOrthonormal(const float3 &a, const float3 &b, float epsilon = 1e-3f);
	static MUST_USE_RESULT bool AreOrthonormal(const float3 &a, const float3 &b, const float3 &c, float epsilon = 1e-3f);

	/// Generates a direction vector of the given length.
	/** The returned vector points at a uniformly random direction.
		@see RandomSphere(), RandomBox(). */
	static MUST_USE_RESULT float3 RandomDir(LCG &lcg, float length = 1.f);
	/// Generates a random point inside a sphere.
	/** The returned point is generated uniformly inside the sphere.
		@see RandomDir(), RandomBox(). */
	static MUST_USE_RESULT float3 RandomSphere(LCG &lcg, const float3 &center, float radius);
	/// Generates a random point inside an axis-aligned box.
	/** The returned point is generated uniformly inside the box.
		@see RandomDir(), RandomSphere(). */
	static MUST_USE_RESULT float3 RandomBox(LCG &lcg, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
	static MUST_USE_RESULT float3 RandomBox(LCG &lcg, const float3 &minValues, const float3 &maxValues);

	/// Returns a random float3 with each entry randomized between the range [minElem, maxElem].
	static MUST_USE_RESULT float3 RandomBox(LCG &lcg, float minElem, float maxElem);

	// Identical to RandomBox, but provided for generic API between float3 and float4 in templates.
	static inline float3 RandomGeneral(LCG &lcg, float minElem, float maxElem) { return RandomBox(lcg, minElem, maxElem); }

	/// Specifies a compile-time constant float3 with value (0, 0, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 zero;
	/// Specifies a compile-time constant float3 with value (1, 1, 1). [similarOverload: zero]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 one;
	/// Specifies a compile-time constant float3 with value (1, 0, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 unitX;
	/// Specifies a compile-time constant float3 with value (0, 1, 0). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 unitY;
	/// Specifies a compile-time constant float3 with value (0, 0, 1). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 unitZ;
	/// A compile-time constant float3 with value (NaN, NaN, NaN).
	/** For this constant, each element has the value of quiet NaN, or Not-A-Number.
		@note Never compare a float3 to this value! Due to how IEEE floats work, "nan == nan" returns false!
			  That is, nothing is equal to NaN, not even NaN itself!
		@note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 nan;
	/// A compile-time constant float3 with value (+infinity, +infinity, +infinity). [similarOverload: nan]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float3 inf;

#ifdef MATH_OGRE_INTEROP
	float3(const Ogre::Vector3 &other):x(other.x), y(other.y), z(other.z) {}
	operator Ogre::Vector3() const { return Ogre::Vector3(x, y, z); }
#endif
#ifdef MATH_QT_INTEROP
	float3(const QVector3D &other):x(other.x()), y(other.y()), z(other.z()) {}
	operator QVector3D() const { return QVector3D(x, y, z); }
	operator QString() const { return "float3(" + QString::number(x) + "," + QString::number(y) + "," + QString::number(z) + ")"; }
	QString toString() const { return (QString)*this; }
	QVector3D ToQVector3D() const { return QVector3D(x, y, z); }
	static float3 FromQVector3D(const QVector3D &v) { return (float3)v; }
	static float3 FromString(const QString &str) { return FromString(str.toStdString()); }
#endif
#ifdef MATH_BULLET_INTEROP
	float3(const btVector3 &other):x(other.x()), y(other.y()), z(other.z()) {}
	operator btVector3() const { return btVector3(x, y, z); }
#endif
#ifdef MATH_URHO3D_INTEROP
	float3(const Urho3D::Vector3 &other) : x(other.x_), y(other.y_), z(other.z_) {}
	operator Urho3D::Vector3() const { return Urho3D::Vector3(x, y, z); }
#endif
#ifdef MATH_IRRKLANG_INTEROP
    float3(const irrklang::vec3df &other) : x(other.X), y(other.Y), z(other.Z) {}
    operator irrklang::vec3df() const { return irrklang::vec3df(x, y, z); }
#endif
};

/// Prints this float3 to the given stream.
#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3 &rhs);
#endif

float3 operator *(float scalar, const float3 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float3 operator /(float scalar, const float3 &rhs) { return float3::FromScalar(scalar) / rhs; }
#endif

inline float Dot(const float3 &a, const float3 &b) { return a.Dot(b); }
inline float3 Cross(const float3 &a, const float3 &b) { return a.Cross(b); }
inline float3 Abs(const float3 &a) { return a.Abs(); }
inline float Length(const float3 &a) { return a.Length(); }
inline float Distance(const float3 &a, const float3 &b) { return a.Distance(b); }
inline float3 Min(const float3 &a, const float3 &b) { return a.Min(b); }
inline float3 Max(const float3 &a, const float3 &b) { return a.Max(b); }
inline float3 Clamp(const float3 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float3 Clamp(const float3 &a, const float3 &floor, const float3 &ceil) { return a.Clamp(floor, ceil); }
inline float3 Clamp01(const float3 &a) { return a.Clamp01(); }
inline float3 Lerp(const float3 &a, const float3 &b, float t) { return a.Lerp(b, t); }

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(float3)
Q_DECLARE_METATYPE(float3*)
#endif

MATH_END_NAMESPACE

#include "float4.h"
#include "MathFunc.h"

MATH_BEGIN_NAMESPACE

#ifdef MATH_AUTOMATIC_SSE
bool EqualAbs(float a, float b, float epsilon);

#define POINT_VEC(...) float4(__VA_ARGS__, 1.f)
#define DIR_VEC(...) float4(__VA_ARGS__, 0.f)

#define POINT_VEC_SCALAR(s) float4(pos_from_scalar_ps(s))
#define DIR_VEC_SCALAR(s) float4(dir_from_scalar_ps(s))

#define POINT_TO_FLOAT3(v) (v).xyz()
#define DIR_TO_FLOAT3(v) (v).xyz()

#define POINT_TO_FLOAT4(v) (v)
#define DIR_TO_FLOAT4(v) (v)

#define FLOAT4_TO_POINT(v) (v)
#define FLOAT4_TO_DIR(v) (v)

/* /// TODO: Enable this:
inline float3 POINT_TO_FLOAT3(const vec &v)
{
	assume(EqualAbs(v.w, 1.f, 1e-4f));
	return v.xyz();
}
inline float3 DIR_TO_FLOAT3(const vec &v)
{
	assume(EqualAbs(v.w, 0.f, 1e-4f));
	return v.xyz();
}
*/
#else
#define POINT_VEC(...) float3(__VA_ARGS__)
#define DIR_VEC(...) float3(__VA_ARGS__)
#define POINT_TO_FLOAT3(x) x
#define DIR_TO_FLOAT3(x) x
#define POINT_VEC_SCALAR(s) float3::FromScalar(s)
#define DIR_VEC_SCALAR(s) float3::FromScalar(s)
#define POINT_TO_FLOAT4(v) float4(v, 1.f)
#define DIR_TO_FLOAT4(v) float4(v, 0.f)
#define FLOAT4_TO_POINT(v) (v).xyz()
#define FLOAT4_TO_DIR(v) (v).xyz()

#endif

vec PointVecFromString(const char *str, const char **outEndStr = 0);
vec DirVecFromString(const char *str, const char **outEndStr = 0);

MATH_END_NAMESPACE
