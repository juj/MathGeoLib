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

/** @file float4.h
	@author Jukka Jylänki
	@brief A 4D (x,y,z,w) homogeneous vector. */
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#include <vector>
#endif
#include "../MathGeoLibFwd.h"
#include "MathConstants.h"
#include "float3.h"
#include "SSEMath.h"
#include "assume.h"

MATH_BEGIN_NAMESPACE

/// A 3D vector of form (x,y,z,w) in a 4D homogeneous coordinate space.
/** This class has two sets of member functions. The functions ending in a suffix '3' operate only on the
	(x, y, z) part, ignoring the w component (or assuming a value of 0 or 1, where expectable). The functions
	without the '3' suffix operate on all four elements of the vector. */
class ALIGN16 float4
{
public:
	enum
	{
		/// Specifies the number of elements in this vector.
		Size = 4
	};

#if defined(MATH_SIMD)
	NAMELESS_UNION_BEGIN // Allow nonstandard nameless struct in union extension on MSC.

	union
	{
		struct
		{
#endif
			/// The x component.
			/** A float4 is 16 bytes in size. This element lies in the memory offsets 0-3 of this class. */
			float x;
			/// The y component. [similarOverload: x]
			/** This element is packed to the memory offsets 4-7 of this class. */
			float y;
			/// The z component. [similarOverload: x]
			/** This element is packed to the memory offsets 8-11 of this class. */
			float z;
			/// The w component. [similarOverload: x]
			/** This element is packed to the memory offsets 12-15 of this class. */
			float w;
#if defined(MATH_SIMD)
		};
		simd4f v;
	};
	NAMELESS_UNION_END
#endif

	/// The default constructor does not initialize any members of this class.
	/** This means that the values of the members x, y, z and w are all undefined after creating a new float4 using
		this default constructor. Remember to assign to them before use.
		@see x, y, z, w. */
	float4() {}

	/// The float4 copy constructor.
	float4(const float4 &rhs) { Set(rhs); }

	/// Constructs a new float4 with the value (x, y, z, w).
	/** @note If you are constructing a float4 from an array of consecutive values, always prefer calling "float4(ptr);" instead of "float4(ptr[0], ptr[1], ptr[2], ptr[3]);"
			because there is a considerable SIMD performance benefit in the first form.
		@see x, y, z, w. */
	float4(float x, float y, float z, float w);

	/// Constructs a new float3 with the value (xyz.x, xyz.y, xyz.z, w).
	/** @see x, y, z, w. */
	float4(const float3 &xyz, float w);

	float4(float x, float y, const float2 &zw);
	float4(float x, const float2 &yz, float w);
	float4(float x, const float3 &yzw);
	float4(const float2 &xy, const float2 &zw);

	/// Constructs a new float3 with the value (xy.x, xy.y, z, w).
	/** @see x, y, z, w. */
	float4(const float2 &xy, float z, float w);

	/// Constructs this float4 from a C array, to the value (data[0], data[1], data[2], data[3]).
	/** @param data An array containing four elements for x, y, z and w. This pointer may not be null. */
	explicit float4(const float *data);

	/// Casts this float4 to a C array.
	/** This function does not allocate new memory or make a copy of this float4. This function simply
		returns a C pointer view to this data structure. Use ptr()[0] to access the x component of this float4,
		ptr()[1] to access y, ptr()[2] to access z, and ptr()[3] to access the w component of this float4.
		@note Since the returned pointer points to this class, do not dereference the pointer after this
			float3 has been deleted. You should never store a copy of the returned pointer.
		@note This function is provided for compatibility with other APIs which require raw C pointer access
			to vectors. Avoid using this function in general, and instead always use the operator [] of this
			class to access the elements of this vector by index.
		@return A pointer to the first float element of this class. The data is contiguous in memory.
		@see operator [](). */
	FORCE_INLINE float *ptr() { return &x; }
	FORCE_INLINE const float *ptr() const { return &x; }

	/// Accesses an element of this vector using array notation.
	/** @param index The element to get. Pass in 0 for x, 1 for y, 2 for z and 3 for w.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec[1] = 10.f; would set the y-component of this vector. */
	FORCE_INLINE float &operator [](int index) { return At(index); }
	FORCE_INLINE CONST_WIN32 float operator [](int index) const { return At(index); }

	/// Accesses an element of this vector.
	/** @param index The element to get. Pass in 0 for x, 1 for y, 2 for z and 3 for w.
		@note If you have a non-const instance of this class, you can use this notation to set the elements of
			this vector as well, e.g. vec.At(1) = 10.f; would set the y-component of this vector. */
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
		@return float4(x + v.x, y + v.y, z + v.z, w + v.w); */
	float4 operator +(const float4 &v) const;

	/// Performs an unary negation of this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Neg().
		@return float4(-x, -y, -z, -w). */
	float4 operator -() const;

	/// Subtracts the given vector from this vector. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Sub().
		@return float4(x - v.x, y - v.y, z - v.z, w - v.w); */
	float4 operator -(const float4 &v) const;

	/// Multiplies this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Mul().
		@return float4(x * scalar, y * scalar, z * scalar, w * scalar); */
	float4 operator *(float scalar) const;

	/// Divides this vector by a scalar. [similarOverload: operator+] [hideIndex]
	/** This function is identical to the member function Div().
		@return float4(x / scalar, y / scalar, z / scalar, w * scalar); */
	float4 operator /(float scalar) const;

	/// Unary operator + allows this structure to be used in an expression '+x'.
	float4 operator +() const { return *this; }

	/// Assigns a vector to another.
	/** @return A reference to this. */
	float4 &operator =(const float4 &v);

	/// Adds a vector to this vector, in-place. [indexTitle: operators +=,-=,*=,/=]
	/** @return A reference to this. */
	float4 &operator +=(const float4 &v);

	/// Subtracts a vector from this vector, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float4 &operator -=(const float4 &v);

	/// Multiplies this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @note If w != 0, multiplying by a scalar does <b>not</b> have the effect of scaling the length of this 3D vector.
		@return A reference to this. */
	float4 &operator *=(float scalar);

	/// Divides this vector by a scalar, in-place. [similarOverload: operator+=] [hideIndex]
	/** @return A reference to this. */
	float4 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
	// In math textbooks, pointwise multiplication of vectors is not defined within a linear space.
	// However, in programming it is often useful for e.g. modulating colors via pointwise multiplication.
	// If you #define MATH_ENABLE_UNCOMMON_OPERATIONS, you'll get these operations upgraded to handy
	// operator * and / notation and can use vec * vec and vec / vec. Otherwise, use the notation
	// vec.Mul(vec) and vec.Div(vec) for pointwise notation. MATH_ENABLE_UNCOMMON_OPERATIONS also enables
	// the operation scalar / vec.
	float4 operator *(const float4 &vector) const { return this->Mul(vector); }
	float4 operator /(const float4 &vector) const { return this->Div(vector); }
	float4 &operator *=(const float4 &vector) { *this = this->Mul(vector); return *this; }
	float4 &operator /=(const float4 &vector) { *this = this->Div(vector); return *this; }
#endif

	/// Adds a vector to this vector. [IndexTitle: Add/Sub/Mul/Div]
	/// @return (x+v.x, y+v.y, z+v.z, w+v.w).
	float4 Add(const float4 &rhs) const { return *this + rhs; }

	/// Adds the vector (s,s,s,s) to this vector.
	/// @note Mathematically, the addition of a vector and scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x+s, y+s, z+s, w+s).
	float4 Add(float s) const;

	/// Subtracts a vector from this vector. [similarOverload: Add] [hideIndex]
	/// @return (x-v.x, y-v.y, z-v.z, w-v.w).
	float4 Sub(const float4 &rhs) const { return *this - rhs; }

	/// Subtracts the vector (s,s,s,s) from this vector. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a vector by a scalar is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x-s, y-s, z-s, w-s).
	float4 Sub(float s) const;

	/// Subtracts this vector from the vector (s,s,s,s). [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the subtraction of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s-x, s-y, s-z, s-w).
	float4 SubLeft(float s) const;

	/// Multiplies this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the multiplication of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x*v.x, y*v.y, z*v.z, w*v.w).
	float4 Mul(const float4 &v) const;

	/// Multiplies this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @note If w != 0, multiplying by a scalar does <b>not</b> have the effect of scaling the length of this 3D vector.
	/// @return (x*s, y*s, z*s, w*s).
	float4 Mul(float s) const { return *this * s; }

	/// Divides this vector by a vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of two vectors is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (x/v.x, y/v.y, z/v.z, w/v.w).
	float4 Div(const float4 &v) const;

	/// Divides this vector by a scalar. [similarOverload: Add] [hideIndex]
	/// @return (x/s, y/s, z/s, w/s).
	float4 Div(float s) const { return *this / s; }
	
	/// Divides the vector (s,s,s,s) by this vector, element-wise. [similarOverload: Add] [hideIndex]
	/// @note Mathematically, the division of a scalar by a vector is not defined in linear space structures,
	///	 but this function is provided here for syntactical convenience.
	/// @return (s/x, s/y, s/z, s/w).
	float4 DivLeft(float s) const;

	/// Returns the (x, y) part of this vector.
	float2 xy() const;

	/// Returns the (x, y, z) part of this vector.
	float3 xyz() const;

	/// Reinterpret-casts this float4 to a vec2d, which is either a float2 if building without SSE/NEON enabled,
	/// or a float4 if building with SSE enabled. (practically projects this 4D vector to 2D x-y part).
	FORCE_INLINE const vec2d &ToVec2D() const { return *reinterpret_cast<const vec2d*>(this); }

	/// Reinterpret-casts the (x, y, z) part of this vector.
	/** @note This aliases into this float4! Use xyz() to make a copy.
		@see xyz() */
	float3 &Float3Part() { return *reinterpret_cast<float3*>(this); }
	const float3 &Float3Part() const { return *reinterpret_cast<const float3*>(this); }

	/// Performs a swizzled access to this vector.
	/** For example, Swizzled(2,1,0) return float3(z,y,x). Swizzled(3,3,3,3) returns float4(w,w,w,w).
		@param i Chooses the element of this vector to pick for the x value of the returned vector, in the range [0, 3].
		@param j Chooses the element of this vector to pick for the y value of the returned vector, in the range [0, 3].
		@param k Chooses the element of this vector to pick for the z value of the returned vector, in the range [0, 3].
		@param l Chooses the element of this vector to pick for the w value of the returned vector, in the range [0, 3].
		@see xyz(). */
	float4 Swizzled(int i, int j, int k, int l) const;
	float3 Swizzled(int i, int j, int k) const;
	float2 Swizzled(int i, int j) const;

	float4 xxxx() const;
	float4 xxxw() const;
	float4 yyyy() const;
	float4 yyyw() const;
	float4 zzzz() const;
	float4 zzzw() const;
	float4 wwww() const;

	/// Returns float4(scalar, scalar, scalar, scalar).
	/** @see float4::float4(float scalar), SetFromScalar(). */
	static float4 FromScalar(float scalar);

	/// Returns float4(scalar, scalar, scalar, w).
	static float4 FromScalar(float scalar, float w);

	/// Sets this float4 to (scalar, scalar, scalar, scalar).
	/** @see float4::float4(float scalar), FromScalar(). */
	void SetFromScalar(float scalar);

	/// Sets this float4 to (scalar, scalar, scalar, w).
	void SetFromScalar(float scalar, float w);

	/// Converts the given vector represented in spherical coordinates to an euclidean vector (x,y,z,w=0) triplet.
	/** @param azimuth The direction, or yaw, of the vector. This function uses the convention that the X-Z plane is
			the 2D horizontal "map" plane, with the vector (0,0,radius) corresponding to the vector in the direction azimuth=0 and inclination=0.
			This value is typically in the range [-pi, pi] (, or [0, 2pi]).
		@param inclination The elevation, or pitch, of the vector. This function uses the convention that the +Y axis
			points towards up, i.e. +Y is the "Zenith direction". This value is typically in the range [-pi/2, pi/2].
		@param radius The magnitude of the vector. This is usually >= 0, although passing in the zero vector as radius returns (0,0,0), and passing
			in a negative radius mirrors the coordinate along the origin.
		@note The returned vector will be a direction vector with w==0.
		@see FromSphericalCoordinates, ToSphericalCoordinates, ToSphericalCoordinatesNormalized. */
	void SetFromSphericalCoordinates(float azimuth, float inclination, float radius);
	void SetFromSphericalCoordinates(const float3 &spherical) { SetFromSphericalCoordinates(spherical.x, spherical.y, spherical.z); }
	static MUST_USE_RESULT float4 FromSphericalCoordinates(float azimuth, float inclination, float radius);
	static MUST_USE_RESULT float4 FromSphericalCoordinates(const float3 &spherical) { return FromSphericalCoordinates(spherical.x, spherical.y, spherical.z); }

	/// Converts the given vector represented in spherical coordinates to an euclidean direction vector.
	/** @param azimuth The direction, or yaw, of the vector. This function uses the convention that the X-Z plane is
			the 2D horizontal "map" plane, with the vector (0,0,radius) corresponding to the vector in the direction azimuth=0 and inclination=0.
			This value is typically in the range [-pi, pi] (, or [0, 2pi]).
		@param inclination The elevation, or pitch, of the vector. This function uses the convention that the +Y axis
			points towards up, i.e. +Y is the "Zenith direction". This value is typically in the range [-pi/2, pi/2]. */
	void SetFromSphericalCoordinates(float azimuth, float inclination);
	static MUST_USE_RESULT float4 FromSphericalCoordinates(float azimuth, float inclination);

	/// Converts this euclidean (x,y,z) vector to spherical coordinates representation in the form (azimuth, inclination, radius).
	/** @note This corresponds to the matrix operation R_y * R_x * (0,0,radius), where R_y is a rotation about the y-axis by azimuth,
			and R_x is a rotation about the x-axis by inclination.
		@note It is valid for the magnitude of this vector to be (very close to) zero, in which case the return value is the zero vector.
		@see FromSphericalCoordinates, SetFromSphericalCoordinates, ToSphericalCoordinatesNormalized. */
	float3 ToSphericalCoordinates() const;

	/// Converts this normalized euclidean (x,y,z) vector to spherical coordinates representation in the form (azimuth, inclination)
	/** @note This function requires that this vector is normalized. This function is identical to ToSphericalCoordinates, but is slightly
			faster in the case this vector is known to be normalized in advance.
		@note This corresponsds to the matrix operation R_y * R_x * (0,0,radius), where R_y is a rotation about the y-axis by azimuth,
			and R_x is a rotation about the x-axis by inclination.
		@see ToSphericalCoordinates, FromSphericalCoordinates, SetFromSphericalCoordinates. */
	float2 ToSphericalCoordinatesNormalized() const;

	/// Sets all elements of this vector.
	/** @see x, y, z, w, At(). */
	void Set(float x, float y, float z, float w);
	void Set(const float4 &rhs);

	/// Computes the squared length of the (x, y, z) part of this vector.
	/** Calling this function is faster than calling Length3(), since this function avoids computing a square root.
		If you only need to compare lengths to each other, but are not interested in the actual length values,
		you can compare by using LengthSq3(), instead of Length3(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@note This function ignores the w component of this vector.
		@return x*x + y*y + z*z.
		@see Length3(), LengthSq4(), Length4(), Normalize3(), Normalize4(). */
	float LengthSq3() const;

	/// Computes the length of the (x, y, z) part of this vector.
	/** @note This function ignores the w component of this vector.
		@return Sqrt(x*x + y*y + z*z).
		@see LengthSq3(), LengthSq4(), Length4(), Normalize3(), Normalize4(). */
	float Length3() const;

	/// Computes the squared length of this vector.
	/** Calling this function is faster than calling Length4(), since this function avoids computing a square root.
		If you only need to compare lengths to each other, but are not interested in the actual length values,
		you can compare by using LengthSq4(), instead of Length4(), since Sqrt() is an order-preserving
		(monotonous and non-decreasing) function.
		@return x*x + y*y + z*z + w*w.
		@see Length3(), LengthSq3(), Length4(), Normalize3(), Normalize4(). */
	float LengthSq4() const;
	inline float LengthSq() const { return LengthSq4(); }

	/// Computes the length of this vector.
	/** @return Sqrt(x*x + y*y + z*z + w*w).
		@see LengthSq3(), Length3(), LengthSq4(), Normalize3(), Normalize4(). */
	float Length4() const;
	inline float Length() const { return Length4(); }
	/// Normalizes the (x, y, z) part of this vector.
	/** @note This function ignores the w component of this vector, retaining whatever value was set there.
		@note This function fails silently. If you expect to receive an error message in case the normalization
			fails, use the Normalized3() function.
		@note This function operates in-place.
		@return The old length of this vector, or 0 if normalization failed. In the case of failure,
			this vector is set to (1, 0, 0, oldW), so that Normalize() function will never result in an unnormalized vector.
		@see Length3(), Length4(), Normalized3(), Normalize4(), Normalized4(). */
	float Normalize3();

	/// Normalizes this vector.
	/** @note This function fails silently. If you expect to receive an error message in case the normalization
			fails, use the Normalized3() function.
		@note This function operates in-place.
		@return The old length of this vector, or 0 if normalization failed. In the case of failure,
		this vector is set to (1, 0, 0, 0), so that Normalize() function will never result in an unnormalized vector.
		@see Length3(), Length4(), Normalize3(), Normalized3(), Normalized4(). */
	float Normalize4();
	inline float Normalize() { return Normalize4(); }

	/// Returns a copy of this vector with the (x, y, z) part normalized.
	/** The w component of this vector is carried over unchanged.
		@return A copy of this vector that has the (x, y, z) part normalized, and w set to the same value as in this vector.
			If the normalization fails, an error message is printed and the vector (1, 0, 0, oldW) is returned.
		@see Length3(), Length4(), Normalize3(), Normalize4(), Normalized4(). */
	float4 Normalized3() const;

	/// Returns a normalized copy of this vector.
	/** @return A copy of this vector that has the (x, y, z) part normalized, and w set to the same value as in this vector.
			If the normalization fails, an error message is printed and the vector (1, 0, 0, oldW) is returned.
		@see Length3(), Length4(), Normalize3(), Normalize4(), Normalized3(). */
	float4 Normalized4() const;
	inline float4 Normalized() const { return Normalized4(); }

	/// Divides each element by w to produce a float4 of form (x, y, z, 1).
	/** This function performs the <b>perspective divide</b> or the <b>homogeneous divide</b> on this vector, which is the
		process of dividing each element of this vector by w. If the w component of this vector is zero before division, the
		result of this vector will be undefined.
		@note This function operates in-place.
		@see IsWZeroOrOne(). */
	void NormalizeW();

	/// Returns true if the w component of this float4 is either 0 or 1.
	/** This is a required condition for several functions to work correctly.
		@see NormalizeW(), IsZero3(), IsZero4(), IsNormalized3(), IsNormalized4(). */
	bool IsWZeroOrOne(float epsilon = 1e-3f) const;

	/// Tests if the (x, y, z) part of this vector is equal to (0,0,0), up to the given epsilon.
	/** @see NormalizeW(), IsWZeroOrOne(), IsZero4(), IsNormalized3(), IsNormalized4(). */
	bool IsZero3(float epsilonSq = 1e-6f) const;

	/// Returns true if this vector is equal to (0,0,0,0), up to the given epsilon.
	/** @see NormalizeW(), IsWZeroOrOne(), IsZero3(), IsNormalized3(), IsNormalized4(). */
	bool IsZero4(float epsilonSq = 1e-6f) const;
	bool IsZero(float epsilonSq = 1e-6f) const { return IsZero4(epsilonSq); }

	/// Tests if the length of the (x, y, z) part of this vector is one, up to the given epsilon.
	/** @see NormalizeW(), IsWZeroOrOne(), IsZero3(), IsZero4(), IsNormalized4(). */
	bool IsNormalized3(float epsilonSq = 1e-5f) const;

	/// Returns true if the length of this vector is 1, up to the given epsilon.
	/** This function takes into account all the four components of this vector when calculating the norm.
		@see NormalizeW(), IsWZeroOrOne(), IsZero3(), IsZero4(), IsNormalized3(). */
	bool IsNormalized4(float epsilonSq = 1e-5f) const;
	bool IsNormalized(float epsilonSq = 1e-5f) const { return IsNormalized4(epsilonSq); }

	/// Multiplies the (x, y, z) part of this vector by the given scalar.
	/** Sets this vector to (x*scalar, y*scalar, z*scalar, w).
		@note This function operates in-place.
		@see Length3(), Mul(), ScaleToLength3(), ScaledToLength3(). */
	void Scale3(float scalar);

	/// Scales the (x, y, z) part of this vector so that its new length is as given.
	/** This is effectively the same as normalizing the vector first and then multiplying by newLength.
		@return False if the length of this vector is zero, and the vector could not be scaled to the specified length.
		@see Length3(), Mul(), Scale3(), ScaledToLength3(). */
	float ScaleToLength3(float newLength);

	float ScaleToLength(float newLength);

	/// Returns a scaled copy of this vector which has its new length as given.
	/** This function assumes the length of this vector is not zero.
		@see Length3(), Mul(), Scale3(), ScaleToLength3(). */
	float4 ScaledToLength3(float newLength) const;
	float4 ScaledToLength(float newLength) const;

	/// Tests if this vector contains valid finite elements.
	bool IsFinite() const;

	/// Tests if the (x, y, z) parts of two vectors are perpendicular to each other.
	bool IsPerpendicular3(const float4 &other, float epsilonSq = 1e-5f) const;

	bool IsPerpendicular(const float4 &other, float epsilonSq = 1e-5f) const;

	/// Makes the given vectors linearly independent.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
	The vector a is kept unmodified, and vector b is modified to be perpendicular to a.
	Finally, if specified, the vector c is adjusted to be perpendicular to a and b.
	@note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthogonal.
	@see AreOrthogonal(), Orthonormalize(), AreOrthonormal(). */
	static void Orthogonalize(const float4 &a, float4 &b);
	static void Orthogonalize(const float4 &a, float4 &b, float4 &c);

	/// Returns true if the given vectors are orthogonal to each other.
	/** @see Orthogonalize(), Orthonormalize(), AreOrthonormal(), AreCollinear(). */
	static MUST_USE_RESULT bool AreOrthogonal(const float4 &a, const float4 &b, float epsilon = 1e-3f);
	static MUST_USE_RESULT bool AreOrthogonal(const float4 &a, const float4 &b, const float4 &c, float epsilon = 1e-3f);

	/// Tests if the points p1, p2 and p3 lie on a straight line, up to the given epsilon.
	/** @see AreOrthogonal(), AreOrthonormal(), Line::AreCollinear(). */
	static MUST_USE_RESULT bool AreCollinear(const float4 &p1, const float4 &p2, const float4 &p3, float epsilon = 1e-7f);

	/// Makes the given vectors linearly independent and normalized in length.
	/** This function directly follows the Gram-Schmidt procedure on the input vectors.
	The vector a is first normalized, and vector b is modified to be perpendicular to a, and also normalized.
	Finally, if specified, the vector c is adjusted to be perpendicular to a and b, and normalized.
	@note If any of the input vectors is zero, then the resulting set of vectors cannot be made orthonormal.
	@see Orthogonalize(), AreOrthogonal(), AreOrthonormal(). */
	static void Orthonormalize(float4 &a, float4 &b);
	static void Orthonormalize(float4 &a, float4 &b, float4 &c);

	/// Returns true if the given direction vectors are orthogonal to each other and all of length 1.
	/** @note As 4D vectors, the w component is included in the computations, so call this function only for direction vectors for which w=0.
		@see Orthogonalize(), AreOrthogonal(), Orthonormalize(), AreCollinear(). */
	static MUST_USE_RESULT bool AreOrthonormal(const float4 &a, const float4 &b, float epsilon = 1e-3f);
	static MUST_USE_RESULT bool AreOrthonormal(const float4 &a, const float4 &b, const float4 &c, float epsilon = 1e-3f);

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x, y, z, w)".
	StringT ToString() const;

	/// Returns "x,y,z,w". This is the preferred format for the float4 if it has to be serialized to a string for machine transfer.
	StringT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static float4 FromString(const StringT &str) { return FromString(str.c_str()); }
#endif

	/// Parses a string that is of form "x,y,z,w" or "(x,y,z,w)" or "(x;y;z;w)" or "x y z w" to a new float4.
	static float4 FromString(const char *str, const char **outEndStr = 0);

	/// @return x + y + z + w.
	float SumOfElements() const;

	/// @return x * y * z * w.
	float ProductOfElements() const;

	/// @return (x+y+z+w)/4.
	float AverageOfElements() const;

	/// @return Min(x, y, z, w).
	/** @see MinElementIndex(). */
	float MinElement() const;

	/// Returns the index that has the smallest value in this vector.
	/** @see MinElement(). */
	int MinElementIndex() const;

	/// @return Max(x, y, z, w).
	/** @see MaxElementIndex(). */
	float MaxElement() const;

	/// Returns the index that has the smallest value in this vector.
	/** @see MaxElement(). */
	int MaxElementIndex() const;

	/// Takes the element-wise absolute value of this vector.
	/** @return float4(|x|, |y|, |z|, |w|).
		@see Neg3(), Neg4(). */
	float4 Abs() const;

	/// Returns a copy of this vector with the x, y and z elements negated.
	/** @return float4(-x, -y, -z, w).
		@see Abs(), Neg4(). */
	float4 Neg3() const;

	/// Returns a copy of this vector with each element negated.
	/** @return float4(-x, -y, -z, -w).
		@see Abs(), Neg3(). */
	float4 Neg4() const;

	/// Computes the element-wise reciprocal of the three first elements of this vector.
	/** This function returns a new vector where the x, y and z elements of the original vector are replaced by
		the values 1/x, 1/y and 1/z.
		@return float4(1/x, 1/y, 1/z, w). */
	float4 Recip3() const;

	/// Computes the element-wise reciprocal of this vector.
	/** This function returns a new vector where each element x of the original vector is replaced by the value 1/x.
		This function operates on all four elements of this vector.
		@return float4(1/x, 1/y, 1/z, 1/w). */
	float4 Recip4() const;

	/// Computes the element-wise reciprocal of this vector using a fast approximation (SSE rcp instruction).
	/** This function returns a new vector where each element x of the original vector is replaced by the value 1/x.
		This function operates on all four elements of this vector.
		@return float4(1/x, 1/y, 1/z, 1/w). */
	float4 RecipFast4() const;

	/// Returns an element-wise minimum of this and the vector (ceil, ceil, ceil, ceil).
	/** Each element that is larger than ceil is replaced by ceil. */
	float4 Min(float ceil) const;

	/// Returns an element-wise minimum of this and the given vector.
	/** Each element that is larger than ceil is replaced by ceil.
		@see Max(), Clamp(). */
	float4 Min(const float4 &ceil) const;

	/// Returns an element-wise maximum of this and the vector (floor, floor, floor, floor).
	/** Each element that is smaller than floor is replaced by floor. */
	float4 Max(float floor) const;

	/// Returns an element-wise maximum of this and the given vector.
	/** Each element that is smaller than floor is replaced by floor.
		@see Min(), Clamp(). */
	float4 Max(const float4 &floor) const;

	/// Returns a vector that has floor <= this[i] <= ceil for each element.
	float4 Clamp(float floor, float ceil) const;

	/// Limits each element of this vector between the corresponding elements in floor and ceil.
	/** @see Min(), Max(), Clamp01(). */
	float4 Clamp(const float4 &floor, const float4 &ceil) const;

	/// Limits each element of this vector in the range [0, 1].
	/** @see Min(), Max(), Clamp(). */
	float4 Clamp01() const;

	/// Linearly interpolates between this and the vector b.
	/** This function assumes that the w components of this and the other vector are equal.
		@param b The target endpoint to lerp towards to.
		@param t The interpolation weight, in the range [0, 1].
		Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
		Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on. */
	float4 Lerp(const float4 &b, float t) const;
	static float4 Lerp(const float4 &a, const float4 &b, float t);

	/// Computes the squared distance between the (x, y, z) parts of this and the given float4.
	/** @note This function ignores the w component of this and rhs vector (assumes w=0 or w=1 are the same for both vectors).
		@see Distance3(), Length3Sq(), Length3().
		@return (x-rhs.x)^2 + (y-rhs.y)^2 + (z-rhs.z)^2. */
	float Distance3Sq(const float4 &rhs) const;

	/// Computes the distance between the (x, y, z) parts of this and the given float4.
	/** @note This function ignores the w component of this and rhs vector (assumes w=0 or w=1 are the same for both vectors).
		@see Distance3Sq(), Length3Sq(), Length3().
		@return Sqrt((x-rhs.x)^2 + (y-rhs.y)^2 + (z-rhs.z)^2). */
	float Distance3(const float4 &rhs) const;

	/// Computes the squared distance between this and the given float4.
	/** @note This function computes the square of the Euclidean distance of the two vectors in 4D space (taking into account the w component).
		@see Distance4Sq(), Distance3(), Distance3Sq(), Length3Sq(), Length3().
		@return (x-rhs.x)^2 + (y-rhs.y)^2 + (z-rhs.z)^2 + (w-rhs.w)^2. */
	float Distance4Sq(const float4 &rhs) const;
	inline float DistanceSq(const float4 &rhs) const { return Distance4Sq(rhs); }

	/// Computes the distance between this and the given float4.
	/** @note This function computes the Euclidean distance of the two vectors in 4D space (taking into account the w component).
		@see Distance4Sq(), Distance3(), Distance3Sq(), Length3Sq(), Length3().
		@return Sqrt((x-rhs.x)^2 + (y-rhs.y)^2 + (z-rhs.z)^2 + (w-rhs.w)^2). */
	float Distance4(const float4 &rhs) const;
	inline float Distance(const float4 &rhs) const { return Distance4(rhs); }

	/// Computes the dot product of the (x, y, z) parts of this and the given float4.
	/** @note This function ignores the w component of this vector (assumes w=0).
		@see Dot4(), Cross3(). */
	float Dot3(const float3 &rhs) const;
	float Dot3(const float4 &rhs) const;

	/// Computes the dot product of this and the given float4, taking into account the w component.
	/** @see Dot3(), Cross3(). */
	float Dot4(const float4 &rhs) const;

	inline float Dot(const float4 &rhs) const { return Dot4(rhs); }

	/// Computes the cross product of the (x, y, z) parts of this and the given vector. Returns a vector with w=0.
	/** @see Dot3(), Dot4(). */
	float4 Cross3(const float3 &rhs) const;
	float4 Cross3(const float4 &rhs) const;

	float4 Cross(const float4 &rhs) const { return Cross3(rhs); }

	/// Computes the outer product of this and the given vector.
	float4x4 OuterProduct(const float4 &rhs) const;

	/// Returns a new normalized direction vector that points as close as possible towards the given hint vector.
	float4 Perpendicular3(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;
	float4 Perpendicular(const float4 &hint = float4(0,1,0,0), const float4 &hint2 = float4(0,0,1,0)) const;

	/// Returns another vector that is perpendicular to this vector and the vector returned by Perpendicular3(hint).
	/** @todo Enforce that (x: this, y: Perpendicular3(), z: AnotherPerpendicular3) form a right-handed basis.
		@see Perpendicular3(). */
	float4 AnotherPerpendicular3(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;
	float4 AnotherPerpendicular(const float4 &hint = float4(0,1,0,0), const float4 &hint2 = float4(0,0,1,0)) const;

	// Completes this vector to generate a perpendicular basis.
	/** This function computes two new vectors b and c which are both orthogonal to this vector and to each other.
		That is, the set { this, b, c} is an orthogonal set. The vectors b and c that are outputted are also normalized.
		@param outB [out] Receives vector b.
		@param outC [out] Receives vector c.
		@note When calling this function, this vector should not be zero! */
	void PerpendicularBasis(float4 &outB, float4 &outC) const;

	/// Generates a random vector that is perpendicular to this vector.
	/** The distribution is uniformly random. */
	float4 RandomPerpendicular(LCG &rng) const;

	/// Returns this vector reflected about a plane with the given normal.
	/** By convention, both this and the reflected vector point away from the plane with the given normal.
		@note This function ignores the w component of this vector (assumes w=0). */
	float4 Reflect3(const float3 &normal) const;
	float4 Reflect(const float4 &normal) const;

	float4 Refract(const float4 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

	/// Returns the angle between this vector and the specified vector, in radians.
	/** @note This function takes into account that this vector or the other vector can be unnormalized, and
			normalizes the computations.
		@note This function ignores the w component of this vector (assumes w=0).
		@see Dot3(), AngleBetweenNorm3(), AngleBetween4(), AngleBetweenNorm4(). */
	float AngleBetween3(const float4 &other) const;

	/// Returns the angle between this vector and the specified normalized vector, in radians.
	/** @note This vector must be normalized to call this function.
		@note This function ignores the w component of this vector (assumes w=0).
		@see Dot3(), AngleBetween3(), AngleBetween4(), AngleBetweenNorm4(). */
	float AngleBetweenNorm3(const float4 &normalizedVector) const;

	/// Returns the angle between this vector and the specified vector, in radians.
	/** @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
		@see Dot3(), AngleBetween3(), AngleBetweenNorm3(), AngleBetweenNorm4(). */
	float AngleBetween4(const float4 &other) const;

	/// Returns the angle between this vector and the specified normalized vector, in radians.
	/** @note This vector must be normalized to call this function.
		@see Dot3(), AngleBetween3(), AngleBetweenNorm3(), AngleBetween4(). */
	float AngleBetweenNorm4(const float4 &normalizedVector) const;

	/// Projects this vector onto the given vector.
	/** @note This function treats this and target vector as direction vectors.
		@note This function ignores the w component of this vector (assumes w=0 or 1) and returns it unmodified.
		@see ProjectToNorm3(). */
	float4 ProjectTo3(const float3 &target) const;

	float4 ProjectTo(const float4 &target) const;

	/// Projects this vector onto the given vector.
	/** @param target The direction vector to project onto. This vector must be normalized.
		@note This function treats this and target vector as direction vectors.
		@note This function ignores the w component of this vector (assumes w=0 or 1) and returns it unmodified.
		@see ProjectTo3(). */
	float4 ProjectToNorm3(const float3 &target) const;

	float4 ProjectToNorm(const float4 &target) const;

	/// Returns true if this vector is equal to the given vector, up to given per-element epsilon.
	bool Equals(const float4 &other, float epsilon = 1e-3f) const;
	bool Equals(float x, float y, float z, float w, float epsilon = 1e-3f) const;

	/// Compares whether this float4 and the given float4 are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const float4 &other) const;

	/// Generates a direction vector of the given length pointing at a uniformly random direction.
	/* The w-component for the returned vector is 0.
	@see RandomSphere(), RandomBox(). */
	static MUST_USE_RESULT float4 RandomDir(LCG &lcg, float length = 1.f);
	/// Generates a random point inside a sphere.
	/** The returned point is generated uniformly inside the sphere.
	@see RandomDir(), RandomBox(). */
	static MUST_USE_RESULT float4 RandomSphere(LCG &lcg, const float4 &center, float radius);
	/// Generates a random point inside an axis-aligned box.
	/** The returned point is generated uniformly inside the box.
	@see RandomDir(), RandomSphere(). */
	static MUST_USE_RESULT float4 RandomBox(LCG &lcg, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
	static MUST_USE_RESULT float4 RandomBox(LCG &lcg, const float4 &minValues, const float4 &maxValues);

	/// Returns a random float3 with each entry randomized between the range [minElem, maxElem].
	static MUST_USE_RESULT float4 RandomBox(LCG &lcg, float minElem, float maxElem);

	/// Returns a random float4 with each entry randomized between the range [minElem, maxElem].
	/** Warning: The vectors returned by this function generally have w != 0 or 1, so they don't do not represent
		well-formed 3D points or direction vectors.
		This function is mostly used for testing and debugging purposes only. */
	static float4 RandomGeneral(LCG &lcg, float minElem, float maxElem);

	/// Specifies a compile-time constant float4 with value (0, 0, 0, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 zero;

	/// Specifies a compile-time constant float4 with value (1, 1, 1, 1). [similarOverload: zero]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 one;

	/// Specifies a compile-time constant float4 with value (1, 0, 0, 0).
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 unitX;

	/// Specifies a compile-time constant float4 with value (0, 1, 0, 0). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 unitY;

	/// Specifies a compile-time constant float4 with value (0, 0, 1, 0). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 unitZ;

	/// Specifies a compile-time constant float4 with value (0, 0, 0, 1). [similarOverload: unitX]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 unitW;

	/// A compile-time constant float4 with value (NaN, NaN, NaN, NaN).
	/** For this constant, each element has the value of quiet NaN, or Not-A-Number.
		@note Never compare a float4 to this value! Due to how IEEE floats work, "nan == nan" returns false!
			  That is, nothing is equal to NaN, not even NaN itself!
		@note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 nan;

	/// A compile-time constant float4 with value (+infinity, +infinity, +infinity, +infinity). [similarOverload: nan]
	/** @note Due to static data initialization order being undefined in C++, do NOT use this
			member to initialize other static data in other compilation units! */
	static const float4 inf;

#ifdef MATH_SIMD
	float4(simd4f vec):v(vec) {}

	///\todo All the _SSE() functions will be deleted in favor of C SSE API.
	simd4f Swizzled_SSE(int i, int j, int k, int l) const;
	simd4f LengthSq3_SSE() const;
	simd4f Length3_SSE() const;
	simd4f LengthSq4_SSE() const;
	simd4f Length4_SSE() const;
	simd4f Normalize4_SSE();
	void Normalize3_Fast_SSE();
	void Normalize4_Fast_SSE();
	void NormalizeW_SSE();
	simd4f SumOfElements_SSE() const;

	inline float4 &operator =(simd4f vec) { v = vec; return *this; }

	inline operator simd4f() const { return v; }
#endif
};

struct float4_storage
{
	float x,y,z,w;
	float4_storage(){}
	float4_storage(const float4 &rhs)
	{
		// Copy with scalar. TODO: Revisit if this is avoidable.
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		// Would like to do the following to get SSE benefit, but
		// Visual Studio generates unaligned temporaries to this struct
		// in debug builds, so can't do that.
		//*reinterpret_cast<float4*>(this) = rhs;
	}
	operator float4() const { return *reinterpret_cast<const float4*>(this); }
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float4 to the given stream.
std::ostream &operator <<(std::ostream &out, const float4 &rhs);
#endif

/// Multiplies the x, y, z and w components of the vector by the given scalar. Note that if w != 0,
/// this does NOT scale the length of the homogeneous 3D vector.
float4 operator *(float scalar, const float4 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float4 operator /(float scalar, const float4 &rhs) { return float4::FromScalar(scalar) / rhs; }
#endif

inline float Dot3(const float4 &a, const float4 &b) { return a.Dot3(b); }
inline float Dot4(const float4 &a, const float4 &b) { return a.Dot4(b); }
inline float Dot(const float4 &a, const float4 &b) { return a.Dot(b); }
inline float4 Cross3(const float4 &a, const float4 &b) { return a.Cross3(b); }
inline float4 Cross(const float4 &a, const float4 &b) { return a.Cross(b); }
inline float4 Abs(const float4 &a) { return a.Abs(); }
inline float Length3(const float4 &a) { return a.Length3(); }
inline float Length4(const float4 &a) { return a.Length4(); }
inline float Distance3(const float4 &a, const float4 &b) { return a.Distance3(b); }
inline float Distance4(const float4 &a, const float4 &b) { return a.Distance4(b); }
inline float4 Min(const float4 &a, const float4 &b) { return a.Min(b); }
inline float4 Max(const float4 &a, const float4 &b) { return a.Max(b); }
inline float4 Clamp(const float4 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float4 Clamp(const float4 &a, const float4 &floor, const float4 &ceil) { return a.Clamp(floor, ceil); }
inline float4 Clamp01(const float4 &a) { return a.Clamp01(); }
inline float4 Lerp(const float4 &a, const float4 &b, float t) { return a.Lerp(b, t); }

float4 Perp2D(const float4 &v);
float4 Mul2D(const float3x3 &transform, const float4 &v);
float4 MulPos2D(const float3x4 &transform, const float4 &v);
float4 MulPos2D(const float4x4 &transform, const float4 &v);
float4 MulDir2D(const float3x4 &transform, const float4 &v);
float4 MulDir2D(const float4x4 &transform, const float4 &v);

MATH_END_NAMESPACE
