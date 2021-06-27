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

/** @file Quat.h
	@author Jukka Jylänki
	@brief Quaternions represent rotations and orientations of 3D objects. */
#pragma once

#include "../MathBuildConfig.h"
#include "SSEMath.h"
#include "float4.h"

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#endif
#include "../MathGeoLibFwd.h"

MATH_BEGIN_NAMESPACE

/// Represents a rotation or an orientation of a 3D object.
class ALIGN16 Quat
{
public:

#ifdef MATH_SIMD
	NAMELESS_UNION_BEGIN // Allow nonstandard nameless struct in union extension on MSC.

	union
	{
		struct
		{
#endif
			
			float x; ///< The factor of i.
			float y; ///< The factor of j. [similarOverload: x]
			float z; ///< The factor of k. [similarOverload: x]
			float w; ///< The scalar part. Sometimes also referred to as 'r'. [similarOverload: x]
#ifdef MATH_SIMD
		};
		simd4f q;
	};

	NAMELESS_UNION_END

	Quat(simd4f quat):q(quat) {}
	inline Quat &operator =(simd4f quat) { q = quat; return *this; }
	inline operator simd4f() const { return q; }
#endif

	/// @note The default ctor does not initialize any member values.
	Quat() {}

	Quat(const Quat &rhs) = default; //{ Set(rhs); }

	/// Constructs a quaternion from the given data buffer.
	/// @param data An array of four floats to use for the quaternion, in the order 'x, y, z, w'. (== 'i, j, k, r')
	/// @note The input data is not normalized after construction, this has to be done manually.
	explicit Quat(const float *data);

	explicit Quat(const float3x3 &rotationMatrix) { Set(rotationMatrix); }
	explicit Quat(const float3x4 &rotationMatrix) { Set(rotationMatrix); }
	explicit Quat(const float4x4 &rotationMatrix) { Set(rotationMatrix); }

	/// @param x The factor of i.
	/// @param y The factor of j.
	/// @param z The factor of k.
	/// @param w The scalar factor (or 'w').
	/// @note The input data is not normalized after construction, this has to be done manually.
	Quat(float x, float y, float z, float w);

	/// Constructs this quaternion by specifying a rotation axis and the amount of rotation to be performed
	/// about that axis.
	/** @param rotationAxis The normalized rotation axis to rotate about. If using the float4 version of the constructor, the w component of this vector must be 0.
		@param rotationAngleRadians The angle to rotate by, in radians. For example, Pi/4.f equals to 45 degrees, Pi/2.f is 90 degrees, and Pi is 180 degrees.
		@see DegToRad(). */
	Quat(const float3 &rotationAxis, float rotationAngleRadians) { SetFromAxisAngle(rotationAxis, rotationAngleRadians); }
	Quat(const float4 &rotationAxis, float rotationAngleRadians) { SetFromAxisAngle(rotationAxis, rotationAngleRadians); }

	/// Returns the local +X axis in the post-transformed coordinate space. This is the same as transforming the vector (1,0,0) by this quaternion.
	vec WorldX() const;
	/// Returns the local +Y axis in the post-transformed coordinate space. This is the same as transforming the vector (0,1,0) by this quaternion.
	vec WorldY() const;
	/// Returns the local +Z axis in the post-transformed coordinate space. This is the same as transforming the vector (0,0,1) by this quaternion.
	vec WorldZ() const;

	/// Returns the axis of rotation for this quaternion.
	vec Axis() const;

	/// Returns the angle of rotation for this quaternion, in radians.
	float Angle() const;

	/// Computes the dot product of this and the given quaternion.
	/// Dot product is commutative.
	float Dot(const Quat &rhs) const;

	float LengthSq() const;

	float Length() const;

	/// Normalizes this quaternion in-place.
	/// Returns the old length of this quaternion, or 0 if normalization failed.
	float Normalize();

	/// Returns a normalized copy of this quaternion.
	Quat Normalized() const;

	/// Returns true if the length of this quaternion is one.
	bool IsNormalized(float epsilon = 1e-5f) const;

	bool IsInvertible(float epsilon = 1e-3f) const;

	/// Returns true if the entries of this quaternion are all finite.
	bool IsFinite() const;

	/// Returns true if this quaternion equals rhs, up to the given epsilon.
	bool Equals(const Quat &rhs, float epsilon = 1e-3f) const;

	/// Compares whether this Quat and the given Quat are identical bit-by-bit in the underlying representation.
	/** @note Prefer using this over e.g. memcmp, since there can be SSE-related padding in the structures. */
	bool BitEquals(const Quat &other) const;

	/// @return A pointer to the first element (x). The data is contiguous in memory.
	/// ptr[0] gives x, ptr[1] is y, ptr[2] is z and ptr[3] is w.
	FORCE_INLINE float *ptr() { return &x; }
	FORCE_INLINE const float *ptr() const { return &x; }

	/// Inverses this quaternion in-place.
	/// @note For optimization purposes, this function assumes that the quaternion is unitary, in which
	///	   case the inverse of the quaternion is simply just the same as its conjugate. This function
	///	   does not detect whether the operation succeeded or failed.
	void Inverse();

	/// Returns an inverted copy of this quaternion.
	MUST_USE_RESULT Quat Inverted() const;

	/// Inverses this quaternion in-place.
	/// Call this function when the quaternion is not known beforehand to be normalized. This function
	/// computes the inverse proper, and normalizes the result.
	/// @note Because of the normalization, it does not necessarily hold that q * q.InverseAndNormalize() == id.
	/// @return Returns the old length of this quaternion (not the old length of the inverse quaternion).
	float InverseAndNormalize();

	/// Computes the conjugate of this quaternion in-place.
	void Conjugate();

	/// Returns a conjugated copy of this quaternion.
	MUST_USE_RESULT Quat Conjugated() const;

	/// Rotates the given vector by this quaternion.
	MUST_USE_RESULT float3 Transform(float x, float y, float z) const;
	MUST_USE_RESULT float3 Transform(const float3 &vec) const;

	/// Rotates the given vector by this quaternion. The w component of the vector is assumed to be zero or one.
	MUST_USE_RESULT float4 Transform(const float4 &vec) const;

	MUST_USE_RESULT Quat Lerp(const Quat &target, float t) const;
	static FORCE_INLINE MUST_USE_RESULT Quat Lerp(const Quat &source, const Quat &target, float t) { return source.Lerp(target, t); }
	MUST_USE_RESULT Quat Slerp(const Quat &target, float t) const;
	static FORCE_INLINE MUST_USE_RESULT Quat Slerp(const Quat &source, const Quat &target, float t) { return source.Slerp(target, t); }

	/// Returns the 'from' vector rotated towards the 'to' vector by the given normalized time parameter.
	/** This function slerps the given 'from' vector towards the 'to' vector.
		@param from A normalized direction vector specifying the direction of rotation at t=0.
		@param to A normalized direction vector specifying the direction of rotation at t=1.
		@param t The interpolation time parameter, in the range [0,1]. Input values outside this range are
			silently clamped to the [0, 1] interval.
		@return A spherical linear interpolation of the vector 'from' towards the vector 'to'. */
	static MUST_USE_RESULT float3 SlerpVector(const float3 &from, const float3 &to, float t);

	/// Returns the 'from' vector rotated towards the 'to' vector by the given absolute angle, in radians.
	/** This function slerps the given 'from' vector towards the 'to' vector.
		@param from A normalized direction vector specifying the direction of rotation at angleRadians=0.
		@param to A normalized direction vector specifying the target direction to rotate towards.
		@param angleRadians The maximum angle to rotate the 'from' vector by, in the range [0, pi]. If the
			angle between 'from' and 'to' is smaller than this angle, then the vector 'to' is returned.
			Input values outside this range are silently clamped to the [0, pi] interval.
		@return A spherical linear interpolation of the vector 'from' towards the vector 'to'. */
	static MUST_USE_RESULT float3 SlerpVectorAbs(const float3 &from, const float3 &to, float angleRadians);

	/// Returns the angle between this and the target orientation (the shortest route) in radians.
	MUST_USE_RESULT float AngleBetween(const Quat &target) const;
	/// Returns the axis of rotation to get from this orientation to target orientation (the shortest route).
	MUST_USE_RESULT vec AxisFromTo(const Quat &target) const;

	/// Returns the rotation axis and angle of this quaternion.
	/// @param rotationAxis [out] Received the normalized axis of the rotation.
	/// @param rotationAngleRadians [out] Receives the angle of rotation around the given axis. This parameter is returned in the range [0, 2pi].
	void ToAxisAngle(float3 &rotationAxis, float &rotationAngleRadians) const;
	void ToAxisAngle(float4 &rotationAxis, float &rotationAngleRadians) const;
	/// Sets this quaternion by specifying the axis about which the rotation is performed, and the angle of rotation.
	/** @param rotationAxis The axis of rotation. This vector must be normalized to call this function. If using the float4 version of this function, 
		then the w component must be zero.
		@param rotationAngleRadians The angle of rotation in radians. */
	void SetFromAxisAngle(const float3 &rotationAxis, float rotationAngleRadians);
	void SetFromAxisAngle(const float4 &rotationAxis, float rotationAngleRadians);

	/// Sets this quaternion to represent the same rotation as the given matrix.
	void Set(const float3x3 &matrix);
	void Set(const float3x4 &matrix);
	void Set(const float4x4 &matrix);
	/// Sets all elements of this quaternion.
	/// @note This sets the raw elements, which do *not* correspond directly to the axis and angle of the rotation. Use
	///	   SetFromAxisAngle to define this Quat using a rotation axis and an angle.
	void Set(float x, float y, float z, float w);
	void Set(const Quat &q);
	void Set(const float4 &v);

	/// Creates a LookAt quaternion.
	/** A LookAt quaternion is a quaternion that orients an object to face towards a specified target direction.
		@param localForward Specifies the forward direction in the local space of the object. This is the direction
			the model is facing at in its own local/object space, often +X (1,0,0), +Y (0,1,0) or +Z (0,0,1). The
			vector to pass in here depends on the conventions you or your modeling software is using, and it is best
			pick one convention for all your objects, and be consistent.			
			This input parameter must be a normalized vector.
		@param targetDirection Specifies the desired world space direction the object should look at. This function
			will compute a quaternion which will rotate the localForward vector to orient towards this targetDirection
			vector. This input parameter must be a normalized vector.
		@param localUp Specifies the up direction in the local space of the object. This is the up direction the model
			was authored in, often +Y (0,1,0) or +Z (0,0,1). The vector to pass in here depends on the conventions you
			or your modeling software is using, and it is best to pick one convention for all your objects, and be
			consistent. This input parameter must be a normalized vector. This vector must be perpendicular to the
			vector localForward, i.e. localForward.Dot(localUp) == 0.
		@param worldUp Specifies the global up direction of the scene in world space. Simply rotating one vector to
			coincide with another (localForward->targetDirection) would cause the up direction of the resulting
			orientation to drift (e.g. the model could be looking at its target its head slanted sideways). To keep
			the up direction straight, this function orients the localUp direction of the model to point towards the
			specified worldUp direction (as closely as possible). The worldUp and targetDirection vectors cannot be
			collinear, but they do not need to be perpendicular either.
		@return A quaternion that maps the given local space forward direction vector to point towards the given target
			direction, and the given local up direction towards the given target world up direction. For the returned
			quaternion Q it holds that M * localForward = targetDirection, and M * localUp lies in the plane spanned
			by the vectors targetDirection and worldUp.
		@see RotateFromTo(). */
	static MUST_USE_RESULT Quat LookAt(const float3 &localForward, const float3 &targetDirection, const float3 &localUp, const float3 &worldUp);

	/// Creates a new quaternion that rotates about the positive X axis by the given angle.
	static MUST_USE_RESULT Quat RotateX(float angleRadians);
	/// Creates a new quaternion that rotates about the positive Y axis by the given angle.
	static MUST_USE_RESULT Quat RotateY(float angleRadians);
	/// Creates a new quaternion that rotates about the positive Z axis by the given angle.
	static MUST_USE_RESULT Quat RotateZ(float angleRadians);

	/// Creates a new Quat that rotates about the given axis by the given angle.
	static MUST_USE_RESULT Quat RotateAxisAngle(const float3 &axisDirection, float angleRadians);

	/// Creates a new quaternion that rotates sourceDirection vector (in world space) to coincide with the
	/// targetDirection vector (in world space).
	/// Rotation is performed around the origin.
	/// The vectors sourceDirection and targetDirection are assumed to be normalized.
	/// @note There are multiple such rotations - this function returns the rotation that has the shortest angle
	/// (when decomposed to axis-angle notation).
	static MUST_USE_RESULT Quat RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection);
	static MUST_USE_RESULT Quat RotateFromTo(const float4 &sourceDirection, const float4 &targetDirection);

	/// Creates a new quaternion that
	/// 1. rotates sourceDirection vector to coincide with the targetDirection vector, and then
	/// 2. rotates sourceDirection2 (which was transformed by 1.) to targetDirection2, but keeping the constraint that
	///	sourceDirection must look at targetDirection.
	static MUST_USE_RESULT Quat RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection,
		const float3 &sourceDirection2, const float3 &targetDirection2);

	/// Creates a new Quat from the given sequence of Euler rotation angles (in radians).
	/** The FromEulerABC function returns a matrix M = A(a) * B(b) * C(c). Rotation
		C is applied first, followed by B and then A. [indexTitle: FromEuler***] */
	static MUST_USE_RESULT Quat FromEulerXYX(float x2, float y, float x);
	static MUST_USE_RESULT Quat FromEulerXZX(float x2, float z, float x); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerYXY(float y2, float x, float y); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerYZY(float y2, float z, float y); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerZXZ(float z2, float x, float z); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerZYZ(float z2, float y, float z); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerXYZ(float x, float y, float z); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerXZY(float x, float z, float y); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerYXZ(float y, float x, float z); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerYZX(float y, float z, float x); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerZXY(float z, float x, float y); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static MUST_USE_RESULT Quat FromEulerZYX(float z, float y, float x); ///< [similarOverload: FromEulerXYX] [hideIndex]

	/// Returns a uniformly random unitary quaternion.
	static MUST_USE_RESULT Quat RandomRotation(LCG &lcg);

	/// Extracts the rotation part of this quaternion into Euler rotation angles (in radians).
	/** @note It is better to think about the returned float3 as an array of three floats, and
			not as a triple of xyz, because e.g. the .y component returned by ToEulerYXZ() does
			not return the amount of rotation about the y axis, but contains the amount of rotation
			in the second axis, in this case the x axis.
		@return A float3 which specifies the rotation of this quaternion in radian Euler angles.
			The function ToEulerABC returns a float3 where the first
			element ([0], or x) specifies the rotation about the axis A (not necessarily the X axis!),
			[1] or y specifies the rotation about the B axis (not necessarily the Y axis!) and
			[2] or z specifies the rotation about the C axis (not necessarily the Z axis!). The
			order of rotations follows the M*v convention, meaning that ToEulerXYZ returns the Euler
			angles for rotating a vector v in the order X * (Y * (Z * v))), i.e. right-to-left. */
	float3 MUST_USE_RESULT ToEulerXYX() const;
	float3 MUST_USE_RESULT ToEulerXZX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerYXY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerYZY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerZXZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerZYZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerXYZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerXZY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerYXZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerYZX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerZXY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 MUST_USE_RESULT ToEulerZYX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]

	float3x3 MUST_USE_RESULT ToFloat3x3() const;
	float3x4 MUST_USE_RESULT ToFloat3x4() const;
	float4x4 MUST_USE_RESULT ToFloat4x4() const;
	float4x4 MUST_USE_RESULT ToFloat4x4(const float3 &translation) const;
	float4x4 MUST_USE_RESULT ToFloat4x4(const float4 &translation) const;

	/// Returns the elements of this quaternion casted to a float4.
	/** @note The returned vector does not have a direct geometric meaning, e.g. it does not
		represent a direction vector or a magnitude or similar. */
	float4 CastToFloat4() const { return float4(x, y, z, w); }

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x,y,z,w)".
	StringT MUST_USE_RESULT ToString() const;

	/// Returns "Quat(axis:(x,y,z) angle:degrees)".
	StringT MUST_USE_RESULT ToString2() const;

	/// Returns "x,y,z,w". This is the preferred format for the quaternion if it has to be serialized to a string for machine transfer.
	StringT MUST_USE_RESULT SerializeToString() const;

	/// Returns a string of C++ code that can be used to construct this object. Useful for generating test cases from badly behaving objects.
	StringT SerializeToCodeString() const;
	static MUST_USE_RESULT Quat FromString(const StringT &str) { return FromString(str.c_str()); }
#endif
	/// Parses a string that is of form "x,y,z,w" or "(x,y,z,w)" or "(x;y;z;w)" or "x y z w" to a new quaternion.
	static MUST_USE_RESULT Quat FromString(const char *str, const char **outEndStr = 0);

	/// Multiplies two quaternions together.
	/// The product q1 * q2 returns a quaternion that concatenates the two orientation rotations. The rotation
	/// q2 is applied first before q1.
	Quat operator *(const Quat &rhs) const;

	/// Transforms the given vector by this Quaternion.
	/// @note Technically, this function does not perform a simple multiplication of 'q * v',
	/// but instead performs a conjugation operation 'q*v*q^-1'. This corresponds to transforming
	/// the given vector by this Quaternion.
	float3 operator *(const float3 &rhs) const;
	float4 operator *(const float4 &rhs) const;

	/// The identity quaternion performs no rotation when applied to a vector.
	/// For quaternions, the identity has the value r = 1, i,j,k = 0.
	static const Quat identity;
	/// A compile-time constant Quat with value (NaN, NaN, NaN, NaN).
	/// For this constant, each element has the value of quiet NaN, or Not-A-Number.
	/// @note Never compare a Quat to this value! Due to how IEEE floats work, "nan == nan" returns false!
	///	   That is, nothing is equal to NaN, not even NaN itself!
	static const Quat nan;

	/// Divides a quaternion by another. Division "a / b" results in a quaternion that rotates the orientation b to coincide with the orientation a.
	Quat operator /(const Quat &rhs) const;

	/// Unary operator + allows this structure to be used in an expression '+x'.
	Quat operator +() const { return *this; }

	/// Multiplies two quaternions in the order 'this * rhs'.
	/// This corresponds to the concatenation of the two operations ('this * rhs * vector' applies the rotation 'rhs' first, followed by the rotation 'this'.
	Quat MUST_USE_RESULT Mul(const Quat &rhs) const;
	/// Converts the given matrix to a quaternion and computes the concatenated transform 'this * rhs'.
	Quat MUST_USE_RESULT Mul(const float3x3 &rhs) const;
	/// Transforms the given vector by this Quaternion.
	/// @note Technically, this function does not perform a simple multiplication of 'q * v',
	/// but instead performs a conjugation operation 'q*v*q^-1'. This corresponds to transforming
	/// the given vector by this Quaternion.
	float3 MUST_USE_RESULT Mul(const float3 &vector) const;
	float4 MUST_USE_RESULT Mul(const float4 &vector) const;

	/// Negates the quaternion.
	/// @note Negating a quaternion will not produce the inverse rotation. Call Quat::Inverse() to generate the inverse rotation.
	Quat Neg() const { return -*this; }

	Quat &operator=(const Quat &rhs) = default;

private: // Hide the unsafe operations from the user, so that he doesn't accidentally invoke an unintended operation.

	/// Multiplies a quaternion by a scalar.
	/// @note Technically, multiplication by scalar would not affect the rotation this quaternion represents, but since
	/// Quat uses conjugation to compute the inverse (to optimize), an unnormalized quaternion will not produce a proper rotation transform.
	/// @note Multiplication by a scalar does not "accumulate" rotations, e.g. "quat * 5.f" will not produce a quaternion that would rotate
	///			"5 times more".
	Quat operator *(float scalar) const;

	Quat operator /(float scalar) const;

	/// Adds two quaternions.
	/// @note Adding two quaternions does not concatenate the two rotation operations. Use quaternion multiplication to achieve that.
	Quat operator +(const Quat &rhs) const;

	Quat operator -(const Quat &rhs) const;

	/// Negates the quaternion.
	/// @note Negating a quaternion will not produce the inverse rotation. Call Quat::Inverse() to generate the inverse rotation.
	Quat operator -() const;
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this Quat to the given stream.
std::ostream &operator <<(std::ostream &out, const Quat &rhs);
#endif

FORCE_INLINE Quat Lerp(const Quat &a, const Quat &b, float t) { return a.Lerp(b, t); }
FORCE_INLINE Quat Slerp(const Quat &a, const Quat &b, float t) { return a.Slerp(b, t); }

MATH_END_NAMESPACE
