/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float3x4.h
	@author Jukka Jyl�nki
	@brief A 3-by-4 matrix for affine operations in 3D space. */
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include "myassert.h"
#endif

#include "../MathGeoLibFwd.h"
#include "../MathBuildConfig.h"
#include "MatrixProxy.h"
#include "float3.h"
#include "SSEMath.h"

MATH_BEGIN_NAMESPACE

/// A 3-by-4 matrix for affine transformations of 3D geometry.
/** This matrix type can represent affine operations in addition to linear ones. Affine operations translate
	the geometry with respect to the origin point, whereas linear transformations retain the origin of the coordinate
	system in place.

	The elements of this matrix are

		m_00, m_01, m_02, m_03
		m_10, m_11, m_12, m_13
		m_20, m_21, m_22, m_23

	The element m_yx is the value on the row y and column x.
	You can access m_yx using the double-bracket notation m[y][x], or using the member function m.At(y, x);

	@note The member functions in this class use the convention that transforms are applied to vectors in the form
	M * v. This means that "float3x4 M, M1, M2; M = M1 * M2;" gives a transformation M that applies M2 first, followed
	by M1 second, i.e. M * v = M1 * M2 * v = M1 * (M2 * v). This is the convention commonly used with OpenGL. The
	opposing convention (v * M) is commonly used with Direct3D.

	@note This class uses row-major storage, which means that the elements are packed in memory in order
	 m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], ...
	The elements for a single row of the matrix hold successive memory addresses. This is the same memory layout as
	 with C++ multidimensional arrays.

 	If preprocessor #define MATH_COLMAJOR_MATRICES is set, column-major storage is used instead, in which the elements
 	are packed in the memory in order m[0][0], m[1][0], m[2][0], m[3][0], m[0][1], m[1][1], ...
	There the elements for a single column of the matrix hold successive memory addresses.
	This is exactly opposite from the standard C++ multidimensional arrays, since if you have e.g.
	int v[10][10], then v[0][9] comes in memory right before v[1][0]. ( [0][0], [0][1], [0][2], ... [1][0], [1][1], ...) */
class ALIGN16 float3x4
{
public:
	/// Specifies the height of this matrix.
	enum { Rows = 3 };

	/// Specifies the width of this matrix.
	enum { Cols = 4 };

	/// Stores the data in this matrix in row-major format.
	/** [noscript] */
	union
	{
#ifdef MATH_COLMAJOR_MATRICES
		float v[Cols][Rows];
#else
		float v[Rows][Cols];
#if defined(MATH_SIMD)
		simd4f row[3];
#endif
#endif
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4201) // warning C4201: nonstandard extension used: nameless struct/union
#endif
		// Alias into the array of elements to allow accessing items from this matrix directly for convenience.
		// This gives human-readable names to the individual matrix elements:
		struct
		{
#ifdef MATH_COLMAJOR_MATRICES
			float scaleX, shearYx, shearZx;
			float shearXy, scaleY, shearZy;
			float shearXz, shearYz, scaleZ;
			float x, y, z;
#else
			float  scaleX, shearXy, shearXz, x;
			float shearYx,  scaleY, shearYz, y;
			float shearZx, shearZy,  scaleZ, z;
#endif
		};
#ifdef _MSC_VER
#pragma warning(pop)
#endif
		// where scaleX/Y/Z specify how much principal axes are scaled by, x/y/z specify translation (position) on the axes,
		// and shearAb specify how much shearing occurs towards axis A from axis b (when vector is multiplied via M*v convention)
	};

	/// A constant matrix that has zeroes in all its entries.
	static const float3x4 zero;

	/// A constant matrix that is the identity.
	/** The identity matrix looks like the following:
		   1 0 0 0
		   0 1 0 0
		   0 0 1 0
		Transforming a vector by the identity matrix is like multiplying a number by one, i.e. the vector is not changed. */
	static const float3x4 identity;

	/// A compile-time constant float3x4 which has NaN in each element.
	/// For this constant, each element has the value of quiet NaN, or Not-A-Number.
	/// @note Never compare a float3x4 to this value! Due to how IEEE floats work, "nan == nan" returns false!
	///	   That is, nothing is equal to NaN, not even NaN itself!
	static const float3x4 nan;

	/// Creates a new float3x4 with uninitialized member values.
	/** [opaque-qtscript] */
	float3x4() {}

	float3x4(const float3x4 &rhs) = default; // { Set(rhs); }

	/// Constructs a new float3x4 by explicitly specifying all the matrix elements.
	/// The elements are specified in row-major format, i.e. the first row first followed by the second and third row.
	/// E.g. The element _10 denotes the scalar at second (index 1) row, first (index 0) column.
	float3x4(float _00, float _01, float _02, float _03,
			 float _10, float _11, float _12, float _13,
			 float _20, float _21, float _22, float _23);

	/// Constructs this float3x4 to represent the same transformation as the given float3x3.
	/** This function expands the last row and column of this matrix with the elements from the identity matrix. */
	float3x4(const float3x3 &other);

	/// Constructs this float3x4 to represent the same transformation as the given float3x3 and the given translation combined.
	/// The resulting matrix have its fourth column equal to the given translate vector, and the 3x3 top-left submatrix equal to
	/// the matrix other. This kind of matrix represents a transformation which first applies the matrix 'other', followed by a
	/// translation specified by the vector 'translate'.
	float3x4(const float3x3 &other, const float3 &translate);

	/// Constructs the matrix by explicitly specifying the four column vectors.
	/** @param col0 The first column. If this matrix represents a change-of-basis transformation, this parameter is the world-space
		direction of the local X axis.
		@param col1 The second column. If this matrix represents a change-of-basis transformation, this parameter is the world-space
		direction of the local Y axis.
		@param col2 The third column. If this matrix represents a change-of-basis transformation, this parameter is the world-space
		direction of the local Z axis.
		@param col3 The fourth column. If this matrix represents a change-of-basis transformation, this parameter is the world-space
		position of the local space pivot. */
	float3x4(const float3 &col0, const float3 &col1, const float3 &col2, const float3 &col3);

	/// Constructs this float3x4 from the given quaternion.
	explicit float3x4(const Quat &orientation);

	/// Constructs this float3x4 from the given quaternion and translation.
	/// Logically, the translation occurs after the rotation has been performed.
	float3x4(const Quat &orientation, const float3 &translation);

	/// Creates a new transformation matrix that translates by the given offset.
	/** @param offset A position or a direction vector specifying the amount of translation. The w-component is ignored. */
	static TranslateOp Translate(float tx, float ty, float tz);
	static TranslateOp Translate(const float3 &offset);
	static TranslateOp Translate(const float4 &offset);

	/// Creates a new float3x4 that rotates about one of the principal axes. [indexTitle: RotateX/Y/Z]
	/** Calling RotateX, RotateY or RotateZ is slightly faster than calling the more generic RotateAxisAngle function.
		@param angleRadians The angle to rotate by, in radians. For example, Pi/4.f equals to 45 degrees, Pi/2.f is 90 degrees, and Pi is 180 degrees.
		@param pointOnAxis If specified, the rotation is performed about an axis that passes through this point, and not
		through the origin. The returned matrix will not be a pure rotation matrix, but will also contain translation.
		@see DegToRad(). */
	static float3x4 RotateX(float angleRadians, const float3 &pointOnAxis);
	static float3x4 RotateX(float angleRadians);
	/** [similarOverload: RotateX] [hideIndex] */
	static float3x4 RotateY(float angleRadians);
	/** [similarOverload: RotateX] [hideIndex] */
	static float3x4 RotateY(float angleRadians, const float3 &pointOnAxis);
	/** [similarOverload: RotateX] [hideIndex] */
	static float3x4 RotateZ(float angleRadians);
	/** [similarOverload: RotateX] [hideIndex] */
	static float3x4 RotateZ(float angleRadians, const float3 &pointOnAxis);

	/// Creates a new float3x4 that rotates about the given axis.
	/** @param axisDirection The axis to rotate about. This vector must be normalized.
		@param angleRadians The angle to rotate by, in radians. Pi/4.f equals to 45 degrees, Pi/2.f is 90 degrees, and Pi is 180 degrees.
		@param pointOnAxis If specified, the rotation is performed about an axis that passes through this point, and not
		through the origin. The returned matrix will not be a pure rotation matrix, but will also contain translation. */
	static float3x4 RotateAxisAngle(const float3 &axisDirection, float angleRadians, const float3 &pointOnAxis);
	static float3x4 RotateAxisAngle(const float3 &axisDirection, float angleRadians);

	/// Creates a new float3x4 that rotates sourceDirection vector to coincide with the targetDirection vector.
	/** @note There are infinite such rotations - this function returns the rotation that has the shortest angle
		(when decomposed to axis-angle notation).
		@param sourceDirection The 'from' direction vector. This vector must be normalized.
		@param targetDirection The 'to' direction vector. This vector must be normalized.
		@param centerPoint If specified, rotation is performed using this point as the coordinate space origin. If omitted,
			the rotation is performed about the coordinate system origin (0,0,0).
		@return A new rotation matrix R for which R*sourceDirection == targetDirection. */
	static float3x4 RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection, const float3 &centerPoint);
	static float3x4 RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection);

	/// Returns a random 3x4 matrix with each entry randomized between the range[minElem, maxElem].
	/** Warning: The matrices returned by this function do not represent well-formed 3D transformations.
		This function is mostly used for testing and debugging purposes only. */
	static float3x4 RandomGeneral(LCG &lcg, float minElem, float maxElem);

	/// Returns a uniformly random 3x4 matrix that performs only rotation.
	/** This matrix produces a random orthonormal basis for an orientation of an object. There is no translation, mirroring
		or scaling present in the generated matrix. Also, naturally since float3x4 cannot represent projection,
		that property are not present either. */
	static float3x4 RandomRotation(LCG &lcg);

	/// Creates a new float3x4 that rotates one coordinate system to coincide with another.
	/** This function rotates the sourceDirection vector to coincide with the targetDirection vector, and then
			rotates sourceDirection2 (which was transformed by 1.) to targetDirection2, but keeping the constraint that
			sourceDirection must look at targetDirection.
			Rotation is performed around the specified centerPoint. */
//	static float3x4 RotateFromTo(const float3 &centerPoint, const float3 &sourceDirection, const float3 &targetDirection,
//		const float3 &sourceDirection2, const float3 &targetDirection2);

	/// Creates a new float3x4 that performs the rotation expressed by the given quaternion.
	static float3x4 FromQuat(const Quat &orientation);
	/** param pointOnAxis If specified, the rotation is performed using this point as the center point. */
	static float3x4 FromQuat(const Quat &orientation, const float3 &pointOnAxis);

	/// Creates a new float3x4 as a combination of translation, rotation and scale.
	/** This function creates a new float3x4 M of the form M = T * R * S, where T is a translation matrix, R a
		(rotation) matrix and S a scale matrix. Transforming a vector v using this matrix computes the vector
		v' == M * v == T*R*S*v == (T * (R * (S * v))), which means that the scale operation is applied to the
		vector first, followed by rotation and finally translation. */
	static float3x4 FromTRS(const float3 &translate, const Quat &rotate, const float3 &scale);
	static float3x4 FromTRS(const float3 &translate, const float3x3 &rotate, const float3 &scale);
	static float3x4 FromTRS(const float3 &translate, const float3x4 &rotate, const float3 &scale);

	/// Attempts to decompose this matrix to a combination of translation, rotation and scale. The components
	/// are written out to the input parameters if true is returned. If false is returned, decomposing did not
	/// succeed.
	void DecomposeToTRS(float3 &translate, float3x3 &rotate, float3 &scale) const;

	/// Creates a new float3x4 from the given sequence of Euler rotation angles (in radians).
	/** The FromEulerABC function returns a matrix M = A(ea) * B(eb) * C(ec). Rotation
		C is applied first, followed by B and then A. [indexTitle: FromEuler***] */
	static float3x4 FromEulerXYX(float ex, float ey, float ex2);
	static float3x4 FromEulerXZX(float ex, float ez, float ex2); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerYXY(float ey, float ex, float ey2); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerYZY(float ey, float ez, float ey2); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerZXZ(float ez, float ex, float ez2); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerZYZ(float ez, float ey, float ez2); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerXYZ(float ex, float ey, float ez); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerXZY(float ex, float ez, float ey); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerYXZ(float ey, float ex, float ez); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerYZX(float ey, float ez, float ex); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerZXY(float ez, float ex, float ey); ///< [similarOverload: FromEulerXYX] [hideIndex]
	static float3x4 FromEulerZYX(float ez, float ey, float ex); ///< [similarOverload: FromEulerXYX] [hideIndex]

	/// Creates a new transformation matrix that scales by the given factors.
	/// This matrix scales with respect to origin.
	static ScaleOp Scale(float sx, float sy, float sz);
	static ScaleOp Scale(const float3 &scale);
	static ScaleOp Scale(const float4 &scale);

	/// Creates a new float3x4 that scales with respect to the given center point.
	/** @param scale The amount of scale to apply to the x, y and z directions.
		@param scaleCenter The coordinate system center point for the scaling. If omitted, the origin (0,0,0) will
			be used as the origin for the scale operation. */
	static float3x4 Scale(const float3 &scale, const float3 &scaleCenter);
	static float3x4 Scale(const float4 &scale, const float4 &scaleCenter);

	/// Creates a new float3x4 that scales points along the given axis.
	/** @param axis A normalized direction vector that specifies the direction of scaling.
		@param scalingFactor The amount of scaling to apply along the specified axis. */
	/** @param scaleCenter If specified, this point will be used as the origin for the scale operation. */
	static float3x4 ScaleAlongAxis(const float3 &axis, float scalingFactor, const float3 &scaleCenter);
	static float3x4 ScaleAlongAxis(const float3 &axis, float scalingFactor);

	/// Creates a new float3x4 that performs uniform scaling by the given amount.
	static ScaleOp UniformScale(float uniformScale);
	static float3x4 UniformScale(float uniformScale, const float3 &scaleCenter);

	/// Returns the scaling performed by this matrix.
	/// GetScale().x specifies the amount of scaling applied to the local x direction vector when it is transformed by this matrix.
	/// i.e. GetScale()[i] equals Col(i).Length();
	float3 GetScale() const;

	/// Produces a matrix that shears along a principal axis.
	/** The shear matrix offsets the two other axes according to the
		position of the point along the shear axis. [indexTitle: ShearX/Y/Z] */
	static float3x4 ShearX(float yFactor, float zFactor);
	static float3x4 ShearY(float xFactor, float zFactor); ///< [similarOverload: ShearX] [hideIndex]
	static float3x4 ShearZ(float xFactor, float yFactor); ///< [similarOverload: ShearX] [hideIndex]

	/// Creates a new matrix that mirrors with respect to the given plane.
	/** Points lying on one side of the plane will end up at the opposite side of the plane, at the same distance of the plane
		they were. */
	static float3x4 Mirror(const Plane &p);

	/// Creates a new float3x4 that performs orthographic projection. [indexTitle: OrthographicProjection/YZ/XZ/XY]
	static float3x4 OrthographicProjection(const Plane &target);
	static float3x4 OrthographicProjectionYZ(); ///< [similarOverload: OrthographicProjection] [hideIndex]
	static float3x4 OrthographicProjectionXZ(); ///< [similarOverload: OrthographicProjection] [hideIndex]
	static float3x4 OrthographicProjectionXY(); ///< [similarOverload: OrthographicProjection] [hideIndex]

	/// Returns the given element. [noscript]
	/** Returns a reference to the element at m[row][col] (or "m[y][x]").
		Remember that zero-based indexing is used, so m[0][0] is the upper-left element of this matrix.
		@note You can use the index notation to set elements of the matrix, e.g. m[0][1] = 5.f;
		@note MatrixProxy is a temporary helper class. Do not store references to it, but always
		directly dereference it with the [] operator.
		For example, m[0][3] Returns the last element on the first row, which is the amount
	 	of translation in the x-direction. This is true independent of
	 	whether MATH_COLMAJOR_MATRICES is set, i.e. the notation is always m[y][x]. */
	FORCE_INLINE MatrixProxy<Rows, Cols> &operator[](int row)
	{
		mgl_assume(row >= 0);
		mgl_assume(row < Rows);
#ifdef MATH_COLMAJOR_MATRICES
		return *(reinterpret_cast<MatrixProxy<Rows, Cols>*>(&v[0][row]));
#else
		return *(reinterpret_cast<MatrixProxy<Rows, Cols>*>(v[row]));
#endif
	}

	FORCE_INLINE const MatrixProxy<Rows, Cols> &operator[](int row) const
	{
		mgl_assume(row >= 0);
		mgl_assume(row < Rows);		
#ifdef MATH_COLMAJOR_MATRICES
		return *(reinterpret_cast<const MatrixProxy<Rows, Cols>*>(&v[0][row]));
#else
		return *(reinterpret_cast<const MatrixProxy<Rows, Cols>*>(v[row]));
#endif
	}

	/// Returns the given element. [noscript]
	/** This function returns the element of this matrix at (row, col)==(i, j)==(y, x).
		If you have a non-const object, you can set values of this matrix through this
		reference, using the notation m.At(row, col) = someValue; */
	float &At(int row, int col);
	const float &At(int row, int col) const;

#ifdef MATH_COLMAJOR_MATRICES
	/// Returns the given column. [noscript]
	/** @param col The zero-based index [0, 3] of the column to get. */
	float3 &Col(int col);
	const float3 &Col(int col) const;
	
	/// Returns the three first elements of the given column. [noscript]
	/** @param col The zero-based index [0, 3] of the column to get. */
	float3 &Col3(int col);
	const float3 &Col3(int col) const;
	
	/// Returns the given row.
	/** @param row The zero-based index [0, 2] of the row to get. */
	CONST_WIN32 float4 Row(int row) const;
	CONST_WIN32 float3 Row3(int row) const { return Row(row).xyz(); }
#else
	/// Returns the given row. [noscript]
	/** @param row The zero-based index [0, 2] of the row to get. */
	float4 &Row(int row);
	const float4 &Row(int row) const;

	/// Returns the three first elements of the given row. [noscript]
	/** @param row The zero-based index [0, 2] of the row to get. */
	float3 &Row3(int row);
	const float3 &Row3(int row) const;

	/// Returns the given column.
	/** @param col The zero-based index [0, 3] of the column to get. */
	CONST_WIN32 float3 Col(int col) const;
	CONST_WIN32 float3 Col3(int col) const { return Col(col); }
#endif

	/// Returns the main diagonal.
	/** The main diagonal consists of the elements at m[0][0], m[1][1], m[2][2]. */
	CONST_WIN32 float3 Diagonal() const;

	/// Scales the three first elements of the given row by a scalar.
	void ScaleRow3(int row, float scalar);

	/// Scales the given row by a scalar.
	void ScaleRow(int row, float scalar);

	/// Scales the given column by a scalar.
	void ScaleCol(int col, float scalar);

	/// Returns the upper-left 3-by-3 part.
	CONST_WIN32 float3x3 Float3x3Part() const;

	/// Returns the translation part.
	/** The translation part is stored in the fourth column of this matrix.
		This is equivalent to decomposing this matrix in the form M = T * M', i.e. this translation is applied last,
		after applying rotation and scale. If this matrix represents a local->world space transformation for an object,
		then this gives the world space position of the object.
		@note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]). */
	CONST_WIN32 float3 TranslatePart() const;

	/// Returns the upper-left 3x3 part of this matrix. This part stores the rotation of this transform.
	CONST_WIN32 float3x3 RotatePart() const;

	// Returns the local right axis in the post-transformed coordinate space, according to the given convention.
	// @note The returned vector might not be normalized if this matrix contains scaling.
	// @note The basis returned by (Right, Up, Forward) might not be of the same handedness as the
	//	   pre-transformed coordinate system, if the matrix contained reflection.
//	template<typename Convention = XposRight_YposUp_ZposForward> float3 Right() const;

	// Returns the local up axis in the post-transformed coordinate space, according to the given convention.
	// @note The returned vector might not be normalized if this matrix contains scaling.
	// @note The basis returned by (Right, Up, Forward) might not be of the same handedness as the
	//	   pre-transformed coordinate system, if the matrix contained reflection.
//	template<typename Convention = XposRight_YposUp_ZposForward> float3 Up() const;

	// Returns the local forward axis in the post-transformed coordinate space, according to the given convention.
	// @note The returned vector might not be normalized if this matrix contains scaling.
	// @note The basis returned by (Right, Up, Forward) might not be of the same handedness as the
	//	   pre-transformed coordinate system, if the matrix contained reflection.
//	template<typename Convention = XposRight_YposUp_ZposForward> float3 Forward() const;

	/// Returns the local +X/+Y/+Z axis in world space.
	/** This is the same as transforming the vector (1,0,0) by this matrix. [indexTitle: PositiveX/Y/Z] */
	float3 WorldX() const;
	/// Returns the local +Y axis in world space.
	/** This is the same as transforming the vector (0,1,0) by this matrix. [similarOverload: PositiveX] [hideIndex] */
	float3 WorldY() const;	
	/// Returns the local +Z axis in world space.
	/** This is the same as transforming the vector (0,0,1) by this matrix. [similarOverload: PositiveX] [hideIndex] */
	float3 WorldZ() const;

	/// Accesses this structure as a float array.
	/// @return A pointer to the upper-left element. The data is contiguous in memory.
	/// ptr[0] gives the element [0][0], ptr[1] is [0][1], ptr[2] is [0][2].
	/// ptr[4] == [1][0], ptr[5] == [1][1], ..., and finally, ptr[15] == [3][3].
	FORCE_INLINE float *ptr() { return &v[0][0]; }
	FORCE_INLINE const float *ptr() const { return &v[0][0]; }

	/// Sets the values of the given row.
	/** @param row The index of the row to set, in the range [0-2].
		@param data A pointer to an array of 4 floats that contain the new x, y, z and w values for the row. */
	void SetRow(int row, const float *data);
	void SetRow(int row, const float3 &rowVector, float m_r3);
	void SetRow(int row, const float4 &rowVector);
	void SetRow(int row, float m_r0, float m_r1, float m_r2, float m_r3);

	/// Sets the values of the given column.
	/** @param column The index of the column to set, in the range [0-3].
		@param data A pointer to an array of 3 floats that contain the new x, y and z values for the column. */
	void SetCol(int column, const float *data);
	void SetCol(int column, const float3 &columnVector);
	void SetCol(int column, float m_0c, float m_1c, float m_2c);

	/// Sets all values of this matrix.
	void Set(float _00, float _01, float _02, float _03,
			 float _10, float _11, float _12, float _13,
			 float _20, float _21, float _22, float _23);

	/// Sets this to be a copy of the matrix rhs.
	void Set(const float3x4 &rhs);

	/// Sets all values of this matrix.
	/// @param values The values in this array will be copied over to this matrix. The source must contain 12 floats in row-major order (the same
	///		order as the Set() function above has its input parameters in).
	void Set(const float *values);

	/// Sets a single element of this matrix.
	/** @param row The row index (y-coordinate) of the element to set, in the range [0-2].
		@param col The col index (x-coordinate) of the element to set, in the range [0-3].
		@param value The new value to set to the cell [row][col]. */
	void Set(int row, int col, float value);

	void Set3x3Part(const float3x3 &rotation);

	/// Sets this matrix to equal the identity.
	void SetIdentity();

	/// Swaps two columns.
	void SwapColumns(int col1, int col2);

	/// Swaps two rows.
	void SwapRows(int row1, int row2);

	/// Sets the translation part of this matrix.
	/** This function sets the translation part of this matrix. These are the three first elements of the fourth column.
		All other entries are left untouched. */
	void SetTranslatePart(float translateX, float translateY, float translateZ) { SetCol(3, translateX, translateY, translateZ); }
	void SetTranslatePart(const float3 &offset) { SetCol(3, offset); }

	/// Sets the 3-by-3 part of this matrix to perform rotation about the positive X axis which passes through
	/// the origin. Leaves all other entries of this matrix untouched. [similarOverload: SetRotatePart] [hideIndex]
	void SetRotatePartX(float angleRadians);
	/// Sets the 3-by-3 part of this matrix to perform rotation about the positive Y axis. Leaves all other
	/// entries untouched. [similarOverload: SetRotatePart] [hideIndex]
	void SetRotatePartY(float angleRadians);
	/// Sets the 3-by-3 part of this matrix to perform rotation about the positive Z axis. Leaves all other
	/// entries untouched. [similarOverload: SetRotatePart] [hideIndex]
	void SetRotatePartZ(float angleRadians);

	/// Sets the 3-by-3 part of this matrix to perform rotation about the given axis and angle. Leaves all other
	/// entries of this matrix untouched. [indexTitle: SetRotatePart/X/Y/Z]
	void SetRotatePart(const float3 &axisDirection, float angleRadians);
	/// Sets the 3-by-3 part of this matrix to perform the rotation expressed by the given quaternion.
	/// Leaves all other entries of this matrix untouched.
	void SetRotatePart(const Quat &orientation);
	/// Sets the 3-by-3 part of this matrix.
	/// @note This is a convenience function which calls Set3x3Part.
	/// @note This function erases the previous top-left 3x3 part of this matrix (any previous rotation, scaling and shearing, etc.). Translation is unaffected.
	void SetRotatePart(const float3x3 &rotation) { Set3x3Part(rotation); }

	/// Creates a LookAt matrix from a look-at direction vector.
	/** A LookAt matrix is a rotation matrix that orients an object to face towards a specified target direction.
		@param localForward Specifies the forward direction in the local space of the object. This is the direction
			the model is facing at in its own local/object space, often +X (1,0,0), +Y (0,1,0) or +Z (0,0,1). The
			vector to pass in here depends on the conventions you or your modeling software is using, and it is best
			pick one convention for all your objects, and be consistent.
			This input parameter must be a normalized vector.
		@param targetDirection Specifies the desired world space direction the object should look at. This function
			will compute a rotation matrix which will rotate the localForward vector to orient towards this targetDirection
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
		@return A matrix that maps the given local space forward direction vector to point towards the given target
			direction, and the given local up direction towards the given target world up direction. The returned
			matrix M is orthonormal with a determinant of +1. For the matrix M it holds that
			M * localForward = targetDirection, and M * localUp lies in the plane spanned by the vectors targetDirection
			and worldUp.
		@note The position of (the translation performed by) the resulting matrix will be set to (0,0,0), i.e. the object
			will be placed to origin. Call SetTranslatePart() on the resulting matrix to set the position of the model.
		@see RotateFromTo(). */
	static float3x4 LookAt(const float3 &localForward, const float3 &targetDirection, const float3 &localUp, const float3 &worldUp);

	/// Creates a LookAt matrix from source and target points.
	/**	A LookAt matrix is a rotation matrix that orients an object to face towards a specified target direction.
		@param eyePos The position the observer is at, i.e. the position of the model.
		@param targetPos The target position the model should be looking at. The vectors eyePos and targetPos
			cannot be equal, and the direction specified by targetPos - eyePos cannot be collinear to the direction
			passed in worldUp.
		@param localForward Specifies the forward direction in the local space of the object. This is the direction
			the model is facing at in its own local/object space, often +X (1,0,0), +Y (0,1,0) or +Z (0,0,1). The
			vector to pass in here depends on the conventions you or your modeling software is using, and it is best
			pick one convention for all your objects, and be consistent.
			This input parameter must be a normalized vector.
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
		@return A matrix that maps the given local space forward direction vector to point towards the given target
			direction, and the given local up direction towards the given target world up direction. The returned
			matrix M is orthonormal with a determinant of +1. For the matrix M it holds that
			M * localForward = targetDirection, and M * localUp lies in the plane spanned by the vectors targetDirection
			and worldUp.
		@note The position of (the translation performed by) the resulting matrix will be set to eyePos, i.e. the object
			will be placed to the given eye position.
		@see RotateFromTo().
		@note Be aware that the convention of a 'LookAt' matrix in MathGeoLib differs from e.g. GLM. In MathGeoLib, the returned
			matrix is a mapping from local space to world space, meaning that the returned matrix can be used as the 'world transform'
			for any 3D object (camera or not). The view space is the local space of the camera, so this function returns the mapping
			view->world. In GLM, the LookAt function is tied to cameras only, and it returns the inverse mapping world->view. */
	static float3x4 LookAt(const float3 &eyePos, const float3 &targetPos, const float3 &localForward,
	                       const float3 &localUp, const float3 &worldUp);

	/// Sets this float3x4 to represent the same transformation as the given float3x3.
	/// @note The translate part of this float3x4 is reset to zero.
	float3x4 &operator =(const float3x3 &rhs);

	/// Sets this float3x4 to represent the same rotation as the given Quat.
	/// @note The translate part of this float3x4 is reset to zero.
	float3x4 &operator =(const Quat &rhs);

	/// Sets this float3x4 to represent the same transformation as the given float3x4.
	float3x4 &operator =(const float3x4 &rhs) = default;

	/// Computes the determinant of this matrix.
	/** If the determinant is nonzero, this matrix is invertible.
		If the determinant is negative, this matrix performs reflection about some axis.
		From http://msdn.microsoft.com/en-us/library/bb204853(VS.85).aspx :
		"If the determinant is positive, the basis is said to be "positively" oriented (or right-handed).
		If the determinant is negative, the basis is said to be "negatively" oriented (or left-handed)." */
	float Determinant() const;

	/// Inverts this matrix using the generic Gauss's method.
	/// @return Returns true on success, false otherwise.
	bool Inverse(float epsilon = 1e-6f);

	/// Returns an inverted copy of this matrix. Uses Gauss's method.
	/// If this matrix does not have an inverse, returns the matrix that was the result of running
	/// Gauss's method on the matrix.
	float3x4 Inverted() const;

	/// Inverts a column-orthogonal matrix.
	/// If a matrix is of form M=T*R*S, where T is an affine translation matrix,
	/// R is a rotation matrix and S is a diagonal matrix with non-zero but potentially non-uniform scaling
	/// factors (possibly mirroring), then the matrix M is column-orthogonal and this function can be used to compute the inverse.
	/// Calling this function is faster than the calling the generic matrix Inverse() function.
	/// Returns true on success. On failure, the matrix is not modified. This function fails if any of the
	/// elements of this vector are not finite, or if the matrix contains a zero scaling factor on X, Y or Z.
	/// @note The returned matrix will be row-orthogonal, but not column-orthogonal in general.
	/// The returned matrix will be column-orthogonal iff the original matrix M was row-orthogonal as well.
	/// (in which case S had uniform scale, InverseOrthogonalUniformScale() could have been used instead)
	bool InverseColOrthogonal();

	/// Inverts a matrix that is a concatenation of only translate, rotate and uniform scale operations.
	/// If a matrix is of form M=T*R*S, where T is an affine translation matrix,
	/// R is a rotation matrix and S is a diagonal matrix with non-zero and uniform scaling factors (possibly mirroring),
	/// then the matrix M is both column- and row-orthogonal and this function can be used to compute the inverse.
	/// This function is faster than calling InverseColOrthogonal() or the generic Inverse().
	/// Returns true on success. On failure, the matrix is not modified. This function fails if any of the
	/// elements of this vector are not finite, or if the matrix contains a zero scaling factor on X, Y or Z.
	/// This function may not be called if this matrix contains any shearing or nonuniform scaling.
	bool InverseOrthogonalUniformScale();

	/// Inverts a matrix that is a concatenation of only translate and rotate operations.
	/// If a matrix is of form M=T*R*S, where T is an affine translation matrix, R is a rotation
	/// matrix and S is either identity or a mirroring matrix, then the matrix M is orthonormal and this function can be used to compute the inverse.
	/// This function is faster than calling InverseOrthogonalUniformScale(), InverseColOrthogonal() or the
	/// generic Inverse().
	/// This function may not be called if this matrix contains any scaling or shearing, but it may contain mirroring.
	void InverseOrthonormal();

	/// Transposes the top-left 3x3 part of this matrix in-place. The fourth column (translation part) will
	/// remain intact.
	/// This operation swaps all elements with respect to the diagonal.
	void Transpose3();

	/// Returns a copy of this matrix which has the top-left 3x3 part transposed.
	float3x4 Transposed3() const;

	/// Computes the inverse transpose of this matrix in-place.
	/** Use the inverse transpose to transform covariant vectors (normal vectors).
		@note This function resets the translation part of this matrix to zero. */
	bool InverseTranspose();

	/// Returns the inverse transpose of this matrix.
	/** Use that matrix to transform covariant vectors (normal vectors).
		@note This function resets the translation part of this matrix to zero. */
	float3x4 InverseTransposed() const;

	/// Returns the sum of the diagonal elements of this matrix.
	float Trace() const;

	/// Orthonormalizes the basis formed by the column vectors of this matrix.
	void Orthonormalize(int firstColumn, int secondColumn, int thirdColumn);

	/// Removes the scaling performed by this matrix. That is, decomposes this matrix M into a form M = M' * S, where
	/// M' has unitary column vectors and S is a diagonal matrix. Then replaces this matrix with M'.
	/// @note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]).
	/// @note This function does not remove reflection (-1 scale along some axis).
	void RemoveScale();

	/// Transforms the given point vector by this matrix M , i.e. returns M * (x, y, z, 1).
	/** The suffix "Pos" in this function means that the w component of the input vector is assumed to be 1, i.e. the input
		vector represents a point (a position). */
	float2 TransformPos(const float2 &pointVector) const;
	float3 TransformPos(const float3 &pointVector) const;
	float3 TransformPos(float x, float y, float z) const;
	float2 TransformPos(float x, float y) const;

	/// Transforms the given direction vector by this matrix M , i.e. returns M * (x, y, z, 0).
	/** The suffix "Dir" in this function just means that the w component of the input vector is assumed to be 0, i.e. the
		input vector represents a direction. The input vector does not need to be normalized. */
	float4 TransformDir(const float4 &directionVector) const;
	float3 TransformDir(const float3 &directionVector) const;
	float2 TransformDir(const float2 &directionVector) const;
	float3 TransformDir(float x, float y, float z) const;
	float2 TransformDir(float x, float y) const;

	/// Transforms the given 4-vector by this matrix M, i.e. returns M * (x, y, z, w).
	float4 Transform(const float4 &vector) const;

	/// Performs a batch transform of the given array of point vectors.
	/** The suffix "Pos" in this function just means that the w components of each input vector are assumed to be 1, i.e. the
		input vectors represent points (positions).
		@param stride If specified, represents the distance in bytes between subsequent vector elements. If stride is not
			specified, the vectors are assumed to be tightly packed in memory. */
	void BatchTransformPos(float3 *pointArray, int numPoints) const;
	void BatchTransformPos(float3 *pointArray, int numPoints, int stride) const;
	void BatchTransformPos(float4 *vectorArray, int numVectors) const { BatchTransform(vectorArray, numVectors); }
	void BatchTransformPos(float4 *vectorArray, int numVectors, int stride) const { BatchTransform(vectorArray, numVectors, stride); }

	/// Performs a batch transform of the given array of direction vectors.
	/** The suffix "Dir" in this function just means that the w components of each input vector are assumed to be 0, i.e. the
		input vectors represent directions. The input vectors do not need to be normalized.
		@param stride If specified, represents the distance in bytes between subsequent vector elements. If stride is not
			specified, the vectors are assumed to be tightly packed in memory. */
	void BatchTransformDir(float3 *dirArray, int numVectors) const;
	void BatchTransformDir(float3 *dirArray, int numVectors, int stride) const;
	void BatchTransformDir(float4 *vectorArray, int numVectors) const { BatchTransform(vectorArray, numVectors); }
	void BatchTransformDir(float4 *vectorArray, int numVectors, int stride) const { BatchTransform(vectorArray, numVectors, stride); }

	/// Performs a batch transform of the given array.
	/** @param stride If specified, represents the distance in bytes between subsequent vector elements. If stride is not
			specified, the vectors are assumed to be tightly packed in memory. */
	void BatchTransform(float4 *vectorArray, int numVectors) const;
	void BatchTransform(float4 *vectorArray, int numVectors, int stride) const;

	/// Treats the float3x3 as a 4-by-4 matrix with the last row and column as identity, and multiplies the two matrices.
	float3x4 operator *(const float3x3 &rhs) const;

	/// Treats the float3x4 as a 4-by-4 matrix with the last row as identity, and multiplies the two matrices.
	float3x4 operator *(const float3x4 &rhs) const;

	/// Converts the quaternion to a float3x4 and multiplies the two matrices together.
	float3x4 operator *(const Quat &rhs) const;

	/// Transforms the given vector by this matrix (in the order M * v).
	/// The fourth element of the vector is preserved.
	float4 operator *(const float4 &rhs) const;

	float3x4 operator *(float scalar) const;
	float3x4 operator /(float scalar) const;
	float3x4 operator +(const float3x4 &rhs) const;
	float3x4 operator -(const float3x4 &rhs) const;
	float3x4 operator -() const;

	/// Unary operator + allows this structure to be used in an expression '+x'.
	float3x4 operator +() const { return *this; }

	float3x4 &operator *=(float scalar);
	float3x4 &operator /=(float scalar);
	float3x4 &operator +=(const float3x4 &rhs);
	float3x4 &operator -=(const float3x4 &rhs);

	/// Tests if this matrix does not contain any NaNs or infs.
	/** @return Returns true if the entries of this float3x4 are all finite, and do not contain NaN or infs. */
	bool IsFinite() const;

	/// Tests if this is the identity matrix.
	/** @return Returns true if this matrix is the identity matrix, up to the given epsilon. */
	bool IsIdentity(float epsilon = 1e-3f) const;

	/// Tests if this matrix is in lower triangular form.
	/** @return Returns true if this matrix is in lower triangular form, up to the given epsilon. */
	bool IsLowerTriangular(float epsilon = 1e-3f) const;

	/// Tests if this matrix is in upper triangular form.
	/** @return Returns true if this matrix is in upper triangular form, up to the given epsilon. */
	bool IsUpperTriangular(float epsilon = 1e-3f) const;

	/// Tests if this matrix has an inverse.
	/** This function treats this matrix as a square 4x4 matrix with the last row having the form [0 0 0 1].
		@return Returns true if this matrix can be inverted, up to the given epsilon. */
	bool IsInvertible(float epsilon = 1e-3f) const;

	/// Tests if this matrix is symmetric (M == M^T).
	/** This function treats this matrix as a square 4x4 matrix with the last row having the form [0 0 0 1].
		The test compares the elements for equality, up to the given epsilon. A matrix is symmetric if it is its own transpose. */
	bool IsSymmetric(float epsilon = 1e-3f) const;

	/// Tests if this matrix is skew-symmetric (M == -M^T).
	/** This function treats this matrix as a square 4x4 matrix with the last row having the form [0 0 0 1].
		The test compares the floating point elements of this matrix up to the given epsilon. A matrix M is skew-symmetric
		the identity M=-M^T holds. */
	bool IsSkewSymmetric(float epsilon = 1e-3f) const;

	/// Returns true if this matrix does not perform any scaling.
	/** A matrix does not do any scaling if the column vectors of this
		matrix are normalized in length, compared to the given epsilon. Note that this matrix may still perform
		reflection, i.e. it has a -1 scale along some axis.
		@note This function only examines the upper 3-by-3 part of this matrix.
		@note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]). */
	bool HasUnitaryScale(float epsilonSq = 1e-6f) const;

	/// Returns true if this matrix performs a reflection along some plane.
	/** In 3D space, an even number of reflections corresponds to a rotation about some axis, so a matrix consisting of
		an odd number of consecutive mirror operations can only reflect about one axis. A matrix that contains reflection reverses
		the handedness of the coordinate system. This function tests if this matrix
		does perform mirroring. This occurs iff this matrix has a negative determinant. */
	bool HasNegativeScale() const;

	/// Returns true if this matrix contains only uniform scaling, compared to the given epsilon.
	/// @note If the matrix does not really do any scaling, this function returns true (scaling uniformly by a factor of 1).
	/// @note This function only examines the upper 3-by-3 part of this matrix.
	/// @note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]).
	bool HasUniformScale(float epsilon = 1e-3f) const;

	/// Returns true if the row vectors of 3x3 top-left submatrix are all perpendicular to each other.
	bool IsRowOrthogonal(float epsilon = 1e-3f) const;

	/// Returns true if the column vectors of 3x3 top-left submatrix are all perpendicular to each other.
	bool IsColOrthogonal(float epsilon = 1e-3f) const;
	bool IsColOrthogonal3(float epsilon = 1e-3f) const { return IsColOrthogonal(epsilon); }

	/// Returns true if the column and row vectors of the 3x3 top-left submatrix form an orthonormal set.
	/// @note In math terms, there does not exist such a thing as 'orthonormal matrix'. In math terms, a matrix
	/// is orthogonal iff its column and row vectors are orthogonal *unit* vectors.
	/// In the terms of this library however, a matrix is orthogonal iff its column and row vectors are orthogonal (no need to be unitary),
	/// and a matrix is orthonormal if the column and row vectors are orthonormal.
	bool IsOrthonormal(float epsilon = 1e-3f) const;

	/// Returns true if this float3x4 is equal to the given float3x4, up to given per-element epsilon.
	bool Equals(const float3x4 &other, float epsilon = 1e-3f) const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns a string representation of form "(m00, m01, m02, m03; m10, m11, m12, m13; ... )".
	StringT ToString() const;
	StringT SerializeToString() const;

	StringT ToString2() const;

	static float3x4 FromString(const char *str, const char **outEndStr = 0);
#endif

	/// Extracts the rotation part of this matrix into Euler rotation angles (in radians). [indexTitle: ToEuler***]
	/// @note It is better to think about the returned float3 as an array of three floats, and
	/// not as a triple of xyz, because e.g. the .y component returned by ToEulerYXZ() does
	/// not return the amount of rotation about the y axis, but contains the amount of rotation
	/// in the second axis, in this case the x axis.
	float3 ToEulerXYX() const;
	float3 ToEulerXZX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerYXY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerYZY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerZXZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerZYZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerXYZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerXZY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerYXZ() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerYZX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerZXY() const; ///< [similarOverload: ToEulerXYX] [hideIndex]
	float3 ToEulerZYX() const; ///< [similarOverload: ToEulerXYX] [hideIndex]

	/// Returns the scale components of this matrix.
	/** This function decomposes this matrix M into a form M = M' * S, where M' has unitary column vectors and S is a diagonal matrix.
		@return ExtractScale returns the diagonal entries of S, i.e. the scale of the columns of this matrix . If this matrix
		represents a local->world space transformation for an object, then this scale represents a 'local scale', i.e.
		scaling that is performed before translating and rotating the object from its local coordinate system to its world
		position.
		@note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]).
		@note This function does not detect and return reflection (-1 scale along some axis). */
	float3 ExtractScale() const;

	/// Decomposes this matrix to translate, rotate and scale parts.
	/** This function decomposes this matrix M to a form M = T * R * S, where T is a translation matrix, R a rotation matrix and S a
		scale matrix.
		@note Remember that in the convention of this class, transforms are applied in the order M * v, so scale is
		applied first, then rotation, and finally the translation last.
		@note This function assumes that this matrix does not contain projection (the fourth row of this matrix is [0 0 0 1]).
		@param translate [out] This vector receives the translation component this matrix performs. The translation is applied last
			after rotation and scaling.
		@param rotate [out] This object receives the rotation part of this transform.
		@param scale [out] This vector receives the scaling along the local (before transformation by R) X, Y and Z axes
			performed by this matrix. */
	void Decompose(float3 &translate, Quat &rotate, float3 &scale) const;
	void Decompose(float3 &translate, float3x3 &rotate, float3 &scale) const;
	void Decompose(float3 &translate, float3x4 &rotate, float3 &scale) const;

	float3x4 Mul(const float3x3 &rhs) const;
	float3x4 Mul(const float3x4 &rhs) const;
	float4x4 Mul(const float4x4 &rhs) const;
	float3x4 Mul(const Quat &rhs) const;
	float2 MulPos(const float2 &pointVector) const;
	float3 MulPos(const float3 &pointVector) const;
	float4 MulPos(const float4 &pointVector) const;
	float2 MulDir(const float2 &directionVector) const;
	float3 MulDir(const float3 &directionVector) const;
	float4 MulDir(const float4 &directionVector) const;
	float4 Mul(const float4 &vector) const;
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float3x4 to the given stream.
std::ostream &operator <<(std::ostream &out, const float3x4 &rhs);
#endif

/// Multiplies two transforms together.
float3x4 operator *(const Quat &lhs, const float3x4 &rhs);
//float3x4 operator *(const float3x4 &lhs, const float3x4 &rhs);
float3x4 operator *(const float3x3 &lhs, const float3x4 &rhs);

/// Transforms the given vector by the given matrix in the order v * M. Note that this form
/// of multiplication is against the convention of this math system. Please use the M * v notation instead.
/// (Remember that M * v != v * M in general).
float4 operator *(const float4 &lhs, const float3x4 &rhs);

MATH_END_NAMESPACE
