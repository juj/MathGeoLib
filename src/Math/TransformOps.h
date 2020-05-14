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

/** @file TransformOps.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "../MathGeoLibFwd.h"
#include "float3.h"
#include "SSEMath.h"

MATH_BEGIN_NAMESPACE

/// A structure that represents the translate operation for 3D objects.
/** This structure is used to optimize special cases of 3D transformation concatenations. The use of this
	class occurs transparently to the user. You do not need to instantiate new TranslateOp objects in your code. */
class ALIGN16 TranslateOp
{
public:
	vec offset;

	/// Constructs an uninitialized TranslateOp.
	TranslateOp() {}

	/// Constructs a TranslateOp that translates the given amount.
	explicit TranslateOp(const float3 &offset);
	explicit TranslateOp(const float4 &offset);
	TranslateOp(float x, float y, float z);

	/// Returns the translation offset (x, y, z).
	vec Offset() const;

	/// Converts this TranslateOp object to a matrix.
	float3x4 ToFloat3x4() const;
	/// Converts this TranslateOp object to a matrix.
	float4x4 ToFloat4x4() const;

	/// Converts this TranslateOp object to a matrix.
	operator float3x4() const;
	/// Converts this TranslateOp object to a matrix.
	operator float4x4() const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x, y, z)".
	StringT ToString() const;
#endif
};

float3x4 operator *(const TranslateOp &lhs, const float3x4 &rhs);
float3x4 operator *(const float3x4 &lhs, const TranslateOp &rhs);

// This form of multiplication is based on the optimization assumption that the last row of rhs is [0,0,0,1], i.e.
// that rhs does not contain a "projective" part. If this does not hold, cast TranslateOp lhs to a float4x4 to 
// perform a full generic 4x4 matrix multiplication instead.
float4x4 operator *(const TranslateOp &lhs, const float4x4 &rhs);
float4x4 operator *(const float4x4 &lhs, const TranslateOp &rhs);

/// A structure that represents the scale operation for 3D objects.
/** This structure is used to optimize special cases of 3D transformation concatenations. The use of this
	class occurs transparently to the user. You do not need to instantiate new ScaleOp objects in your code. */
class ALIGN16 ScaleOp
{
public:
	vec scale;

	/// Constructs an uninitialized ScaleOp.
	ScaleOp() {}

	/// Constructs a ScaleOp with the given scale factors.
	explicit ScaleOp(const float2 &scaleXY, float scaleZ);
	explicit ScaleOp(const float3 &scale);
	explicit ScaleOp(const float4 &scale);
	ScaleOp(float sx, float sy, float sz);

	/// Returns the scale factors (x, y, z).
	vec Offset() const;

	/// Converts this ScaleOp to a matrix.
	operator float3x3() const;
	/// Converts this ScaleOp to a matrix.
	operator float3x4() const;
	/// Converts this ScaleOp to a matrix.
	operator float4x4() const;

	/// Converts this ScaleOp to a matrix.
	float3x3 ToFloat3x3() const;
	/// Converts this ScaleOp to a matrix.
	float3x4 ToFloat3x4() const;
	/// Converts this ScaleOp to a matrix.
	float4x4 ToFloat4x4() const;

#if defined(MATH_ENABLE_STL_SUPPORT) || defined(MATH_CONTAINERLIB_SUPPORT)
	/// Returns "(x, y, z)".
	StringT ToString() const;
#endif
};

float3x3 operator *(const ScaleOp &lhs, const float3x3 &rhs);
float3x3 operator *(const float3x3 &lhs, const ScaleOp &rhs);
float3x4 operator *(const ScaleOp &lhs, const float3x4 &rhs);
float3x4 operator *(const float3x4 &lhs, const ScaleOp &rhs);
float4x4 operator *(const ScaleOp &lhs, const float4x4 &rhs);
float4x4 operator *(const float4x4 &lhs, const ScaleOp &rhs);

float3x4 operator *(const ScaleOp &lhs, const TranslateOp &rhs);
float3x4 operator *(const TranslateOp &lhs, const ScaleOp &rhs);

MATH_END_NAMESPACE
