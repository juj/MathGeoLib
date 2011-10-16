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

/** @file TransformOps.h
    @author Jukka Jylänki
    @brief */
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

MATH_BEGIN_NAMESPACE

/// A structure which represents translation of 3D objects.
/// This structure is used to optimize special cases of 3D transformation concatenations.
class TranslateOp
{
public:
    float x;
    float y;
    float z;

    TranslateOp() {}
    explicit TranslateOp(const float3 &offset);
    TranslateOp(float x, float y, float z);

    float3x4 ToFloat3x4() const;
    float4x4 ToFloat4x4() const;

    float3 Offset() const;

    operator float3x4() const;
    operator float4x4() const;
};

float3x4 operator *(const TranslateOp &lhs, const float3x4 &rhs);
float3x4 operator *(const float3x4 &lhs, const TranslateOp &rhs);
float4x4 operator *(const TranslateOp &lhs, const float4x4 &rhs);
float4x4 operator *(const float4x4 &lhs, const TranslateOp &rhs);

/// A structure which represents scaling of 3D objects.
/// This structure is used to optimize special cases of 3D transformation concatenations.
class ScaleOp
{
public:
    float x;
    float y;
    float z;

    ScaleOp() {}
    explicit ScaleOp(const float3 &scale);
    ScaleOp(float sx, float sy, float sz);

    float3 Offset() const;
    operator float3x3() const;
    operator float3x4() const;
    operator float4x4() const;

    float3x3 ToFloat3x3() const;
    float3x4 ToFloat3x4() const;
    float4x4 ToFloat4x4() const;
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

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(TranslateOp)
Q_DECLARE_METATYPE(TranslateOp*)
Q_DECLARE_METATYPE(ScaleOp)
Q_DECLARE_METATYPE(ScaleOp*)
#endif
