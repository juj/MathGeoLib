/** @file TransformOps.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief A 4-by-4 matrix for affine and homogeneous operations in 3D space.
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

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

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(TranslateOp)
Q_DECLARE_METATYPE(TranslateOp*)
Q_DECLARE_METATYPE(ScaleOp)
Q_DECLARE_METATYPE(ScaleOp*)
#endif
