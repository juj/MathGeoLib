/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Math/float3.h"
/*
struct PositiveX
{ 
    static float3 Pick(const float3x3 &m) { return m.Column(0); }
    static float3 Pick(const float3x4 &m) { return m.Column(0); }
    static float3 Pick(const float4x4 &m) { return m.Column3(0); }
};

struct NegativeX
{ 
    static float3 Pick(const float3x3 &m) { return -m.Column(0); }
    static float3 Pick(const float3x4 &m) { return -m.Column(0); }
    static float3 Pick(const float4x4 &m) { return -m.Column3(0); }
};

struct PositiveY
{ 
    static float3 Pick(const float3x3 &m) { return m.Column(1); }
    static float3 Pick(const float3x4 &m) { return m.Column(1); }
    static float3 Pick(const float4x4 &m) { return m.Column3(1); }
};

struct NegativeY
{ 
    static float3 Pick(const float3x3 &m) { return -m.Column(1); }
    static float3 Pick(const float3x4 &m) { return -m.Column(1); }
    static float3 Pick(const float4x4 &m) { return -m.Column3(1); }
};

struct PositiveZ
{
    static float3 Pick(const float3x3 &m) { return m.Column(2); }
    static float3 Pick(const float3x4 &m) { return m.Column(2); }
    static float3 Pick(const float4x4 &m) { return m.Column3(2); }
};

struct NegativeZ
{ 
    static float3 Pick(const float3x3 &m) { return -m.Column(2); }
    static float3 Pick(const float3x4 &m) { return -m.Column(2); }
    static float3 Pick(const float4x4 &m) { return -m.Column3(2); }
};

/// For more information about coordinate axis conventions, and handedness,
/// see http://msdn.microsoft.com/en-us/library/bb204853(VS.85).aspx .
template<typename ForwardAxis, typename RightAxis, typename UpAxis>
struct CoordinateAxisConvention
{
    template<typename Matrix> static float3 Right(const Matrix &m) { return RightAxis::Pick(m); }
    template<typename Matrix> static float3 Up(const Matrix &m) { return UpAxis::Pick(m); }
    template<typename Matrix> static float3 Forward(const Matrix &m) { return ForwardAxis::Pick(m); }
};

/// This is the default coordinate axis convention used by the math classes. This convention
/// is a left-handed coordinate system.
typedef CoordinateAxisConvention<PositiveX, PositiveY, PositiveZ> XposRight_YposUp_ZposForward;
*/