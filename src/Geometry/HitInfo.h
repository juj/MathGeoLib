/** @file HitInfo.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

struct HitInfo
{
    enum HitResult
    {
        NoHit,
        Intersect,
        AInsideB,
        BInsideA
    };

    /// Specifies the result of the intersection test.
    HitResult result;

    /// Stores the point of intersection.
    float3 point;

    /// Specifies the surface normal of the 'this' object at the point of intersection.
    float3 normalA;

    /// Specifies the surface normal of the other object at the point of intersection.
    float3 normalB;
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(HitInfo)
Q_DECLARE_METATYPE(HitInfo*)
#endif
