/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

/// A 3D cylinder-shaped geometric primitive.
class Cylinder
{
public:
    Cylinder() {}

    // Bounding Cylinder construction.

    bool Contains(const float3 &point) const;
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Cylinder)
Q_DECLARE_METATYPE(Cylinder*)
#endif
