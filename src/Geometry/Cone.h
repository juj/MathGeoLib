/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

#include "Math/MathFwd.h"
#include "Math/float3.h"

/// A 3D pyramid with a circle-shaped base.
class Cone
{
public:
    Cone() {}

    bool Contains(const float3 &point) const;
};

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(Cone)
Q_DECLARE_METATYPE(Cone*)
#endif
