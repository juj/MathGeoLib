/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

#include "Math/MathFwd.h"

/// A proxy class for double brackets [][] element access in matrices.
template<int Cols>
class MatrixProxy
{
private:
    float v[Cols];

public:
    CONST_WIN32 float operator[](int col) const
    {
        assert(col >= 0);
        assert(col < Cols);

        return v[col];
    }
    float &operator[](int col)
    {
        assert(col >= 0);
        assert(col < Cols);

        return v[col];
    }
};
