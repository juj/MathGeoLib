/** @file MathOps.cpp
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/

#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

/** Compares the two values for equality, allowing the given amount of absolute error. */
bool EqualAbs(float a, float b, float epsilon)
{
    return Abs(a-b) < epsilon;
}

bool EqualRel(float a, float b, float maxRelError)
{
    if (a == b) return true; // Handles the special case where a and b are both zero.
    float relativeError = Abs((a-b)/Max(a, b));
    return relativeError <= maxRelError;
}

bool EqualUlps(float a, float b, int maxUlps)
{
    assert(sizeof(float) == sizeof(int));
    assert(maxUlps >= 0);
    assert(maxUlps < 4 * 1024 * 1024);

    int intA = *(int*)&a;
    if (intA < 0) intA = 0x80000000 - intA;
    int intB = *(int*)&b;
    if (intB < 0) intB = 0x80000000 - intB;
    if (Abs(intA - intB) <= maxUlps)
        return true;
    return false;
}

MATH_END_NAMESPACE
