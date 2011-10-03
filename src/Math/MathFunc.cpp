/** @file MathFunc.cpp
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief Common mathematical functions.
*/

#ifdef MATH_ENABLE_STL_SUPPORT
#include <utility>
#include <algorithm>
#endif

#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

bool mathBreakOnAssume = false;

void SetMathBreakOnAssume(bool isEnabled)
{
    mathBreakOnAssume = isEnabled;
}

/// Returns the current state of the math break-on-assume flag.
bool MathBreakOnAssume()
{
    return mathBreakOnAssume;
}

/** Uses a recursive approach, not the fastest/brightest method.
    Note that 13! = 6227020800 overflows already.
    @return n! = n * (n-1) * (n-2) * ... * 1. */
int Factorial(int n)
{
    int result = 1;
    for(int i = 2; i <= n; i++)
        result *= i;
    return result;
}

/** @return Binomial coefficients with recursion, i.e. n choose k, C(n,k) or nCk. */
int CombinatorialRec(int n, int k)
{
    /* We could do: 
            return factorial(n)/(factorial(n-k)*factorial(k));
        But prefer the recursive approach instead, because it's not so prone
        to numerical overflow. This approach uses the idea of the Pascal triangle. */

    if (k <= 0 || k >= n)
        return 1;
    else
        return CombinatorialRec(n-1,k-1) + CombinatorialRec(n-1,k);
}

/** @return Binomial coefficients by tabulation, i.e. n choose k, C(n,k) or nCk. */
int CombinatorialTab(int n, int k)
{
    if (k == 0 || k == n)
        return 1;
    if (k < 0 || k > n)
        return 0;
    // We use two auxiliary tables, one of size n-2 and one of size n-3.
    int *table = new int[2*(k+1)]; ///\todo We can lower this size.
    for(int i = 0; i < 2*(k+1); ++i)
        table[i] = 1;
    int *t1 = &table[0];
    int *t2 = &table[k+1];
    // Iteratively fill the tables.
    for(int i = 2; i <= n; ++i)
    {
        for(int j = Max(1, i-n+k); j <= Min(k,i-1); ++j)
            t1[j] = t2[j] + t2[j-1];
        Swap(t1, t2);
    }
    int c = t2[k];
    delete[] table;
    return c;
}

float PowUInt(float base, u32 exponent)
{
    // 'Fast Exponentiation': We interpret exponent in base two and calculate the power by
    // squaring and multiplying by base.

    // Find the highest bit that is set.
    u32 e = 0x80000000;
    while((exponent & e) == 0 && e > 0)
        e >>= 1;

    float val = 1.f;
    do
    {
        val *= val; // Shifts the exponent one place left
        val *= (exponent & e) != 0 ? base : 1.f; // Adds a 1 as the LSB of the exponent
        e >>= 1;
    } while(e > 0);

    return val;
}

/** @param base Exponent base value.
    @param exponent Integer exponent to raise base to.
    @return pow(base,exponent) but optimized because we only use integer exponent. */
float PowInt(float base, int exponent)
{
    if (exponent == 0)
        return 1.f;
    else if (exponent < 0)
        return 1.f / PowUInt(base, (u32)-exponent);
    else
        return PowUInt(base, (u32)exponent);
}

MATH_END_NAMESPACE
