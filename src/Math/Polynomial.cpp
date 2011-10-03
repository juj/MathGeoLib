#include "Math/Polynomial.h"
#include "Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

int Polynomial::SolveQuadratic(float a, float b, float c, float &root1, float &root2)
{
    // ax^2 + bx + c == 0 => x = [ -b +/- sqrt(b^2 - 4ac) ] / 2a.

    ///\todo numerical float issues: catastrophic cancellation can occur in the subtraction.
    float radicand = b*b - 4.f * a * c;
    if (radicand < -1e-6f) // Add a small epsilon to allow the radicand to be slightly zero.
        return 0;
    float denom = 1.f / (2.f * a);
    if (radicand < 1e-6f) // Consider the radicand to be zero, and hence only one solution.
    {
        root1 = -b * denom; 
        return 1;
    }
    radicand = Sqrt(radicand);
    root1 = -b + radicand * denom;
    root2 = -b - radicand * denom;
    return 2;
}

int Polynomial::SolveCubic(float a, float b, float c, float d, float &root1, float &root2, float &root3)
{
    assume(false && "Not implemented!");
    return 0;
}

int Polynomial::SolveQuartic(float a, float b, float c, float d, float &root1, float &root2, float &root3, float &root4)
{
    assume(false && "Not implemented!");
    return 0;
}

MATH_END_NAMESPACE
