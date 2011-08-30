/** @file Polynomial.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief
*/
#pragma once

class Polynomial
{
public:

    /// Solves a quadratic equation ax^2 + bx + c = 0 for x. Returns the number of roots found.
    static int SolveQuadratic(float a, float b, float c, float &root1, float &root2);

    /// Solves a cubic equation ax^3 + bx^2 + cx + d = 0 for x. Returns the number of roots found.
    static int SolveCubic(float a, float b, float c, float d, float &root1, float &root2, float &root3);

    /// Solves a quartic equation ax^4 + bx^3 + cx^2 + dx + e = 0 for x. Returns the number of roots found.
    static int SolveQuartic(float a, float b, float c, float d, float &root1, float &root2, float &root3, float &root4);

    // @todo add Newton's method functions.
};

