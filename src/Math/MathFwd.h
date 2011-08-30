/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/
#pragma once

// The CONST_WIN32 is a #define which resolves to 'const' on Windows, and null on other
// platforms. This #define is used on Windows to detect accidental programming errors
// occurring from an expression "const float3 vec; vec[1] = 5;". Trying to return
// const float from operator[] on GCC gives a warning "type qualifiers ignored on function return type",
// so hence this is only enabled on Visual Studio.
#ifdef _MSC_VER
#define CONST_WIN32 const
#else
#define CONST_WIN32
#endif

#if !defined(MATH_ENABLE_STL_SUPPORT) && !defined(assert)
#include <stdio.h>
#define assert(x) do { if (!(x)) { printf("Error: assert(%s) failed!\n", #x); } } while(0)
#endif

class float2;
class float3;
class float4;
class float2x2;
class float2x3;
class float3x3;
class float3x4;
class float4x4;
class Quat;

class TranslateOp;
class ScaleOp;

class AABB;
class Capsule;
class Circle;
class Complex;
class Cone;
class Cylinder;
class Ellipsoid;
class Frustum;
struct HitInfo;
class Line;
class LineSegment;
class LCG;
class OBB;
class Plane;
class Polygon;
class Polyhedron;
class Polynomial;
class Quat;
class Ray;
class Rect;
class Sphere;
class TranslateOp;
class Torus;
class ScaleOp;
class Triangle;
