/* Copyright 2011 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float2.h
    @author Jukka Jylänki
    @brief A 2D (x,y) ordered pair. */
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#include <vector>
#endif

#include "Math/MathFwd.h"

#ifdef QT_INTEROP
#include <QVector2D>
#endif

#ifdef OGRE_INTEROP
#include <OgreVector2.h>
#endif

MATH_BEGIN_NAMESPACE

/// A vector of form (x,y).
class float2
{
public:
    enum { Size = 2 };
    float x;
    float y;

    /// This ctor does not initialize the x & y members with a value.
    float2() {}
    /// The copy-ctor for float2 is the trivial copy-ctor, but it is explicitly written to be able to automatically pick up this function for QtScript bindings.
    float2(const float2 &rhs) { x = rhs.x; y = rhs.y; }
    /// Initializes to (x, y).
    float2(float x, float y);

    /// Constructs this float2 from an array. The array must contain at least 2 elements.
    explicit float2(const float *data);

    /// Returns a pointer to first float2 element. The data is contiguous in memory.
    float *ptr();
    /// Returns a pointer to first float2 element. The data is contiguous in memory.
    const float *ptr() const;

    /// Accesses the elements (x,y) using array notation. x: *this[0], y: *this[1].
    CONST_WIN32 float operator [](int index) const;

    /// Accesses the elements (x,y) using array notation. x: *this[0], y: *this[1].
    float &operator [](int index);

    /// Returns the squared length of this vector. Calling this function is faster than calling Length(), so e.g.
    /// instead of comparing the lengths of vectors, you can compare the squared lengths, and avoid expensive square roots.
    float LengthSq() const;
    /// Returns the length of this vector.
    float Length() const;
    /// Normalizes this float2.
    /// Returns the old length of this vector, or 0 if normalization failed. In the case of failure,
    /// this vector is set to (1, 0), so that Normalize() function will never result in an unnormalized vector.
    float Normalize();
    /// Returns a normalized copy of this vector.
    float2 Normalized() const;

    /// Scales this vector so that its new length is as given. This is effectively the same as normalizing the
    /// vector first and then multiplying by newLength.
    float ScaleToLength(float newLength);
	/// Returns a scaled copy of this vector which has its new length as given.
	/// This function assumes the length of this vector is not zero.
	float2 ScaledToLength(float newLength) const;

    /// Tests if the length of this vector is one, up to the given epsilon.
    bool IsNormalized(float epsilonSq = 1e-6f) const;

    /// Tests if this is the null vector, up to the given epsilon.
    bool IsZero(float epsilonSq = 1e-6f) const;

    /// Tests if this vector contains valid finite elements.
    bool IsFinite() const;

    /// Tests if two vectors are perpendicular to each other.
    bool IsPerpendicular(const float2 &other, float epsilon = 1e-3f) const;

    bool Equals(const float2 &rhs, float epsilon = 1e-3f) const;
    bool Equals(float x, float y, float epsilon = 1e-3f) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns "(x, y)".
    std::string ToString() const;

    /// Returns "x y". This is the preferred format for the float2 if it has to be serialized to a string for machine transfer.
    std::string SerializeToString() const;
#endif

    /// Parses a string that is of form "x,y" or "(x,y)" or "(x;y)" or "x y" to a new float2.
    static float2 FromString(const char *str);
#ifdef MATH_ENABLE_STL_SUPPORT
    static float2 FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

    /// Returns x + y.
    float SumOfElements() const;
    /// Returns x * y.
    float ProductOfElements() const;
    /// Returns (x+y)/2.
    float AverageOfElements() const;
    /// Returns Min(x, y).
    float MinElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MinElementIndex() const;
    /// Returns Max(x, y).
    float MaxElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MaxElementIndex() const;
    /// Takes the element-wise absolute value of this vector.
    float2 Abs() const;
    /// Returns a copy of this vector with each element negated.
    /** This function returns a new vector where each element x of the original vector is replaced by the value -x. */
    float2 Neg() const;
    /// Computes the element-wise reciprocal of this vector.
    /** This function returns a new vector where each element x of the original vector is replaced by the value 1/x. */
    float2 Recip() const;
    /// Returns an element-wise minimum of this and the vector (ceil, ceil).
    float2 Min(float ceil) const;
    /// Returns an element-wise minimum of this and the given vector.
    float2 Min(const float2 &ceil) const;
    /// Returns an element-wise maximum of this and the vector (floor, floor).
    float2 Max(float floor) const;
    /// Returns an element-wise maximum of this and the given vector.
    float2 Max(const float2 &floor) const;
    /// Returns a copy of this vector that has floor <= this[i] <= ceil for each element.
    float2 Clamp(float floor, float ceil) const;
    /// Limits each element of this vector between the corresponding elements in floor and ceil.
    float2 Clamp(const float2 &floor, const float2 &ceil) const;
    /// Limits each element of this vector in the range [0, 1].
    float2 Clamp01() const;

    /// Computes the distance between this and the given float2.
    float Distance(const float2 &rhs) const;
    /// Computes the squared distance between this and the given float2.
    float DistanceSq(const float2 &rhs) const;
    /// Computes the dot product of this and the given float2.
    float Dot(const float2 &rhs) const;
    /// Returns this vector with the "perp-operator" applied to it.
    /** The perp operator rotates a vector 90 degrees ccw (around the "z axis"). I.e.
        for a 2D vector (x,y), this function returns the vector (-y, x). */
    float2 Perp() const;
    /// Computes the perp-dot product of this and the given float2 in the order this^perp <dot> rhs.
    float PerpDot(const float2 &rhs) const;

    /// Returns this vector reflected about a plane with the given normal. By convention, both this and the reflected 
    /// vector point away from the plane with the given normal.
    float2 Reflect(const float2 &normal) const;

    /// Refracts this vector about a plane with the given normal. By convention, the this vector points towards the plane, 
    /// and the returned vector points away from the plane.
    /// @param normal Specifies the plane normal direction
    /// @param negativeSideRefractionIndex The refraction index of the material we are exiting.
    /// @param positiveSideRefractionIndex The refraction index of the material we are entering.
    /// When the ray is going from a denser material to a lighter one, total internal reflection can occur.
    /// In this case, this function will just return a reflected vector from a call to Reflect().
    float2 Refract(const float2 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

    /// Projects this vector onto the given unnormalized direction vector.
    float2 ProjectTo(const float2 &direction) const;

    /// Projects this vector onto the given normalized direction vector.
    /// @param direction The vector to project onto. This vector must be normalized.
    float2 ProjectToNorm(const float2 &direction) const;

    /// Returns the angle between this vector and the specified vector, in radians.
    /// @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
    float AngleBetween(const float2 &other) const;

    /// Returns the angle between this vector and the specified normalized vector, in radians.
    /// @note This vector must be normalized to call this function.
    float AngleBetweenNorm(const float2 &normalizedVector) const;

    /// Breaks this vector down into parallel and perpendicular components with respect to the given direction.
    /// @param direction The direction vector along which to decompose. This vector is assumed to be normalized beforehand.
    void Decompose(const float2 &direction, float2 &outParallel, float2 &outPerpendicular) const;

    /// Linearly interpolates between this and the vector b.
    /// @param t The interpolation weight, in the range [0, 1].
    /// Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
    /// Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on.
    float2 Lerp(const float2 &b, float t) const;
    static float2 Lerp(const float2 &a, const float2 &b, float t);

    /// Makes the given vectors linearly independent.
    /// This function adjusts the vector b in-place so that it is orthogonal to the vector a.
    /// The vector a is assumed to be non-zero, but does not have to be normalized.
    /// The vector b may be zero (in which case it stays zero after orthonormalization. Strictly speaking, a and b will not be orthonormal after that).
    /// After this function returns, b may be zero if it was parallel to a to start with.
    static void Orthogonalize(const float2 &a, float2 &b);

    /// Makes the given vectors linearly independent and normalized in length.
    /// The vector a is assumed to be non-zero, but does not have to be normalized.
    /// The vector b may be zero (in which case it stays zero after orthonormalization. Strictly speaking, a and b will not be orthonormal after that).
    /// After this function returns, b may be zero if it was parallel to a to start with.
    static void Orthonormalize(float2 &a, float2 &b);

    /// Generates a new float2 by filling its entries by the given scalar.
    static float2 FromScalar(float scalar);

    /// Fills each entry of this float2 by the given scalar.
    void SetFromScalar(float scalar);

    void Set(float x, float y);

    /// Rotates this vector 90 degrees clock-wise [in place].
    /// This is in a coordinate system on a plane where +x extends to the right, and +y extends upwards.
    void Rotate90CW();

    /// Returns a vector that is perpendicular to this vector (rotated 90 degrees clock-wise).
    float2 Rotated90CW() const;

    /// Rotates this vector 90 degrees counterclock-wise .
    /// This is in a coordinate system on a plane where +x extends to the right, and +y extends upwards.
    void Rotate90CCW();

    /// Returns a vector that is perpendicular to this vector (rotated 90 degrees counter-clock-wise).
    float2 Rotated90CCW() const;

    /// Returns true if the triangle a->b->c is oriented counter-clockwise, when viewed in the XY-plane
    /// where x spans to the right and y spans up.
    /// Another way to think of this is that this function returns true, if the point C lies to the left
    /// of the directed line AB.
    static bool OrientedCCW(const float2 &a, const float2 &b, const float2 &c);

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Computes the 2D convex hull of the given point set.
    static void ConvexHull(const float2 *pointArray, int numPoints, std::vector<float2> &outConvexHull);

    /// Computes the 2D convex hull of the given point set, in-place.
    /// This version of the algorithm works in-place, meaning that when the algorithm finishes,
    /// pointArray will contain the list of the points on the convex hull.
    /// @return The number of points on the convex hull, i.e. the number of elements used in pointArray after the operation.
    static int ConvexHullInPlace(float2 *pointArray, int numPoints);
#endif

    /// Computes the minimum-area rectangle that bounds the given point set. [noscript]
    /// @param center [out] This variable will receive the center point of the rectangle.
    /// @param uDir [out] This variable will receive a normalized direction vector pointing one of the sides of the rectangle.
    /// @param VDir [out] This variable will receive a normalized direction vector pointing the other side of the rectangle.
    static float MinAreaRect(const float2 *pointArray, int numPoints, float2 &center, float2 &uDir, float2 &vDir);

    /// Generates a direction vector of the given length pointing at a uniformly random direction.
    static float2 RandomDir(LCG &lcg, float length = 1.f);

    float2 operator -() const;

    float2 operator +(const float2 &rhs) const;
    float2 operator -(const float2 &rhs) const;
    float2 operator *(float scalar) const;
    float2 operator /(float scalar) const;

    float2 &operator +=(const float2 &rhs);
    float2 &operator -=(const float2 &rhs);
    float2 &operator *=(float scalar);
    float2 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
    float2 operator *(const float2 &rhs) const { return this->Mul(rhs); }
    float2 operator /(const float2 &rhs) const { return this->Div(rhs); }
    float2 &operator *=(const float2 &rhs) { *this = this->Mul(rhs); return *this; }
    float2 &operator /=(const float2 &rhs) { *this = this->Div(rhs); return *this; }
#endif

    float2 Add(const float2 &rhs) const { return *this + rhs; }
    float2 Sub(const float2 &rhs) const { return *this - rhs; }
    float2 Mul(float rhs) const { return *this * rhs; }
    float2 Div(float rhs) const { return *this / rhs; }

    /// Multiplies this vector by rhs *element-wise*.
    float2 Mul(const float2 &rhs) const;
    /// Divides this vector by rhs *element-wise*.
    float2 Div(const float2 &rhs) const;

    /// a compile-time constant float2 with value (0, 0).
    static const float2 zero;
    /// A compile-time constant float2 with value (1, 1).
    static const float2 one;
    /// A compile-time constant float2 with value (1, 0).
    static const float2 unitX;
    /// A compile-time constant float2 with value (0, 1).
    static const float2 unitY;
    /// A compile-time constant float2 with value (NaN, NaN).
    /// For this constant, each element has the value of quiet NaN, or Not-A-Number.
    /// @note Never compare a float2 to this value! Due to how IEEE floats work, for each float x, both the expression "x == nan" and "x == nan" returns false!
    ///       That is, nothing is equal to NaN, not even NaN itself!
    static const float2 nan;
    /// A compile-time constant float2 with value (+infinity, +infinity).
    static const float2 inf;

#ifdef OGRE_INTEROP
    float2(const Ogre::Vector2 &other) { x = other.x; y = other.y; }
    operator Ogre::Vector2() const { return Ogre::Vector2(x, y); }
#endif
#ifdef QT_INTEROP
    float2(const QVector2D &other) { x = other.x(); y = other.y(); }
    operator QVector2D() const { return QVector2D(x, y); }
    operator QString() const { return "float2(" + QString::number(x) + "," + QString::number(y) + ")"; }
    QString toString() const { return (QString)*this; }
    QVector2D ToQVector2D() const { return QVector2D(x, y); }
    static float2 FromQVector2D(const QVector2D &v) { return (float2)v; }
    static float2 FromString(const QString &str) { return FromString(str.toStdString()); }
#endif
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float2 to the given stream.
std::ostream &operator <<(std::ostream &out, const float2 &rhs);
#endif

float2 operator *(float scalar, const float2 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float2 operator /(float scalar, const float2 &rhs) { return float2::FromScalar(scalar) / rhs; }
#endif

inline float Dot(const float2 &a, const float2 &b) { return a.Dot(b); }
inline float2 Abs(const float2 &a) { return a.Abs(); }
inline float2 Min(const float2 &a, const float2 &b) { return a.Min(b); }
inline float2 Max(const float2 &a, const float2 &b) { return a.Max(b); }
inline float2 Clamp(const float2 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp(const float2 &a, const float2 &floor, const float2 &ceil) { return a.Clamp(floor, ceil); }
inline float2 Clamp01(const float2 &a) { return a.Clamp01(); }
inline float2 Lerp(const float2 &a, const float2 &b, float t) { return a.Lerp(b, t); }

MATH_END_NAMESPACE

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(float2)
Q_DECLARE_METATYPE(float2*)
#endif
