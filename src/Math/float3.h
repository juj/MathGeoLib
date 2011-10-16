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

/** @file float3.h
    @author Jukka Jylänki
    @brief A 3D (x,y,z) ordered tuple. */
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#endif

#include "Math/MathFwd.h"

#ifdef MATH_QT_INTEROP
#include <QVector3D>
#endif
/*
#ifdef MATH_IRRLICHT_INTEROP
#include "Math/float3.h"
#endif
*/
#ifdef MATH_OGRE_INTEROP
#include <OgreVector3.h>
#endif

#ifdef MATH_BULLET_INTEROP
#include "LinearMath/btVector3.h"
#endif

MATH_BEGIN_NAMESPACE

/// A vector of form (x,y,z).
class float3
{
public:
    enum { Size = 3 };
    float x;
    float y;
    float z;

    /// This ctor does not initialize the x, y & z members with any value.
    float3() {}
    /// The copy-ctor for float3 is the trivial copy-ctor, but it is explicitly written to be able to automatically pick up this function for QtScript bindings.
    float3(const float3 &rhs) { x = rhs.x; y = rhs.y; z = rhs.z; }
    /// Initializes to (x, y, z).
    float3(float x, float y, float z);
	 /// Initializes to (scalar, scalar, scalar).
	 explicit float3(float scalar);

    float3(const float2 &xy, float z);

    /// Constructs this float3 from an array. The array must contain at least 3 elements. [noscript]
    explicit float3(const float *data);

    /// Returns a pointer to first float3 element. The data is contiguous in memory. [noscript]
    float *ptr();
    /// Returns a pointer to first float3 element. The data is contiguous in memory. [noscript]
    const float *ptr() const;

    /// Returns the given element of this vector using array notation.
    /// x: this[0], y: this[1], z: this[2].
    /// @note Returns a *const* value to avoid accidental copying to a temporary.
    CONST_WIN32 float operator [](int index) const;

    /// Accesses the given element of this vector using array notation.
    /// x: this[0], y: this[1], z: this[2].
    /// @note You can use this notation to set the elements of this vector as well, e.g.
    /// vec[1] = 10.f; would set the y-component of this vector.
    float &operator [](int index);

    /// Returns the (x, y) part of this vector.
    float2 xy() const;

    /// Returns the squared length of this vector. Calling this function is faster than calling Length(), so e.g.
    /// instead of comparing the lengths of vectors, you can compare the squared lengths, and avoid expensive square roots.
    float LengthSq() const;
    /// Returns the length of this vector.
    float Length() const;
    /// Normalizes this float3.
    /// Returns the old length of this vector, or 0 if normalization failed. In the case of failure,
    /// this vector is set to (1, 0, 0), so that Normalize() function will never result in an unnormalized vector.
    float Normalize();
    /// Returns a normalized copy of this vector. If the vector is zero and cannot be normalized, the vector (1, 0, 0) is returned.
    float3 Normalized() const;
    /// Scales this vector so that its new length is as given. This is effectively the same as normalizing the
    /// vector first and then multiplying by newLength.
    float ScaleToLength(float newLength);
	/// Returns a scaled copy of this vector which has its new length as given.
	/// This function assumes the length of this vector is not zero.
	float3 ScaledToLength(float newLength) const;

    /// Tests if the length of this vector is one, up to the given epsilon.
    bool IsNormalized(float epsilonSq = 1e-6f) const;

    /// Tests if this is the null vector, up to the given epsilon.
    bool IsZero(float epsilonSq = 1e-6f) const;

    /// Tests if this vector contains valid finite elements.
    bool IsFinite() const;

    /// Tests if two vectors are perpendicular to each other.
    bool IsPerpendicular(const float3 &other, float epsilon = 1e-3f) const;

    bool Equals(const float3 &rhs, float epsilon = 1e-3f) const;
    bool Equals(float x, float y, float z, float epsilon = 1e-3f) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns "(x, y, z)". [noscript]
    std::string ToString() const;

    /// Returns "x y z". This is the preferred format for the float3 if it has to be serialized to a string for machine transfer.
    std::string SerializeToString() const;
#endif

    /// Parses a string that is of form "x,y,z" or "(x,y,z)" or "(x;y;z)" or "x y z" to a new float3.
    static float3 FromString(const char *str);
#ifdef MATH_ENABLE_STL_SUPPORT
    static float3 FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

    /// Returns x + y + z.
    float SumOfElements() const;
    /// Returns x * y * z.
    float ProductOfElements() const;
    /// Returns the average of x, y and z.
    float AverageOfElements() const;
    /// Returns Min(x, y, z).
    float MinElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MinElementIndex() const;
    /// Returns Max(x, y, z).
    float MaxElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MaxElementIndex() const;
    /// Takes the element-wise absolute value of this vector.
    float3 Abs() const;
    /// Returns a copy of this vector with each element negated.
    /** This function returns a new vector where each element x of the original vector is replaced by the value -x. */
    float3 Neg() const;
    /// Computes the element-wise reciprocal of this vector.
    /** This function returns a new vector where each element x of the original vector is replaced by the value 1/x. */
    float3 Recip() const;
    /// Returns an element-wise minimum of this and the vector (ceil, ceil, ceil).
    float3 Min(float ceil) const;
    /// Returns an element-wise minimum of this and the given vector.
    float3 Min(const float3 &ceil) const;
    /// Returns an element-wise maximum of this and the vector (floor, floor, floor).
    float3 Max(float floor) const;
    /// Returns an element-wise maximum of this and the given vector.
    float3 Max(const float3 &floor) const;
    /// Limits each element of this vector between the corresponding elements in floor and ceil.
    float3 Clamp(const float3 &floor, const float3 &ceil) const;
    /// Limits each element of this vector in the range [0, 1].
    float3 Clamp01() const;
    /// Returns a vector that has floor <= this[i] <= ceil for each element.
    float3 Clamp(float floor, float ceil) const;

    /// Computes the distance between this and the given float3.
    float Distance(const float3 &rhs) const;
    /// Computes the distance between this and the given object.
    float Distance(const Line &rhs) const;
    float Distance(const Ray &rhs) const;
    float Distance(const LineSegment &rhs) const;
    float Distance(const Plane &rhs) const;
    float Distance(const Triangle &rhs) const;
    float Distance(const AABB &rhs) const;
    float Distance(const OBB &rhs) const;
    float Distance(const Sphere &rhs) const;
    float Distance(const Capsule &rhs) const;
    /// Computes the squared distance between this and the given float3.
    float DistanceSq(const float3 &rhs) const;
    /// Computes the dot product of this and the given float3.
    float Dot(const float3 &rhs) const;
    /// Computes the cross product of this and the given vector.
    float3 Cross(const float3 &rhs) const;
    /// Computes the outer product of this and the given vector.
    float3x3 OuterProduct(const float3 &rhs) const;

    /// Returns a new normalized direction vector that is perpendicular to this vector and the specified hint vector.
    /// If this vector points toward the hint vector, the vector hint2 is returned instead.
    float3 Perpendicular(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;
    /// Returns another vector that is perpendicular to this vector and the vector returned by Perpendicular().
    /// The set (this, Perpendicular(), AnotherPerpendicular()) forms a right-handed normalized 3D basis.
    float3 AnotherPerpendicular(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;

    /// Computes [u v w] = (u x v) . w = u . (v x w)
    static float ScalarTripleProduct(const float3 &u, const float3 &v, const float3 &w);

    /// Returns this vector reflected about a plane with the given normal. By convention, both this and the reflected 
    /// vector point away from the plane with the given normal.
    float3 Reflect(const float3 &normal) const;

    /// Refracts this vector about a plane with the given normal. By convention, the this vector points towards the plane, 
    /// and the returned vector points away from the plane.
    /// @param normal Specifies the plane normal direction
    /// @param negativeSideRefractionIndex The refraction index of the material we are exiting.
    /// @param positiveSideRefractionIndex The refraction index of the material we are entering.
    /// When the ray is going from a denser material to a lighter one, total internal reflection can occur.
    /// In this case, this function will just return a reflected vector from a call to Reflect().
    float3 Refract(const float3 &normal, float negativeSideRefractionIndex, float positiveSideRefractionIndex) const;

    /// Projects this vector onto the given unnormalized direction vector.
    float3 ProjectTo(const float3 &direction) const;

    /// Projects this vector onto the given normalized direction vector.
    /// @param direction The vector to project onto. This vector must be normalized.
    float3 ProjectToNorm(const float3 &direction) const;

    /// Returns the angle between this vector and the specified vector, in radians.
    /// @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
    float AngleBetween(const float3 &other) const;

    /// Returns the angle between this vector and the specified normalized vector, in radians.
    /// @note This vector must be normalized to call this function.
    float AngleBetweenNorm(const float3 &normalizedVector) const;

    /// Breaks this vector down into parallel and perpendicular components with respect to the given direction.
    void Decompose(const float3 &direction, float3 &outParallel, float3 &outPerpendicular) const;

    /// Linearly interpolates between this and the vector b.
    /// @param t The interpolation weight, in the range [0, 1].
    /// Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
    /// Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on.
    float3 Lerp(const float3 &b, float t) const;
    static float3 Lerp(const float3 &a, const float3 &b, float t);

    /// Makes the given vectors linearly independent.
    /// The vector a is kept unmodified, and vector b is modified to be perpendicular to a.
    /// Note that if either vector is zero, then the resulting two vectors are not orthogonal.
    static void Orthogonalize(const float3 &a, float3 &b);

    /// Makes the given vectors linearly independent.
    /// The vector a is kept unmodified, and vector b is modified to be perpendicular to a.
    /// Finally, vector c is adjusted to be perpendicular to a and b.
    /// Note that if any of the vectors is zero, then the resulting set of vectors are not orthogonal.
    static void Orthogonalize(const float3 &a, float3 &b, float3 &c);

    /// Returns true if the given vectors are orthogonal to each other.
    static bool AreOrthogonal(const float3 &a, const float3 &b, float epsilon = 1e-3f);
    static bool AreOrthogonal(const float3 &a, const float3 &b, const float3 &c, float epsilon = 1e-3f);

    /// Makes the given vectors linearly independent and normalized in length.
    static void Orthonormalize(float3 &a, float3 &b);

    /// Makes the given vectors linearly independent and normalized in length.
    static void Orthonormalize(float3 &a, float3 &b, float3 &c);

    /// Returns true if the given vectors are orthogonal to each other and all of length 1.
    static bool AreOrthonormal(const float3 &a, const float3 &b, float epsilon = 1e-3f);
    static bool AreOrthonormal(const float3 &a, const float3 &b, const float3 &c, float epsilon = 1e-3f);

    /// Generates a new float3 by filling its entries by the given scalar.
    static float3 FromScalar(float scalar);

    /// Fills each entry of this float3 by the given scalar.
    void SetFromScalar(float scalar);

    void Set(float x, float y, float z);

    /// Returns float4(x,y,z,1).
    float4 ToPos4() const;

    /// Returns float4(x,y,z,0).
    float4 ToDir4() const;

    /// Generates a direction vector of the given length pointing at a uniformly random direction.
    static float3 RandomDir(LCG &lcg, float length = 1.f);
    /// Generates a random point inside a sphere.
    static float3 RandomSphere(LCG &lcg, const float3 &center, float radius);
    /// Generates a random point inside an axis-aligned box.
    static float3 RandomBox(LCG &lcg, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
    static float3 RandomBox(LCG &lcg, const float3 &minValues, const float3 &maxValues);

    float3 operator -() const;

    float3 operator +(const float3 &rhs) const;
    float3 operator -(const float3 &rhs) const;
    float3 operator *(float scalar) const;
    float3 operator /(float scalar) const;

    float3 &operator +=(const float3 &rhs);
    float3 &operator -=(const float3 &rhs);
    float3 &operator *=(float scalar);
    float3 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
    float3 operator *(const float3 &rhs) const { return this->Mul(rhs); }
    float3 operator /(const float3 &rhs) const { return this->Div(rhs); }
    float3 &operator *=(const float3 &rhs) { *this = this->Mul(rhs); return *this; }
    float3 &operator /=(const float3 &rhs) { *this = this->Div(rhs); return *this; }
#endif

    float3 Add(const float3 &rhs) const { return *this + rhs; }
    float3 Sub(const float3 &rhs) const { return *this - rhs; }
    float3 Mul(float rhs) const { return *this * rhs; }
    float3 Div(float rhs) const { return *this / rhs; }

    /// Multiplies this vector by rhs *element-wise*.
    float3 Mul(const float3 &rhs) const;
    /// Divides this vector by rhs *element-wise*.
    float3 Div(const float3 &rhs) const;

    /// Specifies a compile-time constant float3 with value (0, 0, 0).
    static const float3 zero;
    /// Specifies a compile-time constant float3 with value (1, 1, 1).
    static const float3 one;
    /// Specifies a compile-time constant float3 with value (1, 0, 0).
    static const float3 unitX;
    /// Specifies a compile-time constant float3 with value (0, 1, 0).
    static const float3 unitY;
    /// Specifies a compile-time constant float3 with value (0, 0, 1).
    static const float3 unitZ;
    /// A compile-time constant float3 with value (NaN, NaN, NaN).
    /// For this constant, each element has the value of quiet NaN, or Not-A-Number.
    /// @note Never compare a float3 to this value! Due to how IEEE floats work, for each float x, both the expression "x == nan" and "x == nan" returns false!
    ///       That is, nothing is equal to NaN, not even NaN itself!
    static const float3 nan;
    /// A compile-time constant float3 with value (+infinity, +infinity, +infinity).
    static const float3 inf;

#ifdef MATH_OGRE_INTEROP
    float3(const Ogre::Vector3 &other) { x = other.x; y = other.y; z = other.z; }
    operator Ogre::Vector3() const { return Ogre::Vector3(x, y, z); }
#endif
/*
#ifdef MATH_IRRLICHT_INTEROP
    float3(const Vector3df &other) { x = other.x; y = other.y; z = other.z; }
    operator Vector3df() const { return Vector3df(x, y, z); }
#endif
*/
#ifdef MATH_QT_INTEROP
    float3(const QVector3D &other) { x = other.x(); y = other.y(); z = other.z(); }
    operator QVector3D() const { return QVector3D(x, y, z); }
    operator QString() const { return "float3(" + QString::number(x) + "," + QString::number(y) + "," + QString::number(z) + ")"; }
    QString toString() const { return (QString)*this; }
    QVector3D ToQVector3D() const { return QVector3D(x, y, z); }
    static float3 FromQVector3D(const QVector3D &v) { return (float3)v; }
    static float3 FromString(const QString &str) { return FromString(str.toStdString()); }
#endif
#ifdef MATH_BULLET_INTEROP
    float3(const btVector3 &other) { x = other.x(); y = other.y(); z = other.z(); }
    operator btVector3() const { return btVector3(x, y, z); }
#endif
};

/// Prints this float3 to the given stream.
#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3 &rhs);
#endif

float3 operator *(float scalar, const float3 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float3 operator /(float scalar, const float3 &rhs) { return float3::FromScalar(scalar) / rhs; }
#endif

inline float Dot(const float3 &a, const float3 &b) { return a.Dot(b); }
inline float3 Cross(const float3 &a, const float3 &b) { return a.Cross(b); }
inline float3 Abs(const float3 &a) { return a.Abs(); }
inline float3 Min(const float3 &a, const float3 &b) { return a.Min(b); }
inline float3 Max(const float3 &a, const float3 &b) { return a.Max(b); }
inline float3 Clamp(const float3 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float3 Clamp(const float3 &a, const float3 &floor, const float3 &ceil) { return a.Clamp(floor, ceil); }
inline float3 Clamp01(const float3 &a) { return a.Clamp01(); }
inline float3 Lerp(const float3 &a, const float3 &b, float t) { return a.Lerp(b, t); }

MATH_END_NAMESPACE

#ifdef MATH_QT_INTEROP
Q_DECLARE_METATYPE(float3)
Q_DECLARE_METATYPE(float3*)
#endif
