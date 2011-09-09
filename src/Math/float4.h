/** @file float4.h
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief A 4D (x,y,z,w) homogeneous vector.
*/
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <string>
#endif
#include "Math/MathFwd.h"
#include "Math/float3.h"

#ifdef QT_INTEROP
#include <QVector4D>
#endif

#ifdef OGRE_INTEROP
#include <OgreVector4.h>
#endif

/// A 3D vector of form (x,y,z,w) in a 4D homogeneous coordinate space.
/** This class has two sets of member functions. The functions ending in a suffix '3' operate only on the
    (x, y, z) part, ignoring the w component (or assuming a value of 0 or 1, where expectable). The functions
    without the '3' suffix operate on all four elements of the vector. */
class float4
{
public:
    enum
    {
        /// Specifies the number of elements in this vector.
        Size = 4
    };
    float x;
    float y;
    float z;
    float w;

    /// @note This ctor does not initialize the x, y, z & w members with any value.
    float4() {}

    /// The copy-ctor for float4 is the trivial copy-ctor, but it is explicitly written to be able to automatically pick up this function for QtScript bindings.
    float4(const float4 &rhs) { x = rhs.x; y = rhs.y; z = rhs.z; w = rhs.w; }

    /// Initializes to (x, y, z, w).
    float4(float x, float y, float z, float w);

    float4(const float3 &xyz, float w);

    float4(const float2 &xy, float z, float w);

    /// Constructs this float4 from an array. The array must contain at least 4 elements.
    explicit float4(const float *data);

    /// Returns a pointer to first float4 element. The data is contiguous in memory.
    float *ptr();

    /// Returns a pointer to first float4 element. The data is contiguous in memory.
    const float *ptr() const;

    CONST_WIN32 float operator [](int index) const;

    float &operator [](int index);

    /// Returns the (x, y, z) part of this vector.
    float3 xyz() const;

    /// Returns the squared length of the (x,y,z) part of this vector. Calling this function is faster than calling 
    /// Length3(), so e.g. instead of comparing the lengths of vectors, you can compare the squared lengths, and avoid 
    /// expensive square roots. This function ignores the w component of this vector.
    float LengthSq3() const;
    /// Returns the length of the (x, y, z) part of this vector. This function ignores the w component of this vector.
    float Length3() const;
    /// Returns the squared length of this vector, taking into account the w component.
    /// Calling this function is faster than calling Length4(), so e.g. instead of comparing the lengths of vectors, 
    /// you can compare the squared lengths, and avoid expensive square roots.
    float LengthSq4() const;
    /// Returns the length of this vector, taking into account the w component.
    float Length4() const;
    /// Normalizes the (x, y, z) part of this vector. This function ignores the w component of this vector.
    /// Returns the old length of this vector, or 0 if normalization failed.
    float Normalize3();
    /// Normalizes this vector.
    /// Returns the old length of this vector, or 0 if normalization failed.
    float Normalize4();
    /// Returns a copy of this vector where the (x, y, z) part is normalized, and w is carried over unchanged.
    float4 Normalized3() const;
    /// Returns a normalized copy of this vector.
    float4 Normalized4() const;

    /// Divides each element by w to produce a float4 of form (x, y, z, 1). Call this function only if w is nonzero.
    bool NormalizeW();
    /// Returns true if the w component of this float4 is either 0 or 1. This is a required condition for several
    /// functions to work correctly.
    bool IsWZeroOrOne(float epsilon = 1e-3f) const;
    /// Tests if the (x, y, z) part of this vector is equal to (0,0,0), up to the given epsilon.
    bool IsZero3(float epsilonSq = 1e-6f) const;
    /// Returns true if this vector is equal to (0,0,0,0), up to the given epsilon.
    bool IsZero4(float epsilonSq = 1e-6f) const;
    /// Tests if the length of the (x, y, z) part of this vector is one, up to the given epsilon.
    bool IsNormalized3(float epsilonSq = 1e-6f) const;
    /// Returns true if the length of this vector is 1, up to the given epsilon.
    /// This function takes into account all the four components of this vector when calculating the norm.
    bool IsNormalized4(float epsilonSq = 1e-6f) const;
    /// Multiplies the (x, y, z) part of this vector by the given scalar.
    void Scale3(float scalar);
    /// Scales the (x, y, z) part of this vector so that its new length is as given. This is effectively the same as 
    /// normalizing the vector first and then multiplying by newLength.
    /// @return False if the length of this vector is zero, and the vector could not be scaled to the specified length.
    float ScaleToLength3(float newLength);
	/// Returns a scaled copy of this vector which has its new length as given.
	/// This function assumes the length of this vector is not zero.
	float4 ScaledToLength3(float newLength) const;

    /// Tests if this vector contains valid finite elements.
    bool IsFinite() const;

    /// Tests if the (x, y, z) parts of two vectors are perpendicular to each other.
    bool IsPerpendicular3(const float4 &other, float epsilon = 1e-6f) const;

#ifdef MATH_ENABLE_STL_SUPPORT
    /// Returns "(x, y, z, w)".
    std::string ToString() const;
    /// Returns "x y z w". This is the preferred format for the float4 if it has to be serialized to a string for machine transfer.
    std::string SerializeToString() const;
#endif

    /// Parses a string that is of form "x,y,z,w" or "(x,y,z,w)" or "(x;y;z;w)" or "x y z w" to a new float4.
    static float4 FromString(const char *str);
#ifdef MATH_ENABLE_STL_SUPPORT
    static float4 FromString(const std::string &str) { return FromString(str.c_str()); }
#endif

    /// Returns x + y + z + w.
    float SumOfElements() const;
    /// Returns x * y * z * w.
    float ProductOfElements() const;
    /// Returns the average of x, y, z, w.
    float AverageOfElements() const;
    /// Returns Min(x, y, z, w).
    float MinElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MinElementIndex() const;
    /// Returns Max(x, y, z, w).
    float MaxElement() const;
    /// Returns the index that has the smallest value in this vector.
    int MaxElementIndex() const;
    /// Takes the element-wise absolute value of this vector.
    float4 Abs() const;
    /// Returns a copy of this vector with the x, y and z elements negated.
    float4 Neg3() const;
    /// Returns a copy of this vector with each element negated.
    float4 Neg4() const;
    /// Computes the element-wise reciprocal of the three first elements of this vector.
    /** This function returns a new vector where the x, y and z elements of the original vector are replaced by the values 1/x, 1/y and 1/z. */
    float4 Recip3() const;
    /// Computes the element-wise reciprocal of this vector.
    /** This function returns a new vector where each element x of the original vector is replaced by the value 1/x. This function operates on all four elements of this vector. */
    float4 Recip4() const;
    /// Returns an element-wise minimum of this and the vector (ceil, ceil, ceil, ceil).
    float4 Min(float ceil) const;
    /// Returns an element-wise minimum of this and the given vector.
    float4 Min(const float4 &ceil) const;
    /// Returns an element-wise maximum of this and the vector (floor, floor, floor, floor).
    float4 Max(float floor) const;
    /// Returns an element-wise maximum of this and the given vector.
    float4 Max(const float4 &floor) const;
    /// Limits each element of this vector between the corresponding elements in floor and ceil.
    float4 Clamp(const float4 &floor, const float4 &ceil) const;
    /// Limits each element of this vector in the range [0, 1].
    float4 Clamp01() const;
    /// Returns a vector that has floor <= this[i] <= ceil for each element.
    float4 Clamp(float floor, float ceil) const;

    /// Linearly interpolates between this and the vector b.
    /// This function assumes that the w components of this and the other vector are equal.
    /// @param t The interpolation weight, in the range [0, 1].
    /// Lerp(b, 0) returns this vector, Lerp(b, 1) returns the vector b.
    /// Lerp(b, 0.5) returns the vector half-way in between the two vectors, and so on.
    float4 Lerp(const float4 &b, float t) const;
    static float4 Lerp(const float4 &a, const float4 &b, float t);

    /// Computes the squared distance between the (x, y, z) parts of this and the given float4. 
    /// @note This function ignores the w component of this and rhs vector (assumes w=0 or w=1 are the same for both vectors).
    float Distance3Sq(const float4 &rhs) const;
    /// Computes the distance between the (x, y, z) parts of this and the given float4. 
    /// @note This function ignores the w component of this and rhs vector (assumes w=0 or w=1 are the same for both vectors).
    float Distance3(const float4 &rhs) const;
    /// Computes the dot product of the (x, y, z) parts of this and the given float4.
    /// @note This function ignores the w component of this vector (assumes w=0).
    float Dot3(const float3 &rhs) const;
    float Dot3(const float4 &rhs) const;
    /// Computes the dot product of this and the given float4, taking into account the w component.
    float Dot4(const float4 &rhs) const;
    /// Computes the cross product of the (x, y, z) parts of this and the given vector. Returns a vector with w=0.
    float4 Cross3(const float3 &rhs) const;
    float4 Cross3(const float4 &rhs) const;

    /// Computes the outer-product of this and the given vector.
    float4x4 OuterProduct(const float4 &rhs) const;

    /// Returns a new normalized direction vector that points as close as possible towards the given hint vector.
    float4 Perpendicular3(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;
    /// Returns another vector that is perpendicular to this vector and the vector returned by Perpendicular3(hint).
    /// @todo Enforce that (x: this, y: Perpendicular3(), z: AnotherPerpendicular3) form a right-handed basis.
    float4 AnotherPerpendicular3(const float3 &hint = float3(0,1,0), const float3 &hint2 = float3(0,0,1)) const;
    /// Returns this vector reflected about a plane with the given normal. By convention, both this and the reflected 
    /// vector point away from the plane with the given normal.
    /// @note This function ignores the w component of this vector (assumes w=0).
    float4 Reflect3(const float3 &normal) const;
    /// Returns the angle between this vector and the specified vector, in radians.
    /// @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
    /// @note This function ignores the w component of this vector (assumes w=0).
    float AngleBetween3(const float4 &other) const;
    /// Returns the angle between this vector and the specified normalized vector, in radians.
    /// @note This vector must be normalized to call this function.
    /// @note This function ignores the w component of this vector (assumes w=0).
    float AngleBetweenNorm3(const float4 &normalizedVector) const;
    /// Returns the angle between this vector and the specified vector, in radians.
    /// @note This function takes into account that this vector or the other vector can be unnormalized, and normalizes the computations.
    float AngleBetween4(const float4 &other) const;
    /// Returns the angle between this vector and the specified normalized vector, in radians.
    /// @note This vector must be normalized to call this function.
    float AngleBetweenNorm4(const float4 &normalizedVector) const;
    /// Projects this vector onto the given vector.
    /// @note This function treats this and target vector as direction vectors.
    /// @note This function ignores the w component of this vector (assumes w=0 or 1) and returns it unmodified.
    float4 ProjectTo3(const float3 &target) const;

    /// Projects this vector onto the given vector.
    /// @param target The direction vector to project onto. This vector must be normalized.
    /// @note This function treats this and target vector as direction vectors.
    /// @note This function ignores the w component of this vector (assumes w=0 or 1) and returns it unmodified.
    float4 ProjectToNorm3(const float3 &target) const;

    /// Returns float4(scalar, scalar, scalar, scalar).
    static float4 FromScalar(float scalar);
    /// Returns float4(scalar, scalar, scalar, w).
    static float4 FromScalar(float scalar, float w);

    /// Sets this float4 to (scalar, scalar, scalar, scalar).
    void SetFromScalar(float scalar);
    /// Sets this float4 to (scalar, scalar, scalar, w).
    void SetFromScalar(float scalar, float w);

    void Set(float x, float y, float z, float w);

    /// Returns true if this vector is equal to the given vector, up to given per-element epsilon.
    bool Equals(const float4 &other, float epsilon = 1e-3f) const;

    bool Equals(float x, float y, float z, float w, float epsilon = 1e-3f) const;

    /// Generates a direction vector of the given length pointing at a uniformly random direction.
    /// The w-component for the returned vector is 0.
    static float4 RandomDir(LCG &lcg, float length = 1.f);

    /// Sums all four components elementwise.
    float4 operator +(const float4 &rhs) const;
    /// Computes the difference from all four components elementwise.
    float4 operator -(const float4 &rhs) const;
    /// Negates vector.
    float4 operator -() const;
    /// Multiplies the x, y, z and w components of this vector by the given scalar. Note that if w != 0, 
    /// this does NOT scale the length of the homogeneous 3D vector.
    float4 operator *(float scalar) const;
    /// Divides the x, y, z and w components of this vector by the given scalar. Note that if w != 0, 
    /// this does NOT scale the length of the homogeneous 3D vector.
    float4 operator /(float scalar) const;

    float4 &operator +=(const float4 &rhs);
    float4 &operator -=(const float4 &rhs);
    /// Multiplies the x, y, z and w components of this vector by the given scalar. Note that if w != 0, 
    /// this does NOT scale the length of the homogeneous 3D vector.
    float4 &operator *=(float scalar);
    /// Divides the x, y, z and w components of this vector by the given scalar. Note that if w != 0, 
    /// this does NOT scale the length of the homogeneous 3D vector.
    float4 &operator /=(float scalar);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
    float4 operator *(const float4 &rhs) const { return this->Mul(rhs); }
    float4 operator /(const float4 &rhs) const { return this->Div(rhs); }
    float4 &operator *=(const float4 &rhs) { *this = this->Mul(rhs); return *this; }
    float4 &operator /=(const float4 &rhs) { *this = this->Div(rhs); return *this; }
#endif

    /// Specifies a compile-time constant float4 with value (0, 0, 0, 0).
    static const float4 zero;
    /// Specifies a compile-time constant float4 with value (1, 1, 1, 1).
    static const float4 one;
    /// Specifies a compile-time constant float4 with value (1, 0, 0, 0).
    static const float4 unitX;
    /// Specifies a compile-time constant float4 with value (0, 1, 0, 0).
    static const float4 unitY;
    /// Specifies a compile-time constant float4 with value (0, 0, 1, 0).
    static const float4 unitZ;
    /// Specifies a compile-time constant float4 with value (0, 0, 0, 1).
    static const float4 unitW;
    /// A compile-time constant float4 with value (NaN, NaN, NaN, NaN).
    /// For this constant, each element has the value of quiet NaN, or Not-A-Number.
    /// @note Never compare a float4 to this value! Due to how IEEE floats work, for each float x, both the expression "x == nan" and "x == nan" returns false!
    ///       That is, nothing is equal to NaN, not even NaN itself!
    static const float4 nan;
    /// A compile-time constant float4 with value (+infinity, +infinity, +infinity, +infinity).
    static const float4 inf;

    float4 Add(const float4 &rhs) const { return *this + rhs; }
    float4 Sub(const float4 &rhs) const { return *this - rhs; }
    float4 Mul(float rhs) const { return *this * rhs; }
    float4 Div(float rhs) const { return *this / rhs; }
    float4 Neg() const { return -*this; }

    /// Multiplies this vector by rhs *element-wise*, including the w-component.
    float4 Mul(const float4 &rhs) const;
    /// Divides this vector by rhs *element-wise*, including the w-component.
    float4 Div(const float4 &rhs) const;

#ifdef OGRE_INTEROP
    float4(const Ogre::Vector4 &other) { x = other.x; y = other.y; z = other.z; w = other.w; }
    float4 &operator =(const Ogre::Vector4 &other) { x = other.x; y = other.y; z = other.z; w = other.w; return *this; }
    operator Ogre::Vector4() const { return Ogre::Vector4(x, y, z, w); }
#endif
#ifdef QT_INTEROP
    float4(const QVector4D &other) { x = other.x(); y = other.y(); z = other.z(); w = other.w(); }
    operator QVector4D() const { return QVector4D(x, y, z, w); }
    operator QString() const { return "float4(" + QString::number(x) + "," + QString::number(y) + "," + QString::number(z) + "," + QString::number(w) + ")"; }
    QString toString() const { return (QString)*this; }
    QVector4D ToQVector4D() const { return QVector4D(x, y, z, w); }
    static float4 FromQVector4D(const QVector4D &v) { return (float4)v; }
    static float4 FromString(const QString &str) { return FromString(str.toStdString()); }
#endif
#ifdef BULLET_INTEROP
    // Bullet uses the same btVector3 class for both 3- and 4 -tuples (due to SSE).
    float4(const btVector3 &other) { x = other.x(); y = other.y(); z = other.z(); w = other.w(); }
    operator btVector3() const { btVector3 v(x, y, z); v.setW(w); return v; }
#endif
};

#ifdef MATH_ENABLE_STL_SUPPORT
/// Prints this float4 to the given stream.
std::ostream &operator <<(std::ostream &out, const float4 &rhs);
#endif

/// Multiplies the x, y, z and w components of the vector by the given scalar. Note that if w != 0, 
/// this does NOT scale the length of the homogeneous 3D vector.
float4 operator *(float scalar, const float4 &rhs);

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS
inline float4 operator /(float scalar, const float4 &rhs) { return float4::FromScalar(scalar) / rhs; }
#endif

inline float Dot3(const float4 &a, const float4 &b) { return a.Dot3(b); }
inline float Dot4(const float4 &a, const float4 &b) { return a.Dot4(b); }
inline float4 Cross3(const float4 &a, const float4 &b) { return a.Cross3(b); }
inline float4 Abs(const float4 &a) { return a.Abs(); }
inline float4 Min(const float4 &a, const float4 &b) { return a.Min(b); }
inline float4 Max(const float4 &a, const float4 &b) { return a.Max(b); }
inline float4 Clamp(const float4 &a, float floor, float ceil) { return a.Clamp(floor, ceil); }
inline float4 Clamp(const float4 &a, const float4 &floor, const float4 &ceil) { return a.Clamp(floor, ceil); }
inline float4 Clamp01(const float4 &a) { return a.Clamp01(); }
inline float4 Lerp(const float4 &a, const float4 &b, float t) { return a.Lerp(b, t); }

#ifdef QT_INTEROP
Q_DECLARE_METATYPE(float4)
Q_DECLARE_METATYPE(float4*)
#endif
