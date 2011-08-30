/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/


#include <string.h>

#include "Math/MathFunc.h"
#include "assume.h"
#include "Math/float3.h"
#include "Math/float4.h"
#include "Math/float3x3.h"
#include "Math/float3x4.h"
#include "Math/float4x4.h"
#include "Matrix.inl"
#include "Math/Quat.h"
#include "Algorithm/Random/LCG.h"
#include "Geometry/Plane.h"
#include "TransformOps.h"

float3x3::float3x3(float _00, float _01, float _02,
         float _10, float _11, float _12,
         float _20, float _21, float _22)
{
    Set(_00, _01, _02,
        _10, _11, _12,
        _20, _21, _22);
}

float3x3::float3x3(const float3 &col0, const float3 &col1, const float3 &col2)
{
    SetCol(0, col0);
    SetCol(1, col1);
    SetCol(2, col2);
}

float3x3::float3x3(const Quat &orientation)
{
    SetRotatePart(orientation);
}

float3x3 float3x3::RotateX(float angle)
{
    float3x3 r;
    r.SetRotatePartX(angle);
    return r;
}

float3x3 float3x3::RotateY(float angle)
{
    float3x3 r;
    r.SetRotatePartY(angle);
    return r;
}

float3x3 float3x3::RotateZ(float angle)
{
    float3x3 r;
    r.SetRotatePartZ(angle);
    return r;
}

float3x3 float3x3::RotateAxisAngle(const float3 &axisDirection, float angleRadians)
{
    float3x3 r;
    r.SetRotatePart(axisDirection, angleRadians);
    return r;
}

float3x3 float3x3::RotateFromTo(const float3 &sourceDirection, const float3 &targetDirection)
{
    float3x3 r;
    r.SetRotatePart(Quat::RotateFromTo(sourceDirection, targetDirection));
    return r;
}

float3x3 float3x3::RandomRotation(LCG &lcg)
{
    // The easiest way to generate a random orientation is through quaternions, so convert a 
    // random quaternion to a rotation matrix.
    return FromQuat(Quat::RandomRotation(lcg));
}

float3x3 float3x3::RandomGeneral(LCG &lcg, float minElem, float maxElem)
{
    float3x3 m;
    for(int y = 0; y < 3; ++y)
        for(int x = 0; x < 3; ++x)
            m[y][x] = lcg.Float(minElem, maxElem);
    return m;
}

float3x3 float3x3::FromQuat(const Quat &orientation)
{
    float3x3 r;
    r.SetRotatePart(orientation);
    return r;
}

Quat float3x3::ToQuat() const
{
    return Quat(*this);
}

float3x3 float3x3::FromRS(const Quat &rotate, const float3 &scale)
{
    return float3x3(rotate) * float3x3::Scale(scale);
}

float3x3 float3x3::FromRS(const float3x3 &rotate, const float3 &scale)
{
    return rotate * float3x3::Scale(scale);
}

float3x3 float3x3::FromEulerXYX(float x2, float y, float x)
{
    float3x3 r;
    Set3x3PartRotateEulerXYX(r, x2, y, x);
    assert(r.Equals(float3x3::RotateX(x2) * float3x3::RotateY(y) * float3x3::RotateX(x)));
    return r;
}

float3x3 float3x3::FromEulerXZX(float x2, float z, float x)
{
    float3x3 r;
    Set3x3PartRotateEulerXZX(r, x2, z, x);
    assert(r.Equals(float3x3::RotateX(x2) * float3x3::RotateZ(z) * float3x3::RotateX(x)));
    return r;
}

float3x3 float3x3::FromEulerYXY(float y2, float x, float y)
{
    float3x3 r;
    Set3x3PartRotateEulerYXY(r, y2, x, y);
    assert(r.Equals(float3x3::RotateY(y2) * float3x3::RotateX(x) * float3x3::RotateY(y)));
    return r;
}

float3x3 float3x3::FromEulerYZY(float y2, float z, float y)
{
    float3x3 r;
    Set3x3PartRotateEulerYZY(r, y2, z, y);
    assert(r.Equals(float3x3::RotateY(y2) * float3x3::RotateZ(z) * float3x3::RotateY(y)));
    return r;
}

float3x3 float3x3::FromEulerZXZ(float z2, float x, float z)
{
    float3x3 r;
    Set3x3PartRotateEulerZXZ(r, z2, x, z);
    assert(r.Equals(float3x3::RotateZ(z2) * float3x3::RotateX(x) * float3x3::RotateZ(z)));
    return r;
}

float3x3 float3x3::FromEulerZYZ(float z2, float y, float z)
{
    float3x3 r;
    Set3x3PartRotateEulerZYZ(r, z2, y, z);
    assert(r.Equals(float3x3::RotateZ(z2) * float3x3::RotateY(y) * float3x3::RotateZ(z)));
    return r;
}

float3x3 float3x3::FromEulerXYZ(float x, float y, float z)
{
    float3x3 r;
    Set3x3PartRotateEulerXYZ(r, x, y, z);
    assert(r.Equals(float3x3::RotateX(x) * float3x3::RotateY(y) * float3x3::RotateX(z)));
    return r;
}

float3x3 float3x3::FromEulerXZY(float x, float z, float y)
{
    float3x3 r;
    Set3x3PartRotateEulerXZY(r, x, z, y);
    assert(r.Equals(float3x3::RotateX(x) * float3x3::RotateZ(z) * float3x3::RotateY(y)));
    return r;
}

float3x3 float3x3::FromEulerYXZ(float y, float x, float z)
{
    float3x3 r;
    Set3x3PartRotateEulerYXZ(r, y, x, z);
    assert(r.Equals(float3x3::RotateY(y) * float3x3::RotateX(x) * float3x3::RotateZ(z)));
    return r;
}

float3x3 float3x3::FromEulerYZX(float y, float z, float x)
{
    float3x3 r;
    Set3x3PartRotateEulerYZX(r, y, z, x);
    assert(r.Equals(float3x3::RotateY(y) * float3x3::RotateZ(z) * float3x3::RotateX(x)));
    return r;
}

float3x3 float3x3::FromEulerZXY(float z, float x, float y)
{
    float3x3 r;
    Set3x3PartRotateEulerZXY(r, z, x, y);
    assert(r.Equals(float3x3::RotateZ(z) * float3x3::RotateX(x) * float3x3::RotateY(y)));
    return r;
}

float3x3 float3x3::FromEulerZYX(float z, float y, float x)
{
    float3x3 r;
    Set3x3PartRotateEulerZYX(r, z, y, x);
    assert(r.Equals(float3x3::RotateZ(z) * float3x3::RotateY(y) * float3x3::RotateX(x)));
    return r;
}

ScaleOp float3x3::Scale(float sx, float sy, float sz)
{
    return ScaleOp(sx, sy, sz);
}

ScaleOp float3x3::Scale(const float3 &scale)
{
    return ScaleOp(scale);
}

float3x3 float3x3::ScaleAlongAxis(const float3 &axis, float scalingFactor)
{
    return Scale(axis * scalingFactor);
}

ScaleOp float3x3::UniformScale(float uniformScale)
{
    return ScaleOp(uniformScale, uniformScale, uniformScale);
}

float3 float3x3::GetScale() const
{
    return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

float3x3 float3x3::ShearX(float yFactor, float zFactor)
{
    return float3x3(1.f, yFactor, zFactor,
                    0.f, 1.f, 0.f,
                    0.f, 0.f, 1.f);
}

float3x3 float3x3::ShearY(float xFactor, float zFactor)
{
    return float3x3(1.f, 0.f, 0.f,
                    xFactor, 1.f, zFactor,
                    0.f, 0.f, 1.f);
}

float3x3 float3x3::ShearZ(float xFactor, float yFactor)
{
    return float3x3(1.f, 0.f, 0.f,
                    0.f, 1.f, 0.f,
                    xFactor, yFactor, 1.f);
}

float3x3 float3x3::Reflect(const Plane &p)
{
    assume(p.PassesThroughOrigin() && "A 3x3 matrix cannot represent reflection about planes which do not pass through the origin! Use float3x4::Reflect instead!");
    float3x3 v;
    SetMatrix3x3LinearPlaneReflect(v, p.normal.x, p.normal.y, p.normal.z);
    return v;
}

float3x3 float3x3::OrthographicProjection(float nearPlaneDistance, float farPlaneDistance, float horizontalViewportSize, float verticalViewportSize)
{
    assume(false && "Not implemented!");
    return float3x3(); ///\todo
}

float3x3 float3x3::OrthographicProjection(const Plane &p)
{
    assume(p.PassesThroughOrigin() && "A 3x3 matrix cannot represent projection onto planes which do not pass through the origin! Use float3x4::OrthographicProjection instead!");
    float3x3 v;
    SetMatrix3x3LinearPlaneProject(v, p.normal.x, p.normal.y, p.normal.z);
    return v;
}

float3x3 float3x3::OrthographicProjectionYZ()
{
    float3x3 v = identity;
    v[0][0] = 0.f;
    return v;
}

float3x3 float3x3::OrthographicProjectionXZ()
{
    float3x3 v = identity;
    v[1][1] = 0.f;
    return v;
}

float3x3 float3x3::OrthographicProjectionXY()
{
    float3x3 v = identity;
    v[2][2] = 0.f;
    return v;
}

MatrixProxy<float3x3::Cols> &float3x3::operator[](int row)
{
    assert(row >= 0);
    assert(row < Rows);

    return *(reinterpret_cast<MatrixProxy<Cols>*>(v[row]));
}

const MatrixProxy<float3x3::Cols> &float3x3::operator[](int row) const
{
    assert(row >= 0);
    assert(row < Rows);

    return *(reinterpret_cast<const MatrixProxy<Cols>*>(v[row]));
}

float &float3x3::At(int row, int col)
{
    return v[row][col];
}

CONST_WIN32 float float3x3::At(int row, int col) const
{
    return v[row][col];
}

float3 &float3x3::Row(int row)
{
    return reinterpret_cast<float3 &>(v[row]);
}

const float3 &float3x3::Row(int row) const
{
    return reinterpret_cast<const float3 &>(v[row]);
}

CONST_WIN32 float3 float3x3::Col(int col) const
{
    return float3(v[0][col], v[1][col], v[2][col]);
}

CONST_WIN32 float3 float3x3::Diagonal() const
{
    return float3(v[0][0], v[1][1], v[2][2]);
}

void float3x3::ScaleRow(int row, float scalar)
{
    Row(row) *= scalar;
}

void float3x3::ScaleCol(int col, float scalar)
{
    v[0][col] *= scalar;
    v[1][col] *= scalar;
    v[2][col] *= scalar;
}

float3 float3x3::WorldX() const
{
    return Col(0);
}

float3 float3x3::WorldY() const
{
    return Col(1);
}

float3 float3x3::WorldZ() const
{
    return Col(2);
}

float *float3x3::ptr()
{
    return &v[0][0];
}

const float *float3x3::ptr() const
{
    return &v[0][0];
}

void float3x3::SetRow(int row, float x, float y, float z)
{
    v[row][0] = x;
    v[row][1] = y;
    v[row][2] = z;
}

void float3x3::SetRow(int row, const float3 &rowVector)
{
    v[row][0] = rowVector.x;
    v[row][1] = rowVector.y;
    v[row][2] = rowVector.z;
}

void float3x3::SetRow(int row, const float *data)
{
    v[row][0] = data[0];
    v[row][1] = data[1];
    v[row][2] = data[2];
}

void float3x3::SetCol(int column, float x, float y, float z)
{
    v[0][column] = x;
    v[1][column] = y;
    v[2][column] = z;
}

void float3x3::SetCol(int column, const float3 &columnVector)
{
    v[0][column] = columnVector.x;
    v[1][column] = columnVector.y;
    v[2][column] = columnVector.z;
}

void float3x3::SetCol(int column, const float *data)
{
    v[0][column] = data[0];
    v[1][column] = data[1];
    v[2][column] = data[2];
}

void float3x3::Set(float _00, float _01, float _02,
                   float _10, float _11, float _12,
                   float _20, float _21, float _22)
{
    v[0][0] = _00; v[0][1] = _01; v[0][2] = _02;
    v[1][0] = _10; v[1][1] = _11; v[1][2] = _12;
    v[2][0] = _20; v[2][1] = _21; v[2][2] = _22;
}

void float3x3::Set(const float3x3 &rhs)
{
    Set(rhs.ptr());
}

void float3x3::Set(const float *values)
{
    memcpy(ptr(), values, sizeof(float) * Rows * Cols);
}

void float3x3::Set(int row, int col, float value)
{
    assume(0 <= row && row <= 2);
    assume(0 <= col && col <= 2);
    v[row][col] = value;
}

void float3x3::SetIdentity()
{
    Set(1,0,0,
        0,1,0,
        0,0,1);
}

void float3x3::SwapColumns(int col1, int col2)
{
    Swap(v[0][col1], v[0][col2]);
    Swap(v[1][col1], v[1][col2]);
    Swap(v[2][col1], v[2][col2]);
}

void float3x3::SwapRows(int row1, int row2)
{
    Swap(v[row1][0], v[row2][0]);
    Swap(v[row1][1], v[row2][1]);
    Swap(v[row1][2], v[row2][2]);
}

void float3x3::SetRotatePartX(float angle)
{
    Set3x3PartRotateX(*this, angle);
}

void float3x3::SetRotatePartY(float angle)
{
    Set3x3PartRotateY(*this, angle);
}

void float3x3::SetRotatePartZ(float angle)
{
    Set3x3PartRotateZ(*this, angle);
}

void float3x3::SetRotatePart(const float3 &axisDirection, float angle)
{
    SetRotatePart(Quat(axisDirection, angle));
}

void float3x3::SetRotatePart(const Quat &q)
{
    SetMatrixRotatePart(*this, q);
}

float3x3 &float3x3::operator =(const Quat &rhs)
{
    SetRotatePart(rhs);
    return *this;
}

float3x3 &float3x3::operator =(const float3x3 &rhs)
{
    memcpy(this, &rhs, sizeof(rhs));
    return *this;
}

float float3x3::Determinant() const
{
    const float a = v[0][0];
    const float b = v[0][1];
    const float c = v[0][2];
    const float d = v[1][0];
    const float e = v[1][1];
    const float f = v[1][2];
    const float g = v[2][0];
    const float h = v[2][1];
    const float i = v[2][2];

    return a*(e*i - f*h) + b*(f*g - d*i) + c*(d*h - e*g);
}

#define SKIPNUM(val, skip) (val < skip ? val : skip + 1)
/* ///\todo Enable when float2x2 is implemented.
float2x2 float3x3::SubMatrix(int i, int j) const
{
    int r0 = SKIPNUM(0, i);
    int r1 = SKIPNUM(1, i);
    int c0 = SKIPNUM(0, j);
    int c1 = SKIPNUM(1, j);

    return float2x2(v[r0][c0], v[r0][c1],
                    v[r1][c0], v[r1][c1]);
}

float float3x3::Minor(int i, int j) const
{
    int r0 = SKIPNUM(0, i);
    int r1 = SKIPNUM(1, i);
    int c0 = SKIPNUM(0, j);
    int c1 = SKIPNUM(1, j);

    return v[r0][c0] * v[r1][c1] - v[r0][c1] * v[r1][c0];
}

float3x3 float3x3::Adjugate() const
{
    float3x3 a;
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            a[y][x] = (((x+y) & 1) != 0) ? -Minor(y, x) : Minor(y, x);

    return a;
}
*/
bool float3x3::Inverse()
{
    // There exists a generic matrix inverse calculator that uses Gaussian elimination.
    // It would be invoked by calling
    // return InverseMatrix(*this);
    // Instead, compute the inverse directly using Cramer's rule.
    float d = Determinant();
    if (EqualAbs(d, 0.f))
        return false;

    d = 1.f / d;
    float3x3 i;
    i[0][0] = d * (v[1][1] * v[2][2] - v[1][2] * v[2][1]);
    i[0][1] = d * (v[0][2] * v[2][1] - v[0][1] * v[2][2]);
    i[0][2] = d * (v[0][1] * v[1][2] - v[0][2] * v[1][1]);

    i[1][0] = d * (v[1][2] * v[0][0] - v[1][0] * v[2][2]);
    i[1][1] = d * (v[0][0] * v[2][2] - v[0][2] * v[2][0]);
    i[1][2] = d * (v[0][2] * v[1][0] - v[0][0] * v[1][2]);

    i[2][0] = d * (v[1][0] * v[2][1] - v[1][1] * v[2][0]);
    i[2][1] = d * (v[2][0] * v[0][1] - v[0][0] * v[2][1]);
    i[2][2] = d * (v[0][0] * v[1][1] - v[0][1] * v[1][0]);
    *this = i;
    return true;
}

float3x3 float3x3::Inverted() const
{
    float3x3 copy = *this;
    copy.Inverse();
    return copy;
}

bool float3x3::InverseOrthogonal()
{
    assume(IsOrthogonal());
    Swap(v[0][1], v[1][0]);
    Swap(v[0][2], v[2][0]);
    Swap(v[1][2], v[2][1]);
    float scale1 = sqrtf(1.f / float3(v[0][0], v[0][1], v[0][2]).LengthSq());
    float scale2 = sqrtf(1.f / float3(v[1][0], v[1][1], v[1][2]).LengthSq());
    float scale3 = sqrtf(1.f / float3(v[2][0], v[2][1], v[2][2]).LengthSq());

    v[0][0] *= scale1; v[0][1] *= scale2; v[0][2] *= scale3;
    v[1][0] *= scale1; v[1][1] *= scale2; v[1][2] *= scale3;
    v[2][0] *= scale1; v[2][1] *= scale2; v[2][2] *= scale3;

    return true;
}

bool float3x3::InverseOrthogonalUniformScale()
{
    assume(IsOrthogonal());
    assume(HasUniformScale());
    Swap(v[0][1], v[1][0]);
    Swap(v[0][2], v[2][0]);
    Swap(v[1][2], v[2][1]);
    const float scale = sqrtf(1.f / float3(v[0][0], v[0][1], v[0][2]).LengthSq());

    v[0][0] *= scale; v[0][1] *= scale; v[0][2] *= scale;
    v[1][0] *= scale; v[1][1] *= scale; v[1][2] *= scale;
    v[2][0] *= scale; v[2][1] *= scale; v[2][2] *= scale;

    return true;
}

void float3x3::InverseOrthonormal()
{
    assume(IsOrthonormal());
    Transpose();
}

void float3x3::Transpose()
{
    Swap(v[0][1], v[1][0]);
    Swap(v[0][2], v[2][0]);
    Swap(v[1][2], v[2][1]);
}

float3x3 float3x3::Transposed() const
{
    float3x3 copy = *this;
    copy.Transpose();
    return copy;
}

bool float3x3::InverseTranspose()
{
    bool success = Inverse();
    Transpose();
    return success;
}

float3x3 float3x3::InverseTransposed() const
{
    float3x3 copy = *this;
    copy.Transpose();
    copy.Inverse();
    return copy;
}

float float3x3::Trace() const
{
    return v[0][0] + v[1][1] + v[2][2];
}

void float3x3::Orthonormalize(int c0, int c1, int c2)
{
    assume(c0 != c1 && c0 != c2 && c1 != c2);
    if (c0 == c1 || c0 == c2 || c1 == c2)
        return;

    ///\todo Optimize away copies.
    float3 v0 = Col(c0);
    float3 v1 = Col(c1);
    float3 v2 = Col(c2);
    float3::Orthonormalize(v0, v1, v2);
    SetCol(c0, v0);
    SetCol(c1, v1);
    SetCol(c2, v2);
}

void float3x3::RemoveScale()
{
    float x = Row(0).Normalize();
    float y = Row(1).Normalize();
    float z = Row(2).Normalize();
    assume(x != 0 && y != 0 && z != 0 && "float3x3::RemoveScale failed!");
}

float3 float3x3::Transform(const float3 &vector) const
{
    return Transform(vector.x, vector.y, vector.z);
}

float3 float3x3::TransformLeft(const float3 &vector) const
{
    return float3(DOT3STRIDED(vector, ptr(), 3),
                  DOT3STRIDED(vector, ptr()+1, 3),
                  DOT3STRIDED(vector, ptr()+2, 3));
}

float3 float3x3::Transform(float x, float y, float z) const
{
    return float3(DOT3_xyz(Row(0), x,y,z),
                  DOT3_xyz(Row(1), x,y,z),
                  DOT3_xyz(Row(2), x,y,z));
}

float4 float3x3::Transform(const float4 &vector) const
{
    return float4(DOT3(Row(0), vector),
                  DOT3(Row(1), vector),
                  DOT3(Row(2), vector),
                  vector.w);
}

void float3x3::BatchTransform(float3 *pointArray, int numPoints) const
{
    for(int i = 0; i < numPoints; ++i)
        pointArray[i] = *this * pointArray[i];
}

void float3x3::BatchTransform(float3 *pointArray, int numPoints, int stride) const
{
    assume(stride >= sizeof(float3));
    u8 *data = reinterpret_cast<u8*>(pointArray);
    for(int i = 0; i < numPoints; ++i)
    {
        float3 *v = reinterpret_cast<float3*>(data + stride*i);
        *v = *this * *v;
    }
}

void float3x3::BatchTransform(float4 *vectorArray, int numVectors) const
{
    for(int i = 0; i < numVectors; ++i)
        vectorArray[i] = *this * vectorArray[i];
}

void float3x3::BatchTransform(float4 *vectorArray, int numVectors, int stride) const
{
    assume(stride >= sizeof(float4));
    u8 *data = reinterpret_cast<u8*>(vectorArray);
    for(int i = 0; i < numVectors; ++i)
    {
        float4 *v = reinterpret_cast<float4*>(data + stride*i);
        *v = *this * *v;
    }
}

float3x3 float3x3::operator *(const float3x3 &rhs) const
{
    float3x3 r;
    const float *c0 = rhs.ptr();
    const float *c1 = rhs.ptr() + 1;
    const float *c2 = rhs.ptr() + 2;
    r[0][0] = DOT3STRIDED(v[0], c0, 3);
    r[0][1] = DOT3STRIDED(v[0], c1, 3);
    r[0][2] = DOT3STRIDED(v[0], c2, 3);

    r[1][0] = DOT3STRIDED(v[1], c0, 3);
    r[1][1] = DOT3STRIDED(v[1], c1, 3);
    r[1][2] = DOT3STRIDED(v[1], c2, 3);

    r[2][0] = DOT3STRIDED(v[2], c0, 3);
    r[2][1] = DOT3STRIDED(v[2], c1, 3);
    r[2][2] = DOT3STRIDED(v[2], c2, 3);

    return r;
}

float3x3 float3x3::operator *(const Quat &rhs) const
{
    return *this * float3x3(rhs);
}

float3 float3x3::operator *(const float3 &rhs) const
{
    return float3(DOT3(v[0], rhs),
                  DOT3(v[1], rhs),
                  DOT3(v[2], rhs));
}

float4 float3x3::operator *(const float4 &rhs) const
{
    return float4(DOT3(v[0], rhs),
                  DOT3(v[1], rhs),
                  DOT3(v[2], rhs),
                  rhs.w);
}

float3x3 float3x3::operator *(float scalar) const
{
    float3x3 r = *this;
    r *= scalar;
    return r;
}

float3x3 float3x3::operator /(float scalar) const
{
    assume(!EqualAbs(scalar, 0));
    float3x3 r = *this;
    r /= scalar;
    return r;
}

float3x3 float3x3::operator +(const float3x3 &rhs) const
{
    float3x3 r = *this;
    r += rhs;
    return r;
}

float3x3 float3x3::operator -(const float3x3 &rhs) const
{
    float3x3 r = *this;
    r -= rhs;
    return r;
}

float3x3 float3x3::operator -() const
{
    float3x3 r;
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            r[y][x] = -v[y][x];
    return r;
}

float3x3 &float3x3::operator *=(float scalar)
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            v[y][x] *= scalar;

    return *this;
}

float3x3 &float3x3::operator /=(float scalar)
{
    assume(!EqualAbs(scalar, 0));
    float invScalar = 1.f / scalar;
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            v[y][x] *= invScalar;

    return *this;
}

float3x3 &float3x3::operator +=(const float3x3 &rhs)
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            v[y][x] += rhs[y][x];

    return *this;
}

float3x3 &float3x3::operator -=(const float3x3 &rhs)
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            v[y][x] -= rhs[y][x];

    return *this;
}

bool float3x3::IsFinite() const
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            if (!isfinite(v[y][x]))
                return false;
    return true;
}

bool float3x3::IsIdentity(float epsilon) const
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            if (!EqualAbs(v[y][x], (x == y) ? 1.f : 0.f, epsilon))
                return false;

    return true;
}

bool float3x3::IsLowerTriangular(float epsilon) const
{
    return EqualAbs(v[0][1], 0.f, epsilon)
        && EqualAbs(v[0][2], 0.f, epsilon)
        && EqualAbs(v[0][3], 0.f, epsilon)
        && EqualAbs(v[1][2], 0.f, epsilon)
        && EqualAbs(v[1][3], 0.f, epsilon)
        && EqualAbs(v[2][3], 0.f, epsilon);
}

bool float3x3::IsUpperTriangular(float epsilon) const
{
    return EqualAbs(v[1][0], 0.f, epsilon)
        && EqualAbs(v[2][0], 0.f, epsilon)
        && EqualAbs(v[3][0], 0.f, epsilon)
        && EqualAbs(v[2][1], 0.f, epsilon)
        && EqualAbs(v[3][1], 0.f, epsilon)
        && EqualAbs(v[3][2], 0.f, epsilon);
}

bool float3x3::IsInvertible(float epsilon) const
{
    ///\todo Optimize.
    float3x3 copy = *this;
    return copy.Inverse();
}

bool float3x3::IsSymmetric(float epsilon) const
{
    for(int y = 0; y < Rows; ++y)
        for(int x = y+1; x < Cols; ++x)
            if (!EqualAbs(v[y][x], v[x][y], epsilon))
                return false;
    return true;
}

bool float3x3::IsSkewSymmetric(float epsilon) const
{
    for(int y = 0; y < Rows; ++y)
        for(int x = y; x < Cols; ++x)
            if (!EqualAbs(v[y][x], -v[x][y], epsilon))
                return false;
    return true;
}

bool float3x3::HasUnitaryScale(float epsilon) const
{
    float3 scale = ExtractScale();
    return scale.Equals(1.f, 1.f, 1.f, epsilon);
}

bool float3x3::HasNegativeScale() const
{
    return Determinant() < 0.f;
}

bool float3x3::HasUniformScale(float epsilon) const
{
    float3 scale = ExtractScale();
    return EqualAbs(scale.x, scale.y, epsilon) && EqualAbs(scale.x, scale.z, epsilon);
}

bool float3x3::IsOrthogonal(float epsilon) const
{
    return Row(0).IsPerpendicular(Row(1), epsilon)
        && Row(0).IsPerpendicular(Row(2), epsilon)
        && Row(1).IsPerpendicular(Row(2), epsilon);
}

bool float3x3::IsOrthonormal(float epsilon) const
{
    ///\todo Epsilon magnitudes don't match.
    return IsOrthogonal(epsilon) && Row(0).IsNormalized(epsilon) && Row(1).IsNormalized(epsilon) && Row(2).IsNormalized(epsilon);
}

bool float3x3::Equals(const float3x3 &other, float epsilon) const
{
    for(int y = 0; y < Rows; ++y)
        for(int x = 0; x < Cols; ++x)
            if (!EqualAbs(v[y][x], other[y][x], epsilon))
                return false;
    return true;
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::string float3x3::ToString() const
{
    char str[256];
    sprintf(str, "(%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)", 
        v[0][0], v[0][1], v[0][2],
        v[1][0], v[1][1], v[1][2],
        v[2][0], v[2][1], v[2][2]);

    return std::string(str);
}

std::string float3x3::ToString2() const
{
    char str[256];
    sprintf(str, "float3x3(X:(%.2f,%.2f,%.2f) Y:(%.2f,%.2f,%.2f) Z:(%.2f,%.2f,%.2f)", 
        v[0][0], v[1][0], v[2][0],
        v[0][1], v[1][1], v[2][1],
        v[0][2], v[1][2], v[2][2]);

    return std::string(str);
}
#endif

float3 float3x3::ToEulerXYX() const { float3 f; ExtractEulerXYX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXZX() const { float3 f; ExtractEulerXZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYXY() const { float3 f; ExtractEulerYXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYZY() const { float3 f; ExtractEulerYZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZXZ() const { float3 f; ExtractEulerZXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZYZ() const { float3 f; ExtractEulerZYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXYZ() const { float3 f; ExtractEulerXYZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerXZY() const { float3 f; ExtractEulerXZY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYXZ() const { float3 f; ExtractEulerYXZ(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerYZX() const { float3 f; ExtractEulerYZX(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZXY() const { float3 f; ExtractEulerZXY(*this, f[0], f[1], f[2]); return f; }
float3 float3x3::ToEulerZYX() const { float3 f; ExtractEulerZYX(*this, f[0], f[1], f[2]); return f; }

float3 float3x3::ExtractScale() const
{
    return float3(Col(0).Length(), Col(1).Length(), Col(2).Length());
}

void float3x3::Decompose(Quat &rotate, float3 &scale) const
{
    assume(this->IsOrthogonal());

    float3x3 r;
    Decompose(r, scale);
    rotate = Quat(r);

    // Test that composing back yields the original float3x3.
    assume(float3x3::FromRS(rotate, scale).Equals(*this, 0.1f));
}

void float3x3::Decompose(float3x3 &rotate, float3 &scale) const
{
    assume(this->IsOrthogonal());

    rotate = *this;
    scale.x = rotate.Col(0).Length();
    scale.y = rotate.Col(1).Length();
    scale.z = rotate.Col(2).Length();
    assume(!EqualAbs(scale.x, 0));
    assume(!EqualAbs(scale.y, 0));
    assume(!EqualAbs(scale.z, 0));
    rotate.ScaleCol(0, 1.f / scale.x);
    rotate.ScaleCol(1, 1.f / scale.y);
    rotate.ScaleCol(2, 1.f / scale.z);

    // Test that composing back yields the original float3x3.
    assume(float3x3::FromRS(rotate, scale).Equals(*this, 0.1f));
}

#ifdef MATH_ENABLE_STL_SUPPORT
std::ostream &operator <<(std::ostream &out, const float3x3 &rhs)
{
    out << rhs.ToString();
    return out;
}
#endif

float3x3 float3x3::Mul(const float3x3 &rhs) const { return *this * rhs; }
float3x4 float3x3::Mul(const float3x4 &rhs) const { return *this * rhs; }
float4x4 float3x3::Mul(const float4x4 &rhs) const { return *this * rhs; }
float3x3 float3x3::Mul(const Quat &rhs) const { return *this * rhs; }
float3 float3x3::Mul(const float3 &rhs) const { return *this * rhs; }

float3x3 operator *(const Quat &lhs, const float3x3 &rhs)
{
    float3x3 lhsRot(lhs);
    return lhsRot * rhs;
}

float3 operator *(const float3 &lhs, const float3x3 &rhs)
{
    return rhs.TransformLeft(lhs);
}

float4 operator *(const float4 &lhs, const float3x3 &rhs)
{
    assume(lhs.IsWZeroOrOne());
    return float4(rhs.TransformLeft(lhs.xyz()), lhs.w);
}

const float3x3 float3x3::zero     = float3x3(0,0,0, 0,0,0, 0,0,0);
const float3x3 float3x3::identity = float3x3(1,0,0, 0,1,0, 0,0,1);
const float3x3 float3x3::nan = float3x3(FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN, FLOAT_NAN);
