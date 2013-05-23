#pragma once

#include "../src/MathBuildConfig.h"
#include "../src/MathGeoLibFwd.h"
#include "../src/Math/assume.h"

MATH_BEGIN_NAMESPACE

namespace TestData
{

float *PosFloatArray();
Quat *QuatArray();
Quat *QuatArray2();
float *FloatArray();
float *UnitFloatArray();
float4x4 *MatrixArray();
float4x4 *MatrixArray2();
float4x4 *OrthogonalMatrixArray();
float4x4 *OrthonormalMatrixArray();
float4x4 *TransposedMatrixArray();
float2 *Float2Array();
float4 *NormalizedVectorArray();
float4 *NormalizedVectorArray2();
float4 *VectorArray();
float4 *VectorArray2();
float4 *VectorArray3();
float4 *VectorArrayWithW0Or1();

// N.B. These must be static and not extern to not generate UDB with initialization order between compilation units!
static DONT_WARN_UNUSED float *f = FloatArray();
static DONT_WARN_UNUSED float *pf = PosFloatArray(); // > 0
static DONT_WARN_UNUSED float *uf = UnitFloatArray(); // [0,1]
static DONT_WARN_UNUSED float4x4 *m = MatrixArray();
static DONT_WARN_UNUSED float4x4 *m2 = MatrixArray2();
static DONT_WARN_UNUSED const float4x4 *om = OrthonormalMatrixArray();
static DONT_WARN_UNUSED const float4x4 *ogm = OrthogonalMatrixArray();
static DONT_WARN_UNUSED float4x4 *tpm = TransposedMatrixArray();
static DONT_WARN_UNUSED float2 *fl_2 = Float2Array();
static DONT_WARN_UNUSED const float4 *nv = NormalizedVectorArray();
static DONT_WARN_UNUSED const float4 *nv2 = NormalizedVectorArray2();
static DONT_WARN_UNUSED float4 *v = VectorArray();
static DONT_WARN_UNUSED float4 *v2 = VectorArray2();
static DONT_WARN_UNUSED float4 *v3 = VectorArray3();
static DONT_WARN_UNUSED float4 *v01 = VectorArrayWithW0Or1();
static DONT_WARN_UNUSED Quat *q = QuatArray();
static DONT_WARN_UNUSED Quat *q2 = QuatArray2();

} // ~TestData

MATH_END_NAMESPACE
