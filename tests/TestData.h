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
vec *VecArray2();
float4 *VectorArray();
float4 *VectorArray2();
float4 *VectorArray3();
float4 *VectorArrayWithW0Or1();
AABB *AABBArray();
OBB *OBBArray();
Frustum *FrustumArray();

void InitTestData();

#ifdef _MSC_VER
#pragma warning(disable:4459) // C4459: declaration of 'f' hides global declaration
#endif

extern float *f;
extern float *pf;
extern float *uf;
extern float4x4 *m;
extern float4x4 *m2;
extern const float4x4 *om;
extern const float4x4 *ogm;
extern float4x4 *tpm;
extern float2 *fl_2;
extern const float4 *nv;
extern const float4 *nv2;
extern float4 *v;
extern float4 *v2;
extern float4 *v3;
extern float4 *v01;
extern vec *ve;
extern Quat *q;
extern Quat *q2;
extern AABB *aabb;
extern OBB *obb;
extern Frustum *frustum;

extern float2 uninitializedFloat2;
extern float3 uninitializedFloat3;
extern float4 uninitializedFloat4;
extern float3x3 uninitializedFloat3x3;
extern float3x4 uninitializedFloat3x4;
extern float4x4 uninitializedFloat4x4;
extern Quat uninitializedQuat;

// An otherwise unused variable, but global so that writing results to this has the effect that compiler won't
// optimize out benchmarks that time how long computations take.
extern int dummyResultInt;
extern vec dummyResultVec;

} // ~TestData

MATH_END_NAMESPACE
