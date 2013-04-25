#include "../src/MathBuildConfig.h"
#include "../src/MathGeoLibFwd.h"

MATH_BEGIN_NAMESPACE

namespace TestData
{

float *PosFloatArray();
Quat *QuatArray();
float *FloatArray();
float4x4 *MatrixArray();
float4x4 *MatrixArray2();
float4x4 *TransposedMatrixArray();
float2 *Float2Array();
float4 *VectorArray();
float4 *VectorArray2();

// N.B. These must be static and not extern to not generate UDB with initialization order between compilation units!
static float *f = FloatArray();
static float *pf = PosFloatArray();
static float4x4 *m = MatrixArray();
static float4x4 *m2 = MatrixArray2();
static float4x4 *tpm = TransposedMatrixArray();
static float2 *fl_2 = Float2Array();
static float4 *v = VectorArray();
static float4 *v2 = VectorArray2();
static Quat *q = QuatArray();

} // ~TestData

MATH_END_NAMESPACE
