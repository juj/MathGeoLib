#include "../src/MathGeoLib.h"
#include "TestRunner.h"
#include "../src/Math/SSEMath.h"
#include "ObjectGenerators.h"

MATH_BEGIN_NAMESPACE

namespace TestData
{

// Number of extra elements to put in to allow for indexing in benchmarks that have unrolled loop bodies.
#define UNROLL_LOOP_PADDING 100

float *PosFloatArray()
{
	LCG lcg;
	static float *arr;
	if (!arr)
	{
		arr = AlignedNew<float>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = lcg.Float(0.f, 100000.f);
	}
	return arr;
}

float *UnitFloatArray()
{
	LCG lcg;
	static float *arr;
	if (!arr)
	{
		arr = AlignedNew<float>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = lcg.Float();
	}
	return arr;
}

Quat *QuatArray()
{
	LCG lcg;
	static Quat *arr;
	if (!arr)
	{
		arr = AlignedNew<Quat>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = Quat::RandomRotation(lcg);
	}
	return arr;
}

Quat *QuatArray2()
{
	LCG lcg;
	static Quat *arr;
	if (!arr)
	{
		arr = AlignedNew<Quat>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = Quat::RandomRotation(lcg);
	}
	return arr;
}

AABB *AABBArray()
{
	LCG lcg;
	static AABB *arr;
	if (!arr)
	{
		arr = AlignedNew<AABB>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
		{
			arr[i].minPoint = POINT_VEC(float3::RandomBox(lcg, -100.f, 100.f));
			arr[i].maxPoint = arr[i].minPoint + POINT_VEC(float3::RandomBox(lcg, 0, 100.f));
		}
	}
	return arr;
}

OBB *OBBArray()
{
	LCG lcg;
	static OBB *arr;
	if (!arr)
	{
		arr = AlignedNew<OBB>(testrunner_numItersPerTest + UNROLL_LOOP_PADDING);
		for (int i = 0; i < testrunner_numItersPerTest + UNROLL_LOOP_PADDING; ++i)
		{
			AABB a;
			a.minPoint = POINT_VEC(float3::RandomBox(lcg, -100.f, 100.f));
			a.maxPoint = a.minPoint + POINT_VEC(float3::RandomBox(lcg, 0, 100.f));
			arr[i] = a.Transform(Quat::RandomRotation(lcg));
		}
	}
	return arr;
}

Frustum *FrustumArray()
{
	LCG lcg(Clock::TickU32());
	static Frustum *arr;
	if (!arr)
	{
		arr = AlignedNew<Frustum>(testrunner_numItersPerTest + UNROLL_LOOP_PADDING);
		for (int i = 0; i < testrunner_numItersPerTest + UNROLL_LOOP_PADDING; ++i)
		{
			arr[i] = RandomFrustumContainingPoint(lcg, POINT_VEC_SCALAR(0.f));
		}
	}
	return arr;
}

float *FloatArray()
{
	LCG lcg;
	static float *arr;
	if (!arr)
	{
		arr = AlignedNew<float>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = lcg.Float(-10.f, 10.f);
	}
	return arr;
}

float4x4 *MatrixArray()
{
	LCG lcg;
	static float4x4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4x4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4x4::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

float4x4 *MatrixArray2()
{
	LCG lcg;
	static float4x4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4x4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4x4::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

float4x4 *OrthonormalMatrixArray()
{
	LCG lcg;
	static float4x4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4x4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4x4(Quat::RandomRotation(lcg), float3::RandomBox(lcg, -10000.f, 10000.f));
	}
	return arr;
}

float4x4 *OrthogonalMatrixArray()
{
	LCG lcg;
	static float4x4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4x4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4x4(Quat::RandomRotation(lcg), float3::RandomBox(lcg, -10000.f, 10000.f)) * float4x4::UniformScale(lcg.Float(0.01f, 1e3f));
	}
	return arr;
}

float4x4 *TransposedMatrixArray()
{
	static float4x4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4x4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		float4x4 *m = MatrixArray();
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = m[i].Transposed();
	}
	return arr;
}

float2 *Float2Array()
{
	LCG lcg;
	static float2 *arr;
	if (!arr)
	{
		arr = AlignedNew<float2>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float2::RandomBox(lcg, -10.f, 10.f);
	}
	return arr;
}

float4 *NormalizedVectorArray()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4::RandomDir(lcg);
	}
	return arr;
}

float4 *NormalizedVectorArray2()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4::RandomDir(lcg);
	}
	return arr;
}

float4 *VectorArray()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

vec *VecArray2()
{
	LCG lcg;
	static vec *arr;
	if (!arr)
	{
		arr = AlignedNew<vec>(testrunner_numItersPerTest + UNROLL_LOOP_PADDING);
		for (int i = 0; i < testrunner_numItersPerTest + UNROLL_LOOP_PADDING; ++i)
			arr[i] = vec::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

float4 *VectorArray2()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

float4 *VectorArray3()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
	}
	return arr;
}

float4 *VectorArrayWithW0Or1()
{
	LCG lcg;
	static float4 *arr;
	if (!arr)
	{
		arr = AlignedNew<float4>(testrunner_numItersPerTest+UNROLL_LOOP_PADDING);
		for(int i = 0; i < testrunner_numItersPerTest+UNROLL_LOOP_PADDING; ++i)
		{
			arr[i] = float4::RandomGeneral(lcg, -10.f, 10.f);
			arr[i].w = (float)lcg.Int(0, 1);
		}
	}
	return arr;
}

float2 uninitializedFloat2;
float3 uninitializedFloat3;
float4 uninitializedFloat4;
float3x3 uninitializedFloat3x3;
float3x4 uninitializedFloat3x4;
float4x4 uninitializedFloat4x4;
Quat uninitializedQuat;

int dummyResultInt = 0;
vec dummyResultVec = vec::zero;

float *f = 0;
float *pf = 0;
float *uf = 0;
float4x4 *m = 0;
float4x4 *m2 = 0;
const float4x4 *om = 0;
const float4x4 *ogm = 0;
float4x4 *tpm = 0;
float2 *fl_2 = 0;
const float4 *nv = 0;
const float4 *nv2 = 0;
float4 *v = 0;
float4 *v2 = 0;
float4 *v3 = 0;
float4 *v01 = 0;
vec *ve = 0;
Quat *q = 0;
Quat *q2 = 0;
AABB *aabb = 0;
OBB *obb = 0;
Frustum *frustum = 0;

void InitTestData()
{
	f = FloatArray();
	pf = PosFloatArray(); // > 0
	uf = UnitFloatArray(); // [0,1]
	m = MatrixArray();
	m2 = MatrixArray2();
	om = OrthonormalMatrixArray();
	ogm = OrthogonalMatrixArray();
	tpm = TransposedMatrixArray();
	fl_2 = Float2Array();
	nv = NormalizedVectorArray();
	nv2 = NormalizedVectorArray2();
	v = VectorArray();
	v2 = VectorArray2();
	v3 = VectorArray3();
	v01 = VectorArrayWithW0Or1();
	ve = VecArray2();
	q = QuatArray();
	q2 = QuatArray2();
	aabb = AABBArray();
	obb = OBBArray();
	frustum = FrustumArray();
}

class FreeTestData
{
public:
	~FreeTestData()
	{
		AlignedFree(FloatArray());
		AlignedFree(PosFloatArray());
		AlignedFree(UnitFloatArray());
		AlignedFree(MatrixArray());
		AlignedFree(MatrixArray2());
		AlignedFree(Float2Array());
		AlignedFree(NormalizedVectorArray());
		AlignedFree(NormalizedVectorArray2());
		AlignedFree(VectorArray());
		AlignedFree(VectorArray2());
		AlignedFree(VectorArray3());
		AlignedFree(TransposedMatrixArray());
		AlignedFree(OrthonormalMatrixArray());
		AlignedFree(OrthogonalMatrixArray());
		AlignedFree(QuatArray());
		AlignedFree(QuatArray2());
		AlignedFree(VectorArrayWithW0Or1());
		AlignedFree(AABBArray());
		AlignedFree(OBBArray());
		AlignedFree(FrustumArray());
		AlignedFree(VecArray2());
	}
};
// At app exit time, this causes all test data arrays to be freed.
FreeTestData testDataDeleter;

} // ~namespace TestData

MATH_END_NAMESPACE
