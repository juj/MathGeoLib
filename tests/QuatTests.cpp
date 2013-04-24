#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

#include "../src/Math/SSEMath.h"

extern float *f;
extern float4x4 *m;
extern float4x4 *m2;
extern float4x4 *tm;
extern float4 *v;
extern float4 *v2;

Quat *QuatArray()
{
	LCG lcg;
	static Quat *arr;
	if (!arr)
	{
		arr = new Quat[testrunner_numItersPerTest+32];
		uintptr_t a = (uintptr_t)arr;
		a = (a + 31) & ~31;
		arr = (Quat*)a;
		for(int i = 0; i < testrunner_numItersPerTest; ++i)
			arr[i] = Quat::RandomRotation(lcg);
	}
	return arr;
}

Quat *q = QuatArray();

BENCHMARK(Quat_Transform_float3)
{
	TIMER_BEGIN
	{
		v2[i].Float3Part() = q[i].Transform(v[i].Float3Part());
	}
	TIMER_END;
}

BENCHMARK(Quat_Transform_float4)
{
	TIMER_BEGIN
	{
		v2[i] = q[i].Transform(v[i]);
	}
	TIMER_END;
}

BENCHMARK(Quat_to_float4x4)
{
	TIMER_BEGIN
	{
		m[i] = q[i].ToFloat4x4();
	}
	TIMER_END;
}

RANDOMIZED_TEST(Quat_Transform)
{
	Quat q = Quat::RandomRotation(rng);
	float3x3 m = float3x3(q);
	float3 v = float3::RandomSphere(rng, float3(0,0,0), 100.f);
	float3 mv = m*v;
	float3 qv = q*v;
	float3 qv2 = q.Transform(v);
	assert(mv.Equals(qv));
	assert(qv.Equals(qv2));

	float4 V(v, (float)rng.Int(0, 1));
	float4x4 M = float4x4(q);
	float4 MV = M*V;
	float4 qV = q*V;
	float4 qV2 = q.Transform(V);
	assert(MV.Equals(qV));
	assert(MV.Equals(qV2));
}

