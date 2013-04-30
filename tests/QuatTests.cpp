#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"

#include "../src/Math/SSEMath.h"
#include "../src/Math/quat_simd.h"

using namespace TestData;

BENCHMARK(Quat_Transform_float3)
{
	v2[i].Float3Part() = q[i].Transform(v[i].Float3Part());
}
BENCHMARK_END;

BENCHMARK(Quat_Transform_float4)
{
	v2[i] = q[i].Transform(v01[i]);
}
BENCHMARK_END;

BENCHMARK(Quat_to_float4x4)
{
	m[i] = q[i].ToFloat4x4();
}
BENCHMARK_END;

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

RANDOMIZED_TEST(Quat_float4x4_conv)
{
	Quat q = Quat::RandomRotation(rng);
	float4x4 m = q.ToFloat4x4();
	assert(m.TranslatePart().Equals(0,0,0));
	assert(!m.ContainsProjection());
	Quat q2 = m.Float3x3Part().ToQuat();
	assert(q.Equals(q2) || q.Equals(q2.Neg()));

	float4 v = float4::RandomGeneral(rng, -1000.f, 1000.f);
	v.w = (float)rng.Int(0,1);
	m = q.ToFloat4x4(v);
	assert(m.TranslatePart().Equals(v.xyz()));
	assert(!m.ContainsProjection());
	q2 = m.Float3x3Part().ToQuat();
	assert(q.Equals(q2) || q.Equals(q2.Neg()));
}

BENCHMARK(Quat_op_mul_Quat)
{
	q2[i] = q[i] * q2[i];
}
BENCHMARK_END

BENCHMARK(Quat_op_div_Quat)
{
	q2[i] = q[i] / q2[i];
}
BENCHMARK_END

#ifdef MATH_SIMD
BENCHMARK(quat_mul_quat)
{
	q2[i].q = quat_mul_quat(q[i].q, q2[i].q);
}
BENCHMARK_END

RANDOMIZED_TEST(quat_mul_quat)
{
	Quat q = Quat::RandomRotation(rng);
	Quat q2 = Quat::RandomRotation(rng);
	Quat correct = q * q2;
	Quat q3 = quat_mul_quat(q.q, q2.q);
	assert(q3.Equals(correct));
}

BENCHMARK(quat_div_quat)
{
	q2[i].q = quat_div_quat(q[i].q, q2[i].q);
}
BENCHMARK_END

RANDOMIZED_TEST(quat_div_quat)
{
	Quat q = Quat::RandomRotation(rng);
	Quat q2 = Quat::RandomRotation(rng);
	Quat correct = q / q2;
	Quat q3 = quat_div_quat(q.q, q2.q);
	assert(q3.Equals(correct));
}
#endif
