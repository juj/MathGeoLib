#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/SSEMath.h"
#include "../src/Math/float4x4_sse.h"
#include "../src/Math/float4_neon.h"

using namespace MATH_NS::TestData;

#ifdef MATH_NEON

BENCHMARK(rsqrtq)
{
	float32x4_t r = v[i];
	float32x4_t rcp = vrsqrteq_f32(r);
	float32x4_t ret = vmulq_f32(vrsqrtsq_f32(vmulq_f32(rcp, rcp), r), rcp);
	v3[i] = ret;
}
BENCHMARK_END

BENCHMARK(rsqrt)
{
	float32x4_t r = v[i];
	float32x2_t rcp = vrsqrte_f32(vget_low_f32(r));
	float32x2_t hi = vget_high_f32(r);
	float32x2_t ret = vmul_f32(vrsqrts_f32(vmul_f32(rcp, rcp), vget_low_f32(r)), rcp);
	v3[i] = vcombine_f32(ret, hi);
}
BENCHMARK_END

UNIQUE_TEST(set_ps_const)
{
	simd4f constant = set_ps_const(4.f, 3.f, 2.f, 1.f);
	float arr[4];
	memcpy(arr, &constant, sizeof(arr));
	asserteq(arr[0], 1.f);
	asserteq(arr[1], 2.f);
	asserteq(arr[2], 3.f);
	asserteq(arr[3], 4.f);
}

UNIQUE_TEST(set_ps_const_hex)
{
	simd4f constant = set_ps_hex_const(0x80000000u, 0x3F800000u /*1.0f*/, 0x42C80000u /*100.0f*/, 0);
	float arr[4];
	memcpy(arr, &constant, sizeof(arr));
	asserteq(arr[0], 0.f);
	asserteq(arr[1], 100.f);
	asserteq(arr[2], 1.0f);
	asserteq(arr[3], -0.0f);
}

#endif
