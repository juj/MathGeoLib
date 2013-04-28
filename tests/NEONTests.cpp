#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/SSEMath.h"
#include "../src/Math/float4x4_sse.h"

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

#endif
