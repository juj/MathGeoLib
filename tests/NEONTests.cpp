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

BENCHMARK(float4_op_add, "float4 + float4")
{
	v3[i] = v[i] + v2[i];
}
BENCHMARK_END;

#ifdef MATH_NEON

BENCHMARK(rsqrtq, "neon")
{
	float32x4_t r = v[i];
	float32x4_t rcp = vrsqrteq_f32(r);
	float32x4_t ret = vmulq_f32(vrsqrtsq_f32(vmulq_f32(rcp, rcp), r), rcp);
	v3[i] = ret;
}
BENCHMARK_END

BENCHMARK(rsqrt, "neon")
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

#ifdef ANDROID

FORCE_INLINE void inline_asm_add(void *v1, void *v2, void *out)
{
	asm(
		"\t vld1.32 {d0, d1}, [%1]\n"
		"\t vld1.32 {d2, d3}, [%2]\n"
		"\t vadd.f32 q0, q0, q1\n"
		"\t vst1.32 {d0, d1}, [%0]\n"
		: /* no outputs by value */
		:"r"(out), "r"(v1), "r"(v2)
		:"memory", "q0", "q1");
}

FORCE_INLINE simd4f inline_asm_add_nice(simd4f v1, simd4f v2)
{
	simd4f temp;
	inline_asm_add(&v1, &v2, &temp);
	return temp;
}

UNIQUE_TEST(inline_asm_add)
{
	float4 v = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v2 = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v3;
	inline_asm_add(v.ptr(), v2.ptr(), v3.ptr());
	float4 correct = v + v2;
	assert(v3.Equals(correct));
}

simd4f inline_asm_add_2(simd4f v1, simd4f v2)
{
	simd4f ret;
	asm(
		"\t vadd.f32 %q0, %q1, %q2\n"
		: "=w"(ret)
		:"w"(v1), "w"(v2));
	return ret;
}

UNIQUE_TEST(inline_asm_add_2)
{
	float4 v = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v2 = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v3;
	v3 = inline_asm_add_2(v, v2);
	float4 correct = v + v2;
	assert(v3.Equals(correct));
}

simd4f inline_asm_add_3(simd4f v1, simd4f v2)
{
	simd4f ret;
	asm(
		"\t vld1.32 {d0, d1}, [%m1]\n"
		"\t vld1.32 {d2, d3}, [%m2]\n"
		"\t vadd.f32 q0, q0, q1\n"
		"\t vst1.32 {d0, d1}, [%m0]\n"
		:
		:"Us"(ret), "Us"(v1), "Us"(v2)
		:"memory", "q0", "q1");
	return ret;
}

UNIQUE_TEST(inline_asm_add_3)
{
	float4 v = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v2 = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 v3;
	v3 = inline_asm_add_3(v, v2);
	float4 correct = v + v2;
	assert(v3.Equals(correct));
}

BENCHMARK(float4_add_neon_intrinsics_once, "test against float4_op_add")
{
	v3[i] = add_ps(v[i], v2[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_once, "test against float4_op_add")
{
	inline_asm_add(v[i].ptr(), v2[i].ptr(), v3[i].ptr());
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_byaddr_once, "test against float4_op_add")
{
	inline_asm_add(&v[i], &v2[i], &v3[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_byaddr2_once, "test against float4_op_add")
{
	inline_asm_add(&v[i].v, &v2[i].v, &v3[i].v);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_nice_once, "test against float4_op_add")
{
	v3[i] = inline_asm_add_nice(v[i], v2[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_2_once, "test against float4_op_add")
{
	v3[i] = inline_asm_add_2(v[i], v2[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_3_once, "test against float4_op_add")
{
	v3[i] = inline_asm_add_3(v[i], v2[i]);
}
BENCHMARK_END;

BENCHMARK(float4_add_scalar_threetimes, "neon")
{
	v3[i] = v[i] + v2[i];
	v3[i] = v2[i] + v3[i];
	v3[i] = v[i] + v3[i];
}
BENCHMARK_END;

BENCHMARK(float4_add_neon_intrinsics_threetimes, "neon")
{
	v3[i] = add_ps(v[i], v2[i]);
	v3[i] = add_ps(v2[i], v3[i]);
	v3[i] = add_ps(v[i], v3[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_threetimes, "neon")
{
	inline_asm_add(v[i].ptr(), v2[i].ptr(), v3[i].ptr());
	inline_asm_add(v2[i].ptr(), v3[i].ptr(), v3[i].ptr());
	inline_asm_add(v[i].ptr(), v3[i].ptr(), v3[i].ptr());
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_byaddr_threetimes, "neon")
{
	inline_asm_add(&v[i], &v2[i], &v3[i]);
	inline_asm_add(&v2[i], &v3[i], &v3[i]);
	inline_asm_add(&v[i], &v3[i], &v3[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_byaddr2_threetimes, "neon")
{
	inline_asm_add(&v[i].v, &v2[i].v, &v3[i].v);
	inline_asm_add(&v2[i].v, &v3[i].v, &v3[i].v);
	inline_asm_add(&v[i].v, &v3[i].v, &v3[i].v);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_nice_threetimes, "neon")
{
	v3[i] = inline_asm_add_nice(v[i], v2[i]);
	v3[i] = inline_asm_add_nice(v2[i], v3[i]);
	v3[i] = inline_asm_add_nice(v[i], v3[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_2_threetimes, "neon")
{
	v3[i] = inline_asm_add_2(v[i], v2[i]);
	v3[i] = inline_asm_add_2(v2[i], v3[i]);
	v3[i] = inline_asm_add_2(v[i], v3[i]);
}
BENCHMARK_END;

BENCHMARK(inline_asm_add_3_threetimes, "neon")
{
	v3[i] = inline_asm_add_3(v[i], v2[i]);
	v3[i] = inline_asm_add_3(v2[i], v3[i]);
	v3[i] = inline_asm_add_3(v[i], v3[i]);
}
BENCHMARK_END;

UNIQUE_TEST(vec4_length_sq_float_asm)
{
	float4 f(1,2,3,4);
	float len = vec4_length_sq_float_asm(&f.v);
	asserteq(len, 30.f);
}

BENCHMARK(vec4_length_sq_float_asm, "neon")
{
	f[i] = vec4_length_sq_float_asm(&v[i].v);
}
BENCHMARK_END;

UNIQUE_TEST(vec4_length_sq_ps_asm)
{
	float4 f(1,2,3,4);
	float4 lensq;
	vec4_length_sq_ps_asm(&f.v, &lensq.v);
	LOGI("%s", lensq.ToString().c_str());
	asserteq(lensq.x, 30.f);
	asserteq(lensq.y, 30.f);
	asserteq(lensq.z, 30.f);
	asserteq(lensq.w, 30.f);
}

BENCHMARK(vec4_length_sq_ps_asm, "neon")
{
	vec4_length_sq_ps_asm(&v[i].v, &v3[i].v);
}
BENCHMARK_END;

#endif // ~ANDROID

#endif
