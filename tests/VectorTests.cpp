#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/float2.h"
#include "../src/Math/float4d.h"
#include "../src/Math/float4_sse.h"
#include "../src/Math/float4_neon.h"

using namespace TestData;

MATH_IGNORE_UNUSED_VARS_WARNING

TEST(Float4Swizzled)
{
	float4 f(float2(1,2),3,4);
	float4 f2 = f.Swizzled(2,0,1,3);
	float f3[4] = { 3, 1, 2, 4 };
	assert(f2.Equals(float4(f3)));
	MARK_UNUSED(f3);
}

#if defined(MATH_AVX) || defined(MATH_NEON)
TEST(vec4_permute)
{
	float4 f(float2(1,2),3,4);
	float4 f2 = vec4_permute(f, 2, 0, 1, 3);
	float f3[4] = { 3, 1, 2, 4 };
	assert(f2.Equals(float4(f3)));
	MARK_UNUSED(f3);
}

BENCHMARK(Float4Swizzle, "float4::Swizzled")
{
	v3[i] = v[i].Swizzled(2, 0, 1, 3);
}
BENCHMARK_END

BENCHMARK(vec4_permute, "test against Float4Swizzle")
{
	v3[i] = vec4_permute(v[i], 2, 0, 1, 3);
}
BENCHMARK_END

#endif

TEST(Float4LengthSq3)
{
	float4 f(-1.f,2.f,3.f,1000000.f);
	assert(EqualAbs(f.LengthSq3(), 14.f));
}

TEST(Float4Length3)
{
	float4 f(-1.f,2.f,3.f,1000000.f);
	assert(EqualAbs(f.Length3(), Sqrt(14.f)));
}

TEST(Float4LengthSq4)
{
	float4 f(-1.f,2.f,3.f,-4.f);
	assert(EqualAbs(f.LengthSq4(), 30.f));
}

TEST(Float4Length4)
{
	float4 f(-1.f,2.f,3.f,-4.f);
	assert(EqualAbs(f.Length4(), Sqrt(30.f)));
}

TEST(Float4Normalize3)
{
	float4 f(-1.f, 2.f, 3.f, 1000.f);
	float oldLength = f.Normalize3();
	assert(oldLength > 0.f);
	MARK_UNUSED(oldLength);
	assert(EqualAbs(f.x, -1.f / Sqrt(14.f)));
	assert(EqualAbs(f.y, 2.f / Sqrt(14.f)));
	assert(EqualAbs(f.z, 3.f / Sqrt(14.f)));
	assert(EqualAbs(f.w, 1000.f));

	float4 f2(0,0,0, 1000.f);
	oldLength = f2.Normalize3();
	MARK_UNUSED(oldLength);
	assert(oldLength == 0.f);
	assert(f2.x == 1.f);
	assert(f2.y == 0.f);
	assert(f2.z == 0.f);
	assert(f2.w == 1000.f);
}

TEST(Float4Normalized3)
{
	float4 f(-1.f, 2.f, 3.f, 1000.f);
	float4 f2 = f.Normalized3();
	assert4(EqualAbs(f2.x, -1.f / Sqrt(14.f)), f, f2, -1.f / Sqrt(14.f), f2.LengthSq3());
	assert(EqualAbs(f2.y, 2.f / Sqrt(14.f)));
	assert(EqualAbs(f2.z, 3.f / Sqrt(14.f)));
	assert(EqualAbs(f2.w, 1000.f));
	assert1(EqualAbs(f2.LengthSq3(), 1.f), f2.LengthSq3());
}

TEST(Float4Normalize4)
{
	float4 f(-1.f, 2.f, 3.f, 4.f);
	float oldLength = f.Normalize4();
	MARK_UNUSED(oldLength);
	assertcmp(oldLength, >, 0);
	assert(EqualAbs(f.x, -1.f / Sqrt(30.f)));
	assert(EqualAbs(f.y, 2.f / Sqrt(30.f)));
	assert(EqualAbs(f.z, 3.f / Sqrt(30.f)));
	assert(EqualAbs(f.w, 4.f / Sqrt(30.f)));

	float4 f2(0,0,0, 0.f);
	oldLength = f2.Normalize4();
	MARK_UNUSED(oldLength);
	assert(oldLength == 0.f);
	assert(f2.x == 1.f);
	assert(f2.y == 0.f);
	assert(f2.z == 0.f);
	assert(f2.w == 0.f);
}

TEST(Float4Normalized4)
{
	float4 f(-1.f, 2.f, 3.f, -4.f);
	float4 f2 = f.Normalized4();
	assert(EqualAbs(f2.x, -1.f / Sqrt(30.f)));
	assert(EqualAbs(f2.y, 2.f / Sqrt(30.f)));
	assert(EqualAbs(f2.z, 3.f / Sqrt(30.f)));
	assert(EqualAbs(f2.w, -4.f / Sqrt(30.f)));
}

TEST(Float4NormalizeW)
{
	float4 f(-2.f, -4.f, 8.f, 2.f);
	f.NormalizeW();
	assert(f.Equals(float4(-1.f, -2.f, 4.f, 1.f)));
}

TEST(Float4Scale3)
{
	float4 f(-2.f, -4.f, 8.f, 1000.f);
	f.Scale3(100.f);
	assert(f.Equals(float4(-200.f, -400.f, 800.f, 1000.f)));
}

TEST(Float4ScaleToLength3)
{
	float4 f(-1.f, 2.f, 3.f, 1000.f);
	float4 f2 = f.ScaledToLength3(10.f);
	float oldLength = f.ScaleToLength3(10.f);
	MARK_UNUSED(oldLength);
	assert(f.Equals(f2));
	assert(EqualAbs(oldLength, Sqrt(14.f)));
	assert(EqualAbs(f.x, -1.f * 10.f / oldLength));
	assert(EqualAbs(f.y, 2.f * 10.f / oldLength));
	assert(EqualAbs(f.z, 3.f * 10.f / oldLength));
	assert(EqualAbs(f.w, 1000.f));
}

TEST(Float4SumOfElements)
{
	float4 f(-1.f, 4.f, 20.f, -100.f);
	assert(EqualAbs(f.SumOfElements(), -77.f));
}

TEST(Float4ProductOfElements)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(EqualAbs(f.ProductOfElements(), 64.f));
}

TEST(Float4AverageOfElements)
{
	float4 f(-2.f, 2.f, 4.f, -8.f);
	assert(EqualAbs(f.AverageOfElements(), -1.f));
}

TEST(Float4Abs)
{
	float4 f(-2.f, 2.f, 4.f, -8.f);
	assert(f.Abs().Equals(float4(2.f, 2.f, 4.f, 8.f)));
}

TEST(Float4Neg3)
{
	float4 f(-2.f, 2.f, 4.f, -8.f);
	assert(f.Neg3().Equals(float4(2.f, -2.f, -4.f, -8.f)));
}

TEST(Float4Neg4)
{
	float4 f(-2.f, 2.f, 4.f, -8.f);
	assert(f.Neg4().Equals(float4(2.f, -2.f, -4.f, 8.f)));
}

TEST(Float4Recip3)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.Recip3().Equals(float4(-1.f, 0.5f, 0.25f, -8.f)));
}

TEST(Float4Recip4)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.Recip4().Equals(float4(-1.f, 0.5f, 0.25f, -0.125f)));
}

TEST(Float4RecipFast4)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.RecipFast4().Equals(float4(-1.f, 0.5f, 0.25f, -0.125f)));
}

TEST(Float4Min)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.Min(-4.f).Equals(float4(-4.f, -4.f, -4.f, -8.f)));
	assert(f.Min(float4(-3.f, 20.f, -2.f, 9.f)).Equals(-3.f, 2.f, -2.f, -8.f));
}

TEST(Float4Max)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.Max(-4.f).Equals(float4(-1.f, 2.f, 4.f, -4.f)));
	assert(f.Max(float4(-3.f, 20.f, -2.f, 9.f)).Equals(-1.f, 20.f, 4.f, 9.f));
}

TEST(Float4Clamp)
{
	float4 f(-1.f, 2.f, 4.f, -8.f);
	assert(f.Clamp(-2.f, 2.f).Equals(float4(-1.f, 2.f, 2.f, -2.f)));
	assert(f.Clamp(float4(0.f, -1.f, -10.f, -8.f), float4(10.f, 0.f, 10.f, -8.f)).Equals(float4(0.f, 0.f, 4.f, -8.f)));
}

TEST(Float4Clamp01)
{
	float4 f(-1.f, 0.f, 0.5f, 2.f);
	assert(f.Clamp01().Equals(float4(0.f, 0.f, 0.5f, 1.f)));
}

TEST(Float4DistanceSq3)
{
	float4 f(1.f, 2.f, 3.f, 4.f);
	float4 f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs(f.Distance3Sq(f2), 56.f));
}

TEST(Float4Distance3)
{
	float4 f(1.f, 2.f, 3.f, 4.f);
	float4 f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs(f.Distance3(f2), Sqrt(56.f)));
}

TEST(Float4DistanceSq4)
{
	float4 f(1.f, 2.f, 3.f, 4.f);
	float4 f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs(f.Distance4Sq(f2), 120.f));
}

TEST(Float4Distance4)
{
	float4 f(1.f, 2.f, 3.f, 4.f);
	float4 f2(-1.f, -2.f, -3.f, -4.f);
	assert(EqualAbs(f.Distance4(f2), Sqrt(120.f)));
}

TEST(Float4Dot3)
{
	float4 f(-1.f, 2.f, 3.f, -4.f);
	float3 f2(2.f, -1.f, -3.f);
	float4 f3(2.f, -1.f, 0.f, 4.f);
	assert(EqualAbs(f.Dot3(f2), -13.f));
	assert(EqualAbs(f.Dot3(f3), -4.f));
}

TEST(Float4Dot4)
{
	float4 f(-1.f, 2.f, 3.f, -4.f);
	float4 f2(2.f, -1.f, 0.f, 4.f);
	assert(EqualAbs(f.Dot4(f2), -20.f));
}

TEST(Float4Cross3)
{
	float4 f(-1.f, 2.f, 3.f, -4.f);
	float4 f2(2.f, -1.f, 0.f, 4.f);
	assert(f.Cross3(f2).Equals(f.Cross3(f2.xyz())));

	float4 f3 = f.Cross3(f2);
	assert(f3.Equals(float4(3.f, 6.f, -3.f, 0.f)));

	float4 z = float4::unitX.Cross3(float4::unitY);
	float4 y = float4::unitZ.Cross3(float4::unitX);
	float4 x = float4::unitY.Cross3(float4::unitZ);
	assert(x.Equals(float4(1,0,0,0)));
	assert(y.Equals(float4(0,1,0,0)));
	assert(z.Equals(float4(0,0,1,0)));
}

TEST(Float4FromScalar)
{
	float4 f = float4::FromScalar(1.f);
	assert(f.Equals(float4(1,1,1,1)));
	f.SetFromScalar(-1.f);
	assert(f.Equals(float4(-1,-1,-1,-1)));

	float4 f2(3.f, 3.f, 3.f, -9.f);
	f.SetFromScalar(3.f, -9.f);
	assert(f.Equals(f2));
}

TEST(Float4Set)
{
	float4 f = float4(1,2,3,4);
	f.Set(5,6,7,8);
	assert(f.Equals(float4(5,6,7,8)));
}

TEST(Float4OpAdd)
{
	float4 f = float4(1,2,3,4);
	float4 f2 = float4(-5.f, -6.f, -7.f, -8.f);
	float4 f3 = f + f2;
	assert(f3.Equals(float4(-4.f, -4.f, -4.f, -4.f)));
}

TEST(Float2OpAddUnary)
{
	float2 f(1,2);
	float2 g = +f;
	assert(f.Equals(g));
}

TEST(Float3OpAddUnary)
{
	float3 f(1,2,3);
	float3 g = +f;
	assert(f.Equals(g));
}

TEST(Float4OpAddUnary)
{
	float4 f(1,2,3,4);
	float4 g = +f;
	assert(f.Equals(g));
}

TEST(Float4OpSub)
{
	float4 f = float4(1,2,3,4);
	float4 f2 = float4(-5.f, -6.f, -7.f, -8.f);
	float4 f3 = f - f2;
	assert(f3.Equals(float4(6.f, 8.f, 10.f, 12.f)));
}

TEST(Float4OpNeg)
{
	float4 f = float4(1,2,3,4);
	float4 f3 = -f;
	assert(f3.Equals(float4(-1.f, -2.f, -3.f, -4.f)));
}

TEST(Float4OpMul)
{
	float4 f = float4(1,2,3,4);
	float scalar = -2.f;
	
	float4 f2 = f * scalar;
	float4 f3 = scalar * f;
	assert(f2.Equals(f3));

	assert(f2.Equals(float4(-2.f, -4.f, -6.f, -8.f)));
}

TEST(Float4OpDiv)
{
	float4 f = float4(1,2,4,8);
	float scalar = -2.f;
	
	float4 f2 = f / scalar;
	assert(f2.Equals(float4(-0.5f, -1.f, -2.f, -4.f)));
}

TEST(Float4OpAddAssign)
{
	float4 f = float4(1,2,3,4);
	float4 f2 = float4(-5.f, -6.f, -7.f, -8.f);
	float4 f3 = f;
	f3 += f2;
	assert(f3.Equals(float4(-4.f, -4.f, -4.f, -4.f)));
}

TEST(Float4OpSubAssign)
{
	float4 f = float4(1,2,3,4);
	float4 f2 = float4(-5.f, -6.f, -7.f, -8.f);
	float4 f3 = f;
	f3 -= f2;
	assert(f3.Equals(float4(6.f, 8.f, 10.f, 12.f)));
}

TEST(Float4OpMulAssign)
{
	float4 f = float4(1,2,3,4);
	float scalar = -2.f;
	
	float4 f2 = f;
	f2 *= scalar;
	assert(f2.Equals(float4(-2.f, -4.f, -6.f, -8.f)));
}

TEST(Float4OpDivAssign)
{
	float4 f = float4(1,2,4,8);
	float scalar = -2.f;
	
	float4 f2 = f;
	f2 /= scalar;
	assert(f2.Equals(float4(-0.5f, -1.f, -2.f, -4.f)));
}

TEST(Float4Add)
{
	float4 f = float4(1,2,3,4);
	float scalar = 5.f;
	float4 f2 = f.Add(scalar);
	assert(f2.Equals(float4(6.f, 7.f, 8.f, 9.f)));
}

TEST(Float4Sub)
{
	float4 f = float4(1,2,3,4);
	float scalar = 5.f;
	float4 f2 = f.Sub(scalar);
	assert(f2.Equals(float4(-4.f, -3.f, -2.f, -1.f)));
}

TEST(Float4SubLeft)
{
	float4 f = float4(1,2,3,4);
	float scalar = 5.f;
	float4 f2 = f.SubLeft(scalar);
	assert(f2.Equals(float4(4.f, 3.f, 2.f, 1.f)));
}

TEST(Float4DivLeft)
{
	float4 f = float4(-1,2,4,8);
	float scalar = -8.f;
	float4 f2 = f.DivLeft(scalar);
	assert(f2.Equals(float4(8.f, -4.f, -2.f, -1.f)));
}

TEST(Float4Mul)
{
	float4 f = float4(1,2,3,4);
	float4 f2 = float4(-2.f, -4.f, 2.f, 1.f);
	assert(f.Mul(f2).Equals(float4(-2.f, -8.f, 6.f, 4.f)));
}

TEST(Float4Div)
{
	float4 f = float4(1,1,4,8);
	float4 f2 = float4(-2.f, -4.f, 2.f, 1.f);
	assert(f.Div(f2).Equals(float4(-0.5f, -0.25f, 2.f, 8.f)));
}

#ifdef MATH_SSE

// Testing various strategies for loading a single 'float' scalar to a __m128.

/* 	VS2010 with AVX generates, BAD! 
	vmovss	xmm1, DWORD PTR [edx+eax*4]
	vxorps	xmm0, xmm0, xmm0
	vmovss	xmm0, xmm0, xmm1
	Without AVX: GOOD
	movss	xmm0, DWORD PTR [edx+eax*4] */
BENCHMARK(float_to_Float4_ss, "sse")
{
	simd4f scale = _mm_set_ss(f[i]);
	v[i] = scale;
}
BENCHMARK_END;

/* 	VS2010 with AVX generates the following: GOOD if want a 4-vector [x,x,x,x].
		vmovss	xmm0, DWORD PTR [edx+eax*4]
		vshufps	xmm0, xmm0, xmm0, 0
	without AVX:
		movss	xmm0, DWORD PTR [edx+eax*4]
		shufps	xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_set1, "sse")
{
	simd4f scale = set1_ps(f[i]);
	v[i] = scale;
}
BENCHMARK_END;

/* 	VS2010 generates: vshufps is/should be unneeded, if not interested in higher channels!
	vmovss	xmm0, DWORD PTR [edx+eax*4]
	vshufps	xmm0, xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_load1, "sse")
{
	simd4f scale = load1_ps(&f[i]);
	v[i] = scale;
}
BENCHMARK_END;

/*
#ifdef _MSC_VER
// Manually trying to generate only 'vmovss', but this is not working out too well!
BENCHMARK(float_to_Float4_inline_asm)
{
	__m128 scale;
	float *f_addr = &f[i];
	__asm {
		movss xmm0, f_addr
		movss scale, xmm0
	}
	v[i] = scale;
}
BENCHMARK_END;
#endif
*/
// Test what setx_ps produces:
/* 	vmovss	xmm1, DWORD PTR [edx+eax*4]
	vxorps	xmm0, xmm0, xmm0
	vmovss	xmm1, xmm0, xmm1 */
BENCHMARK(float_to_Float4_macro1, "sse")
{
	simd4f scale = setx_ps(f[i]);
	v[i] = scale;
}
BENCHMARK_END;

#ifdef MATH_SSE
/* VS2010 with AVX enabled generates this BAD code(!):
	vmovss	xmm0, DWORD PTR [edx+eax*4]
	vxorps	xmm1, xmm1, xmm1
	vmovss	xmm0, xmm1, xmm0
	vshufps	xmm0, xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_load_swizzle, "sse")
{
	simd4f scale = shuffle1_ps(_mm_load_ss(&f[i]), _MM_SHUFFLE(0,0,0,0));
	v[i] = scale;
}
BENCHMARK_END;

/* VS2010 with AVX enabled generates this BAD code(!):
	vmovss	xmm0, DWORD PTR [edx+eax*4]
	vxorps	xmm1, xmm1, xmm1
	vmovss	xmm0, xmm1, xmm0
	vshufps	xmm0, xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_macro_swizzle, "sse")
{
	simd4f scale = shuffle1_ps(setx_ps(f[i]), _MM_SHUFFLE(0,0,0,0));
	v[i] = scale;
}
BENCHMARK_END;

BENCHMARK(sse_shuffle1, "sse")
{
	simd4f scale = shuffle1_ps(v[i].v, _MM_SHUFFLE(0,1,2,3));
	v[i] = scale;
}
BENCHMARK_END;

BENCHMARK(sse_shuffle_ps, "sse")
{
	simd4f scale = _mm_shuffle_ps(v[i].v, v[i].v, _MM_SHUFFLE(0,1,2,3));
	v[i] = scale;
}
BENCHMARK_END;
#endif // ~MATH_SSE

#ifdef MATH_SSE2
BENCHMARK(sse_shuffle_epi32, "sse")
{
	simd4f scale = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((v[i].v)), _MM_SHUFFLE(0,1,2,3)));
	v[i] = scale;
}
BENCHMARK_END;
#endif

#endif // ~MATH_SIMD

// Benchmark scalar ops so that we can compare scalar vs vector primitive ops costs.
BENCHMARK(FloatAdd, "float + float")
{
	f[i] = pf[i] + uf[i];
}
BENCHMARK_END;

BENCHMARK(FloatSub, "float - float")
{
	f[i] = pf[i] - uf[i];
}
BENCHMARK_END;

BENCHMARK(FloatMul, "float * float")
{
	f[i] = pf[i] * uf[i];
}
BENCHMARK_END;

BENCHMARK(FloatDiv, "float / float")
{
	f[i] = uf[i] / pf[i];
}
BENCHMARK_END;

BENCHMARK(Float4_Add, "float4 + float4")
{
	v3[i] = v[i] + v2[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Add_simd, "test against Float4_Add")
{
	v3[i] = add_ps(v[i], v2[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_AddEq, "float4 += float4")
{
	v3[i] += v2[i];
}
BENCHMARK_END;

BENCHMARK(Float4_Sub, "float4 - float4")
{
	v3[i] = v[i] - v2[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Sub_simd, "test against Float4_Sub")
{
	v3[i] = sub_ps(v[i], v2[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_SubEq, "float4 -= float4")
{
	v3[i] -= v2[i];
}
BENCHMARK_END;

BENCHMARK(Float4_Mul, "float4 * scalar")
{
	v3[i] = v[i] * f[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Mul_simd, "test against Float4_Mul")
{
	v3[i] = muls_ps(v[i], f[i]);
}
BENCHMARK_END;
#endif

// Temp: Testing random seen cache(?) behavior, where running the identical code
//       again produces *slower* results.
BENCHMARK(Float4_Mul_Again, "test against Float4_Mul")
{
	v3[i] = v[i] * f[i];
}
BENCHMARK_END;

BENCHMARK(Float4_MulEq, "float4 *= scalar")
{
	v3[i] *= f[i];
}
BENCHMARK_END;

// Temp: Testing random seen cache(?) behavior, where running the identical code
//       again produces *slower* results.
BENCHMARK(Float4_MulEq_Again, "test against Float4_MulEq")
{
	v3[i] *= f[i];
}
BENCHMARK_END;

BENCHMARK(Float4_Mul_scalar, "test against Float4_Mul")
{
	v3[i].x = v[i].x * f[i];
	v3[i].y = v[i].y * f[i];
	v3[i].z = v[i].z * f[i];
	v3[i].w = v[i].w * f[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Mul_sse, "test against Float4_Mul")
{
	v3[i].v = muls_ps(v[i].v, f[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Mul_float4, "float4::Mul(float4)")
{
	v3[i] = v[i].Mul(v2[i]);
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Mul_float4_simd, "test against Float4_Mul_float4")
{
	v3[i] = mul_ps(v[i], v2[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Div, "float4 / scalar")
{
	v3[i] = v[i] / f[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Div_simd, "test against Float4_Div")
{
	v3[i] = div_ps(v[i], set1_ps(f[i]));
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_DivEq, "float4 /= scalar")
{
	v3[i] /= f[i];
}
BENCHMARK_END;

BENCHMARK(Float4_Div_float4, "float4::Div(float4)")
{
	v3[i] = v[i].Div(v2[i]);
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Div_float4_simd, "test against Float4_Div_float4")
{
	v3[i] = div_ps(v[i], v2[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Abs, "float4::Abs")
{
	v3[i] = v[i].Abs();
}
BENCHMARK_END;

BENCHMARK(Float4_Neg, "float4::Neg")
{
	v3[i] = -v[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Neg_simd, "test against Float4_Neg")
{
	v3[i] = neg_ps(v[i]);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Length3, "float4::Length3")
{
	f[i] = v[i].Length3();
}
BENCHMARK_END;

BENCHMARK(Float4_Length4, "float4::Length4")
{
	f[i] = v[i].Length4();
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(vec4_length_float, "test against Float4_Length4")
{
	f[i] = vec4_length_float(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec4_length_ps, "test against Float4_Length4")
{
	v3[i] = vec4_length_ps(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec3_length_float, "test against Float3_Length")
{
	f[i] = vec3_length_float(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec3_length_ps, "test against Float3_Length")
{
	v3[i] = vec3_length_ps(v[i].v);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Length4Sq, "float4::Length4Sq")
{
	f[i] = v[i].LengthSq4();
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(vec4_length_sq_float, "test against Float4_Length4Sq")
{
	f[i] = vec4_length_sq_float(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec4_length_sq_ps, "test against Float4_Length4Sq")
{
	v3[i] = vec4_length_sq_ps(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec3_length_sq_float, "test against Float4_LengthSq")
{
	f[i] = vec3_length_sq_float(v[i].v);
}
BENCHMARK_END;

BENCHMARK(vec3_length_sq_ps, "test against Float4_LengthSq")
{
	v3[i] = vec3_length_sq_ps(v[i].v);
}
BENCHMARK_END;
#endif

BENCHMARK(Float4_Normalize3, "float4::Normalize3")
{
	v[i] = nv[i];
	f[i] = v[i].Normalize3();
}
BENCHMARK_END;

BENCHMARK(Float4_Normalize4, "float4::Normalize4")
{
	v[i] = nv[i];
	f[i] = v[i].Normalize4();
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(vec4_normalize, "test against Float4_Normalize4")
{
	v3[i] = vec4_normalize(v[i]);
}
BENCHMARK_END;

BENCHMARK(vec3_normalize, "test against Float4_Normalize3")
{
	v3[i] = vec3_normalize(v[i]);
}
BENCHMARK_END;

BENCHMARK(float4_Lerp, "float4::Lerp")
{
	v3[i] = nv[i].Lerp(nv2[i], uf[i]);
}
BENCHMARK_END

BENCHMARK(vec4_lerp, "test against float4_Lerp")
{
	v3[i] = vec4_lerp(v[i], v2[i], uf[i]);
}
BENCHMARK_END

RANDOMIZED_TEST(vec4_lerp)
{
	float4 f = float4::RandomGeneral(rng, -100.f, 100.f);
	float4 f2 = float4::RandomGeneral(rng, -100.f, 100.f);
	f.w = f2.w;
	float t = rng.Float();
	float4 correct = f.Lerp(f2, t);
	float4 f3 = vec4_lerp(f, f2, t);
	assert(f3.Equals(correct));
}

#endif

TEST(float4_Lerp)
{
	float4 a(2,2,2,1);
	float4 b(10,10,10,1);
	assert(a.Lerp(b, 0.f).Equals(2,2,2,1));
	assert(a.Lerp(b, 1.f).Equals(10,10,10,1));
	assert(a.Lerp(b, 0.5f).Equals(6,6,6,1));
}

// Test that copying uninitialized memory around is ok in the sense that it doesn't crash or assert:

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#endif

RANDOMIZED_TEST(copy_uninitialized_float2)
{
	float2 a;
	float2 b = a;
	uninitializedFloat2 = b;
}

RANDOMIZED_TEST(copy_uninitialized_float3)
{
	float3 a;
	float3 b = a;
	uninitializedFloat3 = b;
}

RANDOMIZED_TEST(copy_uninitialized_float4)
{
	float4 a;
	float4 b = a;
	uninitializedFloat4 = b;
}

RANDOMIZED_TEST(copy_uninitialized_float3x3)
{
	float3x3 a;
	float3x3 b = a;
	uninitializedFloat3x3 = b;
}

RANDOMIZED_TEST(copy_uninitialized_float3x4)
{
	float3x4 a;
	float3x4 b = a;
	uninitializedFloat3x4 = b;
}

RANDOMIZED_TEST(copy_uninitialized_float4x4)
{
	float4x4 a;
	float4x4 b = a;
	uninitializedFloat4x4 = b;
}

RANDOMIZED_TEST(copy_uninitialized_Quat)
{
	Quat a;
	Quat b = a;
	uninitializedQuat = b;
}

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

// Also, it must be safe to assign NaN types around:

RANDOMIZED_TEST(copy_nan_float2)
{
	float2 a = float2::nan;
	float2 b = a;
	uninitializedFloat2 = b;
}

RANDOMIZED_TEST(copy_nan_float3)
{
	float3 a = float3::nan;
	float3 b = a;
	uninitializedFloat3 = b;
}

RANDOMIZED_TEST(copy_nan_float4)
{
	float4 a = float4::nan;
	float4 b = a;
	uninitializedFloat4 = b;
}

RANDOMIZED_TEST(copy_nan_float3x3)
{
	float3x3 a = float3x3::nan;
	float3x3 b = a;
	uninitializedFloat3x3 = b;
}

RANDOMIZED_TEST(copy_nan_float3x4)
{
	float3x4 a = float3x4::nan;
	float3x4 b = a;
	uninitializedFloat3x4 = b;
}

RANDOMIZED_TEST(copy_nan_float4x4)
{
	float4x4 a = float4x4::nan;
	float4x4 b = a;
	uninitializedFloat4x4 = b;
}

RANDOMIZED_TEST(copy_nan_Quat)
{
	Quat a = Quat::nan;
	Quat b = a;
	uninitializedQuat = b;
}

RANDOMIZED_TEST(vec_PerpendicularBasis)
{
	vec a = vec::RandomBox(rng, -100.f, 100.f);
	if (!a.IsZero())
	{
		vec b, c;

		a.PerpendicularBasis(b, c);
		assert2(a.IsPerpendicular(b), a, b);
		assert2(a.IsPerpendicular(c), a, c);

		a.Normalize();
		a.PerpendicularBasis(b, c);
		assert2(a.IsPerpendicular(b), a, b);
		assert2(a.IsPerpendicular(c), a, c);
		assert1(b.IsNormalized(), b.Length());
		assert1(c.IsNormalized(), c.Length());
	}
}

BENCHMARK(float4_PerpendicularBasis, "float4::PerpendicularBasis")
{
	float4 b, c;
	nv[i].PerpendicularBasis(b, c);
	dummyResultVec += FLOAT4_TO_DIR(b + c);
}
BENCHMARK_END;

// Pixar orthonormal basis code: https://graphics.pixar.com/library/OrthonormalB/paper.pdf
void branchlessONB(const vec &n, vec &b1, vec &b2)
{
	float sign = copysignf(1.0f, n.z);
	const float a = -1.0f / (sign + n.z);
	const float b = n.x * n.y * a;
	b1 = DIR_VEC(1.0f + sign * n.x * n.x * a, sign * b,             -sign * n.x);
	b2 = DIR_VEC(                          b, sign + n.y * n.y * a,        -n.y);
}

BENCHMARK(float4_PerpendicularBasis_Pixar, "float4 Pixar orthonormal basis code")
{
	vec b, c;
	branchlessONB(FLOAT4_TO_DIR(nv[i]), b, c);
	dummyResultVec += b + c;
}
BENCHMARK_END;

BENCHMARK(float3x3_RotateFromTo, "float3x3::RotateFromTo")
{
	uninitializedFloat3x3 = float3x3::RotateFromTo(nv[i].Float3Part(), nv[i+1].Float3Part());
}
BENCHMARK_END;

BENCHMARK(float4x4_RotateFromTo, "float4x4::RotateFromTo")
{
	uninitializedFloat4x4 = float4x4::RotateFromTo(nv[i], nv[i+1]);
}
BENCHMARK_END;

static double sqe(const float3 &v)
{
	double d = Sqrt(float4d(v, 0.f).LengthSq()) - 1.0;
	return d*d;
}

static double sqe(const float4 &v)
{
	double d = Sqrt(float4d(v).LengthSq()) - 1.0;
	return d*d;
}

static double sqe(const float3 &v, const float3 &w)
{
	double d = float4d(v, 0.f).Dot(float4d(w, 0.f));
	return d*d;
}

static double sqe(const float4 &v, const float4 &w)
{
	double d = float4d(v).Dot(float4d(w));
	return d*d;
}

static double sqe(const float3x3 &m)
{
	return (sqe(m.Col(0)) + sqe(m.Col(1)) + sqe(m.Col(2)) + sqe(m.Col(0), m.Col(1)) + sqe(m.Col(1), m.Col(2)) + sqe(m.Col(0), m.Col(2))) / 6.0;
}

static double sqe(const float4x4 &m)
{
	return sqe(m.Float3x3Part());
}

UNIQUE_TEST(float4_PerpendicularBasis_precision)
{
	const int C = 2;
	double maxRelError[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		vec v = vec::RandomDir(rng, 1.f);
		v.Normalize();

		vec b, c;
		v.PerpendicularBasis(b, c);
		double error = (sqe(v) + sqe(b) + sqe(c) + sqe(v, b) + sqe(b, c) + sqe(c, v)) / 6.0;
		maxRelError[0] = Max(error, maxRelError[0]);

		branchlessONB(v, b, c);
		error = (sqe(v) + sqe(b) + sqe(c) + sqe(v, b) + sqe(b, c) + sqe(c, v)) / 6.0;
		maxRelError[1] = Max(error, maxRelError[1]);
	}

	LOGI("Max relative error with float4::PerpendicularBasis: %e", maxRelError[0]);
	LOGI("Max relative error with branchlessONB: %e", maxRelError[1]);
}

UNIQUE_TEST(float4x4_RotateFromTo_precision)
{
	const int C = 3;
	double maxRelError[C] = {};

	for(int i = 0; i < 1000000; ++i)
	{
		vec v = vec::RandomDir(rng, 1.f);
		v.Normalize();
		vec v2 = vec::RandomDir(rng, 1.f);
		v2.Normalize();

		float3x3 r3 = float3x3::RotateFromTo(v.xyz(), v2.xyz());
		vec rotated3 = DIR_VEC(r3.MulDir(v.xyz()));
		double error = rotated3.DistanceSq(v2) + sqe(r3);
		maxRelError[0] = Max(error, maxRelError[0]);

		float4x4 r = float4x4::RotateFromTo(v, v2);
		float4 rotated = r.Mul(DIR_TO_FLOAT4(v));
		error = rotated.DistanceSq(DIR_TO_FLOAT4(v2)) + sqe(r);
		maxRelError[1] = Max(error, maxRelError[1]);

		Quat q = Quat::RotateFromTo(v, v2);
		vec qv = q.Transform(v);
		error = qv.DistanceSq(v2) + sqe(q.ToFloat3x3());
		maxRelError[2] = Max(error, maxRelError[2]);
	}

	LOGI("Max relative error with float3x3::RotateFromTo: %e", maxRelError[0]);
	LOGI("Max relative error with float4x4::RotateFromTo: %e", maxRelError[1]);
	LOGI("Max relative error with Quat::RotateFromTo: %e", maxRelError[2]);
}

UNIQUE_TEST(float2_ConvexHull_Case)
{
	float2 p[4] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1) };
	float2 h[4] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1) };
	int numPointsInConvexHull = float2::ConvexHullInPlace(h, 4);
	assert(numPointsInConvexHull == 4);
	MARK_UNUSED(numPointsInConvexHull);

	for(int i = 0; i < numPointsInConvexHull; ++i)
		assert(float2::ConvexHullContains(h, numPointsInConvexHull, p[i]));
	MARK_UNUSED(p);
	assert(float2::ConvexHullContains(h, numPointsInConvexHull, float2(0,0)));
}

UNIQUE_TEST(float2_ConvexHull_Case2)
{
	std::vector<float2> pts;
	pts.push_back(float2(-13.965818, 24.836142));
	pts.push_back(float2(-10.308306, 14.270721));
	pts.push_back(float2(-44.626469, 113.405243));
	pts.push_back(float2(-72.140152, 192.883652));
	int numPointsInConvexHull = float2::ConvexHullInPlace(&pts[0], pts.size());
	assert(numPointsInConvexHull == 4);
	MARK_UNUSED(numPointsInConvexHull);

	for(int i = 0; i < numPointsInConvexHull; ++i)
		assert(float2::ConvexHullContains(&pts[0], numPointsInConvexHull, pts[i]));
}

UNIQUE_TEST(float2_ConvexHull_Case3)
{
	std::vector<float2> pts;
	pts.push_back(float2(-10.283741, -147.214920));
	pts.push_back(float2(-0.912153, -130.096695));
	pts.push_back(float2(60.997635, -17.011709));
	pts.push_back(float2(61.076935, -16.866882));
	int numPointsInConvexHull = float2::ConvexHullInPlace(&pts[0], pts.size());
	assert(numPointsInConvexHull == 3);
	MARK_UNUSED(numPointsInConvexHull);

	for(int i = 0; i < numPointsInConvexHull; ++i)
		assert(float2::ConvexHullContains(&pts[0], numPointsInConvexHull, pts[i]));
}

RANDOMIZED_TEST(float2_ConvexHull)
{
	const int n = 100;
	float2 h[n];
	float2 p[n];
	for(int i = 0; i < n; ++i)
		h[i] = p[i] = float2::RandomBox(rng, -100.f, 100.f);

	int numPointsInConvexHull = float2::ConvexHullInPlace(h, n);
	assert(numPointsInConvexHull >= 3);
	MARK_UNUSED(numPointsInConvexHull);

	for(int i = 0; i < n; ++i)
		assert(float2::ConvexHullContains(h, numPointsInConvexHull, p[i]));
}

BENCHMARK(float2_ConvexHull, "float2_ConvexHull")
{
	const int n = 100;
	float2 h[n];
	for(int j = 0; j < n; ++j)
		h[j] = float2::RandomBox(rng, -100.f, 100.f);

	dummyResultInt += float2::ConvexHullInPlace(h, n);
}
BENCHMARK_END;

UNIQUE_TEST(float2_MinAreaRect_Case)
{
	float2 p[4] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1) };
	float2 h[4] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1) };
	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, 4, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < 4; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}

UNIQUE_TEST(float2_MinAreaRect_Case_2)
{
	float2 p[5] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1), float2(0.75f, 0.75f) };
	float2 h[5] = { float2(-1, 0), float2(1,0), float2(0,1), float2(0,-1), float2(0.75f, 0.75f) };
	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, 5, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < 5; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}

UNIQUE_TEST(float2_MinAreaRect_Case_3)
{
	float2 p[3] = { float2(-74.0205307f,18.4061508f), float2(55.5148621f,0.11618042f), float2(89.0816193f,-47.8109818f) };
	float2 h[3] = { p[0], p[1], p[2] };

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, 3, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < 3; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}

UNIQUE_TEST(float2_MinAreaRect_Case_4)
{
	const int n = 7;
	float2 p[n];
	p[0] = float2(-10.606602668762207f,-10.f);
	p[1] = float2(31.819807052612305f,15.f);
	p[2] = float2(17.677671432495117f,-10.f);
	p[3] = float2(7.071068286895752f,20.f);
	p[4] = float2(-31.819807052612305f,-10.f);
	p[5] = float2(31.819807052612305f,-10.f);
	p[6] = float2(3.535534143447876f,20.f);
	float2 h[n];
	memcpy(h, p, sizeof(h));

	float2 c[n];
	memcpy(c, p, sizeof(c));

	int numPointsInConvexHull = float2::ConvexHullInPlace(c, n);

	for(int i = 0; i < n; ++i)
		assert(float2::ConvexHullContains(c, numPointsInConvexHull, p[i]));
	MARK_UNUSED(numPointsInConvexHull);

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, n, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < n; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}


UNIQUE_TEST(float2_MinAreaRect_Case_5)
{
	const int n = 15;
	float2 p[n];
	p[0] = float2(-12.480753898620605,-20);
	p[1] = float2(-31.895261764526367,-25);
	p[2] = float2(18.02775764465332,1.9969324682733713e-7);
	p[3] = float2(-18.027755737304688,10);
	p[4] = float2(-9.707253456115723,-35);
	p[5] = float2(-5.9604644775390625e-007,-10);
	p[6] = float2(-19.414506912231445,-30);
	p[7] = float2(36.05551528930664,10);
	p[8] = float2(-37.442264556884766,-15);
	p[9] = float2(-16.641006469726562,-15);
	p[10] = float2(5.547001838684082,-20);
	p[11] = float2(8.320503234863281,-15);
	p[12] = float2(-12.480754852294922,-4.6083059146440064e-8);
	p[13] = float2(20.801258087158203,-5);
	p[14] = float2(8.320503234863281,-15);
	float2 h[n];
	memcpy(h, p, sizeof(h));

	float2 c[n];
	memcpy(c, p, sizeof(c));

	int numPointsInConvexHull = float2::ConvexHullInPlace(c, n);

	for(int i = 0; i < n; ++i)
		assert(float2::ConvexHullContains(c, numPointsInConvexHull, p[i]));
	MARK_UNUSED(numPointsInConvexHull);

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, n, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < n; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}



UNIQUE_TEST(float2_MinAreaRect_Case_6)
{
	const int n = 14;//19;
	float2 p[n];
/*
	p[0] = float2(3.5777089595794678,-6.245760917663574);
	p[1] = float2(-6.260990619659424,-2.24457049369812);
	p[2] = float2(2.2360680103302,-5.855401039123535);
	p[3] = float2(4.0249223709106445,1.7566202878952026);
	p[4] = float2(7.1554179191589355,-2.927700877189636e-1);
	p[5] = float2(-3.130495309829712,2.5373404026031494);
	p[6] = float2(3.5777089595794678,-1.366260290145874);
	p[7] = float2(-4.4721360206604,-5.367451190948486);
	p[8] = float2(7.1554179191589355,-2.9277002811431885e-1);
	p[9] = float2(0,-8.295151710510254);
	p[10] = float2(6.260990619659424,-6.538531303405762);
	p[11] = float2(1.341640830039978,9.759007394313812e-2);
	p[12] = float2(-6.70820426940918,-9.759001731872559e-1);
	p[13] = float2(-4.919349670410156,6.6361212730407715);
	p[14] = float2(2.2360680103302,4.3915510177612305);
	p[15] = float2(1.341640830039978,-5.757811069488525);
	p[16] = float2(-2.2360680103302,3.9036006927490234);
	p[17] = float2(-2.683281660079956,-5.074680805206299);
	p[18] = float2(-7.602631568908691,9.759001433849335e-2);
*/

p[0] = float2(-4.800000190734863,-4.196885108947754);
p[1] = float2(-3.9999985694885254e-1,-7.295520305633545);
p[2] = float2(1.8000000715255737,-6.393386363983154);
p[3] = float2(-2.3999996185302734,3.294750690460205);
p[4] = float2(4.800000190734863,3.216304302215576);
p[5] = float2(7.399999618530273,2.588732957839966);
p[6] = float2(-2.6000003814697266,-6.23649263381958);
p[7] = float2(4.399999618530273,-1.137473225593567);
p[8] = float2(-4.799999713897705,-4.196885585784912);
p[9] = float2(6.000000238418579e-1,8.982118606567383);
p[10] = float2(1.200000286102295,-1.6473760604858398);
p[11] = float2(3.6000001430511475,-2.000385046005249);
p[12] = float2(7,3.1378581523895264);
p[13] = float2(-4.599999904632568,-2.510286331176758);

	float2 h[n];
	memcpy(h, p, sizeof(h));

	float2 c[n];
	memcpy(c, p, sizeof(c));

	int numPointsInConvexHull = float2::ConvexHullInPlace(c, n);

	for(int i = 0; i < n; ++i)
		assert(float2::ConvexHullContains(c, numPointsInConvexHull, p[i]));
	MARK_UNUSED(numPointsInConvexHull);

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(h, n, center, uDir, vDir, minU, maxU, minV, maxV);

	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	const float epsilon = 1e-3f;
	for(int i = 0; i < n; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}
RANDOMIZED_TEST(float2_MinAreaRect)
{
	const int s = 100;
	const int n = rng.Int(3, s);
	float2 h[s];
	float2 p[s];
	for(int i = 0; i < n; ++i)
		h[i] = p[i] = float2::RandomBox(rng, -100.f, 100.f);

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	float2::MinAreaRectInPlace(p, n, center, uDir, vDir, minU, maxU, minV, maxV);

	const float epsilon = 1e-3f;
	float diffUMin = FLOAT_INF, diffUMax = FLOAT_INF, diffVMin = FLOAT_INF, diffVMax = FLOAT_INF;
	for(int i = 0; i < n; ++i)
	{
		float2 d = p[i];
		float x = d.Dot(uDir);
		diffUMin = MATH_NS::Min(diffUMin, x - minU);
		diffUMax = MATH_NS::Min(diffUMax, maxU - x);
		assert3(x >= minU-epsilon && x <= maxU+epsilon, x, minU, maxU);
		float y = d.Dot(vDir);
		diffVMin = MATH_NS::Min(diffVMin, y - minV);
		diffVMax = MATH_NS::Min(diffVMax, maxV - y);
		assert3(y >= minV-epsilon && y <= maxV+epsilon, y, minV, maxV);
	}
	assert1(diffUMin <= 1e-5f, diffUMin);
	assert1(diffUMax <= 1e-5f, diffUMax);
	assert1(diffVMin <= 1e-5f, diffVMin);
	assert1(diffVMax <= 1e-5f, diffVMax);
}

BENCHMARK(float2_MinAreaRect, "float2::MinAreaRect")
{
	const int n = 100;
	float2 p[n];
	for(int j = 0; j < n; ++j)
		p[j] = float2::RandomBox(rng, -100.f, 100.f);

	float2 center, uDir, vDir;
	float minU, maxU, minV, maxV;
	dummyResultInt += (int)float2::MinAreaRectInPlace(p, n, center, uDir, vDir, minU, maxU, minV, maxV);
}
BENCHMARK_END;

#ifdef MATH_NEON
UNIQUE_TEST(MatrixTranspose)
{
	simd4f a = set_ps(1.f,2.f,3.f,4.f);
	simd4f b = set_ps(5.f,6.f,7.f,8.f);
	simd4f c = set_ps(9.f,10.f,11.f,12.f);
	simd4f d = set_ps(13.f,14.f,15.f,16.f);
	
	_MM_TRANSPOSE4_PS(a, b, c, d);
	assert(float4(a).Equals(float4(4.f, 8.f, 12.f, 16.f)));
	assert(float4(b).Equals(float4(3.f, 7.f, 11.f, 15.f)));
	assert(float4(c).Equals(float4(2.f, 6.f, 10.f, 14.f)));
	assert(float4(d).Equals(float4(1.f, 5.f, 9.f, 13.f)));
}
#endif

#ifdef MATH_ENABLE_UNCOMMON_OPERATIONS

UNIQUE_TEST(float2_pointwise)
{
	float2 a(12.f, 8.f);
	float2 b(3.f, 4.f);
	float2 c = a;
	c *= b;
	float2 d = a;
	d /= b;
	assert((a*b).Equals(float2(36.f, 32.f)));
	assert((a/b).Equals(float2(4.f, 2.f)));
	assert(c.Equals(float2(36.f, 32.f)));
	assert(d.Equals(float2(4.f, 2.f)));
	assert((12.f / b).Equals(float2(4.f, 3.f)));
}

UNIQUE_TEST(float3_pointwise)
{
	float3 a(12.f, 8.f, 90.f);
	float3 b(3.f, 4.f, 30.f);
	float3 c = a;
	c *= b;
	float3 d = a;
	d /= b;
	assert((a*b).Equals(float3(36.f, 32.f, 2700.f)));
	assert((a/b).Equals(float3(4.f, 2.f, 3.f)));
	assert(c.Equals(float3(36.f, 32.f, 2700.f)));
	assert(d.Equals(float3(4.f, 2.f, 3.f)));
	assert((360.f / b).Equals(float3(120.f, 90.f, 12.f)));
}

UNIQUE_TEST(float4_pointwise)
{
	float4 a(12.f, 8.f, 90.f, -20.f);
	float4 b(3.f, 4.f, 30.f, 10.f);
	float4 c = a;
	c *= b;
	float4 d = a;
	d /= b;
	assert((a*b).Equals(float4(36.f, 32.f, 2700.f, -200.f)));
	assert((a/b).Equals(float4(4.f, 2.f, 3.f, -2.f)));
	assert(c.Equals(float4(36.f, 32.f, 2700.f, -200.f)));
	assert(d.Equals(float4(4.f, 2.f, 3.f, -2.f)));
	assert((3600.f / b).Equals(float4(1200.f, 900.f, 120.f, 360.f)));
}

#endif
