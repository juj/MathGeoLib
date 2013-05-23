#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "TestData.h"
#include "../src/Math/float4_sse.h"
#include "../src/Math/float4_neon.h"

using namespace TestData;

TEST(Float4Swizzled)
{
	float4 f(float2(1,2),3,4);
	float4 f2 = f.Swizzled(2,0,1,3);
	float f3[4] = { 3, 1, 2, 4 };
	assert(f2.Equals(float4(f3)));
}

#if defined(MATH_AVX) || defined(MATH_NEON)
TEST(vec4_permute)
{
	float4 f(float2(1,2),3,4);
	float4 f2 = vec4_permute(f, 2, 0, 1, 3);
	float f3[4] = { 3, 1, 2, 4 };
	assert(f2.Equals(float4(f3)));
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
	assert(EqualAbs(f.x, -1.f / Sqrt(14.f)));
	assert(EqualAbs(f.y, 2.f / Sqrt(14.f)));
	assert(EqualAbs(f.z, 3.f / Sqrt(14.f)));
	assert(EqualAbs(f.w, 1000.f));

	float4 f2(0,0,0, 1000.f);
	oldLength = f2.Normalize3();
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
	assert(EqualAbs(f2.x, -1.f / Sqrt(14.f)));
	assert(EqualAbs(f2.y, 2.f / Sqrt(14.f)));
	assert(EqualAbs(f2.z, 3.f / Sqrt(14.f)));
	assert(EqualAbs(f2.w, 1000.f));
}

TEST(Float4Normalize4)
{
	float4 f(-1.f, 2.f, 3.f, 4.f);
	float oldLength = f.Normalize4();
	assertcmp(oldLength, >, 0);
	assert(EqualAbs(f.x, -1.f / Sqrt(30.f)));
	assert(EqualAbs(f.y, 2.f / Sqrt(30.f)));
	assert(EqualAbs(f.z, 3.f / Sqrt(30.f)));
	assert(EqualAbs(f.w, 4.f / Sqrt(30.f)));

	float4 f2(0,0,0, 0.f);
	oldLength = f2.Normalize4();
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
	__m128 scale = _mm_set_ss(f[i]);
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
	__m128 scale = _mm_set1_ps(f[i]);
	v[i] = scale;
}
BENCHMARK_END;

/* 	VS2010 generates: vshufps is/should be unneeded, if not interested in higher channels!
	vmovss	xmm0, DWORD PTR [edx+eax*4]
	vshufps	xmm0, xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_load1, "sse")
{
	__m128 scale = _mm_load1_ps(&f[i]);
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
// Test what FLOAT_TO_M128 produces:
/* 	vmovss	xmm1, DWORD PTR [edx+eax*4]
	vxorps	xmm0, xmm0, xmm0
	vmovss	xmm1, xmm0, xmm1 */
BENCHMARK(float_to_Float4_macro1, "sse")
{
	__m128 scale = FLOAT_TO_M128(f[i]);
	v[i] = scale;
}
BENCHMARK_END;

/* VS2010 with AVX enabled generates this BAD code(!):
	vmovss	xmm0, DWORD PTR [edx+eax*4]
	vxorps	xmm1, xmm1, xmm1
	vmovss	xmm0, xmm1, xmm0
	vshufps	xmm0, xmm0, xmm0, 0 */
BENCHMARK(float_to_Float4_load_swizzle, "sse")
{
	__m128 scale = shuffle1_ps(_mm_load_ss(&f[i]), _MM_SHUFFLE(0,0,0,0));
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
	__m128 scale = shuffle1_ps(FLOAT_TO_M128(f[i]), _MM_SHUFFLE(0,0,0,0));
	v[i] = scale;
}
BENCHMARK_END;

BENCHMARK(sse_shuffle1, "sse")
{
	__m128 scale = shuffle1_ps(v[i].v, _MM_SHUFFLE(0,1,2,3));
	v[i] = scale;
}
BENCHMARK_END;

BENCHMARK(sse_shuffle_ps, "sse")
{
	__m128 scale = _mm_shuffle_ps(v[i].v, v[i].v, _MM_SHUFFLE(0,1,2,3));
	v[i] = scale;
}
BENCHMARK_END;

BENCHMARK(sse_shuffle_epi32, "sse")
{
	__m128 scale = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((v[i].v)), _MM_SHUFFLE(0,1,2,3)));
	v[i] = scale;
}
BENCHMARK_END;

#endif

BENCHMARK(Float4_Add, "float4 + float4")
{
	v3[i] = v[i] + v2[i];
}
BENCHMARK_END;

#ifdef MATH_SIMD
BENCHMARK(Float4_Add_simd, "test against Float4_Add")
{
	v3[i] = vec4_add_vec4(v[i], v2[i]);
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
	v3[i] = vec4_sub_vec4(v[i], v2[i]);
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
	v3[i] = vec4_mul_float(v[i], f[i]);
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
	v3[i].v = vec4_mul_float(v[i].v, f[i]);
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
	v3[i] = vec4_mul_vec4(v[i], v2[i]);
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
	v3[i] = vec4_div_float(v[i], f[i]);
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
	v3[i] = vec4_div_vec4(v[i], v2[i]);
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
	v3[i] = negate_ps(v[i]);
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
