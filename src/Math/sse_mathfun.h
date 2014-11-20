// This file taken from http://gruntthepeon.free.fr/ssemath/
// Modifications by Jukka Jylänki:
// - Removed MMX support
// - Added modf to for better precision outside [0, 2pi] range.
// - Simplified constants usage syntax.

/* SIMD (SSE1+MMX or SSE2) implementation of sin, cos, exp and log

   Inspired by Intel Approximate Math library, and based on the
   corresponding algorithms of the cephes math library

   The default is to use the SSE1 version. If you define USE_SSE2 the
   the SSE2 intrinsics will be used in place of the MMX intrinsics. Do
   not expect any significant performance improvement with SSE2.
*/

/* Copyright (C) 2007  Julien Pommier

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  (this is the zlib license)
*/

#ifdef MATH_SSE2

#include "SSEMath.h"

static const __m128 _ps_1 = _mm_set1_ps(1.f);
static const __m128 _ps_0p5 = _mm_set1_ps(0.5f);
static const __m128 _ps_min_norm_pos = set1_ps_hex(0x00800000);
static const __m128 _ps_mant_mask = set1_ps_hex(0x7f800000);
static const __m128 _ps_inv_mant_mask = set1_ps_hex(~0x7f800000);
static const __m128 _ps_sign_mask = set1_ps_hex(0x80000000);
static const __m128 _ps_inv_sign_mask = set1_ps_hex(~0x80000000);

static const __m128i _pi32_1 = _mm_set1_epi32(1);
static const __m128i _pi32_inv1 = _mm_set1_epi32(~1);
static const __m128i _pi32_2 = _mm_set1_epi32(2);
static const __m128i _pi32_4 = _mm_set1_epi32(4);
static const __m128i _pi32_0x7f = _mm_set1_epi32(0x7f);

static const __m128 _ps_cephes_SQRTHF = _mm_set1_ps(0.707106781186547524f);
static const __m128 _ps_cephes_log_p0 = _mm_set1_ps(7.0376836292E-2f);
static const __m128 _ps_cephes_log_p1 = _mm_set1_ps(-1.1514610310E-1f);
static const __m128 _ps_cephes_log_p2 = _mm_set1_ps(1.1676998740E-1f);
static const __m128 _ps_cephes_log_p3 = _mm_set1_ps(-1.2420140846E-1f);
static const __m128 _ps_cephes_log_p4 = _mm_set1_ps(+1.4249322787E-1f);
static const __m128 _ps_cephes_log_p5 = _mm_set1_ps(-1.6668057665E-1f);
static const __m128 _ps_cephes_log_p6 = _mm_set1_ps(+2.0000714765E-1f);
static const __m128 _ps_cephes_log_p7 = _mm_set1_ps(-2.4999993993E-1f);
static const __m128 _ps_cephes_log_p8 = _mm_set1_ps(+3.3333331174E-1f);
static const __m128 _ps_cephes_log_q1 = _mm_set1_ps(-2.12194440e-4f);
static const __m128 _ps_cephes_log_q2 = _mm_set1_ps(0.693359375f);

/* natural logarithm computed for 4 simultaneous float 
   return NaN for x <= 0
*/
FORCE_INLINE __m128 log_ps(__m128 x) {
  __m128 one = _ps_1;
  __m128 invalid_mask = _mm_cmple_ps(x, _mm_setzero_ps());
  x = _mm_max_ps(x, _ps_min_norm_pos);  /* cut off denormalized stuff */
  __m128i emm0 = _mm_srli_epi32(_mm_castps_si128(x), 23);
  /* keep only the fractional part */
  x = _mm_and_ps(x, _ps_inv_mant_mask);
  x = _mm_or_ps(x, _ps_0p5);
  emm0 = _mm_sub_epi32(emm0, _pi32_0x7f);
  __m128 e = _mm_cvtepi32_ps(emm0);
  e = _mm_add_ps(e, one);
  /* part2: 
     if( x < SQRTHF ) {
       e -= 1;
       x = x + x - 1.0;
     } else { x = x - 1.0; }
  */
  __m128 mask = _mm_cmplt_ps(x, _ps_cephes_SQRTHF);
  __m128 tmp = _mm_and_ps(x, mask);
  x = _mm_sub_ps(x, one);
  e = _mm_sub_ps(e, _mm_and_ps(one, mask));
  x = _mm_add_ps(x, tmp);
  __m128 z = _mm_mul_ps(x,x);
  __m128 y = _ps_cephes_log_p0;
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p1);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p2);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p3);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p4);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p5);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p6);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p7);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_log_p8);
  y = _mm_mul_ps(y, _mm_mul_ps(x, z));
  tmp = _mm_mul_ps(e, _ps_cephes_log_q1);
  y = _mm_add_ps(y, tmp);
  tmp = _mm_mul_ps(z, _ps_0p5);
  y = _mm_sub_ps(y, tmp);
  tmp = _mm_mul_ps(e, _ps_cephes_log_q2);
  x = _mm_add_ps(x, _mm_add_ps(y, tmp));
  x = _mm_or_ps(x, invalid_mask); // negative arg will be NAN
  return x;
}

static const __m128 _ps_exp_hi = _mm_set1_ps(88.3762626647949f);
static const __m128 _ps_exp_lo = _mm_set1_ps(-88.3762626647949f);

static const __m128 _ps_cephes_LOG2EF = _mm_set1_ps(1.44269504088896341f);
static const __m128 _ps_cephes_exp_C1 = _mm_set1_ps(0.693359375f);
static const __m128 _ps_cephes_exp_C2 = _mm_set1_ps(-2.12194440e-4f);

static const __m128 _ps_cephes_exp_p0 = _mm_set1_ps(1.9875691500E-4f);
static const __m128 _ps_cephes_exp_p1 = _mm_set1_ps(1.3981999507E-3f);
static const __m128 _ps_cephes_exp_p2 = _mm_set1_ps(8.3334519073E-3f);
static const __m128 _ps_cephes_exp_p3 = _mm_set1_ps(4.1665795894E-2f);
static const __m128 _ps_cephes_exp_p4 = _mm_set1_ps(1.6666665459E-1f);
static const __m128 _ps_cephes_exp_p5 = _mm_set1_ps(5.0000001201E-1f);

FORCE_INLINE __m128 exp_ps(__m128 x) {
  __m128 one = _ps_1;

  x = _mm_min_ps(x, _ps_exp_hi);
  x = _mm_max_ps(x, _ps_exp_lo);

  /* express exp(x) as exp(g + n*log(2)) */
  __m128 fx = _mm_mul_ps(x, _ps_cephes_LOG2EF);
  fx = _mm_add_ps(fx, _ps_0p5);

  /* how to perform a floorf with SSE: just below */
  __m128i emm0 = _mm_cvttps_epi32(fx);
  __m128 tmp  = _mm_cvtepi32_ps(emm0);
  /* if greater, substract 1 */
  __m128 mask = _mm_cmpgt_ps(tmp, fx);
  mask = _mm_and_ps(mask, one);
  fx = _mm_sub_ps(tmp, mask);
  tmp = _mm_mul_ps(fx, _ps_cephes_exp_C1);
  __m128 z = _mm_mul_ps(fx, _ps_cephes_exp_C2);
  x = _mm_sub_ps(x, _mm_add_ps(tmp, z));
  z = _mm_mul_ps(x,x);
  __m128 y = _ps_cephes_exp_p0;
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_exp_p1);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_exp_p2);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_exp_p3);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_exp_p4);
  y = _mm_mul_ps(y, x);
  y = _mm_add_ps(y, _ps_cephes_exp_p5);
  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _mm_add_ps(x, one));

  /* build 2^n */
  emm0 = _mm_cvttps_epi32(fx);
  emm0 = _mm_add_epi32(emm0, _pi32_0x7f);
  emm0 = _mm_slli_epi32(emm0, 23);
  __m128 pow2n = _mm_castsi128_ps(emm0);
  y = _mm_mul_ps(y, pow2n);
  return y;
}

static const __m128 _ps_minus_cephes_DP1 = _mm_set1_ps(-0.78515625f);
static const __m128 _ps_minus_cephes_DP2 = _mm_set1_ps(-2.4187564849853515625e-4f);
static const __m128 _ps_minus_cephes_DP3 = _mm_set1_ps(-3.77489497744594108e-8f);
static const __m128 _ps_sincof_p0 = _mm_set1_ps(-1.9515295891E-4f);
static const __m128 _ps_sincof_p1 = _mm_set1_ps( 8.3321608736E-3f);
static const __m128 _ps_sincof_p2 = _mm_set1_ps(-1.6666654611E-1f);
static const __m128 _ps_coscof_p0 = _mm_set1_ps( 2.443315711809948E-005f);
static const __m128 _ps_coscof_p1 = _mm_set1_ps(-1.388731625493765E-003f);
static const __m128 _ps_coscof_p2 = _mm_set1_ps( 4.166664568298827E-002f);
static const __m128 _ps_cephes_FOPI = _mm_set1_ps(1.27323954473516f); // 4 / M_PI

/* evaluation of 4 sines at onces, using only SSE1+MMX intrinsics so
   it runs also on old athlons XPs and the pentium III of your grand
   mother.

   The code is the exact rewriting of the cephes sinf function.
   Precision is excellent as long as x < 8192 (I did not bother to
   take into account the special handling they have for greater values
   -- it does not return garbage for arguments over 8192, though, but
   the extra precision is missing).

   Note that it is such that sinf((float)M_PI) = 8.74e-8, which is the
   surprising but correct result.

   Performance is also surprisingly good, 1.33 times faster than the
   macos vsinf SSE2 function, and 1.5 times faster than the
   __vrs4_sinf of amd's ACML (which is only available in 64 bits). Not
   too bad for an SSE1 function (with no special tuning) !
   However the latter libraries probably have a much better handling of NaN,
   Inf, denormalized and other special arguments..

   On my core 1 duo, the execution of this function takes approximately 95 cycles.

   From what I have observed on the experiments with Intel AMath lib, switching to an
   SSE2 version would improve the perf by only 10%.

   Since it is based on SSE intrinsics, it has to be compiled at -O2 to
   deliver full speed.
*/
FORCE_INLINE __m128 sin_ps(__m128 x) { // any x
#if 0 // Currently expect user to round manually if he knows input can be unbounded.
#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
  // XXX Added in MathGeoLib: Take a modulo of the input in 2pi to try to enhance the precision with large input values.
  x = modf_ps(x, _mm_set1_ps(2.f*pi));
#endif
#endif

  /* extract the sign bit (upper one) */
  __m128 sign_bit = _mm_and_ps(x, _ps_sign_mask);
  /* take the absolute value */
  x = _mm_xor_ps(x, sign_bit);

  /* scale by 4/Pi */
  __m128 y = _mm_mul_ps(x, _ps_cephes_FOPI);

  /* store the integer part of y in mm0 */
  __m128i emm2 = _mm_cvttps_epi32(y);
  /* j=(j+1) & (~1) (see the cephes sources) */
  emm2 = _mm_add_epi32(emm2, _pi32_1);
  emm2 = _mm_and_si128(emm2, _pi32_inv1);
  y = _mm_cvtepi32_ps(emm2);

  /* get the swap sign flag */
  __m128i emm0 = _mm_and_si128(emm2, _pi32_4);
  emm0 = _mm_slli_epi32(emm0, 29);
  /* get the polynom selection mask 
     there is one polynom for 0 <= x <= Pi/4
     and another one for Pi/4<x<=Pi/2

     Both branches will be computed.
  */
  emm2 = _mm_and_si128(emm2, _pi32_2);
  emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
  
  __m128 swap_sign_bit = _mm_castsi128_ps(emm0);
  __m128 poly_mask = _mm_castsi128_ps(emm2);
  sign_bit = _mm_xor_ps(sign_bit, swap_sign_bit);
  
  /* The magic pass: "Extended precision modular arithmetic" 
     x = ((x - y * DP1) - y * DP2) - y * DP3; */
  __m128 xmm1 = _mm_mul_ps(y, _ps_minus_cephes_DP1);
  __m128 xmm2 = _mm_mul_ps(y, _ps_minus_cephes_DP2);
  __m128 xmm3 = _mm_mul_ps(y, _ps_minus_cephes_DP3);
  x = _mm_add_ps(_mm_add_ps(x, xmm1), _mm_add_ps(xmm2, xmm3));

  /* Evaluate the first polynom  (0 <= x <= Pi/4) */
  y = _ps_coscof_p0;
  __m128 z = _mm_mul_ps(x,x);

  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p1);
  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p2);
  y = _mm_mul_ps(y, _mm_mul_ps(z, z));
  __m128 tmp = _mm_mul_ps(z, _ps_0p5);
  y = _mm_sub_ps(y, tmp);
  y = _mm_add_ps(y, _ps_1);
  
  /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

  __m128 y2 = _ps_sincof_p0;
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p1);
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p2);
  y2 = _mm_mul_ps(y2, _mm_mul_ps(z, x));
  y2 = _mm_add_ps(y2, x);

  /* select the correct result from the two polynoms */  
  xmm3 = poly_mask;
  y2 = _mm_and_ps(xmm3, y2); //, xmm3);
  y = _mm_andnot_ps(xmm3, y);
  y = _mm_add_ps(y,y2);
  /* update the sign */
  y = _mm_xor_ps(y, sign_bit);
  return y;
}

/* almost the same as sin_ps */
FORCE_INLINE __m128 cos_ps(__m128 x) { // any x

#if 0 // Currently expect user to round manually if he knows input can be unbounded.
#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
  // XXX Added in MathGeoLib: Take a modulo of the input in 2pi to try to enhance the precision with large input values.
  x = modf_ps(x, _mm_set1_ps(2.f*pi));
#endif
#endif

  /* take the absolute value */
  x = _mm_and_ps(x, _ps_inv_sign_mask);
  /* scale by 4/Pi */
  __m128 y = _mm_mul_ps(x, _ps_cephes_FOPI);
  
  /* store the integer part of y in mm0 */
  __m128i emm2 = _mm_cvttps_epi32(y);
  /* j=(j+1) & (~1) (see the cephes sources) */
  emm2 = _mm_add_epi32(emm2, _pi32_1);
  emm2 = _mm_and_si128(emm2, _pi32_inv1);
  y = _mm_cvtepi32_ps(emm2);

  emm2 = _mm_sub_epi32(emm2, _pi32_2);
  
  /* get the swap sign flag */
  __m128i emm0 = _mm_andnot_si128(emm2, _pi32_4);
  emm0 = _mm_slli_epi32(emm0, 29);
  /* get the polynom selection mask */
  emm2 = _mm_and_si128(emm2, _pi32_2);
  emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
  
  __m128 sign_bit = _mm_castsi128_ps(emm0);
  __m128 poly_mask = _mm_castsi128_ps(emm2);
  /* The magic pass: "Extended precision modular arithmetic" 
     x = ((x - y * DP1) - y * DP2) - y * DP3; */
  __m128 xmm1 = _mm_mul_ps(y, _ps_minus_cephes_DP1);
  __m128 xmm2 = _mm_mul_ps(y, _ps_minus_cephes_DP2);
  __m128 xmm3 = _mm_mul_ps(y, _ps_minus_cephes_DP3);
  x = _mm_add_ps(_mm_add_ps(x, xmm1), _mm_add_ps(xmm2, xmm3));
  
  /* Evaluate the first polynom  (0 <= x <= Pi/4) */
  y = _ps_coscof_p0;
  __m128 z = _mm_mul_ps(x,x);

  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p1);
  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p2);
  y = _mm_mul_ps(y, _mm_mul_ps(z, z));
  __m128 tmp = _mm_mul_ps(z, _ps_0p5);
  y = _mm_sub_ps(y, tmp);
  y = _mm_add_ps(y, _ps_1);
  
  /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

  __m128 y2 = _ps_sincof_p0;
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p1);
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p2);
  y2 = _mm_mul_ps(y2, _mm_mul_ps(z, x));
  y2 = _mm_add_ps(y2, x);

  /* select the correct result from the two polynoms */  
  xmm3 = poly_mask;
  y2 = _mm_and_ps(xmm3, y2); //, xmm3);
  y = _mm_andnot_ps(xmm3, y);
  y = _mm_add_ps(y,y2);
  /* update the sign */
  y = _mm_xor_ps(y, sign_bit);

  return y;
}

/* since sin_ps and cos_ps are almost identical, sincos_ps could replace both of them..
   it is almost as fast, and gives you a free cosine with your sine */
FORCE_INLINE void sincos_ps(__m128 x, __m128 *s, __m128 *c) {
#if 0
#ifdef MATH_SSE41 // _mm_round_ps is SSE4.1
  // XXX Added in MathGeoLib: Take a modulo of the input in 2pi to try to enhance the precision with large input values.
  x = modf_ps(x, _mm_set1_ps(2.f*3.141592654f));
#endif
#endif

  /* extract the sign bit (upper one) */
  __m128 sign_bit_sin = _mm_and_ps(x, _ps_sign_mask);
  /* take the absolute value */
  x = _mm_xor_ps(x, sign_bit_sin);
  
  /* scale by 4/Pi */
  __m128 y = _mm_mul_ps(x, _ps_cephes_FOPI);
    
  /* store the integer part of y in emm2 */
  __m128i emm2 = _mm_cvttps_epi32(y);

  /* j=(j+1) & (~1) (see the cephes sources) */
  emm2 = _mm_add_epi32(emm2, _pi32_1);
  emm2 = _mm_and_si128(emm2, _pi32_inv1);
  y = _mm_cvtepi32_ps(emm2);

  __m128i emm4 = emm2;

  /* get the swap sign flag for the sine */
  __m128i emm0 = _mm_and_si128(emm2, _pi32_4);
  emm0 = _mm_slli_epi32(emm0, 29);
  __m128 swap_sign_bit_sin = _mm_castsi128_ps(emm0);

  /* get the polynom selection mask for the sine*/
  emm2 = _mm_and_si128(emm2, _pi32_2);
  emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
  __m128 poly_mask = _mm_castsi128_ps(emm2);
  /* The magic pass: "Extended precision modular arithmetic" 
     x = ((x - y * DP1) - y * DP2) - y * DP3; */
  __m128 xmm1 = _mm_mul_ps(y, _ps_minus_cephes_DP1);
  __m128 xmm2 = _mm_mul_ps(y, _ps_minus_cephes_DP2);
  __m128 xmm3 = _mm_mul_ps(y, _ps_minus_cephes_DP3);
  x = _mm_add_ps(_mm_add_ps(x, xmm1), _mm_add_ps(xmm2, xmm3));

  emm4 = _mm_sub_epi32(emm4, _pi32_2);
  emm4 = _mm_andnot_si128(emm4, _pi32_4);
  emm4 = _mm_slli_epi32(emm4, 29);
  __m128 sign_bit_cos = _mm_castsi128_ps(emm4);

  sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);
  
  /* Evaluate the first polynom  (0 <= x <= Pi/4) */
  __m128 z = _mm_mul_ps(x,x);
  y = _ps_coscof_p0;

  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p1);
  y = _mm_mul_ps(y, z);
  y = _mm_add_ps(y, _ps_coscof_p2);
  y = _mm_mul_ps(y, _mm_mul_ps(z, z));
  __m128 tmp = _mm_mul_ps(z, _ps_0p5);
  y = _mm_sub_ps(y, tmp);
  y = _mm_add_ps(y, _ps_1);
  
  /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

  __m128 y2 = _ps_sincof_p0;
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p1);
  y2 = _mm_mul_ps(y2, z);
  y2 = _mm_add_ps(y2, _ps_sincof_p2);
  y2 = _mm_mul_ps(y2, _mm_mul_ps(z, x));
  y2 = _mm_add_ps(y2, x);

  /* select the correct result from the two polynoms */  
  xmm3 = poly_mask;
  __m128 ysin2 = _mm_and_ps(xmm3, y2);
  __m128 ysin1 = _mm_andnot_ps(xmm3, y);
  y2 = _mm_sub_ps(y2,ysin2);
  y = _mm_sub_ps(y, ysin1);

  xmm1 = _mm_add_ps(ysin1,ysin2);
  xmm2 = _mm_add_ps(y,y2);
 
  /* update the sign */
  *s = _mm_xor_ps(xmm1, sign_bit_sin);
  *c = _mm_xor_ps(xmm2, sign_bit_cos);
}

#endif
