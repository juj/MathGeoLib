/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file float4_neon.h
	@author Jukka Jylänki
	@brief ARM NEON code for float4-related computations. */

#pragma once

#ifdef MATH_NEON
typedef float32x4_t simd4f;
#elif defined(MATH_SSE)
typedef __m128 simd4f;
#endif

FORCE_INLINE simd4f vec4_add_float(simd4f vec, float f)
{
#ifdef MATH_SSE
	return _mm_add_ps(vec, _mm_set1_ps(f));
#elif defined(MATH_NEON)
	return vaddq_f32(vec, vdupq_n_f32(f));
#endif
}

FORCE_INLINE simd4f vec4_add_vec4(simd4f vec, simd4f vec2)
{
#ifdef MATH_SSE
	return _mm_add_ps(vec, vec2);
#elif defined(MATH_NEON)
	return vaddq_f32(vec, vec2);
#endif
}

FORCE_INLINE simd4f vec4_sub_float(simd4f vec, float f)
{
#ifdef MATH_SSE
	return _mm_sub_ps(vec, _mm_set1_ps(f));
#elif defined(MATH_NEON)
	return vsubq_f32(vec, vdupq_n_f32(f));
#endif
}

FORCE_INLINE simd4f float_sub_vec4(float f, simd4f vec)
{
#ifdef MATH_SSE
	return _mm_sub_ps(_mm_set1_ps(f), vec);
#elif defined(MATH_NEON)
	return vsubq_f32(vdupq_n_f32(f), vec);
#endif
}

FORCE_INLINE simd4f vec4_sub_vec4(simd4f vec, simd4f vec2)
{
#ifdef MATH_SSE
	return _mm_sub_ps(vec, vec2);
#elif defined(MATH_NEON)
	return vsubq_f32(vec, vec2);
#endif
}

#ifdef MATH_NEON
FORCE_INLINE simd4f negate_ps(simd4f vec)
{
	return float_sub_vec4(0.f, vec);
}
#endif

FORCE_INLINE simd4f vec4_mul_float(simd4f vec, float f)
{
#ifdef MATH_SSE
	return _mm_mul_ps(vec, _mm_set1_ps(f));
#elif defined(MATH_NEON)
	return vmulq_f32(vec, vdupq_n_f32(f));
#endif
}

FORCE_INLINE simd4f vec4_mul_vec4(simd4f vec, simd4f vec2)
{
#ifdef MATH_SSE
	return _mm_mul_ps(vec, vec2);
#elif defined(MATH_NEON)
	return vmulq_f32(vec, vec2);
#endif
}

FORCE_INLINE simd4f vec4_div_float(simd4f vec, float f)
{
#ifdef MATH_SSE
	return _mm_div_ps(vec, _mm_set1_ps(f));
#elif defined(MATH_NEON)
	simd4f v = vdupq_n_f32(f);
	simd4f rcp = vrecpeq_f32(v);
	rcp = vmulq_f32(vrecpsq_f32(v, rcp), rcp);
	rcp = vmulq_f32(vrecpsq_f32(v, rcp), rcp);
	return vmulq_f32(vec, rcp);
#endif
}

FORCE_INLINE simd4f float_div_vec4(float f, simd4f vec)
{
#ifdef MATH_SSE
	return _mm_div_ps(_mm_set1_ps(f), vec);
#elif defined(MATH_NEON)
	simd4f rcp = vrecpeq_f32(vec);
	rcp = vmulq_f32(vrecpsq_f32(vec, rcp), rcp);
	rcp = vmulq_f32(vrecpsq_f32(vec, rcp), rcp);
	return vmulq_f32(vdupq_n_f32(f), rcp);
#endif
}

FORCE_INLINE simd4f vec4_div_vec4(simd4f vec, simd4f vec2)
{
#ifdef MATH_SSE
	return _mm_div_ps(vec, vec2);
#elif defined(MATH_NEON)
	simd4f rcp = vrecpeq_f32(vec2);
	rcp = vmulq_f32(vrecpsq_f32(vec2, rcp), rcp);
	rcp = vmulq_f32(vrecpsq_f32(vec2, rcp), rcp);
	return vmulq_f32(vec, rcp);
#endif
}
