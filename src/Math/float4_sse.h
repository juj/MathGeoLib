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

/** @file float4_sse.h
	@author Jukka Jylänki
	@brief SSE code for float4-related computations. */
#pragma once

#include "MathBuildConfig.h"

#ifdef MATH_SSE

#include "Types.h"
#include "SSEMath.h"

// Input: [w,z,y,x], Output: x+y+z in all four registers.
FORCE_INLINE __m128 sum_xyz_ps(__m128 m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = _mm_and_ps(m, sseMaskXYZ); // Clear w to zero.
	m = _mm_hadd_ps(m, m); // m = (x+y, z+w, x+y, z+w).
	m = _mm_hadd_ps(m, m); // m = (x+y+z+w, x+y+z+w, x+y+z+w, x+y+z+w).
	return m; // Each index of the output will contain the sum x+y+z.
#else // We only have SSE 1, and must individually shuffle.
	__m128 Y = shuffle1_ps(m, _MM_SHUFFLE(1,1,1,1)); // Load Y to lowest index. (others don't matter)
	__m128 Z = shuffle1_ps(m, _MM_SHUFFLE(2,2,2,2)); // Load Z to lowest index. (others don't matter)
	__m128 XYZ = _mm_add_ps(m, _mm_add_ps(Y, Z));
	return XYZ; // Each index of the output will contain the sum x+y+z.
#endif
}

/// The returned SP FP contains x+y+z+w in all channels of the vector.
FORCE_INLINE __m128 sum_xyzw_ps(__m128 m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = _mm_hadd_ps(m, m); // m = (x+y, z+w, x+y, z+w).
	m = _mm_hadd_ps(m, m); // m = (x+y+z+w, x+y+z+w, x+y+z+w, x+y+z+w).
	return m; // Each index of the output will contain the sum x+y+z+w.
#else // We only have SSE 1, and must individually shuffle.
	__m128 v2 = shuffle1_ps(m, _MM_SHUFFLE(1,0,3,2)); // = [y, x, w, z]
	v2 = _mm_add_ps(v2, m); // = [w+y, z+x, y+w, x+z]
	__m128 v3 = shuffle1_ps(v2, _MM_SHUFFLE(0,3,2,1)); // = [x+z, w+y, z+x, y+w]
	return _mm_add_ps(v2, v3); // = [w+y+x+z, z+x+w+y, y+w+z+x, x+z+y+w]
#endif
}

FORCE_INLINE __m128 mul_xyzw_ps(__m128 v)
{
	__m128 v2 = shuffle1_ps(v, _MM_SHUFFLE(1, 0, 3, 2)); // v2 = [y, x, w, z]
	v2 = _mm_mul_ps(v, v2); // v2 = [w*y, z*x, y*w, x*z]
	__m128 v3 = shuffle1_ps(v2, _MM_SHUFFLE(2, 1, 0, 3)); // v3 = [z*x, y*w, x*z, w*y]
	return _mm_mul_ps(v2, v3); // v3 = [w*y*z*x, z*x*y*w, y*w*x*z, x*z*w*y]
}

// Returns the dot-product of the x,y,z components in all channels of the output vector.
FORCE_INLINE __m128 dot3_ps(__m128 a, __m128 b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	return _mm_dp_ps(a, b, 0x7F); // Choose to multiply x, y and z (0x70 = 0111 0000), and store the output to all indices (0x0F == 0000 1111).
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	__m128 v2 = sum_xyz_ps(_mm_mul_ps(a, b));
#endif
}

/// The dot product is stored in each channel of the returned vector.
FORCE_INLINE __m128 dot4_ps(__m128 a, __m128 b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	return _mm_dp_ps(a, b, 0xFF); // Choose to multiply x, y, z and w (0xF0 = 1111 0000), and store the output to all indices (0x0F == 0000 1111).
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	return sum_xyzw_ps(_mm_mul_ps(a, b));
#endif
}

FORCE_INLINE __m128 cross_ps(__m128 a, __m128 b)
{
	__m128 a_xzy = shuffle1_ps(a, _MM_SHUFFLE(3, 0, 2, 1)); // a_xzy = [a.w, a.x, a.z, a.y]
	__m128 b_yxz = shuffle1_ps(b, _MM_SHUFFLE(3, 1, 0, 2)); // b_yxz = [b.w, b.y, b.x, b.z]

	__m128 a_yxz = shuffle1_ps(a, _MM_SHUFFLE(3, 1, 0, 2)); // a_yxz = [a.w, a.y, a.x, a.z]
	__m128 b_xzy = shuffle1_ps(b, _MM_SHUFFLE(3, 0, 2, 1)); // b_xzy = [b.w, b.x, b.z, b.y]

	__m128 x = _mm_mul_ps(a_xzy, b_yxz); // [a.w*b.w, a.x*b.y, a.z*b.x, a.y*b.z]
	__m128 y = _mm_mul_ps(a_yxz, b_xzy); // [a.w*b.w, a.y*b.x, a.x*b.z, a.z*b.y]

	return _mm_sub_ps(x, y); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]
}

#endif
