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

#include "../MathBuildConfig.h"

#ifdef MATH_SSE

#include "MathTypes.h"
#include "SSEMath.h"

// Input: [w,z,y,x], Output: x+y+z in all four channels.
FORCE_INLINE simd4f sum_xyz_ps(simd4f m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = and_ps(m, sseMaskXYZ); // Clear w to zero.
	m = _mm_hadd_ps(m, m); // m = (x+y, z, x+y, z).
	m = _mm_hadd_ps(m, m); // m = (x+y+z, x+y+z, x+y+z, x+y+z).
	return m; // Each index of the output will contain the sum x+y+z.
#else // We only have SSE 1, and must individually shuffle.
	simd4f X = xxxx_ps(m);
	simd4f Y = yyyy_ps(m);
	simd4f Z = zzzz_ps(m);
	simd4f XYZ = add_ps(X, add_ps(Y, Z)); 
	return XYZ; // Each index of the output will contain the sum x+y+z.
#endif
}

// Input: [w,z,y,x], Output: x+y+z in three lowest channels, w is undefined.
FORCE_INLINE simd4f sum_xyz_ps3(simd4f m)
{
	simd4f yzx = shuffle1_ps(m, _MM_SHUFFLE(3,0,2,1)); // [_, x, z, y]
	simd4f zxy = shuffle1_ps(m, _MM_SHUFFLE(3,1,0,2)); // [_, y, x, z]
	simd4f XYZ = add_ps(m, add_ps(yzx, zxy)); // [_, x+y+z, x+y+z, x+y+z]
	return XYZ; // The three lowest elements will contain the sum x+y+z. Highest element is undefined.
}

FORCE_INLINE float sum_xyz_float(simd4f m)
{
	return s4f_x(sum_xyz_ps3(m));
}

/// The returned SP FP contains x+y+z+w in all channels of the vector.
FORCE_INLINE simd4f sum_xyzw_ps(simd4f m)
{
#ifdef MATH_SSE3 // If we have SSE 3, we can use the haddps (horizontal add) instruction, _mm_hadd_ps intrinsic.
	m = _mm_hadd_ps(m, m); // m = (x+y, z+w, x+y, z+w).
	m = _mm_hadd_ps(m, m); // m = (x+y+z+w, x+y+z+w, x+y+z+w, x+y+z+w).
	return m; // Each index of the output will contain the sum x+y+z+w.
#else // We only have SSE 1, and must individually shuffle.
	simd4f v2 = shuffle1_ps(m, _MM_SHUFFLE(1,0,3,2)); // = [y, x, w, z]
	v2 = add_ps(v2, m); // = [w+y, z+x, y+w, x+z]
	simd4f v3 = shuffle1_ps(v2, _MM_SHUFFLE(0,3,2,1)); // = [x+z, w+y, z+x, y+w]
	return add_ps(v2, v3); // = [w+y+x+z, z+x+w+y, y+w+z+x, x+z+y+w]
#endif
}

FORCE_INLINE float sum_xyzw_float(simd4f m)
{
	return s4f_x(sum_xyzw_ps(m));
}

FORCE_INLINE simd4f mul_xyzw_ps(simd4f v)
{
	simd4f v2 = shuffle1_ps(v, _MM_SHUFFLE(1, 0, 3, 2)); // v2 = [y, x, w, z]
	v2 = mul_ps(v, v2); // v2 = [w*y, z*x, y*w, x*z]
	simd4f v3 = shuffle1_ps(v2, _MM_SHUFFLE(2, 1, 0, 3)); // v3 = [z*x, y*w, x*z, w*y]
	return mul_ps(v2, v3); // v3 = [w*y*z*x, z*x*y*w, y*w*x*z, x*z*w*y]
}

FORCE_INLINE float mul_xyzw_float(simd4f m)
{
	return s4f_x(mul_xyzw_ps(m));
}

// Returns the dot-product of the x,y,z components in all channels of the output vector.
FORCE_INLINE simd4f dot3_ps(simd4f a, simd4f b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	return _mm_dp_ps(a, b, 0x7F); // Choose to multiply x, y and z (0x70 = 0111 0000), and store the output to all indices (0x0F == 0000 1111).
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	return sum_xyz_ps(mul_ps(a, b));
#endif
}

// Returns the dot-product of the x,y,z components in all channels of the output vector.
FORCE_INLINE simd4f dot3_ps3(simd4f a, simd4f b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	return _mm_dp_ps(a, b, 0x7F); // Choose to multiply x, y and z (0x70 = 0111 0000), and store the output to all indices (0x0F == 0000 1111).
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	return sum_xyz_ps3(mul_ps(a, b));
#endif
}

FORCE_INLINE float dot3_float(simd4f a, simd4f b)
{
	return s4f_x(dot3_ps3(a, b));
}

/// The dot product is stored in each channel of the returned vector.
FORCE_INLINE simd4f dot4_ps(simd4f a, simd4f b)
{
#ifdef MATH_SSE41 // If we have SSE 4.1, we can use the dpps (dot product) instruction, _mm_dp_ps intrinsic.
	return _mm_dp_ps(a, b, 0xFF); // Choose to multiply x, y, z and w (0xF0 = 1111 0000), and store the output to all indices (0x0F == 0000 1111).
#else // Otherwise, use SSE3 haddps or SSE1 with individual shuffling.
	return sum_xyzw_ps(mul_ps(a, b));
#endif
}

FORCE_INLINE float dot4_float(simd4f a, simd4f b)
{
	return s4f_x(dot4_ps(a, b));
}

FORCE_INLINE simd4f cross_ps(simd4f a, simd4f b)
{
	simd4f a_xzy = shuffle1_ps(a, _MM_SHUFFLE(3, 0, 2, 1)); // = [a.w, a.x, a.z, a.y]
	simd4f b_xzy = shuffle1_ps(b, _MM_SHUFFLE(3, 0, 2, 1)); // = [b.w, b.x, b.z, b.y]

	simd4f x_yxz = mul_ps(b_xzy, a); // [a.w*b.w, a.z*b.x, a.y*b.z, a.x*b.y]
	simd4f y_yxz = mul_ps(a_xzy, b); // [a.w*b.w, a.z*b.x, a.y*b.z, a.x*b.y]

	return shuffle1_ps(sub_ps(x_yxz, y_yxz), _MM_SHUFFLE(3, 0, 2, 1)); // [0, a.x*b.y - a.y*b.x, a.z*b.x - a.x*b.z, a.y*b.z - a.z*b.y]
}

FORCE_INLINE void basis_ps(simd4f v, simd4f *outB, simd4f *outC)
{
	simd4f a = abs_ps(v);
	simd4f a_min = min_ps(a, min_ps(yyyy_ps(a), _mm_movehl_ps(a, a))); // Horizontal min of x,y,z
	a_min = xxxx_ps(a_min); // Broadcast to all elements.
	a = cmple_ps(a, a_min); // Mask 0xFFFFFFFF to channels that contain the min element.
	// Choose from (1,0,0), (0,1,0), and (0,0,1) the one that's most perpendicular to this vector.
	simd4f q = and_ps(a, set_ps(0.f, 1.f, 1.f, 1.f));

	simd4f v_xzy = shuffle1_ps(v, _MM_SHUFFLE(3, 0, 2, 1));
	simd4f v_yxz = shuffle1_ps(v, _MM_SHUFFLE(3, 1, 0, 2));
	simd4f q_xzy = shuffle1_ps(q, _MM_SHUFFLE(3, 0, 2, 1));
	simd4f b_yxz = sub_ps(mul_ps(q_xzy, v), mul_ps(v_xzy, q));
	simd4f b = shuffle1_ps(b_yxz, _MM_SHUFFLE(3, 0, 2, 1));
	simd4f b_xzy = shuffle1_ps(b_yxz, _MM_SHUFFLE(3, 1, 0, 2));
	simd4f c = sub_ps(mul_ps(b_yxz, v_xzy), mul_ps(v_yxz, b_xzy));

	*outB = mul_ps(b, rsqrt_ps(dot4_ps(b, b)));
	*outC = mul_ps(c, rsqrt_ps(dot4_ps(c, c)));
}

simd4f vec3_length_ps(simd4f vec);
simd4f vec3_length_ps3(simd4f vec);

/// Returns a normalized copy of the given vector. Returns the length of the original vector in outLength.
FORCE_INLINE simd4f vec4_safe_normalize3(simd4f vec, simd4f &outLength)
{
	outLength = vec3_length_ps3(vec);
	simd4f isZero = _mm_cmplt_ps(outLength, simd4fEpsilon); // Was the length zero?
	simd4f normalized = _mm_div_ps(vec, outLength); // Normalize.
	normalized = cmov_ps(normalized, float4::unitX.v, isZero); // If length == 0, output the vector (1,0,0).
	return cmov_ps(vec, normalized, sseMaskXYZ); // Return the original .w component to the vector (this function is supposed to preserve original .w).
}

#endif
