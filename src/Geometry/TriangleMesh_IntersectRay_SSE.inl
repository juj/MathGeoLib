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

/** @file TriangleMesh_IntersectRay_SSE.inl
	@author Jukka Jylänki
	@brief SSE implementation of ray-mesh intersection routines. */
MATH_BEGIN_NAMESPACE

#if defined(MATH_GEN_SSE2) && !defined(MATH_GEN_TRIANGLEINDEX)
float TriangleMesh::IntersectRay_SSE2(const Ray &ray) const
#elif defined(MATH_GEN_SSE2) && defined(MATH_GEN_TRIANGLEINDEX) && !defined(MATH_GEN_UV)
float TriangleMesh::IntersectRay_TriangleIndex_SSE2(const Ray &ray, int &outTriangleIndex) const
#elif defined(MATH_GEN_SSE2) && defined(MATH_GEN_TRIANGLEINDEX) && defined(MATH_GEN_UV)
float TriangleMesh::IntersectRay_TriangleIndex_UV_SSE2(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const
#elif defined(MATH_GEN_SSE41) && !defined(MATH_GEN_TRIANGLEINDEX)
float TriangleMesh::IntersectRay_SSE41(const Ray &ray) const
#elif defined(MATH_GEN_SSE41) && defined(MATH_GEN_TRIANGLEINDEX) && !defined(MATH_GEN_UV)
float TriangleMesh::IntersectRay_TriangleIndex_SSE41(const Ray &ray, int &outTriangleIndex) const
#elif defined(MATH_GEN_SSE41) && defined(MATH_GEN_TRIANGLEINDEX) && defined(MATH_GEN_UV)
float TriangleMesh::IntersectRay_TriangleIndex_UV_SSE41(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const
#endif
{
//	std::cout << numTris << " tris: ";
//	TRACESTART(RayTriMeshIntersectSSE);

	assert(sizeof(float3) == 3*sizeof(float));
	assert(sizeof(Triangle) == 3*sizeof(float3));
#ifdef _DEBUG
	assert(vertexDataLayout == 1); // Must be SoA4 structured!
#endif
	
	const float inf = FLOAT_INF;
	__m128 nearestD = _mm_set1_ps(inf);
#ifdef MATH_GEN_UV
	__m128 nearestU = _mm_set1_ps(inf);
	__m128 nearestV = _mm_set1_ps(inf);
#endif
#ifdef MATH_GEN_TRIANGLEINDEX
	__m128i nearestIndex = _mm_set1_epi32(-1);
#endif

	const __m128 lX = _mm_load1_ps(&ray.pos.x);
	const __m128 lY = _mm_load1_ps(&ray.pos.y);
	const __m128 lZ = _mm_load1_ps(&ray.pos.z);

	const __m128 dX = _mm_load1_ps(&ray.dir.x);
	const __m128 dY = _mm_load1_ps(&ray.dir.y);
	const __m128 dZ = _mm_load1_ps(&ray.dir.z);

	const __m128 epsilon = _mm_set1_ps(1e-4f);
	const __m128 zero = _mm_setzero_ps();
	const __m128 one = _mm_set1_ps(1.f);

    const __m128 sign_mask = _mm_set1_ps(-0.f); // -0.f = 1 << 31

	assert(((uintptr_t)data & 0xF) == 0);

	const float *tris = reinterpret_cast<const float*>(data);

	for(int i = 0; i+4 <= numTriangles; i += 4)
	{
		__m128 v0x = _mm_load_ps(tris);
		__m128 v0y = _mm_load_ps(tris+4);
		__m128 v0z = _mm_load_ps(tris+8);

#ifdef SOA_HAS_EDGES
		__m128 e1x = _mm_load_ps(tris+12);
		__m128 e1y = _mm_load_ps(tris+16);
		__m128 e1z = _mm_load_ps(tris+20);

		__m128 e2x = _mm_load_ps(tris+24);
		__m128 e2y = _mm_load_ps(tris+28);
		__m128 e2z = _mm_load_ps(tris+32);
#else
		__m128 v1x = _mm_load_ps(tris+12);
		__m128 v1y = _mm_load_ps(tris+16);
		__m128 v1z = _mm_load_ps(tris+20);

		__m128 v2x = _mm_load_ps(tris+24);
		__m128 v2y = _mm_load_ps(tris+28);
		__m128 v2z = _mm_load_ps(tris+32);

		// Edge vectors
		__m128 e1x = _mm_sub_ps(v1x, v0x);
		__m128 e1y = _mm_sub_ps(v1y, v0y);
		__m128 e1z = _mm_sub_ps(v1z, v0z);

		__m128 e2x = _mm_sub_ps(v2x, v0x);
		__m128 e2y = _mm_sub_ps(v2y, v0y);
		__m128 e2z = _mm_sub_ps(v2z, v0z);
#endif
//		_mm_prefetch((const char *)(tris+36), _MM_HINT_T0);

		// begin calculating determinant - also used to calculate U parameter
		__m128 px = _mm_sub_ps(_mm_mul_ps(dY, e2z), _mm_mul_ps(dZ, e2y));
		__m128 py = _mm_sub_ps(_mm_mul_ps(dZ, e2x), _mm_mul_ps(dX, e2z));
		__m128 pz = _mm_sub_ps(_mm_mul_ps(dX, e2y), _mm_mul_ps(dY, e2x));

		// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
		__m128 det = _mm_add_ps(_mm_add_ps(_mm_mul_ps(e1x, px), _mm_mul_ps(e1y, py)), _mm_mul_ps(e1z, pz));

		// If determinant is near zero, ray lies in plane of triangle.

//		if (fabs(det) <= epsilon)
//			return FLOAT_INF;
		__m128 recipDet = _mm_rcp_ps(det);

		__m128 absdet = _mm_andnot_ps(sign_mask, det);
		__m128 out = _mm_cmple_ps(absdet, epsilon);

		// Calculate distance from v0 to ray origin
		__m128 tx = _mm_sub_ps(lX, v0x);
		__m128 ty = _mm_sub_ps(lY, v0y);
		__m128 tz = _mm_sub_ps(lZ, v0z);

		// Output barycentric u
		__m128 u = _mm_mul_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(tx, px), _mm_mul_ps(ty, py)), _mm_mul_ps(tz, pz)), recipDet);

//		if (u < 0.f || u > 1.f)
//			return FLOAT_INF; // Barycentric U is outside the triangle - early out.
		__m128 out2 = _mm_cmplt_ps(u, zero);
		out = _mm_or_ps(out, out2);
		out2 = _mm_cmpgt_ps(u, one);
		out = _mm_or_ps(out, out2);

		// Prepare to test V parameter
		__m128 qx = _mm_sub_ps(_mm_mul_ps(ty, e1z), _mm_mul_ps(tz, e1y));
		__m128 qy = _mm_sub_ps(_mm_mul_ps(tz, e1x), _mm_mul_ps(tx, e1z));
		__m128 qz = _mm_sub_ps(_mm_mul_ps(tx, e1y), _mm_mul_ps(ty, e1x));

		// Output barycentric v
		__m128 v = _mm_mul_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(dX, qx), _mm_mul_ps(dY, qy)), _mm_mul_ps(dZ, qz)), recipDet);

//		if (v < 0.f || u + v > 1.f) // Barycentric V or the combination of U and V are outside the triangle - no intersection.
//			return FLOAT_INF;
		out2 = _mm_cmplt_ps(v, zero);
		out = _mm_or_ps(out, out2);
		__m128 uv = _mm_add_ps(u, v);
		out2 = _mm_cmpgt_ps(uv, one);
		out = _mm_or_ps(out, out2);

		// Output signed distance from ray to triangle.
		__m128 t = _mm_mul_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(e2x, qx), _mm_mul_ps(e2y, qy)), _mm_mul_ps(e2z, qz)), recipDet);

		// t < 0?
		out2 = _mm_cmplt_ps(t, zero);
		out = _mm_or_ps(out, out2);

		// Worse than previous result?
		out2 = _mm_cmpge_ps(t, nearestD);
		out = _mm_or_ps(out, out2);

		// The mask 'out' now contains 0xFF in all indices which are worse than previous, and
		// 0x00 in indices which are better.

#ifdef MATH_GEN_SSE41
		nearestD = _mm_blendv_ps(t, nearestD, out);
#else
		// If SSE 4.1 is not available:
		nearestD = _mm_and_ps(out, nearestD);
		t = _mm_andnot_ps(out, t);
		nearestD = _mm_or_ps(t, nearestD);
#endif

#ifdef MATH_GEN_UV

#ifdef MATH_GEN_SSE41
		nearestU = _mm_blendv_ps(u, nearestU, out); // 'blend' requires SSE4.1!
		nearestV = _mm_blendv_ps(v, nearestV, out); // 'blend' requires SSE4.1!
#else
		// If SSE 4.1 is not available:
		nearestU = _mm_and_ps(out, nearestU);
		nearestV = _mm_and_ps(out, nearestV);
		u = _mm_andnot_ps(out, u);
		v = _mm_andnot_ps(out, v);
		nearestU = _mm_or_ps(u, nearestU);
		nearestV = _mm_or_ps(v, nearestV);
#endif

#endif

#ifdef MATH_GEN_TRIANGLEINDEX
		__m128i hitIndex = _mm_set1_epi32(i);
#ifdef MATH_GEN_SSE41
		nearestIndex = _mm_castps_si128(_mm_blendv_ps(_mm_castsi128_ps(hitIndex), _mm_castsi128_ps(nearestIndex), out)); // 'blend' requires SSE4.1!
#else
		// If SSE 4.1 is not available:
		// Store the index of the triangle that was hit.
		nearestIndex = _mm_and_si128(_mm_castps_si128(out), nearestIndex);
		hitIndex = _mm_andnot_si128(_mm_castps_si128(out), hitIndex);
		nearestIndex = _mm_or_si128(hitIndex, nearestIndex);
#endif

#endif

		tris += 36;
	}

	float ds[16];
	float *alignedDS = (float*)(((uintptr_t)ds + 0xF) & ~0xF);

#ifdef MATH_GEN_UV
	float su[16];
	float *alignedU = (float*)(((uintptr_t)su + 0xF) & ~0xF);

	float sv[16];
	float *alignedV = (float*)(((uintptr_t)sv + 0xF) & ~0xF);

	_mm_store_ps(alignedU, nearestU);
	_mm_store_ps(alignedV, nearestV);
#endif

#ifdef MATH_GEN_TRIANGLEINDEX
	u32 ds2[16];
	u32 *alignedDS2 = (u32*)(((uintptr_t)ds2 + 0xF) & ~0xF);

	_mm_store_si128((__m128i*)alignedDS2, nearestIndex);
#endif

	_mm_store_ps(alignedDS, nearestD);

	float smallestT = FLOAT_INF;
//	float u = FLOAT_NAN, v = FLOAT_NAN;
	for(int i = 0; i < 4; ++i)
		if (alignedDS[i] < smallestT)
		{
			smallestT = alignedDS[i];
#ifdef MATH_GEN_TRIANGLEINDEX
			outTriangleIndex = alignedDS2[i]+i;
#endif
#ifdef MATH_GEN_UV
			outU = alignedU[i];
			outV = alignedV[i];
#endif
		}

//	TRACEEND(RayTriMeshIntersectSSE);

//	static double avgtimes = 0.f;
//	static double nAvgTimes = 0;
//	static double processedBytes;
	
//	processedBytes += numTris * 3 * 4;

//	avgtimes += Clock::TicksToMillisecondsD(time_RayTriMeshIntersectSSE);
//	++nAvgTimes;
//	std::cout << "Total avg (SSE): " << (avgtimes / nAvgTimes) << std::endl;
//	std::cout << "Hit distance (SSE): " << smallestT << ", index: " << hitTriangleIndex << ", UV: (" << u << ", " << v << ")" << std::endl;
//	std::cout << "(SSE) " << processedBytes / avgtimes * 1000.0 / 1024.0 / 1024.0 / 1024.0 << "GB/sec." << std::endl;

	return smallestT;
}


//float TriangleMesh::IntersectRay_AVX(const Ray &ray) const
//float TriangleMesh::IntersectRay_TriangleIndex_AVX(const Ray &ray, int &outIndex) const
//float TriangleMesh::IntersectRay_TriangleIndex_UV_AVX(const Ray &ray, int &outIndex, float &outU, float &outV) const




#ifdef MATH_GEN_SSE2
#undef MATH_GEN_SSE2
#endif
#ifdef MATH_GEN_SSE41
#undef MATH_GEN_SSE41
#endif
#ifdef MATH_GEN_TRIANGLEINDEX
#undef MATH_GEN_TRIANGLEINDEX
#endif
#ifdef MATH_GEN_UV
#undef MATH_GEN_UV
#endif

MATH_END_NAMESPACE
