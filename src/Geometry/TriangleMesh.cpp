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

/** @file TriangleMesh.cpp
	@author Jukka Jylänki
	@brief Implementation for the TriangleMesh geometry object. */
#include "TriangleMesh.h"
#include <stdlib.h>
#include <string.h>
#include "../Math/float3.h"
#include "Triangle.h"
#include "Ray.h"
#include "Polyhedron.h"
#include "../MathGeoLibFwd.h"
#include "../Math/MathConstants.h"
#include "../Math/myassert.h"

#include <vector>

#include "../Math/SSEMath.h"

// If defined, we preprocess our TriangleMesh data structure to contain (v0, v1-v0, v2-v0)
// instead of (v0, v1, v2) triplets for faster ray-triangle mesh intersection.
#define SOA_HAS_EDGES

MATH_BEGIN_NAMESPACE

enum SIMDCapability
{
	SIMD_NONE,
	SIMD_SSE,
	SIMD_SSE2,
//	SIMD_SSE3,
//	SIMD_SSSE3,
//	SIMD_SSE4,
	SIMD_SSE41,
//	SIMD_SSE42,
	SIMD_AVX
};

SIMDCapability DetectSIMDCapability()
{
#ifdef WIN32 ///\todo SIMD detection for other x86 platforms.

#ifdef MATH_SSE
	int CPUInfo[4] = {-1};

	unsigned    nIds;//, nExIds, i;
	int nFeatureInfo = 0;
//	bool    bSSE3Instructions = false;
//	bool    bSupplementalSSE3 = false;
//	bool    bCMPXCHG16B = false;
#ifdef MATH_SSE41
	bool    bSSE41Extensions = false;
#endif
//	bool    bSSE42Extensions = false;
//	bool    bPOPCNT = false;

//	bool    bLAHF_SAHFAvailable = false;
//	bool    bCmpLegacy = false;
//	bool    bLZCNT = false;
//	bool    bSSE4A = false;
//	bool    bMisalignedSSE = false;
//	bool    bPREFETCH = false;
//	bool    bMMXExtensions = false;
//	bool    b3DNowExt = false;
//	bool    b3DNow = false;
//	bool    bFP128 = false;
#ifdef MATH_AVX
	bool    hasAVX = false;
#endif
//	bool    bMOVOptimization = false;

	__cpuid(CPUInfo, 0);
	nIds = CPUInfo[0];

	// Get the information associated with each valid Id
//	for (i=0; i<=nIds; ++i)
	if (nIds >= 1)
	{
	//	__cpuid(CPUInfo, i);

		__cpuid(CPUInfo, 1);
		// Interpret CPU feature information.
//		if  (i == 1)
		{
//			bSSE3Instructions = (CPUInfo[2] & 0x1) || false;
//			bSupplementalSSE3 = (CPUInfo[2] & 0x200) || false;
//			bCMPXCHG16B= (CPUInfo[2] & 0x2000) || false;
//			bSSE41Extensions = (CPUInfo[2] & 0x80000) || false;
//			bSSE42Extensions = (CPUInfo[2] & 0x100000) || false;
//			bPOPCNT= (CPUInfo[2] & 0x800000) || false;
#ifdef MATH_AVX
			hasAVX = (CPUInfo[2] & 0x10000000) || false;
#endif
			nFeatureInfo = CPUInfo[3];
		}
	}

//	const bool hasMMX = (nFeatureInfo & (1 << 23)) != 0;

	// Calling __cpuid with 0x80000000 as the InfoType argument
	// gets the number of valid extended IDs.
//	__cpuid(CPUInfo, 0x80000000);
//	nExIds = CPUInfo[0];
/*
	// Get the information associated with each extended ID.
	for (i=0x80000000; i<=nExIds; ++i)
	{
		__cpuid(CPUInfo, i);

		if  (i == 0x80000001)
		{
			bLAHF_SAHFAvailable = (CPUInfo[2] & 0x1) || false;
			bCmpLegacy = (CPUInfo[2] & 0x2) || false;
			bLZCNT = (CPUInfo[2] & 0x20) || false;
			bSSE4A = (CPUInfo[2] & 0x40) || false;
			bMisalignedSSE = (CPUInfo[2] & 0x80) || false;
			bPREFETCH = (CPUInfo[2] & 0x100) || false;
			bMMXExtensions = (CPUInfo[3] & 0x40000) || false;
			b3DNowExt = (CPUInfo[3] & 0x40000000) || false;
			b3DNow = (CPUInfo[3] & 0x80000000) || false;
		}

		if  (i == 0x8000001A)
		{
			bFP128 = (CPUInfo[0] & 0x1) || false;
			bMOVOptimization = (CPUInfo[0] & 0x2) || false;
		}
	}
*/
#ifdef MATH_AVX
	if (hasAVX)
		return SIMD_AVX;
#endif
#ifdef MATH_SSE41
	if (bSSE41Extensions)
		return SIMD_SSE41;
#endif
#ifdef MATH_SSE2
	const bool hasSSE2 = (nFeatureInfo & (1 << 26)) != 0;
	if (hasSSE2)
		return SIMD_SSE2;
#endif
#ifdef MATH_SSE
	const bool hasSSE = (nFeatureInfo & (1 << 25)) != 0;
	if (hasSSE)
		return SIMD_SSE;
#endif

#endif // ~ MATH_SSE not defined.
#endif
	return SIMD_NONE;
}

const int simdCapability = DetectSIMDCapability();

TriangleMesh::TriangleMesh()
:data(0), numTriangles(0)
{

}

TriangleMesh::~TriangleMesh()
{
	AlignedFree(data);
}

void TriangleMesh::Set(const Polyhedron &polyhedron)
{
	std::vector<Triangle> tris = polyhedron.Triangulate();
	if (!tris.empty())
	{
		int alignment = (simdCapability == SIMD_AVX) ? 8 : ((simdCapability == SIMD_SSE41 || simdCapability == SIMD_SSE2) ? 4 : 1);
		float3 degen(-FLOAT_INF, -FLOAT_INF, -FLOAT_INF);
		Triangle degent(degen, degen, degen);
		while(tris.size() % alignment != 0)
			tris.push_back(degent);
		Set(&tris[0], tris.size());
	}
}

void TriangleMesh::Set(const float *triangleMesh, int numTriangles)
{
	if (simdCapability == SIMD_AVX)
		SetSoA8(triangleMesh, numTriangles);
	else if (simdCapability == SIMD_SSE41 || simdCapability == SIMD_SSE2)
		SetSoA4(triangleMesh, numTriangles);
	else
		SetAoS(triangleMesh, numTriangles);
}

float TriangleMesh::IntersectRay(const Ray &ray) const
{
#ifdef MATH_AVX
	if (simdCapability == SIMD_AVX)
		return IntersectRay_AVX(ray);
#endif
#ifdef MATH_SSE41
	if (simdCapability == SIMD_SSE41)
		return IntersectRay_SSE41(ray);
#endif
#ifdef MATH_SSE2
	if (simdCapability == SIMD_SSE2)
		return IntersectRay_SSE2(ray);
#endif

	int triangleIndex;
	float u, v;
	return IntersectRay_TriangleIndex_UV_CPP(ray, triangleIndex, u, v);
}

float TriangleMesh::IntersectRay_TriangleIndex(const Ray &ray, int &outTriangleIndex) const
{
#ifdef MATH_AVX
	if (simdCapability == SIMD_AVX)
		return IntersectRay_TriangleIndex_AVX(ray, outTriangleIndex);
#endif
#ifdef MATH_SSE41
	if (simdCapability == SIMD_SSE41)
		return IntersectRay_TriangleIndex_SSE41(ray, outTriangleIndex);
#endif
#ifdef MATH_SSE2
	if (simdCapability == SIMD_SSE2)
		return IntersectRay_TriangleIndex_SSE2(ray, outTriangleIndex);
#endif

	float u, v;
	return IntersectRay_TriangleIndex_UV_CPP(ray, outTriangleIndex, u, v);
}

float TriangleMesh::IntersectRay_TriangleIndex_UV(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const
{
#ifdef MATH_AVX
	if (simdCapability == SIMD_AVX)
		return IntersectRay_TriangleIndex_UV_AVX(ray, outTriangleIndex, outU, outV);
#endif
#ifdef MATH_SSE41
	if (simdCapability == SIMD_SSE41)
		return IntersectRay_TriangleIndex_UV_SSE41(ray, outTriangleIndex, outU, outV);
#endif
#ifdef MATH_SSE2
	if (simdCapability == SIMD_SSE2)
		return IntersectRay_TriangleIndex_UV_SSE2(ray, outTriangleIndex, outU, outV);
#endif

	return IntersectRay_TriangleIndex_UV_CPP(ray, outTriangleIndex, outU, outV);
}

void TriangleMesh::ReallocVertexBuffer(int numTris)
{
	AlignedFree(data);
	data = (float*)AlignedMalloc(numTris*3*3*sizeof(float), 32);
	numTriangles = numTris;
}

void TriangleMesh::SetAoS(const float *vertexData, int numTriangles)
{
	ReallocVertexBuffer(numTriangles);
#ifdef _DEBUG
	vertexDataLayout = 0; // AoS
#endif

	memcpy(data, vertexData, numTriangles*3*3*4);
}

void TriangleMesh::SetSoA4(const float *vertexData, int numTriangles)
{
	ReallocVertexBuffer(numTriangles);
#ifdef _DEBUG
	vertexDataLayout = 1; // SoA4
#endif

	assert(numTriangles % 4 == 0); // We must have an evenly divisible amount of triangles, so that the SoA swizzling succeeds.

	// From (xyz xyz xyz) (xyz xyz xyz) (xyz xyz xyz) (xyz xyz xyz)
	// To xxxx yyyy zzzz xxxx yyyy zzzz xxxx yyyy zzzz

	float *o = data;
	for(int i = 0; i + 4 <= numTriangles; i += 4)
	{
		for(int j = 0; j < 9; ++j)
		{
			*o++ = vertexData[0];
			*o++ = vertexData[9];
			*o++ = vertexData[18];
			*o++ = vertexData[27];
			++vertexData;
		}
		vertexData += 9 * 3;
	}

#ifdef SOA_HAS_EDGES
	o = data;
	for(int i = 0; i + 4 <= numTriangles; i += 4)
	{
		for(int j = 12; j < 24; ++j)
			o[j] -= o[j-12];
		for(int j = 24; j < 36; ++j)
			o[j] -= o[j-24];
		o += 36;
	}
#endif
}

void TriangleMesh::SetSoA8(const float *vertexData, int numTriangles)
{
	ReallocVertexBuffer(numTriangles);
#ifdef _DEBUG
	vertexDataLayout = 2; // SoA8
#endif

	assert(numTriangles % 8 == 0); // We must have an evenly divisible amount of triangles, so that the SoA swizzling succeeds.

	// From (xyz xyz xyz) (xyz xyz xyz) (xyz xyz xyz) (xyz xyz xyz)
	// To xxxxxxxx yyyyyyyy zzzzzzzz xxxxxxxx yyyyyyyy zzzzzzzz xxxxxxxx yyyyyyyy zzzzzzzz

	float *o = data;
	for(int i = 0; i + 8 <= numTriangles; i += 8)
	{
		for(int j = 0; j < 9; ++j)
		{
			*o++ = vertexData[0];
			*o++ = vertexData[9];
			*o++ = vertexData[18];
			*o++ = vertexData[27];
			*o++ = vertexData[36];
			*o++ = vertexData[45];
			*o++ = vertexData[54];
			*o++ = vertexData[63];
			++vertexData;
		}
		vertexData += 9 * 7;
	}

#ifdef SOA_HAS_EDGES
	o = data;
	for(int i = 0; i + 8 <= numTriangles; i += 8)
	{
		for(int j = 24; j < 48; ++j)
			o[j] -= o[j-24];
		for(int j = 48; j < 72; ++j)
			o[j] -= o[j-48];
		o += 72;
	}
#endif
}

float TriangleMesh::IntersectRay_TriangleIndex_UV_CPP(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const
{
	assert(sizeof(float3) == 3*sizeof(float));
	assert(sizeof(Triangle) == 3*sizeof(float3));
#ifdef _DEBUG
	assert(vertexDataLayout == 0); // Must be AoS structured!
#endif

	float nearestD = FLOAT_INF;
	float u, v, d;

	const Triangle *tris = reinterpret_cast<const Triangle*>(data);
	for(int i = 0; i < numTriangles; ++i)
	{
		d = Triangle::IntersectLineTri(ray.pos, ray.dir, tris->a, tris->b, tris->c, u, v);
		if (d >= 0.f && d < nearestD)
		{
			nearestD = d;
			outU = u;
			outV = v;
			outTriangleIndex = i;
		}
		++tris;
	}

	return nearestD;
}

MATH_END_NAMESPACE

#ifdef MATH_SSE2
#define MATH_GEN_SSE2
#include "TriangleMesh_IntersectRay_SSE.inl"

#define MATH_GEN_SSE2
#define MATH_GEN_TRIANGLEINDEX
#include "TriangleMesh_IntersectRay_SSE.inl"

#define MATH_GEN_SSE2
#define MATH_GEN_TRIANGLEINDEX
#define MATH_GEN_UV
#include "TriangleMesh_IntersectRay_SSE.inl"
#endif

#ifdef MATH_SSE41
#define MATH_GEN_SSE41
#include "TriangleMesh_IntersectRay_SSE.inl"

#define MATH_GEN_SSE41
#define MATH_GEN_TRIANGLEINDEX
#include "TriangleMesh_IntersectRay_SSE.inl"

#define MATH_GEN_SSE41
#define MATH_GEN_TRIANGLEINDEX
#define MATH_GEN_UV
#include "TriangleMesh_IntersectRay_SSE.inl"
#endif

#ifdef MATH_AVX
#define MATH_GEN_AVX
#include "TriangleMesh_IntersectRay_AVX.inl"

#define MATH_GEN_AVX
#define MATH_GEN_TRIANGLEINDEX
#include "TriangleMesh_IntersectRay_AVX.inl"

#define MATH_GEN_AVX
#define MATH_GEN_TRIANGLEINDEX
#define MATH_GEN_UV
#include "TriangleMesh_IntersectRay_AVX.inl"
#endif
