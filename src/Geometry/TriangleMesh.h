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

/** @file TriangleMesh.h
	@author Jukka Jylänki
	@brief The TriangleMesh geometry object. */
#pragma once

#include "../MathGeoLibFwd.h"
#include "../Math/float3.h"
#include "Triangle.h"

MATH_BEGIN_NAMESPACE

/// Represents an unindiced triangle mesh.
/** This class stores a triangle mesh as flat array, optimized for ray intersections. */
class TriangleMesh
{
public:
	TriangleMesh();
	~TriangleMesh();

	TriangleMesh(const TriangleMesh &rhs);
	TriangleMesh &operator =(const TriangleMesh &rhs);

	/// Specifies the vertex data of this triangle mesh. Replaces any old
	/// specified geometry.
	/// @param vertexSizeBytes The size (stride) of a single vertex in memory.
	void Set(const float *triangleMesh, int numTriangles, int vertexSizeBytes);
	void Set(const float3 *triangleMesh, int numTris) { Set(reinterpret_cast<const float *>(triangleMesh), numTris, sizeof(float3)); }
	void Set(const Triangle *triangleMesh, int numTris) { Set(reinterpret_cast<const float *>(triangleMesh), numTris, sizeof(Triangle)/3); }

	void SetConvex(const Polyhedron &polyhedron);

	float IntersectRay(const Ray &ray) const;
	float IntersectRay_TriangleIndex(const Ray &ray, int &outTriangleIndex) const;
	float IntersectRay_TriangleIndex_UV(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const;

	void SetAoS(const float *vertexData, int numTriangles, int vertexSizeBytes);
	void SetSoA4(const float *vertexData, int numTriangles, int vertexSizeBytes);
	void SetSoA8(const float *vertexData, int numTriangles, int vertexSizeBytes);

	float IntersectRay_TriangleIndex_UV_CPP(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const;

#ifdef MATH_SSE2
	float IntersectRay_SSE2(const Ray &ray) const;
	float IntersectRay_TriangleIndex_SSE2(const Ray &ray, int &outTriangleIndex) const;
	float IntersectRay_TriangleIndex_UV_SSE2(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const;
#endif

#ifdef MATH_SSE41
	float IntersectRay_SSE41(const Ray &ray) const;
	float IntersectRay_TriangleIndex_SSE41(const Ray &ray, int &outTriangleIndex) const;
	float IntersectRay_TriangleIndex_UV_SSE41(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const;
#endif

#ifdef MATH_AVX
	float IntersectRay_AVX(const Ray &ray) const;
	float IntersectRay_TriangleIndex_AVX(const Ray &ray, int &outTriangleIndex) const;
	float IntersectRay_TriangleIndex_UV_AVX(const Ray &ray, int &outTriangleIndex, float &outU, float &outV) const;
#endif

private:
	float *data; // This is always allocated to tightly-packed numTriangles*3*vertexSizeBytes bytes.
	int numTriangles;
	int vertexSizeBytes;
#ifdef _DEBUG
	int vertexDataLayout; // 0 - AoS, 1 - SoA4, 2 - SoA8
#endif
	void ReallocVertexBuffer(int numTriangles, int vertexSizeBytes);
};

MATH_END_NAMESPACE
