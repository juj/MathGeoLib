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

/** @file TriangleMesh_IntersectRay_CPP.inl
	@author Jukka Jylänki
	@brief Non-SIMD implementation of ray-mesh intersection routines. */
MATH_BEGIN_NAMESPACE

float TriangleMesh::IntersectRay_CPP(const Ray &ray) const
float TriangleMesh::IntersectRay_TriangleIndex_CPP(const Ray &ray, int &outIndex) const
float TriangleMesh::IntersectRay_TriangleIndex_UV_CPP(const Ray &ray, int &outIndex, float &outU, float &outV) const
{

float TriangleMesh::IntersectLineTriSSE(const float3 &linePos, const float3 &lineDir,
		const float3 &v0, const float3 &v1, const float3 &v2,
		float &u, float &v)
{
	float3 vE1, vE2;
	float3 vT, vP, vQ;

	const float epsilon = 1e-6f;

	// Edge vectors
	vE1 = v1 - v0;
	vE2 = v2 - v0;

	// begin calculating determinant - also used to calculate U parameter
	vP.x = lineDir.y * vE2.z - lineDir.z * vE2.y;
	vP.y = lineDir.z * vE2.x - lineDir.x * vE2.z;
	vP.z = lineDir.x * vE2.y - lineDir.y * vE2.x;

	// If det < 0, intersecting backfacing tri, > 0, intersecting frontfacing tri, 0, parallel to plane.
	const float det = vE1.x * vP.x + vE1.y * vP.y + vE1.z * vP.z;

	// If determinant is near zero, ray lies in plane of triangle.
	if (fabs(det) <= epsilon)
		return FLOAT_INF;
	const float recipDet = 1.f / det;

	// Calculate distance from v0 to ray origin
	vT = linePos - v0;

	// Output barycentric u
	u = (vT.x * vP.x + vT.y * vP.y + vT.z * vP.z) * recipDet;
	if (u < 0.f || u > 1.f)
		return FLOAT_INF; // Barycentric U is outside the triangle - early out.

	// Prepare to test V parameter
	vQ.x = vT.y * vE1.z - vT.z * vE1.y;
	vQ.y = vT.z * vE1.x - vT.x * vE1.z;
	vQ.z = vT.x * vE1.y - vT.y * vE1.x;

	// Output barycentric v
	v = (lineDir.x * vQ.x + lineDir.y * vQ.y + lineDir.z * vQ.z) * recipDet;
	if (v < 0.f || u + v > 1.f) // Barycentric V or the combination of U and V are outside the triangle - no intersection.
		return FLOAT_INF;

	// Barycentric u and v are in limits, the ray intersects the triangle.
	
	// Output signed distance from ray to triangle.
	return (vE2.x * vQ.x + vE2.y * vQ.y + vE2.z * vQ.z) * recipDet;
}

MATH_END_NAMESPACE
