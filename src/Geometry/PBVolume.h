/* Copyright 2012 Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file PBVolume.h
	@author Jukka Jylänki
	@brief Implements a convex polyhedron data structure. */

#pragma once

#include "../Math/float3.h"
#include "../Math/assume.h"
#include "../Math/Swap.h"
#include "AABB.h"
#include "Plane.h"
#include "Sphere.h"
#include "Polyhedron.h"

MATH_BEGIN_NAMESPACE

/// Reports a result from an approximate culling operation.
enum CullTestResult
{
	// The tested objects don't intersect - they are fully disjoint.
	TestOutside,

	// The tested object is at least not fully contained inside the other object, but no other information is known.
	// The objects might intersect or be disjoint.
	TestNotContained,

	// The tested object is fully contained inside the other object.
	TestInside
};

/// PBVolume is a "plane bounded volume", a convex polyhedron represented by a set
/// of planes. The number of planes is fixed at compile time so that compilers are able to perfectly unroll the loops for
/// best performance. As a fixed convention, the plane normals of the volume point outwards from the plane, so the
/// negative halfspaces are inside the convex volume.
template<int N>
class PBVolume
{
public:
	Plane p[N];

	int NumPlanes() const { return N; }

	bool Contains(const vec &point) const
	{
		for(int i = 0; i < N; ++i)
			if (p[i].SignedDistance(point) > 0.f)
				return false;
		return true;
	}

	/// Performs an *approximate* intersection test between this PBVolume and the given AABB.
	/** This function is best used for high-performance object culling purposes, e.g. for frustum-aabb culling, when
		a small percentage of false positives do not matter.
		@return An enum denoting whether the given object is inside or intersects this PBVolume. See the CullTestResult enum 
			for the interpretation of the return values. */
	CullTestResult InsideOrIntersects(const AABB &aabb) const
	{
		CullTestResult result = TestInside;

		for(int i = 0; i < N; ++i)
		{
			vec nPoint;
			vec pPoint;
			nPoint.x = (p[i].normal.x < 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			nPoint.y = (p[i].normal.y < 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			nPoint.z = (p[i].normal.z < 0.f ? aabb.maxPoint.z : aabb.minPoint.z);
#ifdef MATH_VEC_IS_FLOAT4
			nPoint.w = 1.f;
#endif

			pPoint.x = (p[i].normal.x >= 0.f ? aabb.maxPoint.x : aabb.minPoint.x);
			pPoint.y = (p[i].normal.y >= 0.f ? aabb.maxPoint.y : aabb.minPoint.y);
			pPoint.z = (p[i].normal.z >= 0.f ? aabb.maxPoint.z : aabb.minPoint.z);
#ifdef MATH_VEC_IS_FLOAT4
			pPoint.w = 1.f;
#endif

			/*
			// Find the n and p points of the aabb. (The nearest and farthest corners relative to the plane)
			const vec &sign = npPointsSignLUT[((p[i].normal.z >= 0.f) ? 4 : 0) +
												 ((p[i].normal.y >= 0.f) ? 2 : 0) +
												 ((p[i].normal.x >= 0.f) ? 1 : 0)];
			const vec nPoint = c + sign*r;
			const vec pPoint = c - sign*r;
			*/

			float a = p[i].SignedDistance(nPoint);
			if (a >= 0.f)
				return TestOutside; // The AABB is certainly outside this PBVolume.
			a = p[i].SignedDistance(pPoint);
			if (a >= 0.f)
				result = TestNotContained; // At least one vertex is outside this PBVolume. The whole AABB can't possibly be contained in this PBVolume.
		}

		// We can return here either TestInside or TestNotContained, but it's possible that the AABB was outside the frustum, and we
		// just failed to find a separating axis.
		return result;
	}

	CullTestResult InsideOrIntersects(const Sphere &sphere) const
	{
		CullTestResult result = TestInside;
		for(int i = 0; i < N; ++i)
		{
			float d = p[i].SignedDistance(sphere.pos);
			if (d >= sphere.r)
				return TestOutside;
			else if (d >= -sphere.r)
				result = TestNotContained;
		}
		return result;
	}

private:
	struct CornerPt // A helper struct used only internally in ToPolyhedron.
	{
		int ptIndex; // Index to the Polyhedron list of vertices.
		int j, k; // The two plane faces in addition to the main plane that make up this point.
	};

	bool ContainsExcept(const vec &point, int i, int j, int k) const
	{
		for(int l = 0; l < N; ++l)
			if (l != i && l != j && l != k && p[l].SignedDistance(point) > 0.f)
				return false;
		return true;
	}

public:
	Polyhedron ToPolyhedron() const
	{
		Polyhedron ph;
		std::vector<CornerPt> faces[N];
		for(int i = 0; i < N-2; ++i)
			for(int j = i+1; j < N-1; ++j)
				for(int k = j+1; k < N; ++k)
				{
					vec corner;
					bool intersects = p[i].Intersects(p[j], p[k], 0, &corner);
					if (intersects && ContainsExcept(corner, i, j, k))
					{
						CornerPt pt;

						// Find if this vertex is duplicate of an existing vertex.
						bool found = false;
						for(size_t l = 0; l < ph.v.size(); ++l)
							if (vec(ph.v[l]).Equals(corner))
							{
								found = true;
								pt.ptIndex = (int)l;
								break;
							}

						if (!found) // New vertex?
						{
//							LOGI("New point at corner (%d,%d,%d): %s", i, j, k, corner.ToString().c_str());
							ph.v.push_back(corner);
							pt.ptIndex = (int)ph.v.size()-1;
						}
						else
						{
//							LOGI("Existing point at corner (%d,%d,%d): %s", i, j, k, corner.ToString().c_str());
						}

						pt.j = j;
						pt.k = k;
						faces[i].push_back(pt);

						pt.j = i;
						pt.k = k;
						faces[j].push_back(pt);

						pt.j = i;
						pt.k = j;
						faces[k].push_back(pt);
					}
				}

		// Check if we got a degenerate polyhedron?
		if (ph.v.size() <= 1)
			return ph;
		else if (ph.v.size() == 2)
		{
			// Create a degenerate face that's an edge.
			Polyhedron::Face face;
			face.v.push_back(0);
			face.v.push_back(1);
			ph.f.push_back(face);
			return ph;
		}
		else if (ph.v.size() == 3)
		{
			// Create a degenerate face that's a triangle.
			Polyhedron::Face face;
			face.v.push_back(0);
			face.v.push_back(1);
			face.v.push_back(2);
			ph.f.push_back(face);
			return ph;
		}

		// Connect the edges in each face using selection sort.
		for(int i = 0; i < N; ++i)
		{
			std::vector<CornerPt> &pt = faces[i];
			if (pt.size() < 3)
				continue;
			for(size_t j = 0; j < pt.size()-1; ++j)
			{
				CornerPt &prev = pt[j];
				bool found = false;
				for(size_t k = j+1; k < pt.size(); ++k)
				{
					CornerPt &cur = pt[k];
					if (cur.j == prev.k)
					{
						Swap(cur, pt[j+1]);
						found = true;
						break;
					}
					if (cur.k == prev.k)
					{
						Swap(cur.j, cur.k);
						Swap(cur, pt[j+1]);
						found = true;
						break;
					}
				}
				assert(found);
				MARK_UNUSED(found);
			}
			assert(pt[0].j == pt[pt.size()-1].k);
			Polyhedron::Face face;
			for(size_t j = 0; j < pt.size(); ++j)
			{
				face.v.push_back(pt[j].ptIndex);
			}
			ph.f.push_back(face);
		}

		// Fix up winding directions.
		for(size_t i = 0; i < ph.f.size(); ++i)
		{
			Plane face = ph.FacePlane((int)i);
			for(size_t j = 0; j < ph.v.size(); ++j)
			{
				if (face.SignedDistance(ph.v[j]) > 1e-3f)
				{
					ph.f[i].FlipWindingOrder();
					break;
				}
			}
		}
		return ph;
	}

	/// Computes the set intersection of this PBVolume and the PBVolume rhs.
	/// That is, returns the convex set of points that are contained in both this and rhs.
	/// Set intersection is symmetric, so a.SetIntersection(b) is the same as b.SetIntersection(a).
	/// @note The returned PBVolume may contain redundant planes, these are not pruned.
	template<int M>
	PBVolume<N+M> SetIntersection(const PBVolume<M> &rhs) const
	{
		PBVolume<N+M> res;
		for(int i = 0; i < N; ++i)
			res.p[i] = p[i];
		for(int i = 0; i < M; ++i)
			res.p[N+i] = rhs.p[i];
		return res;
	}
};

MATH_END_NAMESPACE
