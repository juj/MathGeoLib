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

/** @file QuadTree.h
	@author Jukka Jylänki
	@brief A QuadTree spatial query acceleration structure for dynamic data. */
#pragma once

#ifdef MATH_GRAPHICSENGINE_INTEROP
#include "Time/Profiler.h"
#else
#define MGL_PROFILE(x)
#endif
#include "../Math/float2.h"
#include "AABB2D.h"
#include "../Math/MathTypes.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Container/MaxHeap.h"
#endif

MATH_BEGIN_NAMESPACE

/// A fixed split rule for all QuadTrees: A QuadTree leaf node is only ever split if the leaf contains at least this many objects.
/// Leaves containing fewer than this many objects are always kept as leaves until the object count is exceeded.
static const int minQuadTreeNodeObjectCount = 16;

/// A fixed split limit rule for all QuadTrees: If the QuadTree node side length is smaller than this, the node will
/// never be split again into smaller subnodes. This provides a hard limit safety net for infinite/extra long recursion
/// in case multiple identical overlapping objects are placed into the tree.
static const float minQuadTreeQuadrantSize = 0.05f;

/// Helper for interpreting how to place float3 elements into a QuadTree<float3>.
inline float MinX(const float3 &pt) { return pt.x; }
inline float MaxX(const float3 &pt) { return pt.x; }
inline float MinY(const float3 &pt) { return pt.y; }
inline float MaxY(const float3 &pt) { return pt.y; }

//#ifdef _DEBUG
/// If enabled, QuadTree queries generate debug trace/stats logging when invoked. Use only for debugging/behavioral profiling.
//#define QUADTREE_VERBOSE_LOGGING
//#endif

template<typename T>
class QuadTree
{
public:
	/// @note For space compactness, a QuadTree node does not store its own AABB2D extents.
	struct Node
	{
		/// If 0, this node is the root.
		Node *parent;
		/// Indicates the quad of child nodes for this node, or 0xFFFFFFFF if this node is a leaf.
		u32 childIndex;
		/// Stores the actual objects in this node/leaf.
#ifdef MATH_CONTAINERLIB_SUPPORT
		Array<T> objects;
#else
		std::vector<T> objects;
#endif

		float2 center;
		float2 radius;

		bool IsLeaf() const { return childIndex == 0xFFFFFFFF; }

		u32 TopLeftChildIndex() const { return childIndex; }
		u32 TopRightChildIndex() const { return childIndex+1; }
		u32 BottomLeftChildIndex() const { return childIndex+2; }
		u32 BottomRightChildIndex() const { return childIndex+3; }

		/// This assumes that the QuadTree contains unique objects and never duplicates.
		void Remove(const T &object)
		{
			for(size_t i = 0; i < objects.size(); ++i)
				if (objects[i] == object)
				{
					AssociateQuadTreeNode(object, 0); // Mark in the object that it has been removed from the quadtree.
					std::swap(objects[i], objects.back());
					objects.pop_back();
					return;
				}
		}

		AABB2D ComputeAABB() const
		{
			return AABB2D(center - radius, center + radius);
		}

		float DistanceSq(const float2 &point) const
		{
			float2 centered = point - center;
			float2 closestPoint = centered.Clamp(-radius, radius);
			return closestPoint.DistanceSq(centered);
//			float2 diff = Max(Abs(point - center) - radius, float2(0,0));
//			return diff.LengthSq();
		}
	};

	/// Helper struct used when traversing through the tree.
	struct TraversalStackItem
	{
		Node *node;
	};

	QuadTree()
	:rootNodeIndex(-1),
	boundingAABB(float2(0,0), float2(1,1))
#ifdef QUADTREE_VERBOSE_LOGGING
	,totalNumObjectsInTree(0)
#endif
	{
		///\todo Currently storing persistent raw pointers to this array outside the array.
		/// Remove the requirement to never reallocate the vector!
		nodes.reserve(200000);
	}

	/// Removes all nodes and objects in this tree and reinitializes the tree to a single root node.
	void Clear(const float2 &minXY = float2(-1.f, -1.f), const float2 &maxXY = float2(1.f, 1.f));

	/// Places the given object into the proper (leaf) node of the tree. After placing, if the leaf split rule is
	/// satisfied, subdivides the leaf node into 4 subquadrants and reassigns the objects to new leaves.
	void Add(const T &object);

	/// Removes the given object from this tree.
	/// To call this function, you must define a function QuadTree<T>::Node *GetQuadTreeNode(const T &object)
	/// which returns the node of this quadtree where the object resides in.
	void Remove(const T &object);

	/// @return The bounding rectangle for the whole tree.
	/// @note This bounding rectangle does not tightly bound the objects themselves, only the root node of the tree.
	AABB2D BoundingAABB() const { return boundingAABB; }

	/// @return The topmost node in the tree.
	Node *Root();
	const Node *Root() const;

	/// Returns the total number of nodes (all nodes, i.e. inner nodes + leaves) in the tree.
	/// Runs in constant time.
	int NumNodes() const;

	/// Returns the total number of leaf nodes in the tree.
	/// @warning Runs in time linear 'O(n)' to the number of nodes in the tree.
	int NumLeaves() const;

	/// Returns the total number of inner nodes in the tree.
	/// @warning Runs in time linear 'O(n)' to the number of nodes in the tree.
	int NumInnerNodes() const;

	/// Returns the total number of objects stored in the tree.
	/// @warning Runs in time linear 'O(n)' to the number of nodes in the tree.
	int NumObjects() const;

	/// Returns the maximum height of the whole tree (the path from the root to the farthest leaf node).
	int TreeHeight() const;

	/// Returns the height of the subtree rooted at 'node'.
	int TreeHeight(const Node *node) const;

	/// Performs an AABB intersection query in this Quadtreee, and calls the given callback function for each non-empty
	/// node of the tree which intersects the given AABB.
	/** @param aabb The axis-aligned bounding box to intersect this QuadTree with.
		@param callback A function or a function object of prototype
			bool callbackFunction(QuadTree<T> &tree, const AABB2D &queryAABB, QuadTree<T>::Node &node);
		If the callback function returns true, the execution of the query is stopped and this function immediately
		returns afterwards. If the callback function returns false, the execution of the query continues. */
	template<typename Func>
	inline void AABBQuery(const AABB2D &aabb, Func &callback);

	/// Finds all object pairs inside the given AABB which have colliding AABBs. For each such pair, calls the
	/// specified callback function.
	template<typename Func>
	inline void CollidingPairsQuery(const AABB2D &aabb, Func &callback);

#ifdef MATH_CONTAINERLIB_SUPPORT
	/// Performs a node-granular nearest neighbor search on this QuadTree.
	/** This query calls the given nodeCallback function for each node of this QuadTree that contains objects, sorted by closest first
		to the target point. At any given time, the nodeCallback function may terminate the search by returning true in its callback.
		@param point The
		@param nodeCallback A function or a function object of prototype
		   bool NodeCallbackFunction(QuadTree<T> &tree, const float2 &targetPoint, QuadTree<T>::Node &node, float minDistanceSquared);
		   If the callback function returns true, the execution of the query is immediately stopped.
		   If the callback function returns false, the execution of the query continues.
		   tree points to this QuadTree, in which the query is being performed.
		   targetPoint is the point passed in the function call to NearestNeighborObjects.
		   node points to the QuadTree (leaf) node that is being traversed.
		   minDistanceSquared is the squared minimum distance the objects in this node (and all future nodes to be passed to the
		   callback) have to the point that is being queried. */
	template<typename Func>
	inline void NearestNeighborNodes(const float2 &point, Func &nodeCallback);

	/// Performs an object-granular nearest neighbor search on this QuadTree.
	/** This query calls the given objectCallback function for each object in this QuadTree, starting from the object closest to the
		given target point, and proceeding in distance-sorted order.
		@param targetPoint The target point to find the nearest neighbors to.
		@param objectCallback The function object that should be invoked by the query for each object. This function should be of prototype
		   bool NearestNeighborObjectCallback(QuadTree<T> &tree, const float2 &targetPoint, QuadTree<T>::Node *node,
		                                      float distanceSquared, const T &nearestNeighborObject, int nearestNeighborIndex);
		   If this function returns true, the execution of the query is immediately stopped.
		   If the callback function returns false, the execution of the query continues.
		   tree points to this QuadTree, in which the query is being performed.
		   targetPoint is the point passed in the function call to NearestNeighborObjects.
		   node points to the QuadTree (leaf) node where nearestNeighborObject resides in.
		   distanceSquared is the squared distance between targetPoint and nearestNeighborObject.
		   nearestNeighborObject gives the next closest object to targetPoint.
		   nearestNeighborIndex provides a conveniency counter that tells how many nearest neighbors are closer to targetPoint than this object. */
	template<typename Func>
	inline void NearestNeighborObjects(const float2 &targetPoint, Func &objectCallback);
#endif

	/// Performs various consistency checks on the given node. Use only for debugging purposes.
	void DebugSanityCheckNode(Node *n);

private:
	void Add(const T &object, Node *n);

	/// Allocates a sequential 4-tuple of QuadtreeNodes, contiguous in memory.
	int AllocateNodeGroup(Node *parent);

	void SplitLeaf(Node *leaf);

//#ifdef MATH_CONTAINERLIB_SUPPORT
//	Array<Node> nodes;
//#else
	std::vector<Node> nodes;
//#endif

	/// Specifies the index to the root node, or -1 if there is no root (nodes.size() == 0).
	int rootNodeIndex;
	AABB2D boundingAABB;

	void GrowRootTopLeft();
	void GrowRootTopRight();
	void GrowRootBottomLeft();
	void GrowRootBottomRight();

	void GrowImpl(int quadrantForRoot);

#ifdef QUADTREE_VERBOSE_LOGGING
	int totalNumObjectsInTree;
#endif
};

inline void AssociateQuadTreeNode(const float3 &, QuadTree<float3>::Node *) {}

MATH_END_NAMESPACE

#include "QuadTree.inl"
