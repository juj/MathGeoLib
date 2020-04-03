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

/** @file KDTree.inl
	@author Jukka Jylänki
	@brief Implementation for the KDTree object. */
#pragma once

#include "AABB.h"
#include "OBB.h"
#include "Ray.h"
#include "../Math/assume.h"
#include "../Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

template<typename T>
int KdTree<T>::AllocateNodePair()
{
	int index = (int)nodes.size();
	KdTreeNode n;
	n.splitAxis = AxisNone; // The newly allocated nodes will be leaves.
	n.bucketIndex = 0;
	nodes.push_back(n);
	nodes.push_back(n);
	return index;
}

template<typename T>
void KdTree<T>::FreeBuckets()
{
	for(size_t i = 0; i < buckets.size(); ++i)
		delete[] buckets[i];
	buckets.clear();
}

template<typename T>
AABB KdTree<T>::BoundingAABB(const u32 *bucket) const
{
	assert(bucket);

	AABB a;
	a.SetNegativeInfinity();

	while(*bucket != BUCKET_SENTINEL)
		a.Enclose(Object(*bucket++).BoundingAABB());

	return a;
}

template<typename T>
void KdTree<T>::SplitLeaf(int nodeIndex, const AABB &nodeAABB, int numObjectsInBucket, int leafDepth)
{
	if (leafDepth >= maxTreeDepth)
		return; // Exceeded max depth - disallow splitting.

	KdTreeNode *node = &nodes[nodeIndex];
	assert(node->IsLeaf());
	// Choose the longest axis for the split and convert the node from a leaf to an inner node.
	int curBucketIndex = node->bucketIndex; // The existing objects.
	assert(curBucketIndex != 0); // The leaf must contain some objects, otherwise this function should never be called!
	CardinalAxis splitAxis = (CardinalAxis)nodeAABB.Size().MaxElementIndex();
	float splitPos = nodeAABB.CenterPoint()[splitAxis];

	// Compute the new bounding boxes for the left and right children.
	AABB leftAABB = nodeAABB;
	AABB rightAABB = nodeAABB;
	leftAABB.maxPoint[splitAxis] = splitPos;
	rightAABB.minPoint[splitAxis] = splitPos;

	// Sort all objects into the left and right children.
	u32 *leftBucket = new u32[numObjectsInBucket+1];
	u32 *rightBucket = new u32[numObjectsInBucket+1];

	u32 *curObject = buckets[curBucketIndex];
	u32 *l = leftBucket;
	u32 *r = rightBucket;
	int numObjectsLeft = 0;
	int numObjectsRight = 0;
	while(*curObject != BUCKET_SENTINEL)
	{
		AABB aabb = Object(*curObject).BoundingAABB();
		bool left = leftAABB.Intersects(aabb);
		bool right = rightAABB.Intersects(aabb);
		if (!left && !right)
			left = right = true; // Numerical precision issues: bounding box doesn't intersect either anymore, so place into both children.
		if (left)
		{
			*l++ = *curObject;
			++numObjectsLeft;
		}
		if (right)
		{
			*r++ = *curObject;
			++numObjectsRight;
		}
		++curObject;
	}
	*l = BUCKET_SENTINEL;
	*r = BUCKET_SENTINEL;

	// If we cannot split according to the given split axis, abort the whole process.
	if (numObjectsLeft == numObjectsInBucket || numObjectsRight == numObjectsInBucket)
	{
		delete[] leftBucket;
		delete[] rightBucket;
		return;
	}

	// Ok to split. Turn this leaf node into an inner node.
	node->splitAxis = splitAxis;
	node->splitPos = splitPos;

	// Allocate nodes for the children.
	int childIndex = AllocateNodePair();
	node = &nodes[nodeIndex]; // AllocateNodePair() above invalidates the 'node' pointer! Recompute it.
	node->childIndex = childIndex;

	// Recompute tighter AABB's for the children which have now been populated with objects.
	leftAABB = BoundingAABB(leftBucket);
	rightAABB = BoundingAABB(rightBucket);

	// For the left child, reuse the bucket index the parent had. (free the bucket of the parent)
	KdTreeNode *leftChild = &nodes[childIndex];
	delete[] buckets[curBucketIndex];
	buckets[curBucketIndex] = leftBucket;
	leftChild->bucketIndex = curBucketIndex;

	// For the right child, allocate a new bucket.
	KdTreeNode *rightChild = &nodes[childIndex+1];
	rightChild->bucketIndex = (u32)buckets.size();
	buckets.push_back(rightBucket);

	assert(numObjectsLeft < numObjectsInBucket && numObjectsRight < numObjectsInBucket);

	// Recursively split children.
	if (numObjectsLeft > 16)
		SplitLeaf(childIndex, leftAABB, numObjectsLeft, leafDepth + 1);
	if (numObjectsRight > 16)
		SplitLeaf(childIndex+1, rightAABB, numObjectsRight, leafDepth + 1);
}

template<typename T>
KdTree<T>::~KdTree()
{
	FreeBuckets();
}

template<typename T>
u32 *KdTree<T>::Bucket(int bucketIndex)
{
	return buckets[bucketIndex];
}

template<typename T>
const u32 *KdTree<T>::Bucket(int bucketIndex) const
{
	return buckets[bucketIndex];
}

template<typename T>
T &KdTree<T>::Object(int objectIndex)
{
	return (T&)objects[sizeof(T)*objectIndex];
}

template<typename T>
const T &KdTree<T>::Object(int objectIndex) const
{
	return (const T&)objects[sizeof(T)*objectIndex];
}

/// Returns the total number of nodes (all nodes, i.e. inner nodes + leaves) in the tree.
template<typename T>
int KdTree<T>::NumNodes() const
{
	return nodes.size() - 1;
}

/// Returns the total number of leaf nodes in the tree.
template<typename T>
int KdTree<T>::NumLeaves() const
{
	int numLeaves = 0;
	for(size_t i = 1; i < nodes.size(); ++i)
		if (nodes[i].IsLeaf())
			++numLeaves;

	return numLeaves;
}

template<typename T>
int KdTree<T>::NumObjects() const
{
	return (int)(objects.size() / sizeof(T));
}

/// Returns the total number of inner nodes in the tree.
template<typename T>
int KdTree<T>::NumInnerNodes() const
{
	int numInnerNodes = 0;
	for(size_t i = 1; i < nodes.size(); ++i)
		if (!nodes[i].IsLeaf())
			++numInnerNodes;

	return numInnerNodes;
}

template<typename T>
int KdTree<T>::TreeHeight(int nodeIndex) const
{
	const KdTreeNode &node = nodes[nodeIndex];
	if (node.IsLeaf())
		return 1;
	return 1 + std::max(TreeHeight(node.LeftChildIndex()), TreeHeight(node.RightChildIndex()));
}

template<typename T>
int KdTree<T>::TreeHeight() const
{
	return TreeHeight(1);
}

template<typename T>
void KdTree<T>::AddObjects(const T *objects_, int numObjects)
{
	objects.insert(objects.end(), (const u8*)objects_, (const u8*)(objects_ + numObjects));
#ifdef _DEBUG
	needsBuilding = true;
#endif
}

template<typename T>
void KdTree<T>::Build()
{
	nodes.clear();
	FreeBuckets();

	// Allocate a dummy node to be stored at index 0 (for safety).
	KdTreeNode dummy;
	dummy.splitAxis = AxisNone;
	dummy.childIndex = 0;
	dummy.bucketIndex = 0;
	nodes.push_back(dummy); // Index 0 - dummy unused node, "null pointer".

	// Allocate a dummy bucket at index 0, to denote that a leaf is empty.
	buckets.push_back(0);

	// Add a root node for the tree.
	KdTreeNode rootNode;
	rootNode.splitAxis = AxisNone;
	rootNode.childIndex = 0;
	rootNode.bucketIndex = 1;
	nodes.push_back(rootNode);

	// Initially, add all objects to the root node.
	u32 *rootBucket = new u32[NumObjects()+1];
	for(int i = 0; i < NumObjects(); ++i)
		rootBucket[i] = (u32)i;
	rootBucket[NumObjects()] = BUCKET_SENTINEL;
	buckets.push_back(rootBucket);

	rootAABB = BoundingAABB(rootBucket);

	// We now have a single root leaf node which is unsplit and contains all the objects
	// in the kD-tree. Now recursively subdivide until the whole tree is built.
	SplitLeaf(1, rootAABB, NumObjects(), 1);

#ifdef _DEBUG
	needsBuilding = false;
#endif
}

template<typename T>
void KdTree<T>::Clear()
{
	nodes.clear();
	objects.clear();
	buckets.clear();
#ifdef _DEBUG
	needsBuilding = false;
#endif
}

template<typename T>
KdTreeNode *KdTree<T>::Root() { return nodes.size() > 1 ? &nodes[1] : 0; }

template<typename T>
const KdTreeNode *KdTree<T>::Root() const { return nodes.size() > 1 ? &nodes[1] : 0; }

template<typename T>
bool KdTree<T>::IsPartOfThisTree(const KdTreeNode *node) const
{
	if (!Root())
		return false;
	if (!node)
		return false;
	return IsPartOfThisTree(Root(), node);
}

template<typename T>
bool KdTree<T>::IsPartOfThisTree(const KdTreeNode *root, const KdTreeNode *node) const
{
	assert(root);
	assert(node);
	if (root == node)
		return true;
	if (root->IsLeaf())
		return false;
	return IsPartOfThisTree(&nodes[root->LeftChildIndex()], node) || IsPartOfThisTree(&nodes[root->RightChildIndex()], node);
}

// The "recursive B" method from Vlastimil Havran's thesis.
template<typename T>
template<typename Func>
inline void KdTree<T>::RayQuery(const Ray &r, Func &nodeProcessFunc)
{
	float tNear = 0.f, tFar = FLOAT_INF;

	assume(rootAABB.IsFinite());
	assume(!rootAABB.IsDegenerate());
#ifdef _DEBUG
	assume(!needsBuilding);
#endif

	if (!rootAABB.IntersectLineAABB(r.pos, r.dir, tNear, tFar))
		return; // The ray doesn't intersect the root, therefore no collision.

	// tNear and tFar are updated above to the enter and exit distances of the root box.
	// All objects in kD-tree are bound within the root box, so no need to clip these, which
	// gives better numerical precision in case some objects are very close (or actually outside)
	// the computed kD-tree root box.
	tNear = 0.f;
	tFar = FLOAT_INF;

	static const CardinalAxis axes[] = { AxisX, AxisY, AxisZ, AxisX, AxisY };

	typedef int StackPtr; // Pointer to the traversal stack.
	struct StackElem
	{
		KdTreeNode *node;
		float t;
		vec pos; // entry/exit point coordinates
		StackPtr prev; // index (pointer) to the previous item in stack.
	};

	const int cMaxStackItems = 50*2;
	StackElem stack[cMaxStackItems];

	KdTreeNode *farChild;
	KdTreeNode *currentNode = Root();
	StackPtr entryPoint = 0;
	stack[entryPoint].t = tNear;

	// Check if the ray has internal or external origin relative to the scene root node.
	stack[entryPoint].pos = r.pos + Max(tNear, 0.f) * r.dir;

	StackPtr exitPoint = 1; // ==entryPoint+1;
	stack[exitPoint].t = tFar;
	stack[exitPoint].pos = r.pos + r.dir * tFar;
	stack[exitPoint].node = 0;

	// Traverse through the kdTree.
	while(currentNode)
	{
		while(!currentNode->IsLeaf()) // while currentNode is an internal node (not a leaf)
		{
//#ifdef TRACKSTATS
//			++travelledNodes;
//#endif
//#ifdef VISSTATS
//			if (bVisNumVisitedNodes)
//				++statsCounter;
//#endif
			const float splitPos = currentNode->splitPos;
			const CardinalAxis axis = (CardinalAxis)currentNode->splitAxis;
			if (stack[entryPoint].pos[axis] <= splitPos)
			{
				if (stack[exitPoint].pos[axis] <= splitPos)
				{ // Cases N1,N2,N3,P5,Z2 and Z3.
					currentNode = &nodes[currentNode->LeftChildIndex()];
					continue;
				}
				if (EqualAbs(stack[exitPoint].pos[axis], splitPos))
				{ // Case Z1
					currentNode = &nodes[currentNode->RightChildIndex()];
					continue;
				}
				// Case N4:
				farChild = &nodes[currentNode->RightChildIndex()];
				currentNode = &nodes[currentNode->LeftChildIndex()];
			}
			else
			{
				if (splitPos < stack[exitPoint].pos[axis])
				{ // Cases P1,P2,P3 and N5
					currentNode = &nodes[currentNode->RightChildIndex()];
					continue;
				}
				// Case P4:
				farChild = &nodes[currentNode->LeftChildIndex()];
				currentNode = &nodes[currentNode->RightChildIndex()];
			}
			// From above, only cases N4 and P4 pass us through to here:
			const float t = (splitPos - r.pos[axis]) / r.dir[axis];
			StackPtr tempPtr = exitPoint++;
			if (exitPoint == entryPoint) // avoid overwriting data on the stack.
				++exitPoint;
			assert(exitPoint < cMaxStackItems);

			stack[exitPoint].prev = tempPtr;
			stack[exitPoint].t = t;
			stack[exitPoint].node = farChild;
			stack[exitPoint].pos[axis] = splitPos;
			const CardinalAxis axis2 = axes[axis+1];
			const CardinalAxis axis3 = axes[axis+2];
			stack[exitPoint].pos[axis2] = r.pos[axis2] + t * r.dir[axis2];
			stack[exitPoint].pos[axis3] = r.pos[axis3] + t * r.dir[axis3];
		}

		const float dNear = stack[entryPoint].t;
		const float dFar = stack[exitPoint].t;

		bool traversalFinished = nodeProcessFunc(*this, *currentNode, r, dNear, dFar);
//#ifdef VISSTATS
//		if (bVisNumIntersections)
//			statsCounter += nodeProcessFunc.nIntersections;
//#endif
		if (traversalFinished)
			return;

		// Pop from the stack
		entryPoint = exitPoint;
		currentNode = stack[exitPoint].node;
		exitPoint = stack[entryPoint].prev;
	}
}

template<typename T>
template<typename Func>
inline void KdTree<T>::AABBQuery(const AABB &aabb, Func &leafCallback)
{
	const int cMaxStackItems = maxTreeDepth*2;

	KdTreeNode *stack[cMaxStackItems];
	int stackSize = 1;
	stack[0] = Root();

	// Don't enter the main iteration loop at all if no overlap occurs at the top level.
	if (!aabb.Intersects(BoundingAABB()))
		return;

	// Process the degenerate case where the root node is a singular leaf node.
	if (stack[0]->IsLeaf())
	{
		leafCallback(*this, *stack[0], aabb);
		return;
	}

	while(stackSize > 0)
	{
		KdTreeNode *cur = stack[--stackSize];
		assert(!cur->IsLeaf());

		// We know that aabb intersects with the AABB of the current node, which allows
		// most of the AABB-AABB intersection tests to be ignored.

		// Does the aabb overlap with the left child?
		if (aabb.minPoint[cur->splitAxis] <= cur->splitPos)
		{
			KdTreeNode *leftChild = &nodes[cur->LeftChildIndex()];
			if (leftChild->IsLeaf()) // Leafs are processed immediately, no need to put them to stack for later.
			{
				if (leafCallback(*this, *leftChild, aabb))
					return; // The callback requested to terminate the query, so quit.
			}
			else // The left child is an inner node, push it to stack.
				stack[stackSize++] = leftChild;
		}

		// Does the aabb overlap with the right child?
		if (aabb.maxPoint[cur->splitAxis] >= cur->splitPos)
		{
			KdTreeNode *rightChild = &nodes[cur->RightChildIndex()];
			if (rightChild->IsLeaf()) // Leafs are processed immediately, no need to put them to stack for later.
			{
				if (leafCallback(*this, *rightChild, aabb))
					return; // The callback requested to terminate the query, so quit.
			}
			else // The right child is an inner node, push it to stack.
				stack[stackSize++] = rightChild;
		}
	}
}

#if 0 ///\bug Doesn't work properly. Fix up!

struct StackElem
{
	KdTreeNode *thisNode;
	KdTreeNode *tree2Node;
	AABB thisAABB;
	AABB tree2AABB;
	OBB tree2OBB;
};

///\todo Fix up thread-safety!
const int cMaxStackItems = 50*2;
StackElem stack[cMaxStackItems*1000];


template<typename T>
template<typename Func>
inline void KdTree<T>::KdTreeQuery(KdTree<T> &tree2, const float3x4 &thisWorldTransform, const float3x4 &tree2WorldTransform, Func &leafCallback)
{
	float3x4 tree2Transform = thisWorldTransform.Inverted() * tree2WorldTransform;

	int stackSize = 1;
	stack[0].thisNode = Root();
	stack[0].thisAABB = BoundingAABB();
	stack[0].tree2Node = tree2.Root();
	stack[0].tree2AABB = tree2.BoundingAABB();	
	OBB tree2OBB = stack[0].tree2AABB.Transform(tree2Transform);
	stack[0].tree2OBB = tree2OBB;

	if (!stack[0].thisAABB.Intersects(tree2OBB))
		return;

	while(stackSize > 0)
	{
		assert(stackSize < cMaxStackItems*100);
		--stackSize;
		KdTreeNode *thisNode = stack[stackSize].thisNode;
		KdTreeNode *tree2Node = stack[stackSize].tree2Node;
		AABB thisAABB = stack[stackSize].thisAABB;
		AABB tree2AABB = stack[stackSize].tree2AABB;
		OBB tree2OBB = stack[stackSize].tree2OBB;

		if (thisNode->IsLeaf() && tree2Node->IsLeaf())
		{
			if (leafCallback(*this, *thisNode, thisAABB, tree2, *tree2Node, tree2OBB, tree2Transform))
				return;
		}
		else if (thisNode->IsLeaf())
		{
			if (thisNode->IsEmptyLeaf())
				continue; // We hit an empty leaf, no need to test this against tree2.

			// Test this leaf node against both children of the tree2 inner node.
			KdTreeNode *tree2LeftChild = &tree2.nodes[tree2Node->LeftChildIndex()];
			KdTreeNode *tree2RightChild = &tree2.nodes[tree2Node->RightChildIndex()];
			AABB tree2LeftAABB = tree2AABB;
			AABB tree2RightAABB = tree2AABB;
			tree2LeftAABB.maxPoint[tree2Node->splitAxis] = tree2Node->splitPos;
			tree2RightAABB.minPoint[tree2Node->splitAxis] = tree2Node->splitPos;
			OBB tree2LeftOBB = tree2LeftAABB.Transform(tree2Transform);
			OBB tree2RightOBB = tree2RightAABB.Transform(tree2Transform);
		//	if (thisAABB.Intersects(tree2LeftOBB))
			{
				stack[stackSize].tree2Node = tree2LeftChild;
				stack[stackSize].tree2AABB = tree2LeftAABB;
				stack[stackSize].tree2OBB = tree2LeftOBB;
				++stackSize;
			}
		//	if (thisAABB.Intersects(tree2RightOBB))
			{
				stack[stackSize].thisNode = thisNode;
				stack[stackSize].thisAABB = thisAABB;
				stack[stackSize].tree2Node = tree2RightChild;
				stack[stackSize].tree2AABB = tree2RightAABB;
				stack[stackSize].tree2OBB = tree2RightOBB;
				++stackSize;
			}
		}
		else if (tree2Node->IsLeaf())
		{
			if (tree2Node->IsEmptyLeaf())
				continue; // We hit an empty leaf, no need to test the node against this tree further.

			// Test both children of this inner node against the tree2 leaf node.
			KdTreeNode *leftChild = &nodes[thisNode->LeftChildIndex()];
			KdTreeNode *rightChild = &nodes[thisNode->RightChildIndex()];
			AABB leftAABB = thisAABB;
			AABB rightAABB = thisAABB;
			leftAABB.maxPoint[thisNode->splitAxis] = thisNode->splitPos;
			rightAABB.minPoint[thisNode->splitAxis] = thisNode->splitPos;
		//	if (leftAABB.Intersects(tree2OBB))
			{
				stack[stackSize].thisNode = leftChild;
				stack[stackSize].thisAABB = leftAABB;
				++stackSize;
			}
		//	if (rightAABB.Intersects(tree2OBB))
			{
				stack[stackSize].thisNode = rightChild;
				stack[stackSize].thisAABB = rightAABB;
				stack[stackSize].tree2Node = tree2Node;
				stack[stackSize].tree2AABB = tree2AABB;
				stack[stackSize].tree2OBB = tree2OBB;
				++stackSize;
			}
		}
		else
		{
			// Test all child node pairs.
			KdTreeNode *leftChild = &nodes[thisNode->LeftChildIndex()];
			KdTreeNode *rightChild = &nodes[thisNode->RightChildIndex()];
			AABB leftAABB = thisAABB;
			AABB rightAABB = thisAABB;
			leftAABB.maxPoint[thisNode->splitAxis] = thisNode->splitPos;
			rightAABB.minPoint[thisNode->splitAxis] = thisNode->splitPos;

			KdTreeNode *tree2LeftChild = &tree2.nodes[tree2Node->LeftChildIndex()];
			KdTreeNode *tree2RightChild = &tree2.nodes[tree2Node->RightChildIndex()];
			AABB tree2LeftAABB = tree2AABB;
			AABB tree2RightAABB = tree2AABB;
			tree2LeftAABB.maxPoint[tree2Node->splitAxis] = tree2Node->splitPos;
			tree2RightAABB.minPoint[tree2Node->splitAxis] = tree2Node->splitPos;
			OBB tree2LeftOBB = tree2LeftAABB.Transform(tree2Transform);
			OBB tree2RightOBB = tree2RightAABB.Transform(tree2Transform);

			if (leftAABB.Intersects(tree2LeftOBB))
			{
				stack[stackSize].thisNode = leftChild;
				stack[stackSize].thisAABB = leftAABB;
				stack[stackSize].tree2Node = tree2LeftChild;
				stack[stackSize].tree2AABB = tree2LeftAABB;
				stack[stackSize].tree2OBB = tree2LeftOBB;
				++stackSize;
			}
			if (rightAABB.Intersects(tree2LeftOBB))
			{
				stack[stackSize].thisNode = rightChild;
				stack[stackSize].thisAABB = rightAABB;
				stack[stackSize].tree2Node = tree2LeftChild;
				stack[stackSize].tree2AABB = tree2LeftAABB;
				stack[stackSize].tree2OBB = tree2LeftOBB;
				++stackSize;
			}
			if (leftAABB.Intersects(tree2RightOBB))
			{
				stack[stackSize].thisNode = leftChild;
				stack[stackSize].thisAABB = leftAABB;
				stack[stackSize].tree2Node = tree2RightChild;
				stack[stackSize].tree2AABB = tree2RightAABB;
				stack[stackSize].tree2OBB = tree2RightOBB;
				++stackSize;
			}
			if (rightAABB.Intersects(tree2RightOBB))
			{
				stack[stackSize].thisNode = rightChild;
				stack[stackSize].thisAABB = rightAABB;
				stack[stackSize].tree2Node = tree2RightChild;
				stack[stackSize].tree2AABB = tree2RightAABB;
				stack[stackSize].tree2OBB = tree2RightOBB;
				++stackSize;
			}
		}
	}
}

#endif

#ifdef MATH_CONTAINERLIB_SUPPORT
struct NearestObjectsTraversalNode
{
	float d;
	AABB aabb;
	KdTreeNode *node;

	bool operator <(const NearestObjectsTraversalNode &t) const { return d > t.d; }
};

template<typename T>
template<typename Func>
inline void KdTree<T>::NearestObjects(const vec &point, Func &leafCallback)
{
	MaxHeap<NearestObjectsTraversalNode> queue;
	NearestObjectsTraversalNode t;
	t.d = 0.f;
	t.aabb = BoundingAABB();
	t.node = Root();
	queue.Insert(t);

	while(queue.Size() > 0)
	{
		t = queue.Front();
		queue.PopFront();

		if (t.node->IsLeaf())
			leafCallback(*this, point, *t.node, t.aabb, t.d);
		else
		{
			NearestObjectsTraversalNode n;

			// Insert left child node to the traversal queue.
			n.aabb = t.aabb;
			n.aabb.maxPoint[t.node->splitAxis] = t.node->splitPos;
			n.node = &nodes[t.node->LeftChildIndex()];
			n.d = n.aabb.Distance(point);
			queue.Insert(n);

			// Insert right child node to the traversal queue.
			n.aabb.maxPoint[t.node->splitAxis] = t.aabb.maxPoint[t.node->splitAxis]; /// Restore the change done above.
			n.aabb.minPoint[t.node->splitAxis] = t.node->splitPos;
			n.node = &nodes[t.node->RightChildIndex()];
			n.d = n.aabb.Distance(point);
			queue.Insert(n);
		}
	}
}
#endif

MATH_END_NAMESPACE
