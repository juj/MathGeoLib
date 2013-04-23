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

/** @file QuadTree.inl
	@author Jukka Jylänki
	@brief Implementation for the QuadTree object. */
#pragma once

#include "../Math/MathFunc.h"

MATH_BEGIN_NAMESPACE

template<typename T>
void QuadTree<T>::Clear(const float2 &minXY, const float2 &maxXY)
{
	nodes.clear();

	boundingAABB.minPoint = minXY;
	boundingAABB.maxPoint = maxXY;

	assert(!boundingAABB.IsDegenerate());

	rootNodeIndex = AllocateNodeGroup(0);
	assert(Root());

#ifdef QUADTREE_VERBOSE_LOGGING
	totalNumObjectsInTree = 0;
#endif
}

template<typename T>
void QuadTree<T>::Add(const T &object)
{
	PROFILE(QuadTree_Add);
	Node *n = Root();
	assert(n && "Error: QuadTree has not been initialized with a root node! Call QuadTree::Clear() to initialize the root node.");

	assert(boundingAABB.IsFinite());
	assert(!boundingAABB.IsDegenerate());

	AABB2D objectAABB = GetAABB2D(object);
	assert(objectAABB.IsFinite());
	assert(!objectAABB.HasNegativeVolume());

#ifdef QUADTREE_VERBOSE_LOGGING
	++totalNumObjectsInTree;
#endif

	if (objectAABB.minPoint.x >= boundingAABB.minPoint.x)
	{
		// Object fits left.

		if (objectAABB.maxPoint.x <= boundingAABB.maxPoint.x)
		{
			// Object fits left and right.

			if (objectAABB.minPoint.y >= boundingAABB.minPoint.y)
			{
				// Object fits left, right and top.
				if (objectAABB.maxPoint.y <= boundingAABB.maxPoint.y)
				{
					// Object fits the whole root AABB. Can safely add into the existing tree size.
					Add(object, n, boundingAABB);
					return;
				}
				else
				{
					// Object fits left, right and top, but not bottom.
					GrowRootBottomRight(); // Could grow bottom-left as well, wouldn't matter here.
				}
			}
			else
			{
				// Object fits left and right, but not to top.
				GrowRootTopRight(); // Could grow top-left as well, wouldn't matter here.
			}
		}
		else
		{
			// Object fits left, but not to right. We must grow right. Check whether to grow top or bottom.
			if (objectAABB.minPoint.y < boundingAABB.minPoint.y)
				GrowRootTopRight();
			else
				GrowRootBottomRight();
		}
	}
	else
	{
		// We must grow left. Check whether to grow top or bottom.
		if (objectAABB.minPoint.y < boundingAABB.minPoint.y)
			GrowRootTopLeft();
		else
			GrowRootBottomLeft();
	}

	// Now that we have grown the tree root node, try adding again.
	Add(object);
}

template<typename T>
void QuadTree<T>::Remove(const T &object)
{
	Node *n = GetQuadTreeNode(object);
	if (n)
	{
		n->Remove(object);

#ifdef QUADTREE_VERBOSE_LOGGING
		--totalNumObjectsInTree;
#endif
	}
}

template<typename T>
void QuadTree<T>::Add(const T &object, Node *n, AABB2D aabb)
{
	for(;;)
	{
		float halfX = (aabb.minPoint.x + aabb.maxPoint.x) * 0.5f;
		float halfY = (aabb.minPoint.y + aabb.maxPoint.y) * 0.5f;
		// Traverse the QuadTree to decide which quad to place this object into.
		assert(MinX(object) <= MaxX(object));
		bool left = MinX(object) < halfX;
		bool right = MaxX(object) > halfX;
		assert(MinY(object) <= MaxY(object));
		bool top = MinY(object) < halfY;
		bool bottom = MaxY(object) > halfY;

		// Note: It can happen that !left && !right, or !top && !bottom,
		// but the if()s below are set up so that right/bottom is taken if no left/top, so that is ok.

		// We must put the object onto this node if
		// a) the object straddled the parent->child split lines.
		// b) this object is a leaf.
		if (n->IsLeaf() || (left && right) || (top && bottom))
		{
//			n->bucket.push_back(objectId);
			n->objects.push_back(object);
			AssociateQuadTreeNode(object, n);
			if (n->IsLeaf() && (int)n->objects.size() > minQuadTreeNodeObjectCount && aabb.Width() >= minQuadTreeQuadrantSize && aabb.Height() >= minQuadTreeQuadrantSize)
				SplitLeaf(n, aabb);
			return;
		}
		if (left)
		{
			aabb.maxPoint.x = halfX;
			if (top)
			{
				aabb.maxPoint.y = halfY;
				assert(nodes[n->TopLeftChildIndex()].parent == n);
				n = &nodes[n->TopLeftChildIndex()];
			}
			else
			{
				aabb.minPoint.y = halfY;
				assert(nodes[n->BottomLeftChildIndex()].parent == n);
				n = &nodes[n->BottomLeftChildIndex()];
			}
		}
		else
		{
			aabb.minPoint.x = halfX;
			if (top)
			{
				aabb.maxPoint.y = halfY;
				assert(nodes[n->TopRightChildIndex()].parent == n);
				n = &nodes[n->TopRightChildIndex()];
			}
			else
			{
				aabb.minPoint.y = halfY;
				assert(nodes[n->BottomRightChildIndex()].parent == n);
				n = &nodes[n->BottomRightChildIndex()];
			}
		}
	}
}

template<typename T>
typename QuadTree<T>::Node *QuadTree<T>::Root()
{
	return nodes.empty() ? 0 : &nodes[rootNodeIndex];
}

template<typename T>
const typename QuadTree<T>::Node *QuadTree<T>::Root() const
{
	return nodes.empty() ? 0 : &nodes[rootNodeIndex];
}

template<typename T>
int QuadTree<T>::AllocateNodeGroup(Node *parent)
{
#ifdef _DEBUG
	size_t oldCap = nodes.capacity();
#endif
	int index = (int)nodes.size();
	Node n;
	n.parent = parent;
	n.childIndex = 0xFFFFFFFF;
	nodes.push_back(n);
	nodes.push_back(n);
	nodes.push_back(n);
	nodes.push_back(n);
#ifdef _DEBUG
	assert(nodes.capacity() == oldCap); // Limitation: Cannot resize the nodes vector!
#endif
	return index;
}

template<typename T>
void QuadTree<T>::SplitLeaf(Node *leaf, const AABB2D &leafAABB)
{
	assert(leaf->IsLeaf());
	assert(leaf->childIndex == 0xFFFFFFFF);

	leaf->childIndex = AllocateNodeGroup(leaf);

	float halfX = (leafAABB.minPoint.x + leafAABB.maxPoint.x) * 0.5f;
	float halfY = (leafAABB.minPoint.y + leafAABB.maxPoint.y) * 0.5f;

	size_t i = 0;
	while(i < leaf->objects.size())
	{
		const T &object = leaf->objects[i];

		// Traverse the QuadTree to decide which quad to place this object into.
		assert(MinX(object) <= MaxX(object));
		bool left = MinX(object) < halfX;
		bool right = MaxX(object) > halfX;
		assert(MinY(object) <= MaxY(object));
		bool top = MinY(object) < halfY;
		bool bottom = MaxY(object) > halfY;

		// Note: It can happen that !left && !right, or !top && !bottom,
		// but the if()s below are set up so that right/bottom is taken if no left/top, so that is ok.

		// We must leave this object in this node if the object straddled the parent->child split lines.
		if ((left && right) || (top && bottom))
		{
			++i;
			continue;
		}

		AABB2D child;
		if (left)
		{
			child.minPoint.x = leafAABB.minPoint.x;
			child.maxPoint.x = halfX;
			if (top)
			{
				child.minPoint.y = leafAABB.minPoint.y;
				child.maxPoint.y = halfY;
				Add(object, &nodes[leaf->TopLeftChildIndex()], child);
			}
			else
			{
				child.minPoint.y = halfY;
				child.maxPoint.y = leafAABB.maxPoint.y;
				Add(object, &nodes[leaf->BottomLeftChildIndex()], child);
			}
		}
		else
		{
			child.minPoint.x = halfX;
			child.maxPoint.x = leafAABB.maxPoint.x;
			if (top)
			{
				child.minPoint.y = leafAABB.minPoint.y;
				child.maxPoint.y = halfY;
				Add(object, &nodes[leaf->TopRightChildIndex()], child);
			}
			else
			{
				child.minPoint.y = halfY;
				child.maxPoint.y = leafAABB.maxPoint.y;
				Add(object, &nodes[leaf->BottomRightChildIndex()], child);
			}
		}

		// Remove the object we added to a child from this node.
		std::swap(leaf->objects[i], leaf->objects.back());
		leaf->objects.pop_back();
	}
}

template<typename T>
template<typename Func>
inline void QuadTree<T>::AABBQuery(const AABB2D &aabb, Func &callback)
{
	PROFILE(QuadTree_AABBQuery);
	std::vector<TraversalStackItem> stack;
	TraversalStackItem n;
	n.aabb = BoundingAABB();
	n.node = Root();
	if (!n.node || !n.aabb.Intersects(aabb))
		return;
	stack.push_back(n);

	while(!stack.empty())
	{
		TraversalStackItem i = stack.back();
		stack.pop_back();

		float halfX = (i.aabb.minPoint.x + i.aabb.maxPoint.x) * 0.5f;
		float halfY = (i.aabb.minPoint.y + i.aabb.maxPoint.y) * 0.5f;

		// aabb intersects the node's aabb.
		// Which aabb's of the four child quadrants does it intersect?

		if (i.node->objects.size() > 0)
		{
			if (callback(*this, aabb, *i.node, i.aabb))
				return;
		}
		if (!i.node->IsLeaf())
		{
			if (aabb.minPoint.x <= halfX && aabb.minPoint.y <= halfY)
			{
				TraversalStackItem child;
				child.aabb.minPoint.x = i.aabb.minPoint.x;
				child.aabb.minPoint.y = i.aabb.minPoint.y;
				child.aabb.maxPoint.x = halfX;
				child.aabb.maxPoint.y = halfY;
				child.node = &nodes[i.node->TopLeftChildIndex()];
				stack.push_back(child);
			}
			if (aabb.maxPoint.x >= halfX && aabb.maxPoint.y >= halfY)
			{
				TraversalStackItem child;
				child.aabb.minPoint.x = halfX;
				child.aabb.minPoint.y = halfY;
				child.aabb.maxPoint.x = i.aabb.maxPoint.x;
				child.aabb.maxPoint.y = i.aabb.maxPoint.y;
				child.node = &nodes[i.node->BottomRightChildIndex()];
				stack.push_back(child);
			}
			if (aabb.minPoint.x <= halfX && aabb.maxPoint.y >= halfY)
			{
				TraversalStackItem child;
				child.aabb.minPoint.x = i.aabb.minPoint.x;
				child.aabb.minPoint.y = halfY;
				child.aabb.maxPoint.x = halfX;
				child.aabb.maxPoint.y = i.aabb.maxPoint.y;
				child.node = &nodes[i.node->BottomLeftChildIndex()];
				stack.push_back(child);
			}
			if (aabb.maxPoint.x >= halfX && aabb.minPoint.y <= halfY)
			{
				TraversalStackItem child;
				child.aabb.minPoint.x = halfX;
				child.aabb.minPoint.y = i.aabb.minPoint.y;
				child.aabb.maxPoint.x = i.aabb.maxPoint.x;
				child.aabb.maxPoint.y = halfY;
				child.node = &nodes[i.node->TopRightChildIndex()];
				stack.push_back(child);
			}
		}
	}
}

template<typename T, typename Func>
class FindCollidingPairs
{
public:
	Func *collisionCallback;

	bool operator ()(QuadTree<T> & /*tree*/, const AABB2D &queryAABB, typename QuadTree<T>::Node &node, const AABB2D & /*nodeAABB*/)
	{
		for(size_t i = 0; i < node.objects.size(); ++i)
		{
			AABB2D aabbI = GetAABB2D(node.objects[i]);
			if (!queryAABB.Intersects(aabbI))
				continue;

			for(size_t j = i+1; j < node.objects.size(); ++j)
			{
				AABB2D aabbJ = GetAABB2D(node.objects[j]);
				if (aabbI.Intersects(aabbJ))
					(*collisionCallback)(node.objects[i], node.objects[j]);
			}

			typename QuadTree<T>::Node *n = node.parent;
			while(n)
			{
				for(size_t j = 0; j < n->objects.size(); ++j)
				{
					AABB2D aabbJ = GetAABB2D(n->objects[j]);
					if (aabbI.Intersects(aabbJ))
						(*collisionCallback)(node.objects[i], n->objects[j]);
				}
				assert(n != n->parent);
				n = n->parent;
			}
		}
		return false;
	}
};

template<typename T>
template<typename Func>
inline void QuadTree<T>::CollidingPairsQuery(const AABB2D &aabb, Func &callback)
{
	PROFILE(QuadTree_CollidingPairsQuery);
	FindCollidingPairs<T, Func> func;
	func.collisionCallback = &callback;
	AABBQuery(aabb, func);
}

template<typename T>
struct TraversalNode
{
	/// The squared distance of this node to the query point.
	float d;
	/// Stores the 2D bounding rectangle of this node.
	AABB2D aabb;
	typename QuadTree<T>::Node *node;

	/// We compare in reverse order, since we want the node with the smallest distance to be visited first,
	/// and MaxHeap stores the node that compares largest in the root.
	bool operator <(const TraversalNode &t) const { return d > t.d; }
	bool operator ==(const TraversalNode &t) const { return d == t.d; }
};

#ifdef MATH_CONTAINERLIB_SUPPORT
template<typename T>
template<typename Func>
inline void QuadTree<T>::NearestNeighborNodes(const float2 &point, Func &leafCallback)
{
	MaxHeap<TraversalNode<T> > queue;
	TraversalNode<T> t;
	t.d = 0.f;
	t.aabb = BoundingAABB();
	t.node = Root();
	queue.Insert(t);

	while(queue.Size() > 0)
	{
		t = queue.Front();
		queue.PopFront();

		if (t.node->objects.size() > 0)
		{
			bool stopIteration = leafCallback(*this, point, *t.node, t.aabb, t.d);
			if (stopIteration)
				return;
		}
		
		if (!t.node->IsLeaf())
		{
			TraversalNode<T> n;

			float halfX = (t.aabb.minPoint.x + t.aabb.maxPoint.x) * 0.5f;
			float halfY = (t.aabb.minPoint.y + t.aabb.maxPoint.y) * 0.5f;

			// Insert bottom-left child node to the traversal queue.
			n.aabb.minPoint.x = t.aabb.minPoint.x;
			n.aabb.maxPoint.x = halfX;
			n.aabb.minPoint.y = halfY;
			n.aabb.maxPoint.y = t.aabb.maxPoint.y;
			n.node = &nodes[t.node->BottomLeftChildIndex()];
			n.d = n.aabb.DistanceSq(point);
			queue.Insert(n);

			// Insert bottom-right child node to the traversal queue.
			n.aabb.minPoint.x = halfX;
			n.aabb.maxPoint.x = t.aabb.maxPoint.x;
			n.node = &nodes[t.node->BottomRightChildIndex()];
			n.d = n.aabb.DistanceSq(point);
			queue.Insert(n);

			// Insert top-right child node to the traversal queue.
			n.aabb.minPoint.y = t.aabb.minPoint.y;
			n.aabb.maxPoint.y = halfY;
			n.node = &nodes[t.node->TopRightChildIndex()];
			n.d = n.aabb.DistanceSq(point);
			queue.Insert(n);

			// Insert top-left child node to the traversal queue.
			n.aabb.minPoint.x = t.aabb.minPoint.x;
			n.aabb.maxPoint.x = halfX;
			n.node = &nodes[t.node->TopLeftChildIndex()];
			n.d = n.aabb.DistanceSq(point);
			queue.Insert(n);
		}
	}
}

template<typename ObjectCallbackFunc, typename T>
struct NearestNeighborObjectSearch
{
	NearestNeighborObjectSearch()
	:objectCallback(0), numObjectsOutputted(0)
#ifdef QUADTREE_VERBOSE_LOGGING
	,numNodesVisited(0)
#endif
	{
	}

	ObjectCallbackFunc *objectCallback;

	struct NearestObject
	{
		/// The squared distance of this node to the query point.
		float d;

		/// Stores the 2D bounding rectangle of this node.
		AABB2D aabb;
		typename QuadTree<T>::Node *node;

		T *object;

		/// We compare in reverse order, since we want the object with the smallest distance to be visited first,
		/// and MaxHeap stores the object that compares largest in the root.
		bool operator <(const NearestObject &t) const { return d > t.d; }
		bool operator ==(const NearestObject &t) const { return d == t.d; }
	};

	MaxHeap<NearestObject> queue;

	int numObjectsOutputted;

#ifdef QUADTREE_VERBOSE_LOGGING
	int numNodesVisited;
#endif

	bool operator ()(QuadTree<T> &tree, const float2 &point, typename QuadTree<T>::Node &leaf, const AABB2D &aabb, float minDistanceSquared)
	{
#ifdef QUADTREE_VERBOSE_LOGGING
		++numNodesVisited;
#endif

		// Output all points that are closer than the next closest AABB node.
		while(queue.Size() > 0 && queue.Front().d <= minDistanceSquared)
		{
			const NearestObject &nextNearestPoint = queue.Front();
			bool shouldStopIteration = (*objectCallback)(tree, point, nextNearestPoint.node, nextNearestPoint.aabb, nextNearestPoint.d, *nextNearestPoint.object, numObjectsOutputted++);
			if (shouldStopIteration)
			{
#ifdef QUADTREE_VERBOSE_LOGGING
				int numPoints = tree.NumObjects();
				LOGI("Visited %d/%d (%.2f%%) of QuadTree nodes (tree height: %d). Saw %d/%d (%.2f%%) points of the QuadTree before outputting %d points.",
					numNodesVisited, tree.NumNodes(), 100.f * numNodesVisited / tree.NumNodes(), -1/*tree.TreeHeight()*/,
					(int)queue.Size() + numObjectsOutputted, numPoints, ((int)queue.Size() + numObjectsOutputted) * 100.f / numPoints,
					numObjectsOutputted);
#endif
				return true;
			}

			queue.PopFront();
		}

		// Queue up all points in the new AABB node.
		for(size_t i = 0; i < leaf.objects.size(); ++i)
		{
			NearestObject obj;
			obj.d = leaf.objects[i].DistanceSq(point);
			obj.aabb = aabb;
			obj.node = &leaf;
			obj.object = &leaf.objects[i];
			queue.Insert(obj);
		}

		return false;
	}
};

template<typename T>
template<typename Func>
inline void QuadTree<T>::NearestNeighborObjects(const float2 &point, Func &leafCallback)
{
	NearestNeighborObjectSearch<Func, T> search;
	search.objectCallback = &leafCallback;

	NearestNeighborNodes(point, search);
}
#endif

template<typename T>
void QuadTree<T>::GrowRootTopLeft()
{
	boundingAABB.minPoint.x -= boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.minPoint.y -= boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// The old root will become the bottom-right child of the new root, at index 3. Swap the root node to its proper place.
	GrowImpl(3);
}

template<typename T>
void QuadTree<T>::GrowRootTopRight()
{
	boundingAABB.maxPoint.x += boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.minPoint.y -= boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// The old root will become the bottom-left child of the new root, at index 2. Swap the root node to its proper place.
	GrowImpl(2);
}

template<typename T>
void QuadTree<T>::GrowRootBottomLeft()
{
	boundingAABB.minPoint.x -= boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.maxPoint.y += boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// The old root will become the top-right child of the new root, at index 1. Swap the root node to its proper place.
	GrowImpl(1);
}

template<typename T>
void QuadTree<T>::GrowRootBottomRight()
{
	boundingAABB.maxPoint.x += boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.maxPoint.y += boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// The old root will become the top-left child of the new root, at index 0, so no swapping is necessary.
	GrowImpl(0);
}

template<typename T>
void QuadTree<T>::GrowImpl(int quadrantForRoot)
{
	// quadrantForRoot specifies the child quadrant the old root is put to in the new root node.

	// rootNodeIndex always points to the first index of the four quadrants.
	Node *oldRoot = &nodes[rootNodeIndex+quadrantForRoot];

	if (quadrantForRoot != 0)
	{
		Swap(nodes[rootNodeIndex], nodes[rootNodeIndex+quadrantForRoot]);

		// Fix up the refs to the swapped old root node.
		if (!oldRoot->IsLeaf())
		{
			nodes[oldRoot->TopLeftChildIndex()].parent = oldRoot;
			nodes[oldRoot->TopRightChildIndex()].parent = oldRoot;
			nodes[oldRoot->BottomLeftChildIndex()].parent = oldRoot;
			nodes[oldRoot->BottomRightChildIndex()].parent = oldRoot;
		}

		// Fix up object->node associations to the swapped old root node.
		for(size_t i = 0; i < oldRoot->objects.size(); ++i)
			AssociateQuadTreeNode(oldRoot->objects[i], oldRoot);
	}


	int oldRootNodeIndex = rootNodeIndex;
	rootNodeIndex = AllocateNodeGroup(0);
	Node *newRoot = &nodes[rootNodeIndex];
	newRoot->childIndex = oldRootNodeIndex;
	nodes[newRoot->TopLeftChildIndex()].parent = newRoot;
	nodes[newRoot->TopRightChildIndex()].parent = newRoot;
	nodes[newRoot->BottomLeftChildIndex()].parent = newRoot;
	nodes[newRoot->BottomRightChildIndex()].parent = newRoot;

	DebugSanityCheckNode(Root());
}

template<typename T>
int QuadTree<T>::NumNodes() const
{
	return std::max<int>(0, nodes.size() - 3); // The nodes rootNodeIndex+1, rootNodeIndex+2 and rootNodeIndex+3 are dummy unused, since the root node is not a quadrant.
}

template<typename T>
int QuadTree<T>::NumLeaves() const
{
	int numLeaves = 0;
	for(int i = 0; i < (int)nodes.size(); ++i)
		if (i <= rootNodeIndex || i >= rootNodeIndex + 4) // The nodes rootNodeIndex+1, rootNodeIndex+2 and rootNodeIndex+3 are dummy unused, since the root node is not a quadrant.
			if (nodes[i].IsLeaf())
				++numLeaves;

	return numLeaves;
}

template<typename T>
int QuadTree<T>::NumInnerNodes() const
{
	int numInnerNodes = 0;
	for(int i = 0; i < (int)nodes.size(); ++i)
		if (i <= rootNodeIndex || i >= rootNodeIndex + 4) // The nodes rootNodeIndex+1, rootNodeIndex+2 and rootNodeIndex+3 are dummy unused, since the root node is not a quadrant.
			if (!nodes[i].IsLeaf())
				++numInnerNodes;

	return numInnerNodes;
}

template<typename T>
int QuadTree<T>::NumObjects() const
{
#ifdef QUADTREE_VERBOSE_LOGGING
	return totalNumObjectsInTree;
#else
	int numObjects = 0;
	for(int i = 0; i < (int)nodes.size(); ++i)
		numObjects += (int)nodes[i].objects.size();
	return numObjects;
#endif
}

template<typename T>
int QuadTree<T>::TreeHeight(const Node *node) const
{
	if (node->IsLeaf())
		return 1;
	return 1 + Max(TreeHeight(&nodes[node->TopLeftChildIndex()]),
	               TreeHeight(&nodes[node->TopRightChildIndex()]),
	               TreeHeight(&nodes[node->BottomLeftChildIndex()]),
	               TreeHeight(&nodes[node->BottomRightChildIndex()]));
}

template<typename T>
int QuadTree<T>::TreeHeight() const
{
	if (!Root())
		return 0;
	return TreeHeight(Root());
}

template<typename T>
void QuadTree<T>::DebugSanityCheckNode(Node *n)
{
#ifdef _DEBUG
	assert(n);
	assert(n->parent || n == Root()); // If no parent, must be root.
	assert(n != Root() || !n->parent); // If not root, must have a parent.

	// Must have a good AABB.
	AABB2D aabb = ComputeAABB(n);
	assert(aabb.IsFinite());
	assert(aabb.minPoint.x <= aabb.maxPoint.x);
	assert(aabb.minPoint.y <= aabb.maxPoint.y);

	LOGI("Node AABB: %s.", aabb.ToString().c_str());
	// Each object in this node must be contained in this node.
	for(size_t i = 0; i < n->objects.size(); ++i)
	{
		LOGI("Object AABB: %s.", GetAABB2D(n->objects[i]).ToString().c_str());

		assert(aabb.Contains(GetAABB2D(n->objects[i])));
	}

	// Parent <-> child links must be valid.
	if (!n->IsLeaf())
	{
		assert(nodes[n->TopLeftChildIndex()].parent == n);
		assert(nodes[n->TopRightChildIndex()].parent == n);
		assert(nodes[n->BottomLeftChildIndex()].parent == n);
		assert(nodes[n->BottomRightChildIndex()].parent == n);

		DebugSanityCheckNode(&nodes[n->TopLeftChildIndex()]);
		DebugSanityCheckNode(&nodes[n->TopRightChildIndex()]);
		DebugSanityCheckNode(&nodes[n->BottomLeftChildIndex()]);
		DebugSanityCheckNode(&nodes[n->BottomRightChildIndex()]);
	}
#else
	MARK_UNUSED(n);
#endif
}

MATH_END_NAMESPACE
