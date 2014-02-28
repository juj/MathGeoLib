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
	Node *root = Root();
	root->center = (minXY + maxXY) * 0.5f;
	root->radius = maxXY - root->center;

#ifdef QUADTREE_VERBOSE_LOGGING
	totalNumObjectsInTree = 0;
#endif
}

template<typename T>
void QuadTree<T>::Add(const T &object)
{
	MGL_PROFILE(QuadTree_Add);
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
					Add(object, n);
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
void QuadTree<T>::Add(const T &object, Node *n)
{
	for(;;)
	{
		// Traverse the QuadTree to decide which quad to place this object into.
		assert(MinX(object) <= MaxX(object));
		float left = n->center.x - MinX(object); // If left > 0.f, then the object overlaps with the left quadrant.
		float right = MaxX(object) - n->center.x; // If right > 0.f, then the object overlaps with the right quadrant.
		assert(MinY(object) <= MaxY(object));
		float top = n->center.y - MinY(object); // If top > 0.f, then the object overlaps with the top quadrant.
		float bottom = MaxY(object) - n->center.y; // If bottom > 0.f, then the object overlaps with the bottom quadrant.
		float leftAndRight = Min(left, right); // If > 0.f, then the object straddles left-right halves.
		float topAndBottom = Min(top, bottom); // If > 0.f, then the object straddles top-bottom halves.
		float straddledEitherOne = Max(leftAndRight, topAndBottom); // If > 0.f, then the object is in two or more quadrants.

		// Note: It can happen that !left && !right, or !top && !bottom,
		// but the if()s below are set up so that right/bottom is taken if no left/top, so that is ok.

		// We must put the object onto this node if
		// a) the object straddled the parent->child split lines.
		// b) this object is a leaf.
		if (straddledEitherOne > 0.f)
		{
			n->objects.push_back(object);
			AssociateQuadTreeNode(object, n);
			return;
		}
		if (n->IsLeaf())
		{
			n->objects.push_back(object);
			AssociateQuadTreeNode(object, n);
			if ((int)n->objects.size() > minQuadTreeNodeObjectCount && Min(n->radius.x, n->radius.y) >= minQuadTreeQuadrantSize)
				SplitLeaf(n);
			return;
		}
		if (left > 0.f)
		{
			if (top > 0.f)
			{
				assert(nodes[n->TopLeftChildIndex()].parent == n);
				n = &nodes[n->TopLeftChildIndex()];
			}
			else
			{
				assert(nodes[n->BottomLeftChildIndex()].parent == n);
				n = &nodes[n->BottomLeftChildIndex()];
			}
		}
		else
		{
			if (top > 0.f)
			{
				assert(nodes[n->TopRightChildIndex()].parent == n);
				n = &nodes[n->TopRightChildIndex()];
			}
			else
			{
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
	// The nodes are in order top-left (--), top-right, bottom-left, bottom-right.
	if (parent)
	{
		n.radius = parent->radius * 0.5f;
		n.center = parent->center - n.radius;
	}
	nodes.push_back(n);
	if (parent)
		n.center.x = parent->center.x + n.radius.x;
	nodes.push_back(n);
	if (parent)
	{
		n.center.x = parent->center.x - n.radius.x;
		n.center.y = parent->center.y + n.radius.y;
	}
	nodes.push_back(n);
	if (parent)
		n.center.x = parent->center.x + n.radius.x;
	nodes.push_back(n);
#ifdef _DEBUG
	assert(nodes.capacity() == oldCap); // Limitation: Cannot resize the nodes vector!
#endif
	return index;
}

template<typename T>
void QuadTree<T>::SplitLeaf(Node *leaf)
{
	assert(leaf->IsLeaf());
	assert(leaf->childIndex == 0xFFFFFFFF);

	leaf->childIndex = AllocateNodeGroup(leaf);

	size_t i = 0;
	while(i < leaf->objects.size())
	{
		const T &object = leaf->objects[i];

		// Traverse the QuadTree to decide which quad to place this object into.
		assert(MinX(object) <= MaxX(object));
		float left = leaf->center.x - MinX(object); // If left > 0.f, then the object overlaps with the left quadrant.
		float right = MaxX(object) - leaf->center.x; // If right > 0.f, then the object overlaps with the right quadrant.
		assert(MinY(object) <= MaxY(object));
		float top = leaf->center.y - MinY(object); // If top > 0.f, then the object overlaps with the top quadrant.
		float bottom = MaxY(object) - leaf->center.y; // If bottom > 0.f, then the object overlaps with the bottom quadrant.
		float leftAndRight = Min(left, right); // If > 0.f, then the object straddles left-right halves.
		float topAndBottom = Min(top, bottom); // If > 0.f, then the object straddles top-bottom halves.
		float straddledEitherOne = Max(leftAndRight, topAndBottom); // If > 0.f, then the object is in two or more quadrants.

		// Note: It can happen that !left && !right, or !top && !bottom,
		// but the if()s below are set up so that right/bottom is taken if no left/top, so that is ok.

		// We must leave this object in this node if the object straddled the parent->child split lines.
		if (straddledEitherOne > 0.f)
		{
			++i;
			continue;
		}

		if (left > 0.f)
		{
			if (top > 0.f)
			{
				Add(object, &nodes[leaf->TopLeftChildIndex()]);
			}
			else
			{
				Add(object, &nodes[leaf->BottomLeftChildIndex()]);
			}
		}
		else
		{
			if (top > 0.f)
			{
				Add(object, &nodes[leaf->TopRightChildIndex()]);
			}
			else
			{
				Add(object, &nodes[leaf->BottomRightChildIndex()]);
			}
		}

		// Remove the object we added to a child from this node.
		leaf->objects[i] = leaf->objects.back();
		leaf->objects.pop_back();
	}
}

template<typename T>
template<typename Func>
inline void QuadTree<T>::AABBQuery(const AABB2D &aabb, Func &callback)
{
	MGL_PROFILE(QuadTree_AABBQuery);
	std::vector<TraversalStackItem> stack;
	TraversalStackItem n;
	n.node = Root();
	if (!n.node || !aabb.Intersects(BoundingAABB()))
		return;
	stack.push_back(n);

	while(!stack.empty())
	{
		TraversalStackItem i = stack.back();
		stack.pop_back();

		// aabb intersects the node's aabb.
		// Which aabb's of the four child quadrants does it intersect?

		if (i.node->objects.size() > 0)
		{
			if (callback(*this, aabb, *i.node))
				return;
		}
		if (!i.node->IsLeaf())
		{
			if (aabb.minPoint.x <= i.node->center.x && aabb.minPoint.y <= i.node->center.y)
			{
				TraversalStackItem child;
				child.node = &nodes[i.node->TopLeftChildIndex()];
				stack.push_back(child);
			}
			if (aabb.maxPoint.x >= i.node->center.x && aabb.maxPoint.y >= i.node->center.y)
			{
				TraversalStackItem child;
				child.node = &nodes[i.node->BottomRightChildIndex()];
				stack.push_back(child);
			}
			if (aabb.minPoint.x <= i.node->center.x && aabb.maxPoint.y >= i.node->center.y)
			{
				TraversalStackItem child;
				child.node = &nodes[i.node->BottomLeftChildIndex()];
				stack.push_back(child);
			}
			if (aabb.maxPoint.x >= i.node->center.x && aabb.minPoint.y <= i.node->center.y)
			{
				TraversalStackItem child;
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

	bool operator ()(QuadTree<T> & /*tree*/, const AABB2D &queryAABB, typename QuadTree<T>::Node &node)
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
	MGL_PROFILE(QuadTree_CollidingPairsQuery);
	FindCollidingPairs<T, Func> func;
	func.collisionCallback = &callback;
	AABBQuery(aabb, func);
}

template<typename T>
struct TraversalNode
{
	/// The squared distance of this node to the query point.
	float d;
	typename QuadTree<T>::Node *node;

	struct TriCmp { float operator()(const TraversalNode &a, const TraversalNode &b) { return b.d - a.d; } };
	struct EqualCmp { bool operator()(const TraversalNode &a, const TraversalNode &b) { return b.d == a.d; } };

	/// We compare in reverse order, since we want the node with the smallest distance to be visited first,
	/// and MaxHeap stores the node that compares largest in the root.
	bool operator <(const TraversalNode &t) const { return d > t.d; }
	bool operator ==(const TraversalNode &t) const { return d == t.d; }

	static void Swap(TraversalNode &a, TraversalNode &b)
	{
		float x = a.d;
		a.d = b.d;
		b.d = x;
		typename QuadTree<T>::Node *temp = a.node;
		a.node = b.node;
		b.node = temp;
//		std::swap(a, b);
	}
};

#ifdef MATH_CONTAINERLIB_SUPPORT
template<typename T>
template<typename Func>
inline void QuadTree<T>::NearestNeighborNodes(const float2 &point, Func &leafCallback)
{
	MaxHeap<TraversalNode<T>, typename TraversalNode<T>::TriCmp, typename TraversalNode<T>::EqualCmp > queue;

	{
		TraversalNode<T> &rootNode = queue.BeginInsert();
		rootNode.d = 0.f;
		rootNode.node = Root();
		queue.FinishInsert();
	}

	while(queue.Size() > 0)
	{
		const TraversalNode<T> &t = queue.Front();

		if (t.node->objects.size() > 0)
		{
			bool stopIteration = leafCallback(*this, point, *t.node, t.d);
			if (stopIteration)
				return;
		}

		if (!t.node->IsLeaf())
		{
			typename QuadTree<T>::Node *childNode = &nodes[t.node->childIndex];
			queue.PopFront();

			// Insert top-left child node to the traversal queue.
			{
				TraversalNode<T> &n = queue.BeginInsert();
				n.node = childNode; // t.node->TopLeftChildIndex()
				n.d = n.node->DistanceSq(point);
				queue.FinishInsert();
			}

			// Insert top-right child node to the traversal queue.
			{
				TraversalNode<T> &n = queue.BeginInsert();
				n.node = childNode + 1; // t.node->TopRightChildIndex()
				n.d = n.node->DistanceSq(point);
				queue.FinishInsert();
			}

			// Insert bottom-left child node to the traversal queue.
			{
				TraversalNode<T> &n = queue.BeginInsert();
				n.node = childNode + 2; // t.node->BottomLeftChildIndex()
				n.d = n.node->DistanceSq(point);
				queue.FinishInsert();
			}

			// Insert bottom-right child node to the traversal queue.
			{
				TraversalNode<T> &n = queue.BeginInsert();
				n.node = childNode + 3; // t.node->BottomRightChildIndex()
				n.d = n.node->DistanceSq(point);
				queue.FinishInsert();
			}
		}
		else
			queue.PopFront();
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

//		typename QuadTree<T>::Node *node;

		T *object;

		struct TriCmp { float operator()(const NearestObject &a, const NearestObject &b) { return b.d - a.d; } };
		struct EqualCmp { bool operator()(const NearestObject &a, const NearestObject &b) { return b.d == a.d; } };

		/// We compare in reverse order, since we want the object with the smallest distance to be visited first,
		/// and MaxHeap stores the object that compares largest in the root.
		bool operator <(const NearestObject &t) const { return d > t.d; }
		bool operator ==(const NearestObject &t) const { return d == t.d; }

		static void Swap(NearestObject &a, NearestObject &b)
		{
			float x = a.d;
			a.d = b.d;
			b.d = x;
			T *o = a.object;
			a.object = b.object;
			b.object = o;
		}
	};

	MaxHeap<NearestObject, typename NearestObject::TriCmp, typename NearestObject::EqualCmp> queue;

	int numObjectsOutputted;

#ifdef QUADTREE_VERBOSE_LOGGING
	int numNodesVisited;
#endif

	bool operator ()(QuadTree<T> &tree, const float2 &point, typename QuadTree<T>::Node &leaf, float minDistanceSquared)
	{
#ifdef QUADTREE_VERBOSE_LOGGING
		++numNodesVisited;
#endif
		// Output all points that are closer than the next closest AABB node.
		while(queue.Size() > 0 && queue.Front().d <= minDistanceSquared)
		{
			const NearestObject &nextNearestPoint = queue.Front();
			bool shouldStopIteration = (*objectCallback)(tree, point, 0 /*nextNearestPoint.node*/, nextNearestPoint.d, *nextNearestPoint.object, numObjectsOutputted++);
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
			NearestObject &obj = queue.BeginInsert();
			obj.d = leaf.objects[i].DistanceSq(point);
			obj.object = &leaf.objects[i];
			queue.FinishInsert();
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
	newRoot->center = (boundingAABB.minPoint + boundingAABB.maxPoint) * 0.5f;
	newRoot->radius = boundingAABB.maxPoint - newRoot->center;
	newRoot->childIndex = oldRootNodeIndex;

	Node *n = &nodes[newRoot->TopLeftChildIndex()];
	n->parent = newRoot;
	n->radius = newRoot->radius * 0.5f;
	n->center = newRoot->center - n->radius;

	n = &nodes[newRoot->TopRightChildIndex()];
	n->parent = newRoot;
	n->radius = newRoot->radius * 0.5f;
	n->center.x = newRoot->center.x + n->radius.x;
	n->center.y = newRoot->center.y - n->radius.y;

	n = &nodes[newRoot->BottomLeftChildIndex()];
	n->parent = newRoot;
	n->radius = newRoot->radius * 0.5f;
	n->center.x = newRoot->center.x - n->radius.x;
	n->center.y = newRoot->center.y + n->radius.y;

	n = &nodes[newRoot->BottomRightChildIndex()];
	n->parent = newRoot;
	n->radius = newRoot->radius * 0.5f;
	n->center = newRoot->center + n->radius;

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
	AABB2D aabb = n->ComputeAABB();
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
		for(int i = 0; i < 4; ++i)
		{
			Node *child = &nodes[n->TopLeftChildIndex()+i];
			assert(child->parent == n);

			// Must contain all its child nodes.
			assert(aabb.Contains(child->center));
			assert(aabb.Contains(child->ComputeAABB()));

			DebugSanityCheckNode(child);
		}
	}
#else
	MARK_UNUSED(n);
#endif
}

MATH_END_NAMESPACE
