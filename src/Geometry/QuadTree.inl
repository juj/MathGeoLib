#pragma once

template<typename T>
void QuadTree<T>::Clear(const float2 &minXY, const float2 &maxXY)
{
	nodes.clear();

	boundingAABB.minPoint = minXY;
	boundingAABB.maxPoint = maxXY;

	rootNodeIndex = AllocateNodeGroup(0);
	assert(Root());
}

template<typename T>
void QuadTree<T>::Add(const T &object)
{
	PROFILE(QuadTree_Add);
	Node *n = Root();
	assert(n);

	assert(boundingAABB.IsFinite());
	assert(!boundingAABB.IsDegenerate());

	AABB2D objectAABB = GetAABB2D(object);
	assert(objectAABB.IsFinite());
	assert(!objectAABB.IsDegenerate());
	
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
		n->Remove(object);
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
			if (n->IsLeaf() && n->objects.size() > 16 && aabb.Width() >= minQuadrantSize && aabb.Height() >= minQuadrantSize)
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
	return nodes.size() > 0 ? &nodes[rootNodeIndex] : 0;
}

template<typename T>
const typename QuadTree<T>::Node *QuadTree<T>::Root() const
{
	return nodes.size() > 0 ? &nodes[rootNodeIndex] : 0;
}

template<typename T>
int QuadTree<T>::AllocateNodeGroup(Node *parent)
{
#ifdef _DEBUG
	int oldCap = nodes.capacity();
#endif
	int index = nodes.size();
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

	while(stack.size() > 0)
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

	bool operator ()(QuadTree<T> &tree, const AABB2D queryAABB, typename QuadTree<T>::Node &node, const AABB2D &nodeAABB)
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

#ifdef MATH_CONTAINERLIB_SUPPORT
template<typename T>
template<typename Func>
inline void QuadTree<T>::NearestObjects(const float2 &point, Func &leafCallback)
{
	struct TraversalNode
	{
		float d;
		AABB2D aabb;
		Node *node;

		bool operator <(const TraversalNode &t) const { return d > t.d; }
	};

	MaxHeap<TraversalNode> queue;
	TraversalNode t;
	t.d = 0.f;
	t.aabb = BoundingAABB();
	t.node = Root();
	queue.Insert(t);

	while(queue.Size() > 0)
	{
		t = queue.Front();
		queue.PopFront();

		if (t.node->bucket.size() > 0)
		{
			bool stopIteration = leafCallback(*this, point, *t.node, t.aabb, t.d);
			if (stopIteration)
				return;
		}
		else if (!t.node->IsLeaf())
		{
			TraversalNode n;

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
#endif

template<typename T>
void QuadTree<T>::GrowRootTopLeft()
{
	boundingAABB.minPoint.x -= boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.minPoint.y -= boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// rootNodeIndex always points to the first index of the four quadrants.
	// The old root will become the bottom-right child of the new root, at index 3. Swap the root node to its proper place.
	Swap(nodes[rootNodeIndex], nodes[rootNodeIndex+3]);
	Node *oldRoot = &nodes[rootNodeIndex+3];

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

	int oldRootNodeIndex = rootNodeIndex;
	rootNodeIndex = AllocateNodeGroup(0);
	Node *newRoot = &nodes[rootNodeIndex];
	newRoot->childIndex = oldRootNodeIndex;
	nodes[newRoot->TopLeftChildIndex()].parent = newRoot;
	nodes[newRoot->TopRightChildIndex()].parent = newRoot;
	nodes[newRoot->BottomLeftChildIndex()].parent = newRoot;
	nodes[newRoot->BottomRightChildIndex()].parent = newRoot;

	LOGI("TopLeft: New Tree Size %s. %d total nodes. %d inner nodes. %d leaves. %d tree height.", 
		boundingAABB.ToString().c_str(), NumNodes(), NumInnerNodes(), NumLeaves(), TreeHeight());
	DebugSanityCheckNode(Root());
}

template<typename T>
void QuadTree<T>::GrowRootTopRight()
{
	boundingAABB.maxPoint.x += boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.minPoint.y -= boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// rootNodeIndex always points to the first index of the four quadrants.
	// The old root will become the bottom-left child of the new root, at index 2. Swap the root node to its proper place.
	Swap(nodes[rootNodeIndex], nodes[rootNodeIndex+2]);
	Node *oldRoot = &nodes[rootNodeIndex+2];

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

	int oldRootNodeIndex = rootNodeIndex;
	rootNodeIndex = AllocateNodeGroup(0);
	Node *newRoot = &nodes[rootNodeIndex];
	newRoot->childIndex = oldRootNodeIndex;
	nodes[newRoot->TopLeftChildIndex()].parent = newRoot;
	nodes[newRoot->TopRightChildIndex()].parent = newRoot;
	nodes[newRoot->BottomLeftChildIndex()].parent = newRoot;
	nodes[newRoot->BottomRightChildIndex()].parent = newRoot;

	LOGI("TopRight: New Tree Size %s. %d total nodes. %d inner nodes. %d leaves. %d tree height.", 
		boundingAABB.ToString().c_str(), NumNodes(), NumInnerNodes(), NumLeaves(), TreeHeight());
	DebugSanityCheckNode(Root());
}

template<typename T>
void QuadTree<T>::GrowRootBottomLeft()
{
	boundingAABB.minPoint.x -= boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.maxPoint.y += boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// rootNodeIndex always points to the first index of the four quadrants.
	// The old root will become the top-right child of the new root, at index 1. Swap the root node to its proper place.
	Swap(nodes[rootNodeIndex], nodes[rootNodeIndex+1]);
	Node *oldRoot = &nodes[rootNodeIndex+1];

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

	int oldRootNodeIndex = rootNodeIndex;
	rootNodeIndex = AllocateNodeGroup(0);
	Node *newRoot = &nodes[rootNodeIndex];
	newRoot->childIndex = oldRootNodeIndex;
	nodes[newRoot->TopLeftChildIndex()].parent = newRoot;
	nodes[newRoot->TopRightChildIndex()].parent = newRoot;
	nodes[newRoot->BottomLeftChildIndex()].parent = newRoot;
	nodes[newRoot->BottomRightChildIndex()].parent = newRoot;

	LOGI("BottomLeft: New Tree Size %s. %d total nodes. %d inner nodes. %d leaves. %d tree height.", 
		boundingAABB.ToString().c_str(), NumNodes(), NumInnerNodes(), NumLeaves(), TreeHeight());
	DebugSanityCheckNode(Root());
}

template<typename T>
void QuadTree<T>::GrowRootBottomRight()
{
	boundingAABB.maxPoint.x += boundingAABB.maxPoint.x - boundingAABB.minPoint.x;
	boundingAABB.maxPoint.y += boundingAABB.maxPoint.y - boundingAABB.minPoint.y;

	// rootNodeIndex always points to the first index of the four quadrants.
	// The old root will become the top-left child of the new root, at index 0, so no swapping is necessary.

	int oldRootNodeIndex = rootNodeIndex;
	rootNodeIndex = AllocateNodeGroup(0);
	Node *newRoot = &nodes[rootNodeIndex];
	newRoot->childIndex = oldRootNodeIndex;
	nodes[newRoot->TopLeftChildIndex()].parent = newRoot;
	nodes[newRoot->TopRightChildIndex()].parent = newRoot;
	nodes[newRoot->BottomLeftChildIndex()].parent = newRoot;
	nodes[newRoot->BottomRightChildIndex()].parent = newRoot;

	LOGI("BottomRight: New Tree Size %s. %d total nodes. %d inner nodes. %d leaves. %d tree height.", 
		boundingAABB.ToString().c_str(), NumNodes(), NumInnerNodes(), NumLeaves(), TreeHeight());
	DebugSanityCheckNode(Root());
}

/// Returns the total number of nodes (all nodes, i.e. inner nodes + leaves) in the tree.
template<typename T>
int QuadTree<T>::NumNodes() const
{
	return std::max<int>(0, nodes.size() - 3); // The nodes rootNodeIndex+1, rootNodeIndex+2 and rootNodeIndex+3 are dummy unused, since the root node is not a quadrant.
}

/// Returns the total number of leaf nodes in the tree.
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

/// Returns the total number of inner nodes in the tree.
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
#endif
}
