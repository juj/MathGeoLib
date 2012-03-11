#pragma once

template<typename T>
void QuadTree<T>::Clear(const float2 &minXY, const float2 &maxXY)
{
//	FreeBuckets();
	nodes.clear();

	boundingAABB.minPoint = minXY;
	boundingAABB.maxPoint = maxXY;

	int rootNodeGroup = AllocateNodeGroup(0);
	Node *root = Root();
	assert(root);
}

template<typename T>
void QuadTree<T>::Add(const T &object)
{
	Node *n = Root();
	assert(n);

	AABB2D aabb = boundingAABB;
	assert(Contains(aabb, object));

//	u32 objectId = (u32)objects.size();
//	objects.push_back(object);

	Add(object, n, aabb);
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
				n = &nodes[n->TopLeftChildIndex()];
			}
			else
			{
				aabb.minPoint.y = halfY;
				n = &nodes[n->BottomLeftChildIndex()];
			}
		}
		else
		{
			aabb.minPoint.x = halfX;
			if (top)
			{
				aabb.maxPoint.y = halfY;
				n = &nodes[n->TopRightChildIndex()];
			}
			else
			{
				aabb.minPoint.y = halfY;
				n = &nodes[n->BottomRightChildIndex()];
			}
		}
	}
}

/*
template<typename T>
u32 *QuadTree<T>::Bucket(int bucketIndex)
{
	return buckets[bucketIndex];
}

template<typename T>
const u32 *QuadTree<T>::Bucket(int bucketIndex) const
{
	return buckets[bucketIndex];
}

template<typename T>
T &QuadTree<T>::Object(int objectIndex)
{
	return objects[objectIndex];
}

template<typename T>
const T &QuadTree<T>::Object(int objectIndex) const
{
	return objects[objectIndex];
}
*/
template<typename T>
typename QuadTree<T>::Node *QuadTree<T>::Root()
{
	return nodes.size() > 0 ? &nodes[1] : 0;
}

template<typename T>
const typename QuadTree<T>::Node *QuadTree<T>::Root() const
{
	return nodes.size() > 0 ? &nodes[1] : 0;
}

template<typename T>
int QuadTree<T>::AllocateNodeGroup(Node *parent)
{
	int index = nodes.size();
	Node n;
	n.parent = parent;
	n.childIndex = 0;
	nodes.push_back(n);
	nodes.push_back(n);
	nodes.push_back(n);
	nodes.push_back(n);
	return index;
}
/*
template<typename T>
void QuadTree<T>::FreeBuckets()
{
	for(size_t i = 0; i < buckets.size(); ++i)
		delete[] buckets[i];
	buckets.clear();
}
*/

template<typename T>
void QuadTree<T>::SplitLeaf(Node *leaf, const AABB2D &leafAABB)
{
	assert(leaf->IsLeaf());
	assert(leaf->childIndex == 0);

	leaf->childIndex = AllocateNodeGroup(leaf);

	float halfX = (leafAABB.minPoint.x + leafAABB.maxPoint.x) * 0.5f;
	float halfY = (leafAABB.minPoint.y + leafAABB.maxPoint.y) * 0.5f;

	size_t i = 0;
	while(i < leaf->objects.size())
	{
//		u32 objectId = leaf->bucket[i];
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
