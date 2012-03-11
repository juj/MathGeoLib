#pragma once

#include "Math/float2.h"
#include "Types.h"

#ifdef MATH_CONTAINERLIB_SUPPORT
#include "Container/MaxHeap.h"
#endif

struct AABB2D
{
	AABB2D(){}
	AABB2D(const float2 &minPt, const float2 &maxPt)
	:minPoint(minPt),
	maxPoint(maxPt)
	{
	}

	float2 minPoint;
	float2 maxPoint;

	float Width() const { return maxPoint.x - minPoint.x; }
	float Height() const { return maxPoint.y - minPoint.y; }

	float DistanceSq(const float2 &pt) const
	{
		float2 cp = pt.Clamp(minPoint, maxPoint);
		return cp.DistanceSq(pt);
	}

	bool Intersects(const AABB2D &rhs) const
	{
		return maxPoint.x >= rhs.minPoint.x &&
		       maxPoint.y >= rhs.minPoint.y &&
		       rhs.maxPoint.x >= minPoint.x &&
		       rhs.maxPoint.y >= minPoint.y;
	}

	AABB2D operator +(const float2 &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint + pt;
		a.maxPoint = maxPoint + pt;
		return a;
	}

	AABB2D operator -(const float2 &pt) const
	{
		AABB2D a;
		a.minPoint = minPoint - pt;
		a.maxPoint = maxPoint - pt;
		return a;
	}

#ifdef MATH_ENABLE_STL_SUPPORT
	std::string ToString() const
	{
		char str[256];
		sprintf(str, "AABB2D(Min:(%.2f, %.2f) Max:(%.2f, %.2f))", minPoint.x, minPoint.y, maxPoint.x, maxPoint.y);
		return str;
	}
#endif
};

static const float minQuadrantSize = 0.1f;

inline float MinX(const float3 &pt) { return pt.x; }
inline float MaxX(const float3 &pt) { return pt.x; }
inline float MinY(const float3 &pt) { return pt.y; }
inline float MaxY(const float3 &pt) { return pt.y; }

inline bool Contains(const AABB2D &aabb, const float3 &pt)
{
	return aabb.minPoint.x <= pt.x &&
	       aabb.minPoint.y <= pt.y &&
	       pt.x <= aabb.maxPoint.x &&
	       pt.y <= aabb.maxPoint.y;
}

template<typename T>
class QuadTree
{
public:
	struct Node
	{
		Node *parent;
		u32 childIndex;
		std::vector<T> objects;

		bool IsLeaf() const { return childIndex == 0; }

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
	};

	struct TraversalStackItem
	{
		AABB2D aabb;
		Node *node;
	};

	QuadTree() { nodes.reserve(200000); }

	void Clear(const float2 &minXY, const float2 &maxXY);

	void Add(const T &object);

	/// Removes the given object from this tree.
	/// To call this function, you must define a function QuadTree<T>::Node *GetQuadTreeNode(const T &object)
	/// which returns the node of this quadtree where the object resides in.
	void Remove(const T &object);
	
	AABB2D BoundingAABB() const { return boundingAABB; }

	AABB2D ComputeAABB(const Node *node) const
	{
		if (!node->parent)
			return BoundingAABB();
		AABB2D aabb = ComputeAABB(node->parent);
		float halfX = (aabb.minPoint.x + aabb.maxPoint.x) * 0.5f;
		float halfY = (aabb.minPoint.y + aabb.maxPoint.y) * 0.5f;
		u32 parentChildIndex = node->parent->childIndex;
		const Node *parentChildBase = &nodes[parentChildIndex];
		int quadrant = node - parentChildBase;
		switch(quadrant)
		{
		case 0: 
			aabb.maxPoint.x = halfX;
			aabb.maxPoint.y = halfY;
			return aabb;
		case 1:
			aabb.minPoint.x = halfX;
			aabb.maxPoint.y = halfY;
			return aabb;
		case 2:
			aabb.maxPoint.x = halfX;
			aabb.minPoint.y = halfY;
			return aabb;
		case 3:
			aabb.minPoint.x = halfX;
			aabb.minPoint.y = halfY;
			return aabb;
		default:
			assert(false);
			return aabb; // Dummy to hide compiler warning.
		}
	}

	/// Returns an object bucket by the given bucket index.
	/// An object bucket is a contiguous C array of object indices, terminated with a sentinel value BUCKET_SENTINEL.
	/// To fetch the actual object based on an object index, call the Object() method.
//	u32 *Bucket(int bucketIndex);
//	const u32 *Bucket(int bucketIndex) const;

	/// Returns an object by the given object index.
//	T &Object(int objectIndex);
//	const T &Object(int objectIndex) const;

	Node *Root();
	const Node *Root() const;

//	int NumObjects() const { return objects.size(); }

	/// Performs an AABB intersection query in this Quadtreee, and calls the given callback function for each non-empty
	/// node of the tree which intersects the given AABB.
	/// @param callback A function or a function object of prototype
	///    bool callbackFunction(QuadTree<T> &tree, const AABB2D &queryAABB, QuadTree<T>::Node &node, const AABB2D &nodeAABB);
	///    If the callback function returns true, the execution of the query is stopped and this function immediately
	///    returns afterwards. If the callback function returns false, the execution of the query continues.
	template<typename Func>
	inline void AABBQuery(const AABB2D &aabb, Func &callback);

	/// Finds all object pairs inside the given AABB which have colliding AABBs. For each such pair, calls the
	/// specified callback function.
	template<typename Func>
	inline void CollidingPairsQuery(const AABB2D &aabb, Func &callback);

#ifdef MATH_CONTAINERLIB_SUPPORT
	/// Performs a nearest neighbor search on this QuadTree.
	/// @param leafCallback A function or a function object of prototype
	///    bool LeafCallbackFunction(QuadTree<T> &tree, const float2 &point, Node &leaf, const AABB2D &aabb, float minDistanceSquared);
	///    If the callback function returns true, the execution of the query is stopped and this function immediately
	///    returns afterwards. If the callback function returns false, the execution of the query continues.
	///	   minDistance is the minimum distance the objects in this leaf (and all future leaves to be passed to the
	///    callback) have to the point that is being queried.
	template<typename Func>
	inline void NearestObjects(const float2 &point, Func &leafCallback);
#endif

private:
	void Add(const T &object, Node *n, AABB2D aabb);

	/// Allocates a sequential 4-tuple of QuadtreeNodes, contiguous in memory.
	int AllocateNodeGroup(Node *parent);

	void SplitLeaf(Node *leaf, const AABB2D &leafAABB);
//	void FreeBuckets();

	std::vector<Node> nodes;
//	std::vector<T> objects;

	AABB2D boundingAABB;
};

#include "QuadTree.inl"
