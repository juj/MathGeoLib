#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"
#include "../tests/TestData.h"

using namespace TestData;

// Test that doing an AABBQuery into a KdTree with a single triangle works out ok.
UNIQUE_TEST(KdTree_SingleTriangle_AABBQuery)
{
	// Build the tree
	KdTree<Triangle> tree;

	Triangle triangle(POINT_VEC_SCALAR(0.f), POINT_VEC(1.f, 0.f, 0.f), POINT_VEC(0.f, 1.f, 0.f));

	tree.AddObjects(&triangle, 1);
	tree.Build();

	// Query
	AABB bbox(POINT_VEC_SCALAR(-10.f), POINT_VEC_SCALAR(10.f));

	bool intersected = false;
	auto callback = [&](const KdTree<Triangle>& tree, const KdTreeNode& leaf, const AABB& /*aabb*/)
	{
		auto bucket = tree.Bucket(leaf.bucketIndex);
		auto triangle = tree.Object(*bucket);
		
		// ... do stuff with triangle ...
		intersected = true;
		return false;
	};
	tree.AABBQuery(bbox, callback);
	assert(intersected);
}
