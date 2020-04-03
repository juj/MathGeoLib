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

	Triangle triangle(float3::zero, float3::unitX, float3::unitY);

	tree.AddObjects(&triangle, 1);
	tree.Build();

	// Query
	AABB bbox(float3::one * -10, float3::one * 10);

	bool intersected = false;
	auto callback = [&](const KdTree<Triangle>& tree, const KdTreeNode& leaf, const AABB& aabb)
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
