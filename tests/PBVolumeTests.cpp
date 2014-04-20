#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "../src/Geometry/PBVolume.h"
#include "TestRunner.h"
#include "ObjectGenerators.h"

MATH_IGNORE_UNUSED_VARS_WARNING

RANDOMIZED_TEST(AABBPBVolumeIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Frustum b = RandomFrustumContainingPoint(pt);
	AABB a = RandomAABBContainingPoint(pt, 10.f);
	assert(a.Intersects(b));
	assert(b.Intersects(a));

	bool contained = b.Contains(a);

	assert(b.Contains(b.CenterPoint()));

	PBVolume<6> pbVolume = ToPBVolume(b);
	assert(pbVolume.Contains(b.CenterPoint()));
	CullTestResult r = pbVolume.InsideOrIntersects(a);
	assert(r == TestInside || r == TestNotContained);
	if (contained)
		assert(r == TestInside);
}

RANDOMIZED_TEST(AABBPBVolumeNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	AABB a = RandomAABBInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));

	PBVolume<6> pbVolume = ToPBVolume(b);
	assert(pbVolume.Contains(b.CenterPoint()));
	CullTestResult r = pbVolume.InsideOrIntersects(a);
	assert(r == TestOutside || r == TestNotContained);
}

RANDOMIZED_TEST(SpherePBVolumeIntersect)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere a = RandomSphereContainingPoint(pt, 10.f);
	Frustum b = RandomFrustumContainingPoint(pt);
	assert(a.Intersects(b));
	assert(b.Intersects(a));

//	bool contained = b.Contains(a);

	assert(b.Contains(b.CenterPoint()));

	PBVolume<6> pbVolume = ToPBVolume(b);
	assert(pbVolume.Contains(b.CenterPoint()));
	CullTestResult r = pbVolume.InsideOrIntersects(a);
	assert(r == TestInside || r == TestNotContained);
//	if (contained)
//		assert(r == TestInside);
}

RANDOMIZED_TEST(SpherePBVolumeNoIntersect)
{
	Plane p(vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE)), vec::RandomDir(rng));
	Sphere a = RandomSphereInHalfspace(p, 10.f);
	p.ReverseNormal();
	Frustum b = RandomFrustumInHalfspace(p);
	assert2(!a.Intersects(b), a, b);
	assert(!b.Intersects(a));

	PBVolume<6> pbVolume = ToPBVolume(b);
	assert(pbVolume.Contains(b.CenterPoint()));
	CullTestResult r = pbVolume.InsideOrIntersects(a);
	assert(r == TestOutside || r == TestNotContained);
}
