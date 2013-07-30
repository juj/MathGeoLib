#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

Frustum GenIdFrustum_GL_RH()
{
	Frustum f;
	f.type = PerspectiveFrustum;
	f.handedness = FrustumRightHanded;
	f.projectiveSpace = FrustumSpaceGL;
	f.pos = float3::zero;
	f.front = -float3::unitZ;
	f.up = float3::unitY;
	f.nearPlaneDistance = 1.f;
	f.farPlaneDistance = 100.f;
	f.horizontalFov = pi/2.f;
	f.verticalFov = pi/2.f;

	return f;
}

UNIQUE_TEST(Frustum_AspectRatio)
{
	Frustum f = GenIdFrustum_GL_RH();
	asserteq(f.AspectRatio(), 1.f);
}

UNIQUE_TEST(Frustum_WorldRight)
{
	Frustum f = GenIdFrustum_GL_RH();
	assert(f.WorldRight().Equals(float3::unitX));
}

UNIQUE_TEST(Frustum_Chirality)
{
	Frustum f = GenIdFrustum_GL_RH();
	assert(f.WorldMatrix().Determinant() > 0.f);
	assert(f.ViewMatrix().Determinant() > 0.f);
	assert(f.ProjectionMatrix().Determinant4() < 0.f); // In OpenGL, the view -> projection space transform changes handedness.
}

UNIQUE_TEST(Frustum_Planes)
{
	Frustum f = GenIdFrustum_GL_RH();
	assert(f.NearPlane().normal.Equals(float3::unitZ));
	asserteq(f.NearPlane().d, -1.f);

	assert(f.FarPlane().normal.Equals(-float3::unitZ));
	asserteq(f.FarPlane().d, 100.f);
}

UNIQUE_TEST(Frustum_Corners)
{
	Frustum f = GenIdFrustum_GL_RH();

	// Corner points are returned in XYZ order: 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++
	assert(f.CornerPoint(0).Equals(-1.f, -1.f, -1.f));
	assert(f.CornerPoint(1).Equals(-100.f, -100.f, -100.f));
	assert(f.CornerPoint(2).Equals(-1.f, 1, -1.f));
	assert(f.CornerPoint(3).Equals(-100.f, 100.f, -100.f));
	assert(f.CornerPoint(4).Equals(1.f, -1.f, -1.f));
	assert(f.CornerPoint(5).Equals(100.f, -100.f, -100.f));
	assert(f.CornerPoint(6).Equals(1.f, 1.f, -1.f));
	assert(f.CornerPoint(7).Equals(100.f, 100.f, -100.f));
}

UNIQUE_TEST(Frustum_Contains)
{
	Frustum f = GenIdFrustum_GL_RH();

	for(int i = 0; i < 8; ++i)
		assert(f.Contains(f.CornerPoint(i)));

	assert(f.Contains(f.CenterPoint()));
}

UNIQUE_TEST(Frustum_Matrices)
{
	Frustum f = GenIdFrustum_GL_RH();

	float3x4 wm = f.WorldMatrix();
	assert(wm.IsIdentity());

	float3x4 vm = f.ViewMatrix();
	assert(vm.IsIdentity());
}

UNIQUE_TEST(Frustum_Projection)
{
	Frustum f = GenIdFrustum_GL_RH();

	// Corner points are returned in XYZ order: 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++
	assert(f.Project(f.CornerPoint(0)).Equals(-1, -1, -1));
	assert(f.Project(f.CornerPoint(1)).Equals(-1, -1,  1));
	assert(f.Project(f.CornerPoint(2)).Equals(-1,  1, -1));
	assert(f.Project(f.CornerPoint(3)).Equals(-1,  1,  1));
	assert(f.Project(f.CornerPoint(4)).Equals( 1, -1, -1));
	assert(f.Project(f.CornerPoint(5)).Equals( 1, -1,  1));
	assert(f.Project(f.CornerPoint(6)).Equals( 1,  1, -1));
	assert(f.Project(f.CornerPoint(7)).Equals( 1,  1,  1));
}

UNIQUE_TEST(Frustum_UnProject)
{
	Frustum f = GenIdFrustum_GL_RH();

	Ray r = f.UnProject(0, 0);
	assert(r.pos.Equals(f.pos));
	assert(r.pos.Equals(0,0,0));
	assert(r.dir.Equals(0,0,-1));

	r = f.UnProject(-1, -1);
	assert(r.pos.Equals(f.pos));
	assert(r.pos.Equals(0,0,0));
	assert(r.dir.Equals((f.CornerPoint(1)-f.CornerPoint(0)).Normalized()));

	r = f.UnProject(1, 1);
	assert(r.pos.Equals(f.pos));
	assert(r.pos.Equals(0,0,0));
	assert(r.dir.Equals((f.CornerPoint(7)-f.CornerPoint(6)).Normalized()));
}

UNIQUE_TEST(Frustum_UnProjectFromNearPlane)
{
	Frustum f = GenIdFrustum_GL_RH();

	Ray r = f.UnProjectFromNearPlane(0, 0);
	assert(r.pos.Equals(0,0,-1));
	assert(r.dir.Equals(0,0,-1));

	r = f.UnProjectFromNearPlane(-1, -1);
	assert(r.pos.Equals(f.CornerPoint(0)));
	assert(r.dir.Equals((f.CornerPoint(1)-f.CornerPoint(0)).Normalized()));

	r = f.UnProjectFromNearPlane(1, 1);
	assert(r.pos.Equals(f.CornerPoint(6)));
	assert(r.dir.Equals((f.CornerPoint(7)-f.CornerPoint(6)).Normalized()));
}

UNIQUE_TEST(Frustum_UnProjectLineSegment)
{
	Frustum f = GenIdFrustum_GL_RH();

	LineSegment ls = f.UnProjectLineSegment(0, 0);
	assert(ls.a.Equals(0,0,-1));
	assert(ls.b.Equals(0,0,-100));

	ls = f.UnProjectLineSegment(-1, -1);
	assert(ls.a.Equals(f.CornerPoint(0)));
	assert(ls.b.Equals(f.CornerPoint(1)));

	ls = f.UnProjectLineSegment(1, 1);
	assert(ls.a.Equals(f.CornerPoint(6)));
	assert(ls.b.Equals(f.CornerPoint(7)));
}

UNIQUE_TEST(Frustum_NearPlanePos)
{
	Frustum f = GenIdFrustum_GL_RH();

	assert(f.NearPlanePos(-1,-1).Equals(-1,-1,-1));
	assert(f.NearPlanePos(0,0).Equals(0,0,-1));
	assert(f.NearPlanePos(1,1).Equals(1,1,-1));
}

UNIQUE_TEST(Frustum_FarPlanePos)
{
	Frustum f = GenIdFrustum_GL_RH();

	assert(f.FarPlanePos(-1,-1).Equals(-100,-100,-100));
	assert(f.FarPlanePos(0,0).Equals(0,0,-100));
	assert(f.FarPlanePos(1,1).Equals(100,100,-100));
}

UNIQUE_TEST(Frustum_Finite)
{
	Frustum f = GenIdFrustum_GL_RH();
	assert(f.IsFinite());
}

UNIQUE_TEST(Frustum_MinimalEnclosingAABB)
{
	Frustum f = GenIdFrustum_GL_RH();
	AABB a = f.MinimalEnclosingAABB();
	assert(a.Contains(f));
}

UNIQUE_TEST(Frustum_MinimalEnclosingOBB)
{
	Frustum f = GenIdFrustum_GL_RH();
	OBB o = f.MinimalEnclosingOBB();
	assert(o.Contains(f));
}
