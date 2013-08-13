#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

Frustum GenIdFrustum(FrustumHandedness h, FrustumProjectiveSpace p)
{
	Frustum f;
	f.type = PerspectiveFrustum;
	f.handedness = h;
	f.projectiveSpace = p;
	f.pos = float3::zero;
	f.front = -float3::unitZ; // In right-handed convention, local view space looks towards -Z.
	f.up = float3::unitY;
	f.nearPlaneDistance = 1.f;
	f.farPlaneDistance = 100.f;
	f.horizontalFov = pi/2.f;
	f.verticalFov = pi/2.f;

	return f;
}

#define FOR_EACH_FRUSTUM_CONVENTION(f) \
	for(int ii = 0; ii < 4; ++ii) { \
		switch(ii) { \
		case 0: f = GenIdFrustum(FrustumLeftHanded, FrustumSpaceGL); break; \
		case 1: f = GenIdFrustum(FrustumRightHanded, FrustumSpaceGL); break; \
		case 2: f = GenIdFrustum(FrustumLeftHanded, FrustumSpaceD3D); break; \
		case 3: f = GenIdFrustum(FrustumRightHanded, FrustumSpaceD3D); break; \
		} \
		
UNIQUE_TEST(Frustum_AspectRatio)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		assert(EqualAbs(f.AspectRatio(), 1.f));
	}
}

UNIQUE_TEST(Frustum_WorldRight)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		if (f.handedness == FrustumRightHanded)
			assert(f.WorldRight().Equals(float3::unitX));
		else // In the test func, all cameras look down to -Z, so left-handed cameras need to point their right towards -X then.
			assert(f.WorldRight().Equals(-float3::unitX));
	}
}

UNIQUE_TEST(Frustum_Chirality)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		assert(f.WorldMatrix().Determinant() > 0.f);
		assert(f.ViewMatrix().Determinant() > 0.f);
		if (f.handedness == FrustumLeftHanded)
			assert(f.ProjectionMatrix().Determinant4() > 0.f); // left-handed view -> projection space transform does not change handedness.
		else
			assert(f.ProjectionMatrix().Determinant4() < 0.f); // but right-handed transform should.
	}
}

UNIQUE_TEST(Frustum_Planes)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		assert(f.NearPlane().normal.Equals(float3::unitZ));
		assert(EqualAbs(f.NearPlane().d, -1.f));

		assert(f.FarPlane().normal.Equals(-float3::unitZ));
		assert(EqualAbs(f.FarPlane().d, 100.f));
	}
}

UNIQUE_TEST(Frustum_Corners)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)

		// Corner points are returned in XYZ order: 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++
		if (f.handedness == FrustumLeftHanded)
		{
			assert(f.CornerPoint(0).Equals(1.f, -1.f, -1.f));
			assert(f.CornerPoint(1).Equals(100.f, -100.f, -100.f));
			assert(f.CornerPoint(2).Equals(1.f, 1, -1.f));
			assert(f.CornerPoint(3).Equals(100.f, 100.f, -100.f));
			assert(f.CornerPoint(4).Equals(-1.f, -1.f, -1.f));
			assert(f.CornerPoint(5).Equals(-100.f, -100.f, -100.f));
			assert(f.CornerPoint(6).Equals(-1.f, 1.f, -1.f));
			assert(f.CornerPoint(7).Equals(-100.f, 100.f, -100.f));
		}
		else
		{
			assert(f.CornerPoint(0).Equals(-1.f, -1.f, -1.f));
			assert(f.CornerPoint(1).Equals(-100.f, -100.f, -100.f));
			assert(f.CornerPoint(2).Equals(-1.f, 1, -1.f));
			assert(f.CornerPoint(3).Equals(-100.f, 100.f, -100.f));
			assert(f.CornerPoint(4).Equals(1.f, -1.f, -1.f));
			assert(f.CornerPoint(5).Equals(100.f, -100.f, -100.f));
			assert(f.CornerPoint(6).Equals(1.f, 1.f, -1.f));
			assert(f.CornerPoint(7).Equals(100.f, 100.f, -100.f));
		}
	}
}

UNIQUE_TEST(Frustum_Contains)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		for(int i = 0; i < 8; ++i)
			assert4(f.Contains(f.CornerPoint(i)), i, f.CornerPoint(i), f, f.Distance(f.CornerPoint(i)));

		assert3(f.Contains(f.CenterPoint()), f, f.CenterPoint(), f.Distance(f.CenterPoint()));
	}
}

UNIQUE_TEST(Frustum_Matrices)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		if (f.handedness == FrustumRightHanded)
		{
			float3x4 wm = f.WorldMatrix();
			assert(wm.IsIdentity());

			float3x4 vm = f.ViewMatrix();
			assert(vm.IsIdentity());
		}
		else
		{
			float3x4 wm = f.WorldMatrix() * float3x4::RotateY(pi);
			assert(wm.IsIdentity());

			float3x4 vm = f.ViewMatrix() * float3x4::RotateY(pi);
			assert(vm.IsIdentity());
		}
	}
}

UNIQUE_TEST(Frustum_Projection)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		const float nearD = (f.projectiveSpace == FrustumSpaceD3D) ? 0.f : -1.f;

		// Corner points are returned in XYZ order: 0: ---, 1: --+, 2: -+-, 3: -++, 4: +--, 5: +-+, 6: ++-, 7: +++
		assert(f.Project(f.CornerPoint(0)).Equals(-1, -1, nearD));
		assert(f.Project(f.CornerPoint(1)).Equals(-1, -1,  1));
		assert(f.Project(f.CornerPoint(2)).Equals(-1,  1, nearD));
		assert(f.Project(f.CornerPoint(3)).Equals(-1,  1,  1));
		assert(f.Project(f.CornerPoint(4)).Equals( 1, -1, nearD));
		assert(f.Project(f.CornerPoint(5)).Equals( 1, -1,  1));
		assert(f.Project(f.CornerPoint(6)).Equals( 1,  1, nearD));
		assert(f.Project(f.CornerPoint(7)).Equals( 1,  1,  1));
	}
}

UNIQUE_TEST(Frustum_UnProject)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
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
}

UNIQUE_TEST(Frustum_UnProjectFromNearPlane)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
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
}

UNIQUE_TEST(Frustum_UnProjectLineSegment)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
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
}

UNIQUE_TEST(Frustum_NearPlanePos)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		if (f.handedness == FrustumLeftHanded)
		{
			assert(f.NearPlanePos(1,-1).Equals(-1,-1,-1));
			assert(f.NearPlanePos(-1,1).Equals(1,1,-1));
		}
		else
		{
			assert(f.NearPlanePos(-1,-1).Equals(-1,-1,-1));
			assert(f.NearPlanePos(1,1).Equals(1,1,-1));
		}
		assert(f.NearPlanePos(0,0).Equals(0,0,-1));
	}
}

UNIQUE_TEST(Frustum_FarPlanePos)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		if (f.handedness == FrustumLeftHanded)
		{
			assert(f.FarPlanePos(1,-1).Equals(-100,-100,-100));
			assert(f.FarPlanePos(-1,1).Equals(100,100,-100));
		}
		else
		{
			assert(f.FarPlanePos(-1,-1).Equals(-100,-100,-100));
			assert(f.FarPlanePos(1,1).Equals(100,100,-100));
		}
		assert(f.FarPlanePos(0,0).Equals(0,0,-100));
	}
}

UNIQUE_TEST(Frustum_Finite)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		assert(f.IsFinite());
	}
}

UNIQUE_TEST(Frustum_MinimalEnclosingAABB)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		AABB a = f.MinimalEnclosingAABB();
		assert(a.Contains(f));
	}
}

UNIQUE_TEST(Frustum_MinimalEnclosingOBB)
{
	Frustum f;
	FOR_EACH_FRUSTUM_CONVENTION(f)
		OBB o = f.MinimalEnclosingOBB();
		if (!o.Contains(f))
		{
			LOGE("OBB: %s", o.ToString().c_str());
			LOGE("Frustum: %s", f.ToString().c_str());
		}
		assert(o.Contains(f));
	}
}

UNIQUE_TEST(Frustum_AspectRatio_NearPlanePos)
{
	Frustum f;
	f.type = PerspectiveFrustum;
	f.handedness = FrustumRightHanded;
	f.projectiveSpace = FrustumSpaceGL;
	f.pos = float3::zero;
	f.front = -float3::unitZ;
	f.up = float3::unitY;
	f.nearPlaneDistance = 0.5f;
	f.farPlaneDistance = 10.f;
	f.horizontalFov = DegToRad(140.f);
	f.verticalFov = DegToRad(30.f);

	assert(EqualAbs(f.NearPlaneWidth(), 0.5f*Tan(DegToRad(140.f)/2.f)*2.f));
	assert(EqualAbs(f.NearPlaneHeight(), 0.5f*Tan(DegToRad(30.f)/2.f)*2.f));

	float aspect = f.NearPlaneWidth() / f.NearPlaneHeight();
	assert(EqualAbs(aspect, f.AspectRatio()));
}
