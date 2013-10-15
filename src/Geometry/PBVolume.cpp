#include "PBVolume.h"
#include "Frustum.h"

MATH_BEGIN_NAMESPACE

PBVolume<6> ToPBVolume(const Frustum &frustum)
{
	PBVolume<6> frustumVolume;
	frustumVolume.p[0] = frustum.NearPlane();
	frustumVolume.p[1] = frustum.LeftPlane();
	frustumVolume.p[2] = frustum.RightPlane();
	frustumVolume.p[3] = frustum.TopPlane();
	frustumVolume.p[4] = frustum.BottomPlane();
	frustumVolume.p[5] = frustum.FarPlane();

	return frustumVolume;
}

MATH_END_NAMESPACE
