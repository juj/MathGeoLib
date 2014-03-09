#include <stdio.h>
#include <stdlib.h>
#include <locale.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"
#include "ObjectGenerators.h"

MATH_BEGIN_NAMESPACE
bool IsNeutralCLocale();
MATH_END_NAMESPACE

UNIQUE_TEST(StartupLocale)
{
	LOGI("Startup locale for LC_ALL is '%s'", setlocale(LC_ALL, NULL));
	LOGI("Startup locale for LC_NUMERIC is '%s'", setlocale(LC_NUMERIC, NULL));
#ifndef ANDROID // Not implemented on Android, see float2.cpp/IsNeutralCLocale().
	lconv *lc = localeconv();
	LOGI("Decimal point is '%s'", lc->decimal_point);
#endif

	bool isGoodLocale = MATH_NS::IsNeutralCLocale();
	if (isGoodLocale)
		LOGI("This locale setup is appropriate for string serialization.");
	else
		LOGI("This locale setup is NOT appropriate for string serialization!");
	assert(isGoodLocale);
}

#define assertveq(a,b) if (!(a).Equals((b))) { LOGE("%s != %s (%s != %s)", #a, #b, (a).ToString().c_str(), (b).ToString().c_str()); } assert((a).Equals(b));

TEST(Float2FromString)
{
	const char *locales[] = { "C", "en" /*, "fi" */ }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assertveq(float2::FromString("-1.0000, +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000, +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000, +2)"), float2(-1.0000,2));

		assertveq(float2::FromString("-1.0000 ; +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000 ; +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000 ; +2)"), float2(-1.0000,2));

		assertveq(float2::FromString("-1.0000,+2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000,+2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000,+2)"), float2(-1.0000,2));

		assertveq(float2::FromString("-1.0000 +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000 +2"), float2(-1.0000,2));
		assertveq(float2::FromString("(-1.0000 +2)"), float2(-1.0000,2));
	}
}

TEST(Float3FromString)
{
	const char *locales[] = { "C", "en" /*, "fi" */ }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assertveq(float3::FromString("+2, 3.1, -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2, 3.1, -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2, 3.1, -4)"), float3(2,3.1f,-4));

		assertveq(float3::FromString("+2; 3.1; -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2; 3.1; -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2; 3.1; -4)"), float3(2,3.1f,-4));

		assertveq(float3::FromString("+2,3.1, -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2,3.1, -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2,3.1, -4)"), float3(2,3.1f,-4));

		assertveq(float3::FromString("+2 3.1  -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2 3.1  -4"), float3(2,3.1f,-4));
		assertveq(float3::FromString("(+2 3.1  -4)"), float3(2,3.1f,-4));
	}
}
TEST(Float4FromString)
{
	const char *locales[] = { "C", "en" /*, "fi" */ }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assertveq(float4::FromString("1, +2, 3.1, -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1, +2, 3.1, -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1, +2, 3.1, -4)"), float4(1,2,3.1f,-4));

		assertveq(float4::FromString("1 ; +2; 3.1; -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1 ; +2; 3.1; -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1 ; +2; 3.1; -4)"), float4(1,2,3.1f,-4));

		assertveq(float4::FromString("1,+2,3.1, -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1,+2,3.1, -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1,+2,3.1, -4)"), float4(1,2,3.1f,-4));

		assertveq(float4::FromString("1 +2 3.1  -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1 +2 3.1  -4"), float4(1,2,3.1f,-4));
		assertveq(float4::FromString("(1 +2 3.1  -4)"), float4(1,2,3.1f,-4));
	}
}

TEST(QuatFromString)
{
	const char *locales[] = { "C", "en" /*, "fi" */ }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assertveq(Quat::FromString("1, +2, 3.1, -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1, +2, 3.1, -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1, +2, 3.1, -4)"), Quat(1,2,3.1f,-4));

		assertveq(Quat::FromString("1 ; +2; 3.1; -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1 ; +2; 3.1; -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1 ; +2; 3.1; -4)"), Quat(1,2,3.1f,-4));

		assertveq(Quat::FromString("1,+2,3.1, -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1,+2,3.1, -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1,+2,3.1, -4)"), Quat(1,2,3.1f,-4));

		assertveq(Quat::FromString("1 +2 3.1  -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1 +2 3.1  -4"), Quat(1,2,3.1f,-4));
		assertveq(Quat::FromString("(1 +2 3.1  -4)"), Quat(1,2,3.1f,-4));
	}
}

RANDOMIZED_TEST(SerializeFloat)
{
	u32 u = rng.Int() ^ (rng.Int() << 16);
	// Make u a quiet NaN - since quiet vs signalling nans might not work on all platforms, and MGL
	// is not interested in using or serializing sNans anyways.
	u |= 0x400000;
	float f = ReinterpretAsFloat(u);
	char str[256];
	SerializeFloat(f, str);
	float f2 = DeserializeFloat(str);
	u32 u2 = ReinterpretAsU32(f2);
	assert4(u == u2, u, u2, f, f2);
}

#define LARGESCALE 1e8f

RANDOMIZED_TEST(float2_Serialize)
{
	float2 o = float2::RandomBox(rng, -LARGESCALE, LARGESCALE);

	std::string s = o.SerializeToString();
	float2 o2 = float2::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = float2::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = float2::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(float3_Serialize)
{
	float3 o = float3::RandomBox(rng, float3::FromScalar(-LARGESCALE), float3::FromScalar(LARGESCALE));

	std::string s = o.SerializeToString();
	float3 o2 = float3::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = float3::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = float3::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(float4_Serialize)
{
	float4 o = float4::RandomBox(rng, float4::FromScalar(-LARGESCALE), float4::FromScalar(LARGESCALE));

	std::string s = o.SerializeToString();
	float4 o2 = float4::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = float4::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = float4::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Quat_Serialize)
{
	float4 fo = float4::RandomBox(rng, float4::FromScalar(-LARGESCALE), float4::FromScalar(LARGESCALE));
	Quat o(fo.x, fo.y, fo.z, fo.w);

	std::string s = o.SerializeToString();
	Quat o2 = Quat::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = Quat::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = Quat::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(AABB_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-LARGESCALE), POINT_VEC_SCALAR(LARGESCALE));
	AABB o = RandomAABBContainingPoint(pt, LARGESCALE);
	std::string s = o.SerializeToString();
	AABB o2 = AABB::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = AABB::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = AABB::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(OBB_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-LARGESCALE), POINT_VEC_SCALAR(LARGESCALE));
	OBB o = RandomOBBContainingPoint(pt, LARGESCALE);
	std::string s = o.SerializeToString();
	OBB o2 = OBB::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.SerializeToCodeString();
	o2 = OBB::FromString(s);
	assert(o.Equals(o2));
	assert(o.BitEquals(o2));

	s = o.ToString();
	o2 = OBB::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Sphere_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Sphere o = RandomSphereContainingPoint(pt, SCALE);
	std::string s = o.SerializeToString();
	Sphere o2 = Sphere::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Sphere::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Sphere::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Plane_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Plane o = RandomPlaneContainingPoint(pt);
	std::string s = o.SerializeToString();
	Plane o2 = Plane::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Plane::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Plane::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Triangle_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Triangle o = RandomTriangleContainingPoint(pt);
	std::string s = o.SerializeToString();
	Triangle o2 = Triangle::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Triangle::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Triangle::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Ray_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Ray o = RandomRayContainingPoint(pt);
	std::string s = o.SerializeToString();
	Ray o2 = Ray::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Ray::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Ray::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(LineSegment_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	LineSegment o = RandomLineSegmentContainingPoint(pt);
	std::string s = o.SerializeToString();
	LineSegment o2 = LineSegment::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = LineSegment::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = LineSegment::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Line_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Line o = RandomLineContainingPoint(pt);
	std::string s = o.SerializeToString();
	Line o2 = Line::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Line::FromString(s);
	assert(o.Equals(o2));
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Line::FromString(s);
	o2.dir.Normalize();
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Capsule_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Capsule o = RandomCapsuleContainingPoint(pt);
	std::string s = o.SerializeToString();
	Capsule o2 = Capsule::FromString(s);
	assert2(o.Equals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.SerializeToCodeString();
	o2 = Capsule::FromString(s);
	assert2(o.Equals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Capsule::FromString(s);
	assert(o.Equals(o2, 0.1f));
}

RANDOMIZED_TEST(Polygon_Serialize)
{
	vec pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polygon o = RandomPolygonContainingPoint(pt);
	std::string s = o.SerializeToString();
	Polygon o2 = Polygon::FromString(s);
	assert2(o.Equals(o2), o.SerializeToString(), o2.SerializeToString());
	assert2(o.BitEquals(o2), o.SerializeToString(), o2.SerializeToString());
/*
	s = o.SerializeToCodeString();
	o2 = Polygon::FromString(s);
	assert2(o.Equals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());
	assert2(o.BitEquals(o2), o.SerializeToCodeString(), o2.SerializeToCodeString());

	s = o.ToString();
	o2 = Polygon::FromString(s);
	assert(o.Equals(o2, 0.1f));
*/
}
