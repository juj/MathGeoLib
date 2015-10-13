#include <stdio.h>
#include <stdlib.h>
#include <locale.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#ifdef MATH_WITH_GRISU3
#include "../src/Math/grisu3.h"
#endif
#include "TestRunner.h"
#include "ObjectGenerators.h"
#include "TestData.h"

using namespace TestData;

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

std::string U64Str(uint64_t u)
{
	char str[256];
	sprintf(str, "%X%X", (uint32_t)(u >> 32), (uint32_t)u);
	return str;
}

#ifdef MATH_WITH_GRISU3
void test_double_to_string(double d, const char *d_in_src)
{
	std::string s = dtoa_grisu3_string(d);
	double d1 = d;
	double d2 = DeserializeDouble(s.c_str(), 0);
	uint64_t u1 = *(uint64_t*)&d1;
	uint64_t u2 = *(uint64_t*)&d2;
	if (u1 != u2)
	{
		int64_t ulpsDifference = Abs<int64_t>(u1 - u2);
#ifdef _MSC_VER
		bool isDenormal = (u1 & 0x7FF0000000000000ULL) == 0 && (u2 & 0x7FF0000000000000ULL) == 0;
		if (isDenormal && ulpsDifference <= 1) return; // Ignore 1 ulp differences on denormals! (MSVC bug?)
#endif
		LOGE("%.17g to str -> %s does not match:\nOriginal: %.17g (0x%s)\nExpected: %.17g (0x%s)\n   %d ulps difference!", d, s.c_str(), d1, U64Str(u1).c_str(), d2, U64Str(u2).c_str(), ulpsDifference);
	}
	assert(u1 == u2);

	if (d_in_src && strcmp(d_in_src, s.c_str()))// && strlen(d_in_src) < s.length())
	{
		LOGE("%.17g to str -> string does not match shortest representation:\nExpected: %s\nReceived: %s!", d1, d_in_src, s.c_str());
		assert(false);
	}
}

// Test that we get the short representation as presented in source code.
#define TEST_D_GIVEN(d) test_double_to_string((double)d, #d)

#define TEST_D_TO_STR(d) test_double_to_string(d, 0)

UNIQUE_TEST(grisu3)
{
	test_double_to_string((double)-FLOAT_INF, "-inf");
	TEST_D_GIVEN(-3.3885015522551284e+186);
	TEST_D_GIVEN(-16777216);
	TEST_D_GIVEN(-99.9);
	TEST_D_GIVEN(-1);
	TEST_D_GIVEN(-.5);
	TEST_D_GIVEN(-1.3447622391509214e-5);
	TEST_D_GIVEN(-1.0971427264533918e-308); // grisu3 fails.
	TEST_D_GIVEN(-5e-324);
	uint64_t negZero = 0x8000000000000000ULL;
	double d = *(double*)&negZero;
	test_double_to_string(d, "-0");
	TEST_D_GIVEN(0);
	TEST_D_GIVEN(5e-324); // min denormal double
	TEST_D_TO_STR(1.0971427264533918e-308); // grisu3 fails.
	TEST_D_GIVEN(1.5450561895576766e-308);
	TEST_D_TO_STR(4.1878166087191307e-23); // grisu3 fails.
	TEST_D_GIVEN(1.2345e-2);
	TEST_D_TO_STR(0.0020599371828568034); // grisu3 fails.
	TEST_D_GIVEN(.5);
	test_double_to_string(.0009, "9e-4");
	TEST_D_GIVEN(.009);
	TEST_D_GIVEN(.09);
	TEST_D_GIVEN(.9);
	TEST_D_GIVEN(1);
	TEST_D_TO_STR(1.0444224918822551); // grisu3 fails.
	TEST_D_GIVEN(1.1);
	TEST_D_GIVEN(2);
	TEST_D_GIVEN(10);
	TEST_D_GIVEN(11.1);
	TEST_D_GIVEN(99.9);
	TEST_D_GIVEN(100);
	test_double_to_string(1000, "1e3");
	TEST_D_GIVEN(1e5);
	TEST_D_GIVEN(100000.00000001);
	TEST_D_GIVEN(16777216);
	TEST_D_GIVEN(12345e4);
	TEST_D_GIVEN(1.2345e-1);
	TEST_D_TO_STR(232889534317.95139);
	TEST_D_GIVEN(232889534317.9514);
	TEST_D_TO_STR(1075193546584750.7); // grisu3 fails.
	TEST_D_TO_STR(3.3885015522551284e186); // grisu3 fails.
	TEST_D_TO_STR(1.7976931348623157e308); // max double
	TEST_D_GIVEN(17976931348623157e292); // max double, but in one char less.
	test_double_to_string((double)FLOAT_INF, "inf");
	TEST_D_TO_STR((double)FLOAT_NAN);
}

RANDOMIZED_TEST(grisu3_random)
{
	uint64_t u = (uint64_t)rng.Int() ^ ((uint64_t)rng.Int() << 15) ^ ((uint64_t)rng.Int() << 30) ^ ((uint64_t)rng.Int() << 45)  ^ ((uint64_t)rng.Int() << 60);
	double d = *(double*)&u;
	test_double_to_string(d, 0);
}
#endif
char dummy_str[256] = {};

// Benchmark 'dtoa_sprintf': dtoa_sprintf
//    Best: 967.948 nsecs / 1646 ticks, Avg: 1.181 usecs, Worst: 2.403 usecs
BENCHMARK(dtoa_sprintf, "dtoa_sprintf")
{
	sprintf(dummy_str, "%.17g", (double)f[i]);
}
BENCHMARK_END;

#ifdef MATH_WITH_GRISU3
// Benchmark 'dtoa_grisu': dtoa_grisu
//    Best: 213.828 nsecs / 363.596 ticks, Avg: 229.754 nsecs, Worst: 338.511 nsecs
BENCHMARK(dtoa_grisu3, "dtoa_grisu3")
{
	dtoa_grisu3(f[i], dummy_str);
}
BENCHMARK_END;
#endif

BENCHMARK(float4_SerializeToString, "float4::SerializeToString")
{
	strcpy(dummy_str, v[i].SerializeToString().c_str());
}
BENCHMARK_END;
