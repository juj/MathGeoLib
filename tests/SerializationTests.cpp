#include <stdio.h>
#include <stdlib.h>
#include <locale.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

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
