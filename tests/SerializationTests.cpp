#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

void TestFloat2FromString()
{
	const char *locales[] = { "C", "en", "fi" }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assert(float2::FromString("-1.0000, +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000, +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000, +2)").Equals(float2(-1.0000,2)));

		assert(float2::FromString("-1.0000 ; +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000 ; +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000 ; +2)").Equals(float2(-1.0000,2)));

		assert(float2::FromString("-1.0000,+2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000,+2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000,+2)").Equals(float2(-1.0000,2)));

		assert(float2::FromString("-1.0000 +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000 +2").Equals(float2(-1.0000,2)));
		assert(float2::FromString("(-1.0000 +2)").Equals(float2(-1.0000,2)));
	}
}

void TestFloat3FromString()
{
	const char *locales[] = { "C", "en", "fi" }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assert(float3::FromString("+2, 3.1, -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2, 3.1, -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2, 3.1, -4)").Equals(float3(2,3.1f,-4)));

		assert(float3::FromString("+2; 3.1; -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2; 3.1; -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2; 3.1; -4)").Equals(float3(2,3.1f,-4)));

		assert(float3::FromString("+2,3.1, -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2,3.1, -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2,3.1, -4)").Equals(float3(2,3.1f,-4)));

		assert(float3::FromString("+2 3.1  -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2 3.1  -4").Equals(float3(2,3.1f,-4)));
		assert(float3::FromString("(+2 3.1  -4)").Equals(float3(2,3.1f,-4)));
	}
}
void TestFloat4FromString()
{
	const char *locales[] = { "C", "en", "fi" }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assert(float4::FromString("1, +2, 3.1, -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1, +2, 3.1, -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1, +2, 3.1, -4)").Equals(float4(1,2,3.1f,-4)));

		assert(float4::FromString("1 ; +2; 3.1; -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1 ; +2; 3.1; -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1 ; +2; 3.1; -4)").Equals(float4(1,2,3.1f,-4)));

		assert(float4::FromString("1,+2,3.1, -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1,+2,3.1, -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1,+2,3.1, -4)").Equals(float4(1,2,3.1f,-4)));

		assert(float4::FromString("1 +2 3.1  -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1 +2 3.1  -4").Equals(float4(1,2,3.1f,-4)));
		assert(float4::FromString("(1 +2 3.1  -4)").Equals(float4(1,2,3.1f,-4)));
	}
}

void TestQuatFromString()
{
	const char *locales[] = { "C", "en", "fi" }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assert(Quat::FromString("1, +2, 3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1, +2, 3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1, +2, 3.1, -4)").Equals(Quat(1,2,3.1f,-4)));

		assert(Quat::FromString("1 ; +2; 3.1; -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1 ; +2; 3.1; -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1 ; +2; 3.1; -4)").Equals(Quat(1,2,3.1f,-4)));

		assert(Quat::FromString("1,+2,3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1,+2,3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1,+2,3.1, -4)").Equals(Quat(1,2,3.1f,-4)));

		assert(Quat::FromString("1 +2 3.1  -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1 +2 3.1  -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1 +2 3.1  -4)").Equals(Quat(1,2,3.1f,-4)));
	}
}

void AddSerializationTests()
{
	AddTest("Float2::FromString", TestFloat2FromString);
	AddTest("Float3::FromString", TestFloat3FromString);
	AddTest("Float4::FromString", TestFloat4FromString);
	AddTest("Quat::FromString", TestQuatFromString);
}
