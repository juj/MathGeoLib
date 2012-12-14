#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"

void TestQuat4FromString()
{
	const char *locales[] = { "C", "en", "fi" }; // From http://www.loc.gov/standards/iso639-2/php/code_list.php

	for(int i = -1; i < (int)(sizeof(locales)/sizeof(locales[0])); ++i)
	{
		if (i != -1)
			setlocale(LC_ALL, locales[i]);
		assert(Quat::FromString("1, +2, 3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1, +2, 3.1, -4").Equals(Quat(1,2,3.1f,-4)));
		assert(Quat::FromString("(1, +2, 3.1, -4)").Equals(Quat(1,2,3.1f,-4)));

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
	AddTest("Quat4::FromString", TestQuat4FromString);
}
