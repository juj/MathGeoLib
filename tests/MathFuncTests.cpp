#include <stdio.h>
#include <stdlib.h>

#include "MathGeoLib.h"
#include "myassert.h"
#include "TestRunner.h"
#include <cmath>

#ifndef _MSC_VER // MSVC compilers do not have std::isfinite
void TestCXX11StdFinite()
{
	// When using MathGeoLib, users should still be able to invoke C++11 std::isfinite function.
	// http://en.cppreference.com/w/cpp/numeric/math/isfinite
	assert(std::isfinite(5.f));
	assert(std::isfinite(5.0));
	assert(std::isfinite(5.0L));

	using namespace std;

	assert(isfinite(5.f));
	assert(isfinite(5.0));
	assert(isfinite(5.0L));
}
#endif

void TestIsFinite()
{
	assert(IsFinite(5));
	assert(IsFinite(5.f));
	assert(IsFinite(5.0));
	assert(IsFinite(5.0L));

	assert(!IsFinite(FLOAT_NAN));
	assert(!IsFinite((double)FLOAT_NAN));
	assert(!IsFinite((long double)FLOAT_NAN));

	assert(!IsFinite(FLOAT_INF));
	assert(!IsFinite((double)FLOAT_INF));
	assert(!IsFinite((long double)FLOAT_INF));

	assert(IsFinite(FLT_MAX));
	assert(IsFinite((double)FLT_MAX));
	assert(IsFinite((long double)FLT_MAX));
}

void AddMathFuncTests()
{
#ifndef _MSC_VER
	AddTest("C++11 std::isfinite", TestCXX11StdFinite);
#endif
	AddTest("IsFinite", TestIsFinite);
}
