#pragma once

/// Adjust this #define to choose the name of the namespace math is given to.
#define MATH_NAMESPACE_NAME math

#define MATH_ENABLE_NAMESPACE

#ifdef MATH_ENABLE_NAMESPACE

#define MATH_BEGIN_NAMESPACE namespace MATH_NAMESPACE_NAME {
#define MATH_END_NAMESPACE }

#define MATH_NS math

#define USE_MATH_NAMESPACE using namespace math;

#else

// Don't embed the math library into a namespace.
#define MATH_BEGIN_NAMESPACE
#define MATH_END_NAMESPACE

#define MATH_NS

#define USE_MATH_NAMESPACE

#endif

MATH_BEGIN_NAMESPACE
MATH_END_NAMESPACE

// It is very unconventional to put a 'using namespace' inside a .h file, but here
// the math namespace only exists to allow shadowing all symbols outside the math libraries.
// This enables hiding the unwanted stuff from the math libraries (e.g. Windows GDI Polygon function).
USE_MATH_NAMESPACE
