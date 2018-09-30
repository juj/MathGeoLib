#pragma once

#include "../MathBuildConfig.h"
#include "MathNamespace.h"
#include "MathTypes.h"

#if __cplusplus >= 201103L
#include <utility>
#endif

MATH_BEGIN_NAMESPACE

#if __cplusplus >= 201103L
/// Swaps the two values.
template<typename T>
FORCE_INLINE void Swap(T &a, T &b)
{
	T temp = std::move(a);
	a = std::move(b);
	b = std::move(temp);
}
#else
/// Swaps the two values.
template<typename T>
FORCE_INLINE void Swap(T &a, T &b)
{
	T temp = a;
	a = b;
	b = temp;
}
#endif

MATH_END_NAMESPACE
