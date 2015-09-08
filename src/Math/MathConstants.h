/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file MathConstants.h
	@author Jukka Jylänki
	@brief Common mathematical constants.

	See
	http://www.worldwideschool.org/library/books/sci/math/MiscellaneousMathematicalConstants/toc.html
*/
#pragma once

#include "MathTypes.h"
#include "MathNamespace.h"

#include <limits>
#include <math.h>
#include <float.h>

#ifdef __GNUC__
#define NOT_NECESSARILY_USED __attribute__ ((unused))
#else
#define NOT_NECESSARILY_USED
#endif

#if defined(_MSC_VER) || defined(EMSCRIPTEN)
#define FLOAT_NAN ((float)std::numeric_limits<float>::quiet_NaN())
#define FLOAT_INF ((float)std::numeric_limits<float>::infinity())
#else
#define FLOAT_NAN ((float)NAN)
#define FLOAT_INF ((float)INFINITY)
#endif

MATH_BEGIN_NAMESPACE

/// \f$\frac{1}{\sqrt{2\pi}}\f$
const float NOT_NECESSARILY_USED recipSqrt2Pi = (float)0.3989422804014326779399460599343818684758586311649346576659258296706579258993018385012523339073069364;
/// \f$\cos{1}\f$
const float NOT_NECESSARILY_USED cos1 =         (float)0.5403023058681397174009366074429766037323104206179222276700972553811003947744717645179518560871830893;
/// \f$\ln{2}\f$
const float NOT_NECESSARILY_USED ln2 =          (float)0.6931471805599453094172321214581765680755001343602552541206800094933936219696947156058633269964186875;
/// \f$\sqrt[3]{3}\f$
const float NOT_NECESSARILY_USED cbrt3 =        (float)1.4422495703074083823216383107801095883918692534993505775464161945416875968299973398547554797056452567;
/// \f$\frac{1}{\ln{2}}\f$
const float NOT_NECESSARILY_USED recipLn2 =     (float)1.4426950408889634073599246810018921374266459541529859341354494069311092191811850798855266228935063444;
/// Golden ratio, or \f$\frac{1+\sqrt{5}}{2}\f$
const float NOT_NECESSARILY_USED goldenRatio =  (float)1.6180339887498948482045868343656381177203091798057628621354486227052604628189024497072072041893911375;
/// \f$\ln{10}\f$
const float NOT_NECESSARILY_USED ln10 =         (float)2.3025850929940456840179914546843642076011014886287729760333279009675726096773524802359972050895982983;
/// \f$e\f$
const float NOT_NECESSARILY_USED euler =        (float)2.7182818284590452353602874713526624977572470936999595749669676277240766303535475945713821785251664274;
/// \f$\pi\f$
const float NOT_NECESSARILY_USED pi =           (float)3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679;
/// \f$e^2\f$
const float NOT_NECESSARILY_USED euler2 =       (float)7.3890560989306502272304274605750078131803155705518473240871278225225737960790577633843124850791217948;
/// A very small epsilon value to use in floating point equality comparisons.
const float NOT_NECESSARILY_USED c_eps =       (float)1e-5f;
/// The floating point representation for +\f$\inf\f$.
const float NOT_NECESSARILY_USED inf =          FLOAT_INF;
/// The floating point representation for -\f$\inf\f$.
const float NOT_NECESSARILY_USED negInf =       -FLOAT_INF;
/// Represents a floating-point not-a-number. \note Never compare a float against nan, use IsFinite() instead!
const float NOT_NECESSARILY_USED nan =          FLOAT_NAN;
/// Stores the largest positive non-infinite value for a float.
const float NOT_NECESSARILY_USED floatMax =     FLT_MAX;
/// Stores the largest negative non-infinite value for a float.
const float NOT_NECESSARILY_USED floatMin =     -FLT_MAX;

/// Integral base to an integral power.
template<u32 Base, u32 Power>
class PowT
{
public:
	enum { val = Base * PowT<Base,Power-1>::val };
};

/** @cond FULL */

/// End recursion for Base^1.
template<u32 Base>
class PowT<Base, 1>
{
public:
	enum { val = Base };
};
/// @endcond

/// Factorial<N> unfolds to N!.
template<int N>
class FactorialT
{
public:
	enum { val = N * FactorialT<N-1>::val };
};

/** @cond FULL */

/// Specialize 0! = 1 to end factorial recursion.
template<>
class FactorialT<0>
{
public:
	enum { val = 1 };
};
/// @endcond

/// Combinatorial<N, K> unfolds to (N nCr K).
template<int N, int K>
class CombinatorialT
{
public:
	enum { val = CombinatorialT<N-1,K-1>::val + CombinatorialT<N-1,K>::val };
};

/** @cond FULL */

/// Specialize (N nCr 0) = 1 to end recursion.
template<int N>
class CombinatorialT<N, 0>
{
public:
	enum { val = 1 };
};

/// Specialize (N nCr N) = 1 to end recursion.
template<int N>
class CombinatorialT<N, N>
{
public:
	enum { val = 1 };
};
/// @endcond

MATH_END_NAMESPACE
