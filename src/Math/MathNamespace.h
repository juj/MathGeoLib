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

/** @file MathNamespace.h
	@author Jukka Jylänki
	@brief Defines a C++ namespace for all Math objects (optional). */
#pragma once

#include "../MathBuildConfig.h"

#ifdef MATH_ENABLE_NAMESPACE

#define MATH_BEGIN_NAMESPACE namespace MATH_NAMESPACE_NAME {
#define MATH_END_NAMESPACE }

#define MATH_NS MATH_NAMESPACE_NAME

#define USE_MATH_NAMESPACE using namespace MATH_NAMESPACE_NAME;

#else

// Don't embed the math library into a namespace.
#define MATH_BEGIN_NAMESPACE
#define MATH_END_NAMESPACE

#define MATH_NS

#define USE_MATH_NAMESPACE

#endif

MATH_BEGIN_NAMESPACE
MATH_END_NAMESPACE

#ifdef MATH_AUTO_USE_NAMESPACE
// It is very unconventional to put a 'using namespace' inside a .h file, but here
// the math namespace only exists to allow shadowing all symbols outside the math libraries.
// This enables hiding the unwanted stuff from the math libraries (e.g. Windows GDI Polygon function).
USE_MATH_NAMESPACE
#endif
