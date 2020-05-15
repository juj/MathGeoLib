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

/** @file Interpolate.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "assume.h"
#include "MathFunc.h"
#include "MathConstants.h"
#include "MathNamespace.h"

MATH_BEGIN_NAMESPACE

// Interpolates [0,1]->[0,1] in a way that starts smoothly, but stops sharply.
// Sometimes also referred to as EaseIn or EaseInQuad.
inline float SmoothStart(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  return t*t;
}

// Like SmoothStart, but even smoother start (and sharper stop)
inline float SmoothStart3(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  return t*t*t;
}

inline float SmoothStart4(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float tt = t*t;
  return tt*tt;
}

inline float SmoothStart5(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float tt = t*t;
  return tt*tt*t;
}

// Starts sharply at (0,0), but stops smoothly to (1,1). I.e. reverse of SmoothStart2.
inline float SmoothStop(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float oneT = 1.f - t;
  return 1.f - oneT*oneT;
}

inline float SmoothStop3(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float oneT = 1.f - t;
  return 1.f - oneT*oneT*oneT;
}

inline float SmoothStop4(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float oneT = 1.f - t;
  oneT *= oneT;
  return 1.f - oneT*oneT;
}

inline float SmoothStop5(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  float oneT = 1.f - t;
  float oneT2 = oneT * oneT;
  return 1.f - oneT2*oneT2*oneT;
}

// Starts out as SmoothStop, and linearly blends to SmoothStart
inline float SharpStep(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  //   t * t^2 + (1-t)*(1 - (1-t)^2)
  // = 2t^3 - 3t^2 + 2t
  // = t(2t^2 + 2 - 3t)
  float tt = t*t;
  return t*(2.f * tt + 2.f - 3.f * t);
}

// Starts out as SmoothStart, and linearly blends to SmoothStop.
// Also called "cubic Hermite interpolation"
inline float SmoothStep(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  //   (1-t) * SmoothStart(t) + t * SmoothStop(t)
  // = (1-t) * t^2 + t*(1 - (1-t)^2)
  // = 3t^2 - 2t^3

  float tt = t*t;
  return 3.f*tt - 2.f*tt*t;
}

// N.b. it is possible to define higher order linear
// blends, like 

//   (1-t) * SmoothStart4(t) + t * SmoothStop4(t)
//  = -2t^5+5t^4-6t^3+4t^2

//   (1-t) * SmoothStart5(t) + t * SmoothStop5(t)
//  = -4t^5+10t^4-10t^3+5t^2

//  Nth order blend: (1-t) * t^n + t * (1-(1-t)^n)

// and so on. As the exponent grows, the function becomes
// "sharper" in the beginning and the end.

inline float SmoothStep5(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  // 6t^5 -15t^4 + 10t^3
  float tt = t*t;
  return tt*t*(6.f*tt - 15.f*t + 10.f);
}

inline float SmoothStep7(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  // -20t^7 + 70t^6 - 84t^5 + 35t^4
  float tt = t*t;
  float tttt = tt*tt;
  return tttt*(-20.f*tt*t + 70.f*tt - 84.f*t + 35.f);
}

inline float CosineStep01(float t)
{
  assume1(t >= 0.f && t <= 1.f, t); // Input should be pre-clamped for performance (combine with Clamp01() from MathFunc.h)
  return 0.5f - Cos(t*pi) * 0.5f;
}

MATH_END_NAMESPACE
