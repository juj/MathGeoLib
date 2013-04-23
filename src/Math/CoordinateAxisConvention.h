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

/** @file CoordinateAxisConvention.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "float3x3.h"
#include "float3x4.h"
#include "float4x4.h"
#include "float3.h"
/*
struct PositiveX
{
	static float3 Pick(const float3x3 &m) { return m.Column(0); }
	static float3 Pick(const float3x4 &m) { return m.Column(0); }
	static float3 Pick(const float4x4 &m) { return m.Column3(0); }
};

struct NegativeX
{
	static float3 Pick(const float3x3 &m) { return -m.Column(0); }
	static float3 Pick(const float3x4 &m) { return -m.Column(0); }
	static float3 Pick(const float4x4 &m) { return -m.Column3(0); }
};

struct PositiveY
{
	static float3 Pick(const float3x3 &m) { return m.Column(1); }
	static float3 Pick(const float3x4 &m) { return m.Column(1); }
	static float3 Pick(const float4x4 &m) { return m.Column3(1); }
};

struct NegativeY
{
	static float3 Pick(const float3x3 &m) { return -m.Column(1); }
	static float3 Pick(const float3x4 &m) { return -m.Column(1); }
	static float3 Pick(const float4x4 &m) { return -m.Column3(1); }
};

struct PositiveZ
{
	static float3 Pick(const float3x3 &m) { return m.Column(2); }
	static float3 Pick(const float3x4 &m) { return m.Column(2); }
	static float3 Pick(const float4x4 &m) { return m.Column3(2); }
};

struct NegativeZ
{
	static float3 Pick(const float3x3 &m) { return -m.Column(2); }
	static float3 Pick(const float3x4 &m) { return -m.Column(2); }
	static float3 Pick(const float4x4 &m) { return -m.Column3(2); }
};

/// For more information about coordinate axis conventions, and handedness,
/// see http://msdn.microsoft.com/en-us/library/bb204853(VS.85).aspx .
template<typename ForwardAxis, typename RightAxis, typename UpAxis>
struct CoordinateAxisConvention
{
	template<typename Matrix> static float3 Right(const Matrix &m) { return RightAxis::Pick(m); }
	template<typename Matrix> static float3 Up(const Matrix &m) { return UpAxis::Pick(m); }
	template<typename Matrix> static float3 Forward(const Matrix &m) { return ForwardAxis::Pick(m); }
};

/// This is the default coordinate axis convention used by the math classes. This convention
/// is a left-handed coordinate system.
typedef CoordinateAxisConvention<PositiveX, PositiveY, PositiveZ> XposRight_YposUp_ZposForward;
*/
