/* Copyright Jukka Jyl�nki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file MatrixProxy.h
	@author Jukka Jyl�nki
	@brief */
#pragma once

#include "myassert.h"
#include "../MathGeoLibFwd.h"

MATH_BEGIN_NAMESPACE

/// A proxy class for double brackets [][] element access in matrices.
#ifdef MATH_COLMAJOR_MATRICES
template<int Rows, int Cols>
class MatrixProxy
{
private:
	float v[Cols*Rows];

public:
	CONST_WIN32 FORCE_INLINE float operator[](int col) const
	{
		mgl_assert(col >= 0);
		mgl_assert(col < Cols);
		
		return v[col*Rows];
	}
	FORCE_INLINE float &operator[](int col)
	{
		mgl_assert(col >= 0);
		mgl_assert(col < Cols);
		
		return v[col*Rows];
	}
};
#else
template<int Rows, int Cols>
class MatrixProxy
{
private:
	float v[Cols];

public:
	CONST_WIN32 FORCE_INLINE float operator[](int col) const
	{
		mgl_assert(col >= 0);
		mgl_assert(col < Cols);

		return v[col];
	}
	FORCE_INLINE float &operator[](int col)
	{
		mgl_assert(col >= 0);
		mgl_assert(col < Cols);

		return v[col];
	}
};
#endif

MATH_END_NAMESPACE
