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

/** @file MatrixProxy.h
	@author Jukka Jylänki
	@brief */
#pragma once

#include "myassert.h"
#include "../MathGeoLibFwd.h"

MATH_BEGIN_NAMESPACE

/// A proxy class for double brackets [][] element access in matrices.
template<int Cols>
class MatrixProxy
{
private:
	float v[Cols];

public:
	CONST_WIN32 float operator[](int col) const
	{
		assert(col >= 0);
		assert(col < Cols);

		return v[col];
	}
	float &operator[](int col)
	{
		assert(col >= 0);
		assert(col < Cols);

		return v[col];
	}
};

MATH_END_NAMESPACE
