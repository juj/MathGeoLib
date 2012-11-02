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

/** @file BitOps.cpp
	@author Jukka Jylänki
	@brief */
#include "BitOps.h"
#include <string.h>


MATH_BEGIN_NAMESPACE

u32 BinaryStringToValue(const char *str)
{
	u32 val = 0;
	int strl = (int)strlen(str);
	for(int i = 0; i < strl && i < 32; ++i)
		if (str[strl-i-1] != '0')
			val |= 1 << i;
	return val;
}

MATH_END_NAMESPACE
