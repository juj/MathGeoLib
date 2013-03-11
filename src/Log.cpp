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

/** @file Log.cpp
	@author Jukka Jylänki
	@brief The LOG and LOGUSER macros. Provides an unified mechanism for logging. */
#include "Log.h"

#if defined(WIN32) && !defined(WIN8RT)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

MATH_BEGIN_NAMESPACE

void SetStdoutTextColor(int newColor)
{
#if defined(WIN32) && !defined(WIN8RT) // Win8 metro apps don't have SetConsoleTextAttribute.
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), (WORD)newColor);
#endif
}

MATH_END_NAMESPACE
