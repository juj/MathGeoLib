/** @file FloatCmp.h
	@author Jukka Jylänki

	This work is copyrighted material and may NOT be used for any kind of commercial or 
	personal advantage and may NOT be copied or redistributed without prior consent
	of the author(s). 
*/
#pragma once

#ifdef MATH_ENABLE_STL_SUPPORT
#include <cassert>
#endif

bool Equal(double a, double b, double epsilon = 1e-6)
{
	return std::abs(a-b) < epsilon;
}
