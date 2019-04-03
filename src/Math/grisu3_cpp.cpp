/* This file is part of an implementation of the "grisu3" double to string
	conversion algorithm described in the research paper

	"Printing Floating-Point Numbers Quickly And Accurately with Integers"
	by Florian Loitsch, available at
	http://www.cs.tufts.edu/~nr/cs257/archive/florian-loitsch/printf.pdf */

#include "grisu3.h"
#include <string>
#include <assert.h>

std::string dtoa_grisu3_string(double v)
{
	char str[32];
	int len = dtoa_grisu3(v, str,32);
	assert(len > 0 && len < 25);
	assert(str[len] == '\0');
	((void)len);
	return str;
}
