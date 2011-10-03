/** @file
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 
*/


#include <string.h>

#include "BitOps.h"

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
