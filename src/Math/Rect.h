/** @file 
    @author Jukka Jylänki

    This work is copyrighted material and may NOT be used for any kind of commercial or 
    personal advantage and may NOT be copied or redistributed without prior consent
    of the author(s). 

    @brief 2D integral axis-aligned rectangle, equivalent to RECT in Windows API.
*/
#pragma once

#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#endif

/// A 2D integral (x,y),(w,h) -rectangle.
class Rect
{
public:
    Rect() { left = top = right = bottom = 0; }
    Rect(int left_, int top_, int width, int height)
        :left(left_), top(top_), right(left + width), bottom(top + height) {}
//    ~Rect() {}

    int Width() { return right - left; }
    int Height() { return bottom - top; }

    int left;
    int top;
    int right;
    int bottom;

#ifdef WIN32
    operator RECT()
    {
        RECT r;
        r.top = top;
        r.left = left;
        r.right = right;
        r.bottom = bottom;
        return r;
    }
    Rect(const RECT &r)
    {
        top = r.top;
        left = r.left;
        bottom = r.bottom;
        right = r.right;
    }
#endif
};
