#include "SSEMath.h"
#include "MathFunc.h"
#include <stdint.h>
#include <stdlib.h>

using namespace std;

MATH_BEGIN_NAMESPACE

void *AlignedMalloc(size_t size, size_t alignment)
{
	alignment = alignment > 0 ? alignment : 1;
	size += alignment;

	uintptr_t ptr = (uintptr_t)malloc(size);
#ifndef __clang_analyzer__ // Hide clang analyzer false positive: Memory is never released; potential leak of memory pointed to by 'ptr'
	if (!ptr)
		return 0;
	++ptr; // Must make room for storing the offset info.
	ptrdiff_t incr = (alignment - (ptr & (alignment-1))) & (alignment-1);
	ptr += incr;
	((u8*)ptr)[-1] = (u8)(incr+1);
#endif
	assert(ptr % alignment == 0);
	return (void*)ptr;
}

void AlignedFree(void *ptr)
{
	if (!ptr)
		return;
	u8 *p = (u8*)ptr;
	p -= p[-1];
	free(p);
}

MATH_END_NAMESPACE
