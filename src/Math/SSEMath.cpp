#include "SSEMath.h"
#include "MathFunc.h"
#include <stdint.h>
#include <stdlib.h>

MATH_BEGIN_NAMESPACE

void *AlignedMalloc(size_t size, size_t alignment)
{
	alignment = alignment > 0 ? alignment : 1;
	size += alignment;

	uintptr_t ptr = (uintptr_t)malloc(size);
	if (!ptr)
		return 0;
	++ptr; // Must make room for storing the offset info.
	ptrdiff_t incr = (alignment - (ptr & (alignment-1))) & (alignment-1);
	ptr += incr;
	((u8*)ptr)[-1] = (u8)(incr+1);
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
