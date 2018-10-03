#pragma once

// From http://cnicholson.net/2009/03/stupid-c-tricks-dowhile0-and-c4127/
#ifdef _MSC_VER
#define MULTI_LINE_MACRO_BEGIN do { \
	__pragma(warning(push)) \
	__pragma(warning(disable:4127))

#define MULTI_LINE_MACRO_END \
	} while(0) \
	__pragma(warning(pop))

#else

#define MULTI_LINE_MACRO_BEGIN do {
#define MULTI_LINE_MACRO_END } while(0)

#endif
