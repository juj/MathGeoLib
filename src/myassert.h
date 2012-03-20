#include "Log.h"

#ifdef assert
#undef assert
#endif

#ifdef OPTIMIZED_RELEASE
#define assert(x)
#else

#ifdef WIN32
#include <cassert>
#else


#ifdef _DEBUG
#define assert(x) do { if (!(x)) LOGW("Assertion failed: " #x); } while(0)
#else
#define assert(x)
#endif

#endif

#endif
