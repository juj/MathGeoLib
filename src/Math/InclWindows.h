// This header file includes Windows.h the way that MathGeoLib wants to use it. No need to follow this in your own projects.

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// Windows GDI has a global function named Polygon. I am not renaming my Polygon object just for its sake, especially since
// I'm not expecting anyone to co-use this library with GDI. Kill the Polygon function from Windows. and force-include
// Windows.h here to erase that function signature.

// If you are getting build errors with conflicting types on Win32 GDI Polygon() and MathGeoLib class Polygon, then
// MathGeoLib is probably including this file too late, after your own project has already included <Windows.h> itself. In
// that case, try moving this file higher up in the include chain.

// Alternatively, consider building MathGeoLib with MATH_ENABLE_NAMESPACE so that class Polygon is enclosed in the math:: namespace
// and also possibly disable MATH_AUTO_USE_NAMESPACE in MathBuildConfig.h.
#ifndef MATH_ENABLE_NAMESPACE
#define Polygon Polygon_unused
#endif
#include <Windows.h>
#ifndef MATH_ENABLE_NAMESPACE
#undef Polygon
#endif

