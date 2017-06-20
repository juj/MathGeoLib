#pragma once

#ifdef __EMSCRIPTEN__

extern "C" {
	extern const char *browser_info();
}

#endif
