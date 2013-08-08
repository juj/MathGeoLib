#pragma once

#ifdef EMSCRIPTEN

extern "C" {
	extern const char *browser_info();
}

#endif
