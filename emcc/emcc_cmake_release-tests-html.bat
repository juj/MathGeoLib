del cmakecache.txt

set PATH=%PATH:cygwin=cygwin_unused%
set PATH=%PATH:git=git_unused%
emconfigure cmake -DEMSCRIPTEN=1 -DFAIL_USING_EXCEPTIONS=1 -DMATH_TESTS_EXECUTABLE=1 -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ..

pause
