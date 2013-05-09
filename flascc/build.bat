@echo off
set PTH=%PATH%
set PATH=%PATH:Windows=Win_dows_unused%
mingw32-make VERBOSE=1
set PATH=%PTH%