#pragma once

#include <stdint.h>

float Float16ToFloat32(uint16_t float16);
uint16_t Float32ToFloat16(float float32);

uint32_t Float32ToMiniFloat(bool signBit, int exponentBits, int mantissaBits, int exponentBias, float value);
float MiniFloatToFloat32(bool signBit, int exponentBits, int mantissaBits, int exponentBias, uint32_t value);
