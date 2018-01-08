#include "../include/Encoder.hpp"
#include <cassert>

float Encoder::mapToRange(float value, float min, float max, float range_max)
{
    if(value < min) {
        return 0.0f;
    }
    else if(value > max) {
        return range_max;
    }
    else {
        // map between 0-255;
        float range = max - min;
        value = (value - min) / range * range_max;
        value = std::max(0.0f, std::min(value, range_max));
        return value;
    }
}

uint32_t Encoder::mapToBit(float value, float min, float max, uint8_t bits)
{
    auto max_compressed = static_cast<uint32_t>(pow(2, bits)-1);
    return (uint32_t) mapToRange(value, min, max, max_compressed);
}

float Encoder::mapFromBit(uint32_t value, float min, float max, uint8_t bits, float invalid)
{
    auto max_compressed = static_cast<uint32_t >(pow(2, bits)-1);
    if(value == 0 || value == max_compressed) {
        return invalid;
    }
    else {
        float range = max - min;
        auto res = (float) value;
        res = (res/(float) max_compressed)*range + min;
        return res;
    }
}

Vec<float> const Encoder::rgbToYuv(Vec<float> const& rgb) {
  Vec<float> yuv;
  yuv.x = rgb.x * 0.299f + rgb.y * 0.587f + rgb.z * 0.114f;
  yuv.y = (rgb.z - yuv.x) * 0.493f;
  yuv.z = (rgb.x - yuv.x) * 0.877f;

  return yuv;
}
