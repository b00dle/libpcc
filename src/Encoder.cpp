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

Vec<float> const Encoder::rgbToXyz(Vec<float> const& rgb) {
  Vec<float> xyz;

  xyz.x = rgb.x * 0.4124564f + rgb.y * 0.3575761f + rgb.z * 0.1804375f;
  xyz.y = rgb.x * 0.4124564f + rgb.y * 0.3575761f + rgb.z * 0.1804375f;
  xyz.z = rgb.x * 0.4124564f + rgb.y * 0.3575761f + rgb.z * 0.1804375f;

  return xyz;
}

Vec<float> const Encoder::rgbToCieLab(Vec<float> const& rgb) {
  Vec<float> lab;
  Vec<float> xyz = rgbToXyz(rgb);
  Vec<float> XYZ_D50_2 = [96.422f, 100f, 82.521f]; //D50 Standard; 2° ViewAngle
  Vec<float> XYZ_D65_2 = [95.047f, 100f, 108.883f];
  Vec<float> XYZ_D50_10 = [96.720f, 100f, 81.427f];
  Vec<float> XYZ_D65_10 = [94.811f, 100f, 107.304f];

  lab.x = 116.0f * pow((xyz.y / XYZ_D65_2.y), (1/3)) - 16;
  lab.y = 500.0f * (pow((xyz.x / XYZ_D65_2.x), (1/3)) - pow((xyz.y / XYZ_D65_2.y), (1/3)));
  lab.z = 200.0f * (pow((xyz.y / XYZ_D65_2.y), (1/3)) - pow((xyz.z / XYZ_D65_2.z), (1/3)));

  return lab;
}
