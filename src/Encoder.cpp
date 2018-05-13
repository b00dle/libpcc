#include "../include/Encoder.hpp"
#include <cassert>
#include <iostream>

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

Vec<float> const Encoder::rgbToXyz(Vec<float> rgb) {
  Vec<float> xyz;

  if(rgb.x > 0.04045f) {
    rgb.x = pow(((rgb.x + 0.055f) / 1.055f), 2.4f);
  } else {
    rgb.x = rgb.x / 12.92f;
  }
  if(rgb.y > 0.04045f) {
    rgb.y = pow(((rgb.y + 0.055f) / 1.055f), 2.4f);
  } else {
    rgb.y = rgb.y / 12.92f;
  }
  if(rgb.z > 0.04045f) {
    rgb.z = pow(((rgb.z + 0.055f) / 1.055f), 2.4f);
  } else {
    rgb.z = rgb.z / 12.92f;
  }

  rgb.x = rgb.x * 100.0f;
  rgb.y = rgb.y * 100.0f;
  rgb.z = rgb.z * 100.0f;

  xyz.x = rgb.x * 0.4124564f + rgb.y * 0.3575761f + rgb.z * 0.1804375f;
  xyz.y = rgb.x * 0.2126729f + rgb.y * 0.7151522f + rgb.z * 0.0721750f;
  xyz.z = rgb.x * 0.0193339f + rgb.y * 0.1191920f + rgb.z * 0.9503041f;

  return xyz;
}

Vec<float> const Encoder::rgbToCieLab(Vec<float> const& rgb) {
  Vec<float> lab;
  Vec<float> xyz = rgbToXyz(rgb);

  Vec<float> XYZ_D50_2 = Vec<float>(96.422f, 100.0f, 82.521f); //D50 Standard; 2° ViewAngle
  Vec<float> XYZ_D65_2 = Vec<float>(95.047f, 100.0f, 108.883f);
  Vec<float> XYZ_D50_10 = Vec<float>(96.720f, 100.0f, 81.427f);
  Vec<float> XYZ_D65_10 = Vec<float>(94.811f, 100.0f, 107.304f);

  lab.x = 116.0f * cbrt(xyz.y / XYZ_D65_2.y) - 16;
  lab.y = 500.0f * (cbrt(xyz.x / XYZ_D65_2.x) - cbrt(xyz.y / XYZ_D65_2.y));
  lab.z = 200.0f * (cbrt(xyz.y / XYZ_D65_2.y) - cbrt(xyz.z / XYZ_D65_2.z));

  return lab;
}

Vec<float> const Encoder::bit8ToRgb(const unsigned char from[4])
{
    return Vec<float>(
        from[0] / 255.0f,
        from[1] / 255.0f,
        from[2] / 255.0f
    );
}

const Vec<uint64_t> Encoder::mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<uint8_t>& bits)
{
    Vec<uint64_t> res;
    res.x = mapToBit(from.x, bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit(from.y, bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit(from.z, bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<uint64_t> Encoder::mapVec(const unsigned char from[4], BoundingBox const& bb, const Vec<uint8_t>& bits)
{
    Vec<uint64_t> res;
    res.x = mapToBit((uint64_t) from[1], bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit((uint64_t) from[2], bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit((uint64_t) from[3], bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<uint64_t> Encoder::mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<BitCount>& bits)
{
    Vec<uint64_t> res;
    res.x = mapToBit(from.x, bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit(from.y, bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit(from.z, bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<uint64_t> Encoder::mapVec(const unsigned char from[4], BoundingBox const& bb, const Vec<BitCount>& bits)
{
    Vec<uint64_t> res;
    res.x = mapToBit((uint64_t) from[1], bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit((uint64_t) from[2], bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit((uint64_t) from[3], bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<float> Encoder::mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<uint8_t>& bits)
{
    Vec<float> res;
    res.x = mapFromBit((uint32_t) from.x, bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from.y, bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from.z, bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<float> Encoder::mapVecToFloat(const unsigned char from[4], BoundingBox const& bb, const Vec<uint8_t>& bits)
{
    Vec<float> res;
    res.x = mapFromBit((uint32_t) from[1], bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from[2], bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from[3], bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<float> Encoder::mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<BitCount>& bits)
{
    Vec<float> res;
    res.x = mapFromBit((uint32_t) from.x, bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from.y, bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from.z, bb.min.z, bb.max.z, bits.z);
    return res;
}

const Vec<float> Encoder::mapVecToFloat(const unsigned char from[4], BoundingBox const& bb, const Vec<BitCount>& bits)
{
    Vec<float> res;
    res.x = mapFromBit((uint32_t) from[1], bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from[2], bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from[3], bb.min.z, bb.max.z, bits.z);
    return res;
}
