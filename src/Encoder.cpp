#include "../include/Encoder.hpp"
#include <cassert>

uint8_t Encoder::mapTo8Bit(float value, float min, float max)
{
    if(value < min) {
        return 0;
    }
    else if(value > max) {
        return 255;
    }
    else {
        // map between 0-255;
        float range = max - min;
        value = (value - min) / range * 255;
        value = std::max(0.0f, std::min(value, 255.0f));
        return (uint8_t) value;
    }
}

float Encoder::mapTo32Bit(uint8_t value, float min, float max, float invalid)
{
    if(value == 0 || value == 255) {
        return invalid;
    }
    else {
        float range = max - min;
        auto res = (float) value;
        res = (res/255.0f)*range + min;
        return res;
    }
}

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

Vec8 const Encoder::VecFloatToVec8(Vec<float> const& v)
{
    BoundingBox bb(0.0f,1.0f,0.0f,1.0f,0.0f,1.0f);
    return VecFloatToVec8(v, bb);
}

Vec8 const Encoder::VecFloatToVec8(Vec<float> const& v, BoundingBox const& bb)
{
    Vec8 res;
    res.x = mapTo8Bit(v.x, bb.min.x, bb.max.x);
    res.y = mapTo8Bit(v.y, bb.min.y, bb.max.y);
    res.z = mapTo8Bit(v.z, bb.min.z, bb.max.z);
    return res;
}

Vec<float> const Encoder::Vec8ToVecFloat(Vec8 const& v)
{
    BoundingBox bb(0.0f,1.0f,0.0f,1.0f,0.0f,1.0f);
    return Vec8ToVecFloat(v, bb);
}

Vec<float> const Encoder::Vec8ToVecFloat(Vec8 const& v, BoundingBox const& bb)
{
    Vec<float> res;
    res.x = mapTo32Bit(v.x, bb.min.x, bb.max.x);
    res.y = mapTo32Bit(v.y, bb.min.y, bb.max.y);
    res.z = mapTo32Bit(v.z, bb.min.z, bb.max.z);
    return res;
}

uint32_t Encoder::VecFloatToUInt32(Vec<float> const& v, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits)
{
    BoundingBox bb(0.0f,1.0f,0.0f,1.0f,0.0f,1.0f);
    return VecFloatToUInt32(v, bb, x_bits, y_bits, z_bits);
}

uint32_t Encoder::VecFloatToUInt32(Vec<float> const& v, BoundingBox const& bb, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits)
{
    assert(x_bits + y_bits + z_bits == 32);
    uint32_t res = 0;
    res = res | mapToBit(v.x, bb.min.x, bb.max.x, x_bits);
    res = res | (mapToBit(v.y, bb.min.y, bb.max.y, y_bits) << x_bits);
    res = res | (mapToBit(v.z, bb.min.z, bb.max.z, z_bits) << (x_bits+y_bits));
    return res;
}

Vec<float> Encoder::UInt32ToVecFloat(uint32_t v, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits)
{
    BoundingBox bb(0.0f,1.0f,0.0f,1.0f,0.0f,1.0f);
    return UInt32ToVecFloat(v, bb, x_bits, y_bits, z_bits);
}

Vec<float> Encoder::UInt32ToVecFloat(uint32_t v, BoundingBox const& bb, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits)
{
    assert(x_bits + y_bits + z_bits == 32);

    Vec<float> res;

    uint32_t x = v & (int) (pow(2, x_bits)-1);
    v = v >> x_bits;
    uint32_t y = v & (int) (pow(2, y_bits)-1);
    v = v >> y_bits;
    uint32_t z = v & (int) (pow(2, z_bits)-1);

    res.x = mapFromBit(x, bb.min.x, bb.max.x, x_bits);
    res.y = mapFromBit(y, bb.min.y, bb.max.y, y_bits);
    res.z = mapFromBit(z, bb.min.z, bb.max.z, z_bits);

    return res;
}
