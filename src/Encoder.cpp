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
    res.x = mapToBit((uint64_t) from[0], bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit((uint64_t) from[1], bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit((uint64_t) from[2], bb.min.z, bb.max.z, bits.z);
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
    res.x = mapToBit((uint64_t) from[0], bb.min.x, bb.max.x, bits.x);
    res.y = mapToBit((uint64_t) from[1], bb.min.y, bb.max.y, bits.y);
    res.z = mapToBit((uint64_t) from[2], bb.min.z, bb.max.z, bits.z);
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
    res.x = mapFromBit((uint32_t) from[0], bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from[1], bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from[2], bb.min.z, bb.max.z, bits.z);
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
    res.x = mapFromBit((uint32_t) from[0], bb.min.x, bb.max.x, bits.x);
    res.y = mapFromBit((uint32_t) from[1], bb.min.y, bb.max.y, bits.y);
    res.z = mapFromBit((uint32_t) from[2], bb.min.z, bb.max.z, bits.z);
    return res;
}