#ifndef LIBPCC_ENCODER_HPP
#define LIBPCC_ENCODER_HPP

#include <cmath>
#include "PointCloud.hpp"
#include "../include/BitVec.hpp"

class Encoder {

public:
    Encoder() = default;
    virtual ~Encoder() = default;

    ////////////////////////////////////////////
    ////////// STATIC MAPPING HELPERS //////////
    ////////////////////////////////////////////

    static float mapToRange(float value, float min, float max, float range_max);
    static uint32_t mapToBit(float value, float min, float max, uint8_t bits);
    static float mapFromBit(uint32_t value, float min, float max, uint8_t bits, float invalid = 0.0f);

    /*
     * Maps 'from' vec into range determined by bb and bit range per component
     * Results is returned as unsigned long to make sure values fit.
     */
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<uint8_t>& bits) {
        Vec<uint64_t> res;
        res.x = mapToBit(from.x, bb.min.x, bb.max.x, bits.x);
        res.y = mapToBit(from.y, bb.min.y, bb.max.y, bits.y);
        res.z = mapToBit(from.z, bb.min.z, bb.max.z, bits.z);
        return res;
    };

    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<uint8_t>& bits) {
        Vec<float> res;
        res.x = mapFromBit((uint32_t) from.x, bb.min.x, bb.max.x, bits.x);
        res.y = mapFromBit((uint32_t) from.y, bb.min.y, bb.max.y, bits.y);
        res.z = mapFromBit((uint32_t) from.z, bb.min.z, bb.max.z, bits.z);
        return res;
    };
    // Color Conversion to YUV standard
    static Vec<float> const rgbToYuv(Vec<float> const& rgb);
    // Color Conversion to CIE - XYZ standard
    static Vec<float> const rgbToXyz(Vec<float> rgb);
    // Color Conversion to CIE LAB standard
    static Vec<float> const rgbToCieLab(Vec<float> const& rgb);
};


#endif // #ifndef  LIBPCC_ENCODER_HPP
