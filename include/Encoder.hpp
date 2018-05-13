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
     
    // Color Conversion to YUV standard
    static Vec<float> const rgbToYuv(Vec<float> const& rgb);
    // Color Conversion to CIE - XYZ standard
    static Vec<float> const rgbToXyz(Vec<float> rgb);
    // Color Conversion to CIE LAB standard
    static Vec<float> const rgbToCieLab(Vec<float> const& rgb);
    // 8 bit representation to [0,1] representation
    static Vec<float> const bit8ToRgb(const unsigned char from[4]);

    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<uint64_t> mapVec(const unsigned char from[4], BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<BitCount>& bits);
    static const Vec<uint64_t> mapVec(const unsigned char from[4], BoundingBox const& bb, const Vec<BitCount>& bits);
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<float> mapVecToFloat(const unsigned char from[4], BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<BitCount>& bits);
    static const Vec<float> mapVecToFloat(const unsigned char from[4], BoundingBox const& bb, const Vec<BitCount>& bits);
};


#endif // #ifndef  LIBPCC_ENCODER_HPP
