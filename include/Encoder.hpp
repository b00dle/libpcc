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
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb, const Vec<BitCount>& bits);
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<uint8_t>& bits);
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<BitCount>& bits);
};


#endif // #ifndef  LIBPCC_ENCODER_HPP

