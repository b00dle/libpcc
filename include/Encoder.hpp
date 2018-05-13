#ifndef LIBPCC_ENCODER_HPP
#define LIBPCC_ENCODER_HPP

#include <cmath>
#include "BoundingBox.hpp"
#include "Vec.hpp"
#include "../include/BitVec.hpp"

/**
 * Defines general static mapping & conversion functions.
 * Used as base class for PointCloudGridEncoder.
*/
class Encoder {
public:
    Encoder() = default;
    virtual ~Encoder() = default;

    /**
     * Maps given value into [min, max] range.
     * range_max defines fall back value for upper bounds clamping.
    */
    static float mapToRange(float value, float min, float max, float range_max);

    /**
     * Quantizes given value in range [min, max].
     * Quantization step size is given by 2^bits.
    */
    static uint32_t mapToBit(float value, float min, float max, uint8_t bits);

    /**
     * Extracts given value from quantization range [min, max].
     * Quantization step size is given by 2^bits.
     * invalid value can be specfied as fallback if de-quantization fails (default is 0.0f).
    */
    static float mapFromBit(uint32_t value, float min, float max, uint8_t bits,
                            float invalid = 0.0f);

    /**
     * Color Conversion to YUV standard.
     * Given rgb components should be in range [0,1].
    */
    static Vec<float> const rgbToYuv(Vec<float> const& rgb);

    /**
     * Color Conversion to XYZ standard.
     * Given rgb components should be in range [0,1].
    */
    static Vec<float> const rgbToXyz(Vec<float> rgb);

    /**
     * Color Conversion to Cielab standard.
     * Given rgb components should be in range [0,1].
    */
    static Vec<float> const rgbToCieLab(Vec<float> const& rgb);

    /**
     * Value conversion from 8 bit [0,255] rgb color to 32 bit [0,1] colors.
    */
    static Vec<float> const bit8ToRgb(const unsigned char from[4]);

    /**
     * Maps 'from' into range determined by bb and bit range per component.
     * Result is returned as unsigned long to make sure values fit.
    */
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb,
                                      const Vec<uint8_t>& bits);

    /**
     * Maps 'from' into range determined by bb and bit range per component.
     * Result is returned as unsigned long to make sure values fit.
    */
    static const Vec<uint64_t> mapVec(const unsigned char from[4], BoundingBox const& bb,
                                      const Vec<uint8_t>& bits);

    /**
     * Maps 'from' into range determined by bb and bit range per component.
     * Result is returned as unsigned long to make sure values fit.
    */
    static const Vec<uint64_t> mapVec(const Vec<float>& from, BoundingBox const& bb,
                                      const Vec<BitCount>& bits);

    /**
     * Maps 'from' into range determined by bb and bit range per component.
     * Result is returned as unsigned long to make sure values fit.
    */
    static const Vec<uint64_t> mapVec(const unsigned char from[4], BoundingBox const& bb,
                                      const Vec<BitCount>& bits);

    /**
     * Performs de-quantization on 'from' within given bounds
     * and using component precision defined by 'bits'.
     * Encoder::mapFromBit is used on every component.
    */
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb,
                                          const Vec<uint8_t>& bits);

    /**
     * Performs de-quantization on 'from' within given bounds
     * and using component precision defined by 'bits'.
     * Encoder::mapFromBit is used on every component.
    */
    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb,
                                          const Vec<BitCount>& bits);
};


#endif // #ifndef  LIBPCC_ENCODER_HPP
