#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <cmath>
#include "PointCloud.hpp"
#include "VariantVec.hpp"
#include "../include/BitVec.hpp"

class Encoder {

public:
    Encoder() = default;
    virtual ~Encoder() = default;

    ////////////////////////////////////////////
    ////////// STATIC MAPPING HELPERS //////////
    ////////////////////////////////////////////

    static uint8_t mapTo8Bit(float value, float min, float max);
    static float mapTo32Bit(uint8_t value, float min, float max, float invalid = 0.0f);
    static float mapToRange(float value, float min, float max, float range_max);
    static uint32_t mapToBit(float value, float min, float max, uint8_t bits);
    static float mapFromBit(uint32_t value, float min, float max, uint8_t bits, float invalid = 0.0f);

    static Vec8 const VecFloatToVec8(Vec<float> const& v);
    static Vec8 const VecFloatToVec8(Vec<float> const& v, BoundingBox const& bb);
    static Vec<float> const Vec8ToVecFloat(Vec8 const& v);
    static Vec<float> const Vec8ToVecFloat(Vec8 const& v, BoundingBox const& bb);
    static uint32_t VecFloatToUInt32(Vec<float> const& v, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits);
    static uint32_t VecFloatToUInt32(Vec<float> const& v, BoundingBox const& bb, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits);
    static Vec<float> UInt32ToVecFloat(uint32_t v, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits);
    static Vec<float> UInt32ToVecFloat(uint32_t v, BoundingBox const& bb, uint8_t x_bits, uint8_t y_bits, uint8_t z_bits);

    template<typename SRC, typename DST>
    static const Vec<DST> mapVec(const Vec<SRC>& from, BoundingBox const& bb) {
        auto bits = static_cast<uint8_t>(Vec<DST>::getComponentSize()*8);
        Vec<DST> res;
        res.x = (DST) mapToBit(from.x, bb.min.x, bb.max.x, bits);
        res.y = (DST) mapToBit(from.y, bb.min.y, bb.max.y, bits);
        res.z = (DST) mapToBit(from.z, bb.min.z, bb.max.z, bits);
        return res;
    };

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

    template <typename SRC>
    static const Vec<float> mapVecToFloat(const Vec<SRC>& from, BoundingBox const& bb) {
        auto bits = static_cast<uint8_t>(Vec<SRC>::getComponentSize()*8);
        Vec<float> res;
        res.x = mapFromBit((uint32_t) from.x, bb.min.x, bb.max.x, bits);
        res.y = mapFromBit((uint32_t) from.y, bb.min.y, bb.max.y, bits);
        res.z = mapFromBit((uint32_t) from.z, bb.min.z, bb.max.z, bits);
        return res;
    };

    static const Vec<float> mapVecToFloat(const Vec<uint64_t>& from, BoundingBox const& bb, const Vec<uint8_t>& bits) {
        Vec<float> res;
        res.x = mapFromBit((uint32_t) from.x, bb.min.x, bb.max.x, bits.x);
        res.y = mapFromBit((uint32_t) from.y, bb.min.y, bb.max.y, bits.y);
        res.z = mapFromBit((uint32_t) from.z, bb.min.z, bb.max.z, bits.z);
        return res;
    };

    static const Vec<float> mapToFloat(const VariantVec& from, BoundingBox const& bb, bool& ok) {
        // TODO: Handle float type
        switch(from.getType()) {
            case VEC_UINT8:
                return mapVecToFloat(from.toVecUInt8(ok), bb);
            case VEC_UINT16:
                return mapVecToFloat(from.toVecUInt16(ok), bb);
            default:
                ok=false;
                return Vec<float>();
        }
    }

private:
};


#endif // #ifndef  ENCODER_HPP

