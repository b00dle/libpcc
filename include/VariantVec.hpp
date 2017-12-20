//
// Created by basti on 14.12.17.
//

#ifndef LIBPCC_VARIANT_VALUE_HPP
#define LIBPCC_VARIANT_VALUE_HPP

#include "../include/PointCloud.hpp"
#include <cstdint>
#include <type_traits>
#include <iostream>

enum VariantVecType {
    NONE,
    VEC_FLOAT,
    VEC_UINT16,
    VEC_UINT8,
};

class VariantVec {
public:
    VariantVec();
    explicit VariantVec(const Vec<float>& value);
    explicit VariantVec(const Vec<uint16_t>& value);
    explicit VariantVec(const Vec<uint8_t>& value);
    VariantVec(const VariantVec& value);
    ~VariantVec();

    VariantVecType getType() const;

    static size_t getByteSize(VariantVecType type) {
        switch(type) {
            case NONE:
                return 0;
            case VEC_FLOAT:
                return 3*sizeof(float);
            case VEC_UINT16:
                return 3*sizeof(uint16_t);
            case VEC_UINT8:
                return 3*sizeof(uint8_t);
            default:
                return 0;
        }
    }

    void set(const Vec<float>& value);
    void set(const Vec<uint16_t>& value);
    void set(const Vec<uint8_t>& value);
    void set(const VariantVec& value);

    VariantVec& operator=(const Vec<float>& value);
    VariantVec& operator=(const Vec<uint16_t>& value);
    VariantVec& operator=(const Vec<uint8_t>& value);
    VariantVec& operator=(const VariantVec& value);

    template <typename C>
    void set(const Vec<C>& value) {
        if (std::is_same<C, float>::value) {
            set(Vec<float>(value.x,value.y,value.z));
        } else if (std::is_same<C, uint16_t>::value) {
            set(Vec<uint16_t>(value.x,value.y,value.z));
        } else if(std::is_same<C, uint8_t >::value) {
            set(Vec<uint8_t>(value.x,value.y,value.z));
        }
    }

    const Vec<float> toVecFloat(bool& ok) const;
    const Vec<uint16_t> toVecUInt16(bool& ok) const;
    const Vec<uint8_t> toVecUInt8(bool& ok) const;

private:
    void clear();
    bool isEmpty() const;

    Vec<float>* v_float_;
    Vec<uint16_t>* v_uint16_;
    Vec<uint8_t>* v_uint8_;
    VariantVecType type_;
};

#endif //LIBPCC_VARIANT_VALUE_HPP
