//
// Created by basti on 25.12.17.
//

#ifndef LIBPCC_BIT_VEC_HPP
#define LIBPCC_BIT_VEC_HPP

#include <cstdlib>
#include <vector>
#include <bitset>

#include "../include/BitValue.hpp"
#include "../include/PointCloud.hpp"

struct BitVec {
    explicit BitVec(uint64_t t_x=0, uint64_t t_y=0, uint64_t t_z=0,
                    BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8);

    explicit BitVec(const std::vector<bool>& packed,
                    BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8);

    ~BitVec();

    void initX(BitCount N, uint64_t val=0);

    void initY(BitCount N, uint64_t val=0);

    void initZ(BitCount N, uint64_t val=0);

    BitCount getNX() const;

    BitCount getNY() const;

    BitCount getNZ() const;

    Vec<uint64_t> const get();

    uint64_t getX() const;

    uint64_t getY() const;

    uint64_t getZ() const;

    void setX(uint64_t x_t);

    void setY(uint64_t y_t);

    void setZ(uint64_t z_t);

    const std::vector<bool> getPackedBitset() const;

    void setFromPackedBitset(const std::vector<bool> &packed);

    AbstractBitValue* x;
    AbstractBitValue* y;
    AbstractBitValue* z;
};

#endif //LIBPCC_BIT_VEC_HPP
