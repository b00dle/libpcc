//
// Created by basti on 31.12.17.
//

#ifndef LIBPCC_BITVECARRAY_HPP
#define LIBPCC_BITVECARRAY_HPP

#include "../include/BitVec.hpp"
#include "../include/PointCloud.hpp"

#include <cmath>

struct BitVecArray {
    explicit BitVecArray(BitCount t_NX=BIT_8, BitCount t_NY=BIT_8, BitCount t_NZ=BIT_8);

    ~BitVecArray();

    static size_t getBitSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz);

    size_t getBitSize() const;

    static size_t getByteSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz);

    size_t getByteSize() const;

    BitCount getNX() const;

    BitCount getNY() const;

    BitCount getNZ() const;

    void init(BitCount t_NX, BitCount t_NY, BitCount t_NZ);

    Vec<uint64_t> const operator[](unsigned i);

    /*
     * Fills data from packed_data.
     * num_elements should hold the total number of BitVec elements
     * encoded by packed_data.
    */
    void unpack(unsigned char* packed_data, size_t num_elmnts);

    /* Returns a byte array of minimum size encoding data. */
    unsigned char* pack();

    void push_back(const Vec<uint64_t>& v);

    unsigned size() const;

    void resize(unsigned s);

    void clear();

private:
    std::vector<BitVec*> data;
    BitCount NX;
    BitCount NY;
    BitCount NZ;
};

#endif //LIBPCC_BITVECARRAY_HPP
