#ifndef LIBPCC_BIT_VEC_ARRAY_HPP
#define LIBPCC_BIT_VEC_ARRAY_HPP

#include "../include/BitVec.hpp"
#include "../include/PointCloud.hpp"

#include <cmath>

/* Container storing vector of BitVec instances with similar component precisions */
class BitVecArray {
public:
    explicit BitVecArray(BitCount t_NX=BIT_8, BitCount t_NY=BIT_8, BitCount t_NZ=BIT_8);

    ~BitVecArray();

    /*
     * Returns the exact count of BITS needed to express num_elements
     * of BitVec type with given x, y & z - component precision.
    */
    static size_t getBitSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz);

    /*
     * Returns the exact count of BITS needed to express all BitVec elements
     * maintained by this instance.
    */
    size_t getBitSize() const;

    /*
     * Returns the exact count of BYTES needed to express num_elements
     * of BitVec type with given x, y & z - component precision.
    */
    static size_t getByteSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz);

    /*
     * Returns the exact count of BYTES needed to express all BitVec elements
     * maintained by this instance.
    */
    size_t getByteSize() const;

    /*
     * Returns the x-component precision uniform among all
     * BitVec elements maintained by this instance.
    */
    BitCount getNX() const;

    /*
     * Returns the y-component precision uniform among all
     * BitVec elements maintained by this instance.
    */
    BitCount getNY() const;

    /*
     * Returns the z-component precision uniform among all
     * BitVec elements maintained by this instance.
    */
    BitCount getNZ() const;

    /*
     * Sets the uniform x, y & z - component precision
     * for BitVec elements maintained by this instance.
     * Note: All existing data will be cleared if
     * any given BitCount differs from existing setting.
    */
    void init(BitCount t_NX, BitCount t_NY, BitCount t_NZ);

    /* Returns element i in data_ */
    Vec<uint64_t> const& operator[](unsigned i) const;

    /* Returns element i in data_ */
    Vec<uint64_t>& operator[](unsigned i);

    /*
     * Fills list of BitVec elements from packed_data.
     * num_elements should hold the total number of BitVec elements
     * encoded by packed_data.
    */
    void unpack(unsigned char* packed_data, size_t num_elmnts);

    /*
     * Returns a byte array of minimum size encoding all
     * BitVec elements maintained by this instance.
    */
    unsigned char* pack();

    /*
     * Appends aN element to data_.
     * Note: component values in v should be expressible by
     * x, y & z - component precision set for this instance.
    */
    void push_back(const Vec<uint64_t>& v);

    /*
     * Appends an element to data_ value set from input parameters.
     * Note: component values in v should be expressible by
     * x, y & z - component precision set for this instance.
    */
    void emplace_back(uint64_t x, uint64_t y, uint64_t z);


    /* Returns the current number of elements in data_. */
    unsigned size() const;

    /* Resizes the container used to store BitVec instances. */
    void resize(unsigned s);

    /* Removes all BitVec instances maintained by this instance. */
    void clear();

private:
    std::vector<Vec<uint64_t>> data_;
    BitCount nx_;
    BitCount ny_;
    BitCount nz_;
};

#endif //LIBPCC_BIT_VEC_ARRAY_HPP
