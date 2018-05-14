#ifndef LIBPCC_BIT_VEC_HPP
#define LIBPCC_BIT_VEC_HPP

#include <cstdlib>
#include <vector>
#include <bitset>

#include "BitValue.hpp"
#include "Vec.hpp"

/**
 * Three component vector class
 * providing interface to adjust component precision.
 * Convenience functions offer setting and getting values
 * from/as Vec<uint64_t>.
*/
class BitVec {
public:
    explicit BitVec(uint64_t t_x=0, uint64_t t_y=0, uint64_t t_z=0,
                    BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8);

    explicit BitVec(const std::vector<bool>& packed,
                    BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8);

    ~BitVec();

    /**
     * Sets x-component precision of this instance.
     * x value will be set to val after initialization.
    */
    void initX(BitCount N, uint64_t val=0);

    /**
     * Sets y-component precision of this instance.
     * y value will be set to val after initialization.
    */
    void initY(BitCount N, uint64_t val=0);

    /**
     * Sets z-component precision of this instance.
     * z value will be set to val after initialization.
    */
    void initZ(BitCount N, uint64_t val=0);

    /**
     * Returns x-component precision.
    */
    BitCount getNX() const;

    /**
     * Returns y-component precision.
    */
    BitCount getNY() const;

    /**
     * Returns z-component precision.
    */
    BitCount getNZ() const;

    /**
     * Returns x, y, z component vector converted to Vec<uint64_t>.
    */
    Vec<uint64_t> const toVecInt64();

    /**
     * Returns x value as uint64_t.
    */
    uint64_t getXInt() const;

    /**
     * Returns y value as uint64_t.
    */
    uint64_t getYInt() const;

    /**
     * Returns z value as uint64_t.
    */
    uint64_t getZInt() const;

    /**
     * Returns x value as const AbstractBitValue*.
     * Exposes interface to reading BITS.
     * see (AbstractBitValue).
    */
    const AbstractBitValue* getX() const;

    /**
     * Returns y value as const AbstractBitValue*.
     * Exposes interface to reading BITS.
     * see (AbstractBitValue).
    */
    const AbstractBitValue* getY() const;

    /**
     * Returns z value as const AbstractBitValue*.
     * Exposes interface to reading BITS.
     * see (AbstractBitValue).
    */
    const AbstractBitValue* getZ() const;

    /**
     * Sets x value from uint64_t.
    */
    void setX(uint64_t x_t);

    /**
     * Sets ith bit of x value.
    */
    void setX(size_t i, bool val);

    /**
     * Sets y value from uint64_t.
    */
    void setY(uint64_t y_t);

    /**
     * Sets ith bit of y value.
    */
    void setY(size_t i, bool val);

    /**
     * Sets z value from uint64_t.
    */
    void setZ(uint64_t z_t);

    /**
     * Sets ith bit of z value.
    */
    void setZ(size_t i, bool val);

    /**
     * Returns vector of bool encoded BITS,
     * retrieved by concatenating x, y, z - component BITS.
     * Returned format is [x_0,x_1,...,x_n,y_0,y_1,...,y_n,z_0,z_1,...,z_n]
    */
    const std::vector<bool> getPackedBitset() const;

    /**
     * Sets component values from vector of bool encoded BITS.
     * Number of elements has to be equal to x,y,z - component precision sum.
     * Format should be [x_0,x_1,...,x_n,y_0,y_1,...,y_n,z_0,z_1,...,z_n].
    */
    void setFromPackedBitset(const std::vector<bool> &packed);

private:
    AbstractBitValue* x_;
    AbstractBitValue* y_;
    AbstractBitValue* z_;
};

#endif //LIBPCC_BIT_VEC_HPP
