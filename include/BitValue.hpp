#ifndef LIBPCC_BITVALUE_HPP
#define LIBPCC_BITVALUE_HPP

#include <cstdlib>
#include <bitset>

/**
 * Used to denote component precision (amount of bits) in a BitValue.
 * Provided range is [1,32].
*/
enum BitCount {
    BIT_1 = 1,
    BIT_2 = 2,
    BIT_3 = 3,
    BIT_4 = 4,
    BIT_5 = 5,
    BIT_6 = 6,
    BIT_7 = 7,
    BIT_8 = 8,
    BIT_9 = 9,
    BIT_10 = 10,
    BIT_11 = 11,
    BIT_12 = 12,
    BIT_13 = 13,
    BIT_14 = 14,
    BIT_15 = 15,
    BIT_16 = 16,
    BIT_17 = 17,
    BIT_18 = 18,
    BIT_19 = 19,
    BIT_20 = 20,
    BIT_21 = 21,
    BIT_22 = 22,
    BIT_23 = 23,
    BIT_24 = 24,
    BIT_25 = 25,
    BIT_26 = 26,
    BIT_27 = 27,
    BIT_28 = 28,
    BIT_29 = 29,
    BIT_30 = 30,
    BIT_31 = 31,
    BIT_32 = 32
};

/**
 * Defines base interface for all BitValues.
 * initBitValue (Helper function) can be called for
 * generic child class instantiation.
*/
struct AbstractBitValue {
    virtual ~AbstractBitValue()
    {}

    virtual BitCount getN() const = 0;

    virtual uint64_t get() const = 0;

    virtual void set(uint64_t) = 0;

    virtual bool getBit(size_t pos) const = 0;

    virtual void setBit(size_t pos, bool val) = 0;
};

/**
 * Template type Wrapping a std::bitset of size N.
 * Offers convenience functions for accessing and setting
 * data independent from template arguments.
*/
template <BitCount N>
struct BitValue : AbstractBitValue {
    explicit BitValue(uint64_t v=0)
        : data(v)
    {}

    ~BitValue()
    {}

    BitCount getN() const override
    {
        return N;
    }

    uint64_t get() const override
    {
        return data.to_ulong();
    }

    void set(uint64_t v) override
    {
        data = std::bitset<N>(v);
    }

    bool getBit(size_t pos) const override
    {
        return data[pos];
    }

    void setBit(size_t pos, bool val) override
    {
        data[pos] = val;
    }

    std::bitset<N> data;
};

/**
 * Helper function to create child class instance
 * BitValue referenced by AbstractBitValue* v
 * with respect to given BitCount.
 * val can be supplied as an initial value for
 * created BitValue.
*/
void initBitValue(AbstractBitValue*& v, BitCount N, uint64_t val=0);

#endif //LIBPCC_BITVALUE_HPP
