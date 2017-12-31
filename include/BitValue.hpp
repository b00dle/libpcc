//
// Created by basti on 31.12.17.
//

#ifndef LIBPCC_BITVALUE_HPP
#define LIBPCC_BITVALUE_HPP

#include <cstdlib>
#import <bitset>

/* Used to denote amount of bits in a BitValue.
 * Range [1,32] */
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

/*
 * Defines base interface for all template type BitValues.
 * initBitValue (Helper function) can be called for
 * generic child class instantiation.
*/
struct AbstractBitValue {
    virtual BitCount getN() const = 0;

    virtual uint64_t get() const = 0;

    virtual void set(uint64_t) = 0;

    virtual bool getBit(size_t pos) = 0;

    virtual void setBit(size_t pos, bool val) = 0;
};

/*
 * Template type BitValue.
 * Wraps a std::bitset of size N.
*/
template <BitCount N>
struct BitValue : AbstractBitValue {
    explicit BitValue(uint64_t v=0)
        : data(v)
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

    bool getBit(size_t pos) override
    {
        return data[pos];
    }

    void setBit(size_t pos, bool val) override
    {
        data[pos] = val;
    }

    std::bitset<N> data;
};

/*
 * Helper function to create child class instance
 * BitValue referenced by AbstractBitValue* v
 * with respect to given BitCount.
 * val can be supplied as an initialization value for
 * created BitValue.
*/
static void initBitValue(AbstractBitValue*& v, BitCount N, uint64_t val=0) {
    if(v != nullptr) {
        if(v->getN() == N) {
            v->set(val);
            return;
        }
        delete v;
    }

    switch(N) {
        case BIT_1:
            v = new BitValue<BIT_1>(val); break;
        case BIT_2:
            v = new BitValue<BIT_2>(val); break;
        case BIT_3:
            v = new BitValue<BIT_3>(val); break;
        case BIT_4:
            v = new BitValue<BIT_4>(val); break;
        case BIT_5:
            v = new BitValue<BIT_5>(val); break;
        case BIT_6:
            v = new BitValue<BIT_6>(val); break;
        case BIT_7:
            v = new BitValue<BIT_7>(val); break;
        case BIT_8:
            v = new BitValue<BIT_8>(val); break;
        case BIT_9:
            v = new BitValue<BIT_9>(val); break;
        case BIT_10:
            v = new BitValue<BIT_10>(val); break;
        case BIT_11:
            v = new BitValue<BIT_11>(val); break;
        case BIT_12:
            v = new BitValue<BIT_12>(val); break;
        case BIT_13:
            v = new BitValue<BIT_13>(val); break;
        case BIT_14:
            v = new BitValue<BIT_14>(val); break;
        case BIT_15:
            v = new BitValue<BIT_15>(val); break;
        case BIT_16:
            v = new BitValue<BIT_16>(val); break;
        case BIT_17:
            v = new BitValue<BIT_17>(val); break;
        case BIT_18:
            v = new BitValue<BIT_18>(val); break;
        case BIT_19:
            v = new BitValue<BIT_19>(val); break;
        case BIT_20:
            v = new BitValue<BIT_20>(val); break;
        case BIT_21:
            v = new BitValue<BIT_21>(val); break;
        case BIT_22:
            v = new BitValue<BIT_22>(val); break;
        case BIT_23:
            v = new BitValue<BIT_23>(val); break;
        case BIT_24:
            v = new BitValue<BIT_24>(val); break;
        case BIT_25:
            v = new BitValue<BIT_25>(val); break;
        case BIT_26:
            v = new BitValue<BIT_26>(val); break;
        case BIT_27:
            v = new BitValue<BIT_27>(val); break;
        case BIT_28:
            v = new BitValue<BIT_28>(val); break;
        case BIT_29:
            v = new BitValue<BIT_29>(val); break;
        case BIT_30:
            v = new BitValue<BIT_30>(val); break;
        case BIT_31:
            v = new BitValue<BIT_31>(val); break;
        case BIT_32:
            v = new BitValue<BIT_32>(val); break;
    }
}

#endif //LIBPCC_BITVALUE_HPP
