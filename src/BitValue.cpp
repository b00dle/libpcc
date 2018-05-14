#include "BitValue.hpp"

void initBitValue(AbstractBitValue*& v, BitCount N, uint64_t val) {
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