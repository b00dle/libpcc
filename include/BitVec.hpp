//
// Created by basti on 25.12.17.
//

#ifndef BITVECTOR_HPP
#define BITVECTOR_HPP

#include <cstdlib>
#include <bitset>

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

struct AbstractBitValue {
    virtual BitCount getN() const = 0;

    virtual uint64_t get() const = 0;

    virtual void set(uint64_t) = 0;

    virtual bool getBit(size_t pos) = 0;

    virtual void setBit(size_t pos, bool val) = 0;
};

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

struct BitVec {
    explicit BitVec(uint64_t t_x=0, uint64_t t_y=0, uint64_t t_z=0, BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8)
        : x(nullptr)
        , y(nullptr)
        , z(nullptr)
    {
        initX(NX, t_x);
        initY(NY, t_y);
        initZ(NZ, t_z);
    }

    explicit BitVec(const std::vector<bool>& packed, BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8)
        : x(nullptr)
        , y(nullptr)
        , z(nullptr)
    {
        initX(NX);
        initY(NY);
        initZ(NZ);
        setFromPackedBitset(packed);
    }

    ~BitVec()
    {
        delete x;
        delete y;
        delete z;
    }

    void initX(BitCount N, uint64_t val=0)
    {
        initBitValue(x, N, val);
    }

    void initY(BitCount N, uint64_t val=0)
    {
        initBitValue(y, N, val);
    }

    void initZ(BitCount N, uint64_t val=0)
    {
        initBitValue(z, N, val);
    }

    BitCount getNX() const
    {
        return x->getN();
    }

    BitCount getNY() const
    {
        return y->getN();
    }

    BitCount getNZ() const
    {
        return z->getN();
    }

    Vec<uint64_t> const get()
    {
        return Vec<uint64_t>(getX(), getY(), getZ());
    }

    uint64_t getX() const
    {
        return x->get();
    }

    uint64_t getY() const
    {
        return y->get();
    }

    uint64_t getZ() const
    {
        return z->get();
    }

    void setX(uint64_t x_t)
    {
        x->set(x_t);
    }

    void setY(uint64_t y_t)
    {
        y->set(y_t);
    }

    void setZ(uint64_t z_t)
    {
        z->set(z_t);
    }

    const std::vector<bool> getPackedBitset() const
    {
        std::vector<bool> packed;
        packed.resize(x->getN()+y->getN()+z->getN());
        for(size_t i = 0; i < x->getN(); ++i)
            packed[i] = x->getBit(i);
        for(size_t i = 0; i < y->getN(); ++i)
            packed[y->getN()+i] = y->getBit(i);
        for(size_t i = 0; i < z->getN(); ++i)
            packed[x->getN()+y->getN()+i] = z->getBit(i);
        return packed;
    }

    void setFromPackedBitset(const std::vector<bool> &packed)
    {
        if(packed.size() != x->getN()+y->getN()+z->getN())
            return;
        for(size_t i = 0; i < x->getN(); ++i)
            x->setBit(i, packed[i]);
        for(size_t i = 0; i < y->getN(); ++i)
            y->setBit(i, packed[x->getN()+i]);
        for(size_t i = 0; i < z->getN(); ++i)
            z->setBit(i, packed[x->getN()+y->getN()+i]);
    }

    AbstractBitValue* x;
    AbstractBitValue* y;
    AbstractBitValue* z;
};

struct BitVecArray {
    BitVecArray(BitCount t_NX=BIT_8, BitCount t_NY=BIT_8, BitCount t_NZ=BIT_8)
        : data()
        , NX(t_NX)
        , NY(t_NY)
        , NZ(t_NZ)
    {}

    ~BitVecArray()
    {
        while(!data.empty()) {
            delete data.back();
            data.pop_back();
        }
    }

    static size_t getBitSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz)
    {
        return num_elmnts * (nx+ny+nz);
    }

    size_t getBitSize() const
    {
        return BitVecArray::getBitSize(data.size(), NX, NY, NZ);
    }

    static size_t getByteSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz)
    {
        return static_cast<size_t>(ceil(getBitSize(num_elmnts, nx, ny, nz)/8.0f));
    }

    size_t getByteSize() const
    {
        return static_cast<size_t>(ceil(getBitSize()/8.0f));
    }

    BitCount getNX() const
    {
        return NX;
    }

    BitCount getNY() const
    {
        return NY;
    }

    BitCount getNZ() const
    {
        return NZ;
    }

    void init(BitCount t_NX, BitCount t_NY, BitCount t_NZ)
    {
        clear();
        if(NX == t_NX && NY == t_NY && NZ == t_NZ)
            return;
        NX = t_NX;
        NY = t_NY;
        NZ = t_NZ;
    }

    Vec<uint64_t> const operator[](unsigned i)
    {
        return data[i]->get();
    }

    /*
     * Fills data from packed_data.
     * num_elements should hold the total number of BitVec elements
     * encoded by packed_data.
    */
    void unpack(unsigned char* packed_data, size_t num_elmnts)
    {
        data.clear();
        data.resize(num_elmnts);

        size_t elmt_idx = 0;
        std::vector<bool> elmt;
        elmt.resize(NX+NY+NZ);
        size_t current_bit = 0;
        std::bitset<8> byte;
        for(size_t i = 0; i < getByteSize(); ++i) {
            byte = std::bitset<8>(static_cast<ulong>(packed_data[i]));
            for(size_t byte_idx = 0; byte_idx < byte.size(); ++byte_idx) {
                elmt[current_bit] = byte[byte_idx];
                current_bit = (current_bit + 1) % elmt.size();
                if(current_bit == 0) {
                    data[elmt_idx] = new BitVec(elmt, NX, NY, NZ);
                    elmt_idx++;
                    if(elmt_idx == data.size())
                        break;
                }
            }
            if(elmt_idx == data.size())
                break;
        }
    }

    /* Returns a byte array of minimum size encoding data. */
    unsigned char* pack()
    {
        unsigned char* packed_data = new unsigned char[getByteSize()];
        std::bitset<8> byte;
        size_t current_byte = 0;
        size_t bit_idx = 0;
        // Pack all BitVec
        for(auto v: data) {
            // pack x-component
            for(size_t i = 0; i < NX; ++i) {
                byte[bit_idx] = v->x->getBit(i);
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack y-component
            for(size_t i = 0; i < NY; ++i) {
                byte[bit_idx] = v->y->getBit(i);
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack z-component
            for(size_t i = 0; i < NZ; ++i) {
                byte[bit_idx] = v->z->getBit(i);
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
        }

        // add padding for last byte if necessary
        if(bit_idx != 0) {
            while(bit_idx != 0) {
                byte[bit_idx] = false;
                bit_idx = (bit_idx + 1) % 8;
            }
            packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
        }

        return packed_data;
    }

    void push_back(const Vec<uint64_t>& v)
    {
        data.push_back(new BitVec(v.x, v.y, v.z, NX, NY, NZ));
    }

    unsigned size() const
    {
        return static_cast<unsigned>(data.size());
    }

    void resize(unsigned s)
    {
        data.resize(s);
    }

    void clear()
    {
        while(!data.empty()) {
            delete data.back();
            data.pop_back();
        }
    }

private:
    std::vector<BitVec*> data;
    BitCount NX;
    BitCount NY;
    BitCount NZ;
};

#endif //BITVECTOR_HPP
