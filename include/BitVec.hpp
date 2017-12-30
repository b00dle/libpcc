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
    BIT_16 = 16
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
    if(v != nullptr && v->getN() != N) {
        delete v;
    }
    else if(v->getN() == N) {
        v->set(val);
        return;
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
    }
}

struct AbstractBitVec {
    AbstractBitVec() = default;

    virtual ~AbstractBitVec() = default;

    size_t getSize() const {
        return getNX() + getNY() + getNZ();
    }

    virtual BitCount getNX() const = 0;

    virtual BitCount getNY() const = 0;

    virtual BitCount getNZ() const = 0;

    virtual uint64_t getX() const = 0;

    virtual uint64_t getY() const = 0;

    virtual uint64_t getZ() const = 0;

    virtual void setX(uint64_t x_t) = 0;

    virtual void setY(uint64_t y_t) = 0;

    virtual void setZ(uint64_t z_t) = 0;
};

struct _BitVec : AbstractBitVec {
    explicit _BitVec(uint64_t t_x=0, uint64_t t_y=0, uint64_t t_z=0, BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8)
        : AbstractBitVec()
        , x(nullptr)
        , y(nullptr)
        , z(nullptr)
    {
        initX(NX, t_x);
        initY(NY, t_y);
        initZ(NZ, t_z);
    }

    explicit _BitVec(const std::vector<bool>& packed, BitCount NX=BIT_8, BitCount NY=BIT_8, BitCount NZ=BIT_8)
        : AbstractBitVec()
        , x(nullptr)
        , y(nullptr)
        , z(nullptr)
    {
        initX(NX);
        initY(NY);
        initZ(NZ);
        setFromPackedBitset(packed);
    }

    ~_BitVec() override
    {
        delete x;
        delete y;
        delete z;
    }

    void initX(BitCount N, uint64_t val=0) {
        initBitValue(x, N, val);
    }

    void initY(BitCount N, uint64_t val=0) {
        initBitValue(y, N, val);
    }

    void initZ(BitCount N, uint64_t val=0) {
        initBitValue(z, N, val);
    }

    BitCount getNX() const override {
        return x->getN();
    }

    BitCount getNY() const override {
        return y->getN();
    }

    BitCount getNZ() const override {
        return z->getN();
    }

    Vec<uint64_t> const get() {
        return Vec<uint64_t>(getX(), getY(), getZ());
    }

    uint64_t getX() const override {
        return x->get();
    }

    uint64_t getY() const override {
        return y->get();
    }

    uint64_t getZ() const override {
        return z->get();
    }

    void setX(uint64_t x_t) override {
        x->set(x_t);
    }

    void setY(uint64_t y_t) override {
        y->set(y_t);
    }

    void setZ(uint64_t z_t) {
        z->set(z_t);
    }

    const std::vector<bool> getPackedBitset() const {
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

    void setFromPackedBitset(const std::vector<bool> &packed) {
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

struct _BitVecArray {
    _BitVecArray(BitCount t_NX=BIT_8, BitCount t_NY=BIT_8, BitCount t_NZ=BIT_8)
        : data()
        , NX(t_NX)
        , NY(t_NY)
        , NZ(t_NZ)
    {}

    ~_BitVecArray()
    {
        while(!data.empty()) {
            delete data.back();
            data.pop_back();
        }
    }

    size_t getByteSize() const
    {
        size_t bit_size = data.size() * (NX+NY+NZ);
        return static_cast<size_t>(ceil(bit_size/8.0f));
    }

    BitCount getNX() const {
        return NX;
    }

    BitCount getNY() const {
        return NY;
    }

    BitCount getNZ() const {
        return NZ;
    }

    void init(BitCount t_NX, BitCount t_NY, BitCount t_NZ) {
        clear();
        if(NX == t_NX && NY == t_NY && NZ == t_NZ)
            return;
        NX = t_NX;
        NY = t_NY;
        NZ = t_NZ;
    }

    Vec<uint64_t> const operator[](unsigned i) {
        return data[i]->get();
    }

    /*
     * Fills data from packed_data.
     * num_elements should hold the total number of BitVec elements
     * encoded by packed_data.
    */
    void unpack(unsigned char* packed_data, size_t num_elmnts) {
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
                    data[elmt_idx] = new _BitVec(elmt, NX, NY, NZ);
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
        data.push_back(new _BitVec(v.x, v.y, v.z, NX, NY, NZ));
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
    std::vector<_BitVec*> data;
    BitCount NX;
    BitCount NY;
    BitCount NZ;
};

template <BitCount NX, BitCount NY, BitCount NZ>
struct BitVec : AbstractBitVec {
    explicit BitVec(uint64_t t_x = 0, uint64_t t_y = 0, uint64_t t_z = 0)
        : AbstractBitVec()
        , x(t_x)
        , y(t_y)
        , z(t_z)
    {}

    explicit BitVec(const std::bitset<NX+NY+NZ>& packed)
        : AbstractBitVec()
        , x()
        , y()
        , z()
    {
        setFromPackedBitset(packed);
    }

    ~BitVec() override = default;

    BitCount getNX() const override {
        return NX;
    }

    BitCount getNY() const override {
        return NY;
    }

    BitCount getNZ() const override {
        return NZ;
    }

    Vec<uint64_t> const get() {
        return Vec<uint64_t>(getX(), getY(), getZ());
    }

    uint64_t getX() const override {
        return x.to_ulong();
    }

    uint64_t getY() const override {
        return y.to_ulong();
    }

    uint64_t getZ() const override {
        return z.to_ulong();
    }

    void setX(uint64_t x_t) override {
        x = std::bitset<NX>(x_t);
    }

    void setY(uint64_t y_t) override {
        y = std::bitset<NY>(y_t);
    }

    void setZ(uint64_t z_t) {
        z = std::bitset<NZ>(z_t);
    }

    const std::bitset<NX+NY+NZ> getPackedBitset() const {
        std::bitset<NX+NY+NZ> packed;
        for(size_t i = 0; i < NX; ++i)
            packed[i] = x[i];
        for(size_t i = 0; i < NY; ++i)
            packed[NX+i] = y[i];
        for(size_t i = 0; i < NZ; ++i)
            packed[NX+NY+i] = x[i];
        return packed;
    }

    void setFromPackedBitset(const std::bitset<NX+NY+NZ> &packed) {
        for(size_t i = 0; i < NX; ++i)
            x[i] = packed[i];
        for(size_t i = 0; i < NY; ++i)
            y[i] = packed[NX+i];
        for(size_t i = 0; i < NZ; ++i)
            z[i] = packed[NX+NY+i];
    }

    std::bitset<NX> x;
    std::bitset<NY> y;
    std::bitset<NZ> z;
};


template <BitCount N>
struct UniformBitVec : BitVec<N,N,N> {};

struct AbstractBitVecArray {
    AbstractBitVecArray() = default;

    virtual ~AbstractBitVecArray() = default;

    static size_t getByteSize(unsigned num_elmts, BitCount NX, BitCount NY, BitCount NZ) {
        size_t bit_size = num_elmts * (NX+NY+NZ);
        return static_cast<size_t>(ceil(bit_size/8.0f));
    }

    virtual size_t getByteSize() const = 0;

    virtual BitCount getNX() const = 0;

    virtual BitCount getNY() const = 0;

    virtual BitCount getNZ() const = 0;

    virtual Vec<uint64_t> const operator[](unsigned i) = 0;

    virtual void unpack(unsigned char* packed_data, size_t num_elmnts) = 0;

    virtual unsigned char* pack() = 0;

    virtual void push_back(const Vec<uint64_t>&) = 0;

    virtual unsigned size() const = 0;

    virtual void resize(unsigned s) = 0;

    virtual void clear() = 0;
};

template <BitCount NX, BitCount NY, BitCount NZ>
struct BitVecArray : AbstractBitVecArray {
    BitVecArray()
        : AbstractBitVecArray()
        , data()
    {}

    ~BitVecArray() override
    {}

    size_t getByteSize() const override
    {
        return AbstractBitVecArray::getByteSize(static_cast<unsigned>(data.size()), NX, NY, NZ);
    }

    BitCount getNX() const override {
        return NX;
    }

    BitCount getNY() const override {
        return NY;
    }

    BitCount getNZ() const override {
        return NZ;
    }

    Vec<uint64_t> const operator[](unsigned i) override {
        return data[i].get();
    }

    /*
     * Fills data from packed_data.
     * num_elements should hold the total number of BitVec elements
     * encoded by packed_data.
    */
    void unpack(unsigned char* packed_data, size_t num_elmnts) {
        data.clear();
        data.resize(num_elmnts);

        size_t elmt_idx = 0;
        std::bitset<NX+NY+NZ> elmt;
        size_t current_bit = 0;
        std::bitset<8> byte;
        for(size_t i = 0; i < getByteSize(); ++i) {
            byte = std::bitset<8>(static_cast<ulong>(packed_data[i]));
            for(size_t byte_idx = 0; byte_idx < byte.size(); ++byte_idx) {
                elmt[current_bit] = byte[byte_idx];
                current_bit = (current_bit + 1) % elmt.size();
                if(current_bit == 0) {
                    data[elmt_idx] = BitVec<NX,NY,NZ>(elmt);
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
                byte[bit_idx] = v.x[i];
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack y-component
            for(size_t i = 0; i < NY; ++i) {
                byte[bit_idx] = v.y[i];
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack z-component
            for(size_t i = 0; i < NZ; ++i) {
                byte[bit_idx] = v.z[i];
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

    void push_back(const Vec<uint64_t>& v) override
    {
        data.emplace_back(v.x, v.y, v.z);
    }

    unsigned size() const override
    {
        return static_cast<unsigned>(data.size());
    }

    void resize(unsigned s) override
    {
        data.resize(s);
    }

    void clear() override
    {
        data.clear();
    }

    std::vector<BitVec<NX,NY,NZ>> data;
};

template <BitCount N>
struct UniformBitVecArr : BitVecArray<N,N,N> {};

#endif //BITVECTOR_HPP
