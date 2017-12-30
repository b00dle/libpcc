//
// Created by basti on 25.12.17.
//

#ifndef BITVECTOR_HPP
#define BITVECTOR_HPP

#include <cstdlib>
#include <bitset>

enum ComponentPrecision {
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

struct AbstractBitVec {
    AbstractBitVec() = default;

    virtual ~AbstractBitVec() = default;

    size_t getSize() const {
        return getNX() + getNY() + getNZ();
    }

    virtual size_t getNX() const = 0;

    virtual size_t getNY() const = 0;

    virtual size_t getNZ() const = 0;

    virtual uint64_t getX() const = 0;

    virtual uint64_t getY() const = 0;

    virtual uint64_t getZ() const = 0;

    virtual void setX(uint64_t x_t) = 0;

    virtual void setY(uint64_t y_t) = 0;

    virtual void setZ(uint64_t z_t) = 0;
};

template <size_t NX, size_t NY, size_t NZ>
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

    size_t getNX() const override {
        return NX;
    }

    size_t getNY() const override {
        return NY;
    }

    size_t getNZ() const override {
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

    const std::bitset<NX+NY+NZ> getPackedBitset() {
        std::bitset<NX+NY+NZ> packed;
        for(size_t i = 0; i < NX; ++i)
            packed[i] = x[i];
        for(size_t i = 0; i < NY; ++i)
            packed[NX+i] = y[i];
        for(size_t i = 0; i < NZ; ++i)
            packed[NX+NY+i] = x[i];
        return packed;
    }

    void setFromPackedBitset(const std::bitset<NX + NY + NZ> &packed) {
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

template <size_t N>
struct UniformBitVec : BitVec<N,N,N> {};

struct AbstractBitVecArray {
    AbstractBitVecArray() = default;

    virtual ~AbstractBitVecArray() = default;

    static size_t getByteSize(unsigned num_elmts, size_t NX, size_t NY, size_t NZ) {
        size_t bit_size = num_elmts * (NX+NY+NZ);
        return static_cast<size_t>(ceil(bit_size/8.0f));
    }

    virtual size_t getByteSize() const = 0;

    virtual size_t getNX() const = 0;

    virtual size_t getNY() const = 0;

    virtual size_t getNZ() const = 0;

    virtual Vec<uint64_t> const operator[](unsigned i) = 0;

    virtual void unpack(unsigned char* packed_data, size_t num_elmnts) = 0;

    virtual unsigned char* pack() = 0;

    virtual void push_back(const Vec<uint64_t>&) = 0;

    virtual unsigned size() const = 0;

    virtual void resize(unsigned s) = 0;

    virtual void clear() = 0;
};

template <size_t NX, size_t NY, size_t NZ>
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

    size_t getNX() const override {
        return NX;
    }

    size_t getNY() const override {
        return NY;
    }

    size_t getNZ() const override {
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

template <size_t N>
struct UniformBitVecArr : BitVecArray<N,N,N> {};

#endif //BITVECTOR_HPP
