//
// Created by basti on 25.12.17.
//

#ifndef BITVECTOR_HPP
#define BITVECTOR_HPP

#include <cstdlib>
#include <bitset>

template <size_t NX, size_t NY, size_t NZ>
struct BitVec {
    explicit BitVec(uint64_t t_x = 0, uint64_t t_y = 0, uint64_t t_z = 0)
        : x(t_x)
        , y(t_y)
        , z(t_z)
    {}

    explicit BitVec(const std::bitset<NX+NY+NZ>& packed)
        : x()
        , y()
        , z()
    {
        setFromPacked(packed);
    }

    ~BitVec() = default;

    static size_t getSize() {
        return NX + NY + NZ;
    }

    static size_t getSizeX() {
        return NX;
    }

    static size_t getSizeY() {
        return NY;
    }

    static size_t getSizeZ() {
        return NZ;
    }

    const std::bitset<NX+NY+NZ> getPacked() {
        std::bitset<NX+NY+NZ> packed;
        for(size_t i = 0; i < NX; ++i)
            packed[i] = x[i];
        for(size_t i = 0; i < NY; ++i)
            packed[NX+i] = y[i];
        for(size_t i = 0; i < NZ; ++i)
            packed[NX+NY+i] = x[i];
        return packed;
    }

    void setFromPacked(const std::bitset<NX+NY+NZ>& packed) {
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

template <size_t NX, size_t NY, size_t NZ>
struct BitVecArray {
    BitVecArray()
        : data()
        , packed_data(0)
    {}

    ~BitVecArray()
    {
        if(packed_data != 0)
            delete [] packed_data;
    }

    size_t getByteSize()
    {
        size_t bit_size = (data.size() * BitVec<NX,NY,NZ>::getSize());
        return static_cast<size_t>(ceil(bit_size/8.0f));
    }

    /* Fills data from packed_data. */
    void fromPackedData(size_t num_elmnts) {
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

    /* Fills packed_data from data. */
    unsigned char* calcPackedData()
    {
        // delete old data
        if (packed_data != 0)
            delete [] packed_data;

        packed_data = new unsigned char[getByteSize()];
        std::bitset<8> byte;
        size_t current_byte = 0;
        size_t bit_idx = 0;
        // Pack all BitVec
        for(auto v: data) {
            // pack x-component
            for(size_t i = 0; i < BitVec<NX,NY,NZ>::getSizeX(); ++i) {
                byte[bit_idx] = v.x[i];
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack y-component
            for(size_t i = 0; i < BitVec<NX,NY,NZ>::getSizeY(); ++i) {
                byte[bit_idx] = v.y[i];
                bit_idx = (bit_idx + 1) % 8;
                if(bit_idx == 0) {
                    packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                    ++current_byte;
                }
            }
            // pack z-component
            for(size_t i = 0; i < BitVec<NX,NY,NZ>::getSizeZ(); ++i) {
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
                byte[bit_idx] = 0;
                bit_idx = (bit_idx + 1) % 8;
            }
            packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
        }

        return packed_data;
    }

    std::vector<BitVec<NX,NY,NZ>> data;
    unsigned char* packed_data;
};

#endif //BITVECTOR_HPP
