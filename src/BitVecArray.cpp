//
// Created by basti on 31.12.17.
//
#include "../include/BitVecArray.hpp"

BitVecArray::BitVecArray(BitCount t_NX, BitCount t_NY, BitCount t_NZ)
        : data()
        , NX(t_NX)
        , NY(t_NY)
        , NZ(t_NZ)
{}

BitVecArray::~BitVecArray()
{
    while(!data.empty()) {
        delete data.back();
        data.pop_back();
    }
}

size_t BitVecArray::getBitSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz)
{
    return num_elmnts * (nx+ny+nz);
}

size_t BitVecArray::getBitSize() const
{
    return BitVecArray::getBitSize(data.size(), NX, NY, NZ);
}

size_t BitVecArray::getByteSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz)
{
    return static_cast<size_t>(ceil(getBitSize(num_elmnts, nx, ny, nz)/8.0f));
}

size_t BitVecArray::getByteSize() const
{
    return static_cast<size_t>(ceil(getBitSize()/8.0f));
}

BitCount BitVecArray::getNX() const
{
    return NX;
}

BitCount BitVecArray::getNY() const
{
    return NY;
}

BitCount BitVecArray::getNZ() const
{
    return NZ;
}

void BitVecArray::init(BitCount t_NX, BitCount t_NY, BitCount t_NZ)
{
    clear();
    if(NX == t_NX && NY == t_NY && NZ == t_NZ)
        return;
    NX = t_NX;
    NY = t_NY;
    NZ = t_NZ;
}

Vec<uint64_t> const BitVecArray::operator[](unsigned i)
{
    return data[i]->get();
}

void BitVecArray::unpack(unsigned char* packed_data, size_t num_elmnts)
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

unsigned char* BitVecArray::pack()
{
    auto* packed_data = new unsigned char[getByteSize()];
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

void BitVecArray::push_back(const Vec<uint64_t>& v)
{
    data.push_back(new BitVec(v.x, v.y, v.z, NX, NY, NZ));
}

unsigned BitVecArray::size() const
{
    return static_cast<unsigned>(data.size());
}

void BitVecArray::resize(unsigned s)
{
    data.resize(s);
}

void BitVecArray::clear()
{
    while(!data.empty()) {
        delete data.back();
        data.pop_back();
    }
}
