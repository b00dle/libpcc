#include "BitVecArray.hpp"

BitVecArray::BitVecArray(BitCount t_NX, BitCount t_NY, BitCount t_NZ)
        : data_()
        , nx_(t_NX)
        , ny_(t_NY)
        , nz_(t_NZ)
{}

BitVecArray::~BitVecArray()
{}

size_t BitVecArray::getBitSize(size_t num_elmnts, BitCount nx, BitCount ny, BitCount nz)
{
    return num_elmnts * (nx+ny+nz);
}

size_t BitVecArray::getBitSize() const
{
    return BitVecArray::getBitSize(data_.size(), nx_, ny_, nz_);
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
    return nx_;
}

BitCount BitVecArray::getNY() const
{
    return ny_;
}

BitCount BitVecArray::getNZ() const
{
    return nz_;
}

void BitVecArray::init(BitCount t_NX, BitCount t_NY, BitCount t_NZ)
{
    clear();
    if(nx_ == t_NX && ny_ == t_NY && nz_ == t_NZ)
        return;
    nx_ = t_NX;
    ny_ = t_NY;
    nz_ = t_NZ;
}

Vec<uint64_t> const& BitVecArray::operator[](unsigned i) const
{
    return data_[i];
}

Vec<uint64_t>& BitVecArray::operator[](unsigned i)
{
    return data_[i];
}

void BitVecArray::unpack(unsigned char* packed_data, size_t num_elmnts)
{
    data_.clear();
    data_.resize(num_elmnts);

    size_t elmt_idx = 0;
    size_t elmt_size = nx_+ny_+nz_;
    size_t current_bit = 0;
    std::bitset<8> byte;
    BitVec bit_vec(0,0,0,nx_,ny_,nz_);
    for(size_t i = 0; i < getByteSize(); ++i) {
        byte = std::bitset<8>(static_cast<ulong>(packed_data[i]));
        for(size_t byte_idx = 0; byte_idx < byte.size(); ++byte_idx) {
            if(current_bit < nx_) {
                bit_vec.setX(current_bit, byte[byte_idx]);
            }
            else if(current_bit < (size_t) nx_+ny_) {
                bit_vec.setY(current_bit-nx_, byte[byte_idx]);
            }
            else {
                bit_vec.setZ(current_bit-nx_-ny_, byte[byte_idx]);
            }
            current_bit = (current_bit + 1) % elmt_size;
            if(current_bit == 0) {
                data_[elmt_idx].x = bit_vec.getXInt();
                data_[elmt_idx].y = bit_vec.getYInt();
                data_[elmt_idx].z = bit_vec.getZInt();
                elmt_idx++;
                if(elmt_idx == data_.size())
                    break;
            }
        }
        if(elmt_idx == data_.size())
            break;
    }
}

unsigned char* BitVecArray::pack()
{
    auto* packed_data = new unsigned char[getByteSize()];
    std::bitset<8> byte;
    size_t current_byte = 0;
    size_t bit_idx = 0;
    BitVec bit_vec(0,0,0,nx_,ny_,nz_);
    // Pack all BitVec
    for(auto v: data_) {
        // convert x,y,z (uint64_t) to bit string
        bit_vec.setX(v.x);
        bit_vec.setY(v.y);
        bit_vec.setZ(v.z);
        // pack x-component
        for(size_t i = 0; i < nx_; ++i) {
            byte[bit_idx] = bit_vec.getX()->getBit(i);
            bit_idx = (bit_idx + 1) % 8;
            if(bit_idx == 0) {
                packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                ++current_byte;
            }
        }
        // pack y-component
        for(size_t i = 0; i < ny_; ++i) {
            byte[bit_idx] = bit_vec.getY()->getBit(i);
            bit_idx = (bit_idx + 1) % 8;
            if(bit_idx == 0) {
                packed_data[current_byte] = static_cast<unsigned char>(byte.to_ulong());
                ++current_byte;
            }
        }
        // pack z-component
        for(size_t i = 0; i < nz_; ++i) {
            byte[bit_idx] = bit_vec.getZ()->getBit(i);
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
    data_.push_back(v);
}

void BitVecArray::emplace_back(uint64_t x, uint64_t y, uint64_t z)
{
    data_.emplace_back(x,y,z);
}

unsigned BitVecArray::size() const
{
    return static_cast<unsigned>(data_.size());
}

void BitVecArray::resize(unsigned s)
{
    data_.resize(s);
}

void BitVecArray::clear()
{
    data_.clear();
}
