//
// Created by basti on 31.12.17.
//

#include "../include/BitVec.hpp"

BitVec::BitVec(uint64_t t_x, uint64_t t_y, uint64_t t_z, BitCount NX, BitCount NY, BitCount NZ)
    : x(nullptr)
    , y(nullptr)
    , z(nullptr)
{
    initX(NX, t_x);
    initY(NY, t_y);
    initZ(NZ, t_z);
}

BitVec::BitVec(const std::vector<bool>& packed, BitCount NX, BitCount NY, BitCount NZ)
    : x(nullptr)
    , y(nullptr)
    , z(nullptr)
{
    initX(NX);
    initY(NY);
    initZ(NZ);
    setFromPackedBitset(packed);
}

BitVec::~BitVec()
{
    delete x;
    delete y;
    delete z;
}

void BitVec::initX(BitCount N, uint64_t val)
{
    initBitValue(x, N, val);
}

void BitVec::initY(BitCount N, uint64_t val)
{
    initBitValue(y, N, val);
}

void BitVec::initZ(BitCount N, uint64_t val)
{
    initBitValue(z, N, val);
}

BitCount BitVec::getNX() const
{
    return x->getN();
}

BitCount BitVec::getNY() const
{
    return y->getN();
}

BitCount BitVec::getNZ() const
{
    return z->getN();
}

Vec<uint64_t> const BitVec::get()
{
    return Vec<uint64_t>(getX(), getY(), getZ());
}

uint64_t BitVec::getX() const
{
    return x->get();
}

uint64_t BitVec::getY() const
{
    return y->get();
}

uint64_t BitVec::getZ() const
{
    return z->get();
}

void BitVec::setX(uint64_t x_t)
{
    x->set(x_t);
}

void BitVec::setY(uint64_t y_t)
{
    y->set(y_t);
}

void BitVec::setZ(uint64_t z_t)
{
    z->set(z_t);
}

const std::vector<bool> BitVec::getPackedBitset() const
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

void BitVec::setFromPackedBitset(const std::vector<bool> &packed)
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
