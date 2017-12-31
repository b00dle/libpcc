#include "../include/BitVec.hpp"

BitVec::BitVec(uint64_t t_x, uint64_t t_y, uint64_t t_z, BitCount NX, BitCount NY, BitCount NZ)
    : x_(nullptr)
    , y_(nullptr)
    , z_(nullptr)
{
    initX(NX, t_x);
    initY(NY, t_y);
    initZ(NZ, t_z);
}

BitVec::BitVec(const std::vector<bool>& packed, BitCount NX, BitCount NY, BitCount NZ)
    : x_(nullptr)
    , y_(nullptr)
    , z_(nullptr)
{
    initX(NX);
    initY(NY);
    initZ(NZ);
    setFromPackedBitset(packed);
}

BitVec::~BitVec()
{
    delete x_;
    delete y_;
    delete z_;
}

void BitVec::initX(BitCount N, uint64_t val)
{
    initBitValue(x_, N, val);
}

void BitVec::initY(BitCount N, uint64_t val)
{
    initBitValue(y_, N, val);
}

void BitVec::initZ(BitCount N, uint64_t val)
{
    initBitValue(z_, N, val);
}

BitCount BitVec::getNX() const
{
    return x_->getN();
}

BitCount BitVec::getNY() const
{
    return y_->getN();
}

BitCount BitVec::getNZ() const
{
    return z_->getN();
}

Vec<uint64_t> const BitVec::toVecInt64()
{
    return Vec<uint64_t>(getXInt(), getYInt(), getZInt());
}

uint64_t BitVec::getXInt() const
{
    return x_->get();
}

uint64_t BitVec::getYInt() const
{
    return y_->get();
}

uint64_t BitVec::getZInt() const
{
    return z_->get();
}

void BitVec::setX(uint64_t x_t)
{
    x_->set(x_t);
}

void BitVec::setY(uint64_t y_t)
{
    y_->set(y_t);
}

void BitVec::setZ(uint64_t z_t)
{
    z_->set(z_t);
}

const std::vector<bool> BitVec::getPackedBitset() const
{
    std::vector<bool> packed;
    packed.resize(x_->getN()+y_->getN()+z_->getN());
    for(size_t i = 0; i < x_->getN(); ++i)
        packed[i] = x_->getBit(i);
    for(size_t i = 0; i < y_->getN(); ++i)
        packed[y_->getN()+i] = y_->getBit(i);
    for(size_t i = 0; i < z_->getN(); ++i)
        packed[x_->getN()+y_->getN()+i] = z_->getBit(i);
    return packed;
}

void BitVec::setFromPackedBitset(const std::vector<bool> &packed)
{
    if(packed.size() != x_->getN()+y_->getN()+z_->getN())
        return;
    for(size_t i = 0; i < x_->getN(); ++i)
        x_->setBit(i, packed[i]);
    for(size_t i = 0; i < y_->getN(); ++i)
        y_->setBit(i, packed[x_->getN()+i]);
    for(size_t i = 0; i < z_->getN(); ++i)
        z_->setBit(i, packed[x_->getN()+y_->getN()+i]);
}

const AbstractBitValue *BitVec::getX() const
{
    return x_;
}

const AbstractBitValue *BitVec::getY() const {
    return y_;
}

const AbstractBitValue *BitVec::getZ() const {
    return z_;
}
