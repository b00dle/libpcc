//
// Created by basti on 14.12.17.
//
#include "../include/VariantValue.hpp"

VariantValue::VariantValue()
    : v_float_(nullptr)
    , v_unit16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{}


VariantValue::VariantValue(const Vec<float> &value)
    : VariantValue()
{
    *this = value;
}

VariantValue::VariantValue(const Vec<uint16_t> &value)
    : VariantValue()
{
    *this = value;
}

VariantValue::VariantValue(const Vec<uint8_t> &value)
    : VariantValue()
{
    *this = value;
}

VariantValue::~VariantValue()
{
    clear();
}

void VariantValue::set(const Vec<float>& value)
{
    if(type_ != VEC_FLOAT) {
        clear();
        type_ = VEC_FLOAT;
        v_float_ = new Vec<float>();
    }
    *v_float_ = value;
}

void VariantValue::set(const Vec<uint16_t>& value)
{
    if(type_ != VEC_UINT16) {
        clear();
        type_ = VEC_UINT16;
        v_unit16_ = new Vec<uint16_t>();
    }
    *v_unit16_ = value;
}

void VariantValue::set(const Vec<uint8_t>& value)
{
    if(type_ != VEC_UINT8) {
        clear();
        type_ = VEC_UINT8;
        v_uint8_ = new Vec<uint8_t>();
    }
    *v_uint8_  = value;
}

void VariantValue::operator=(const Vec<float>& value)
{
    set(value);
}

void VariantValue::operator=(const Vec<uint16_t>& value)
{
    set(value);
}

void VariantValue::operator=(const Vec<uint8_t>& value)
{
    set(value);
}

const Vec<float> VariantValue::toVecFloat(bool& ok) const
{
    if(type_ != VEC_FLOAT) {
        ok = false;
        return Vec<float>();
    }
    ok = true;
    return *v_float_;
}

const Vec<uint16_t> VariantValue::toVecUInt16(bool &ok) const {
    if(type_ != VEC_UINT16) {
        ok = false;
        return Vec<uint16_t >();
    }
    ok = true;
    return *v_unit16_;
}

const Vec<uint8_t> VariantValue::toVecUInt8(bool& ok) const
{
    if(type_ != VEC_UINT8) {
        ok = false;
        return Vec<uint8_t >();
    }
    ok = true;
    return *v_uint8_;
}

VariantValueType VariantValue::getType() const
{
    return type_;
}

void VariantValue::clear()
{
    if(isEmpty())
        return;

    switch(type_) {
        VEC_FLOAT:
            delete v_float_;
            v_float_ = nullptr;
            break;
       VEC_UINT16:
            delete v_unit16_;
            v_unit16_ = nullptr;
            break;
        VEC_UINT8:
            delete v_uint8_;
            v_uint8_ = nullptr;
            break;
        default:
            break;
    }
}

bool VariantValue::isEmpty() const
{
    return type_ == NONE;
}

