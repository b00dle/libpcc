//
// Created by basti on 14.12.17.
//
#include "../include/VariantValue.hpp"

VariantValue::VariantValue()
    : v_float_(nullptr)
    , v_uint16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{}


VariantValue::VariantValue(const Vec<float> &value)
    : v_float_(nullptr)
    , v_uint16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{
    this->set<float>(value);
}

VariantValue::VariantValue(const Vec<uint16_t> &value)
    : v_float_(nullptr)
    , v_uint16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{
    this->set<uint16_t>(value);
}

VariantValue::VariantValue(const Vec<uint8_t> &value)
    : v_float_(nullptr)
    , v_uint16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{
    this->set<uint8_t>(value);
}

VariantValue::VariantValue(const VariantValue &value)
    : v_float_(nullptr)
    , v_uint16_(nullptr)
    , v_uint8_(nullptr)
    , type_(NONE)
{
    set(value);
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
        v_float_ = new Vec<float>(0.0f,0.0f,0.0f);
    }
    v_float_->x = value.x;
    v_float_->y = value.y;
    v_float_->z = value.z;
}

void VariantValue::set(const Vec<uint16_t>& value)
{
    if(type_ != VEC_UINT16) {
        clear();
        type_ = VEC_UINT16;
        v_uint16_ = new Vec<uint16_t>(0,0,0);
    }
    v_uint16_->x = value.x;
    v_uint16_->y = value.y;
    v_uint16_->z = value.z;
}

void VariantValue::set(const Vec<uint8_t>& value)
{
    if(type_ != VEC_UINT8) {
        clear();
        type_ = VEC_UINT8;
        v_uint8_ = new Vec<uint8_t>(0,0,0);
    }
    v_uint8_->x = value.x;
    v_uint8_->y = value.y;
    v_uint8_->z = value.z;
}


void VariantValue::set(const VariantValue &value)
{
    bool parse_ok = false;
    switch(value.type_) {
        case VEC_FLOAT:
            set<float>(value.toVecFloat(parse_ok));
            break;
        case VEC_UINT16:
            set<uint16_t>(value.toVecUInt16(parse_ok));
            break;
        case VEC_UINT8:
            set<uint8_t>(value.toVecUInt8(parse_ok));
            break;
        default:
            break;
    }
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

void VariantValue::operator=(const VariantValue &value)
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
    return *v_uint16_;
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
        case VEC_FLOAT:
            delete v_float_;
            v_float_ = nullptr;
            break;
        case VEC_UINT16:
            delete v_uint16_;
            v_uint16_ = nullptr;
            break;
        case VEC_UINT8:
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

