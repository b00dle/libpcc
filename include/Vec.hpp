//
// Created by basti on 31.12.17.
//

#ifndef LIBPCC_VEC_HPP
#define LIBPCC_VEC_HPP

#include <cstdint>
#include <cstdlib>

/*
 * Template Class to encapsulate 3 component Vector
 * with C as component type.
*/
template <typename C>
struct Vec {
    explicit Vec(C x_t=C(), C y_t=C(), C z_t=C())
            : x(x_t)
            , y(y_t)
            , z(z_t)
    {}

    Vec(Vec<C> const& v)
            : x(v.x)
            , y(v.y)
            , z(v.z)
    {}

    static size_t getComponentSize() {
        return sizeof(C);
    }

    virtual ~Vec() = default;

    const Vec<C> operator+(const Vec<C>& rhs) const {
        return Vec<C>(x+rhs.x, y+rhs.y, z+rhs.z);
    }

    const Vec<C> operator-(const Vec<C>& rhs) const {
        return Vec<C>(x-rhs.x, y-rhs.y, z-rhs.z);
    }

    void operator+=(const Vec<C>& rhs) {
        Vec<C> temp = (*this) + rhs;
        x = temp.x;
        y = temp.y;
        z = temp.z;
    }

    void operator-=(const Vec<C>& rhs) {
        Vec<C> temp = (*this) - rhs;
        x = temp.x;
        y = temp.y;
        z = temp.z;
    }

    C x;
    C y;
    C z;
};

/*
 * Vec of uint8_t Components.
 * Defines interface for calculating a 32 bit integer key
 * unique per possible Vec<uint_8> value.
 * key() can be used for sort operations.
 * Defines basic logical == operation.
*/
struct Vec8 : Vec<uint8_t> {
    explicit Vec8(uint8_t x_t=0, uint8_t y_t=0, uint8_t z_t=0)
            : Vec<uint8_t>(x_t, y_t, z_t)
    {}

    Vec8(const Vec8& v) = default;

    explicit Vec8(const Vec<uint8_t>& v)
            : Vec<uint8_t>(v)
    {}

    /*virtual*/ ~Vec8() = default;

    uint32_t key()
    {
        uint32_t res = 0;
        res = res | x;
        res = res | y << 8;
        res = res | z << 16;
        return res;
    }

    bool operator==(const Vec8& rhs) const {
        return x == rhs.x &&
               y == rhs.y &&
               z == rhs.z;
    }
};

#endif //LIBPCC_VEC_HPP
