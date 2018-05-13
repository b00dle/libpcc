#ifndef LIBPCC_VEC_HPP
#define LIBPCC_VEC_HPP

#include <cstdint>
#include <cstdlib>
#include <iostream>

/**
 * Template Data transfer object to describe a 3 component Vector
 * with C as component type.
 * Provides basic arithmetic interface and hash function.
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

    size_t hash() const {
        size_t h = 0;
        h = h | x;
        h = h | y << 8;
        h = h | z << 16;
        return h;
    }

    bool operator<(const Vec<C>& rhs) const {
        return hash() < rhs.hash();
    }

    friend std::ostream& operator<< (std::ostream &out, const Vec<C>& rhs) 
    {
        out << "[" << /*(float)*/ rhs.x << "," << /*(float)*/ rhs.y << "," << /*(float)*/ rhs.z << "]";
        return out;
    }

    C x;
    C y;
    C z;
};

/**
 * 3D vector of uint8_t component type.
 * Defines interface for calculating a 32 bit integer key
 * unique per possible Vec<uint_8> value.
 * Defines basic logical operations.
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

    bool operator==(const Vec8& rhs) const {
        return x == rhs.x &&
               y == rhs.y &&
               z == rhs.z;
    }

    bool operator!=(const Vec8& rhs) const {
        return !(*this == rhs);
    }
};

#endif //LIBPCC_VEC_HPP
