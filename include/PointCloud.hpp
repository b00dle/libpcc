#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <vector>
#include <cstdint>
#include <cstdlib>

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

// BoundBox for pointcloud sample ranges

struct BoundingBox {
    explicit BoundingBox(float x_min_t=.0f, float x_max_t=.0f, float y_min_t=.0f, float y_max_t=.0f, float z_min_t=.0f, float z_max_t=.0f)
        : min(x_min_t, y_min_t, z_min_t)
        , max(x_max_t, y_max_t, z_max_t)
    {}

    BoundingBox(const Vec<float>& min_t, const Vec<float>& max_t)
        : min(min_t)
        , max(max_t)
    {}

    BoundingBox(BoundingBox const& bb) = default;

    ~BoundingBox() = default;

    bool contains(Vec<float> const& v) const
    {
        return v.x > min.x && v.x < max.x &&
            v.y > min.y && v.y < max.y &&
            v.z > min.z && v.z < max.z;
    }

    Vec<float> const calcRange() const {
        return max - min;
    }

    Vec<float> min;
    Vec<float> max;
};

// PointCloud Template struct

template <typename P, typename C>
struct PointCloud {
    explicit PointCloud(BoundingBox const& bb = BoundingBox())
        : bounding_box(bb)
        , points()
        , colors()
    {}

    ~PointCloud() = default;

    void addVoxel(P const& pos, C const& clr)
    {
        points.push_back(pos);
        colors.push_back(clr);
    }

    unsigned char* pointsData()
    {
        return (unsigned char*) points.data();
    }

    unsigned char* colorsData()
    {
        return (unsigned char*) colors.data();
    }

    unsigned size() const
    {
        return points.size();
    }

    void resize(unsigned s)
    {
        points.resize(s);
        colors.resize(s);
    } 

    void clear()
    {
        points.clear();
        colors.clear();
    }

    BoundingBox bounding_box;
    std::vector<P> points;
    std::vector<C> colors;
};

#endif // #ifndef  POINT_CLOUD_HPP