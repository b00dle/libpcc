#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <vector>
#include <cstdint>

// uncompressed pointcloud

struct Vec32 {
    Vec32(float x_t=0.0f, float y_t=0.0f, float z_t=0.0f)
        : x(x_t)
        , y(y_t)
        , z(z_t)
    {}

    Vec32(Vec32 const& v)
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {}

    ~Vec32()
    {}

    float x;
    float y;
    float z;
};

struct Vec8 {
    Vec8(uint8_t x_t=0, uint8_t y_t=0, uint8_t z_t=0)
        : x(x_t)
        , y(y_t)
        , z(z_t)
    {}

    Vec8(Vec8 const& v)
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {}

    ~Vec8()
    {}

    uint32_t key()
    {
        uint32_t res = 0;
        res = res | x;
        res = res | y << 8;
        res = res | z << 16;
        return res;
    }

    uint8_t x;
    uint8_t y;
    uint8_t z;
};

// BoundBox for pointcloud sample ranges

struct BoundingBox {
    BoundingBox(float x_min_t=.0f, float x_max_t=.0f, float y_min_t=.0f, float y_max_t=.0f, float z_min_t=.0f, float z_max_t=.0f)
        : x_min(x_min_t)
        , x_max(x_max_t)
        , y_min(y_min_t)
        , y_max(y_max_t)
        , z_min(z_min_t)
        , z_max(z_max_t)
    {}

    BoundingBox(BoundingBox const& bb)
        : x_min(bb.x_min)
        , x_max(bb.x_max)
        , y_min(bb.y_min)
        , y_max(bb.y_max)
        , z_min(bb.z_min)
        , z_max(bb.z_max)
    {}

    ~BoundingBox()
    {}

    bool contains(Vec32 const& v) 
    {
        return v.x > x_min && v.x < x_max &&
            v.y > y_min && v.y < y_max &&
            v.z > z_min && v.z < z_max;
    }

    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

// PointCloud Template struct

template <typename P, typename C>
struct PointCloud {
    PointCloud(BoundingBox bb = BoundingBox())
        : bounding_box(bb)
        , points()
        , colors()
    {}

    ~PointCloud()
    {}

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