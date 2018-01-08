#ifndef LIBPCC_POINT_CLOUD_HPP
#define LIBPCC_POINT_CLOUD_HPP


#include <vector>
#include <cstdint>
#include <cstdlib>

#include "../include/Vec.hpp"
#include "../include/BoundingBox.hpp"
#include "../include/UncompressedVoxel.hpp"

/*
 * Data transfer object for template type precision based PointCloud data.
*/
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

    size_t size() const
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

#endif // #ifndef  LIBPCC_POINT_CLOUD_HPP