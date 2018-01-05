#ifndef LIBPCC_POINT_CLOUD_GRID_HPP
#define LIBPCC_POINT_CLOUD_GRID_HPP

#include "../include/BitVecArray.hpp"
#include "../include/PointCloud.hpp"

struct GridCell {
    GridCell() = default;

    ~GridCell() = default;

    void initPoints(BitCount NX, BitCount NY, BitCount NZ)
    {
        points.init(NX, NY, NZ);
    }

    void initColors(BitCount NX, BitCount NY, BitCount NZ)
    {
        colors.init(NX, NY, NZ);
    }

    void addVoxel(const Vec<uint64_t>& p, const Vec<uint64_t>& c)
    {
        points.emplace_back(p.x, p.y, p.z);
        colors.emplace_back(c.x, c.y, c.z);
    }

    unsigned size()
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

    BitVecArray points;
    BitVecArray colors;
};

struct PointCloudGrid {
    explicit PointCloudGrid(Vec8 const& t_dimensions=Vec8(4,4,4), const BoundingBox& t_bb=BoundingBox())
        : dimensions(t_dimensions)
        , bounding_box(t_bb)
        , cells()
    {
        initCells();
    }

    ~PointCloudGrid() {
        deleteCells();
    }

    void clear() {
        for(auto c : cells)
            c->clear();
    }

    void deleteCells() {
        while(!cells.empty()) {
            cells.back()->clear();
            delete cells.back();
            cells.pop_back();
        }
    }

    void resize(Vec8 const& t_dimensions) {
        if(t_dimensions == dimensions)
            return;
        deleteCells();
        dimensions = t_dimensions;
        initCells();
    }

    GridCell* operator[](unsigned cell_idx) {
        return cells[cell_idx];
    }

    Vec8 dimensions;
    BoundingBox bounding_box;
    std::vector<GridCell*> cells;

private:
    void initCells() {
        unsigned idx_count = dimensions.x * dimensions.y * dimensions.z;
        for(unsigned i=0; i < idx_count; ++i) {
            cells.push_back(new GridCell);
        }
    }
};

#endif //LIBPCC_POINT_CLOUD_GRID_HPP
