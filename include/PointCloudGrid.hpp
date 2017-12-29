#ifndef POINT_CLOUD_GRID_HPP
#define POINT_CLOUD_GRID_HPP

#include <vector>
#include "../include/PointCloud.hpp"
#include "VariantVec.hpp"

template <typename P, typename C>
struct GridCell {
    GridCell()
        : points()
        , colors()
    {}

    virtual void addVoxel(P const& pos, C const& clr)
    {
        points.push_back(pos);
        colors.push_back(clr);
    }

    unsigned size() const
    {
        return static_cast<unsigned>(points.size());
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

    std::vector<P> points;
    std::vector<C> colors;
};

template <typename P, typename C>
struct PointCloudGrid {
    explicit PointCloudGrid(Vec8 const& t_dimensions=Vec8(4,4,4), const BoundingBox& t_bb=BoundingBox())
        : dimensions(t_dimensions)
        , bounding_box(t_bb)
        , cells()
    {
        initCells();
    }

    virtual ~PointCloudGrid() {
        while(cells.size() > 0) {
            delete cells.back();
            cells.pop_back();
        }
    }

    void clear() {
        while(cells.size() > 0) {
            cells.back()->clear();
            delete cells.back();
            cells.pop_back();
        }
    }

    void resize(Vec8 const& t_dimensions) {
        if(t_dimensions == dimensions)
            return;
        clear();
        dimensions = t_dimensions;
        initCells();
    }

    Vec8 dimensions;
    BoundingBox bounding_box;
    std::vector<GridCell<P,C>*> cells;

private:
    void initCells() {
        unsigned idx_count = dimensions.x * dimensions.y * dimensions.z;
        for(unsigned i=0; i < idx_count; ++i) {
            cells.push_back(new GridCell<P, C>());
        }
    }
};

struct VariantPointCloudGrid : PointCloudGrid<VariantVec, VariantVec> {
    explicit VariantPointCloudGrid(Vec8 const& t_dimensions=Vec8(4,4,4), const BoundingBox& t_bb=BoundingBox())
        : PointCloudGrid(t_dimensions, t_bb)
    {}

    /*virtual*/ ~VariantPointCloudGrid() = default;

    /*virtual*/ void addVoxel(unsigned cell_idx, const VariantVec& pos, const VariantVec& clr) {
        if(cell_idx >= cells.size())
            return;
        if(cells[cell_idx]->size() == 0) {
            cells[cell_idx]->addVoxel(pos, clr);
        }
        else if(cells[cell_idx]->points[0].getType() == pos.getType() &&
                cells[cell_idx]->colors[0].getType() == clr.getType()) {
            cells[cell_idx]->addVoxel(pos, clr);
        }
    }

    VariantVecType getPointType(unsigned cell_idx) const {
        if(cell_idx >= cells.size())
            return NONE;
        if(cells[cell_idx]->points.size() == 0)
            return NONE;
        return cells[cell_idx]->points[0].getType();
    }

    VariantVecType getColorType(unsigned cell_idx) const {
        if(cell_idx >= cells.size())
            return NONE;
        if(cells[cell_idx]->colors.size() == 0)
            return NONE;
        return cells[cell_idx]->colors[0].getType();
    }
};

#endif //POINT_CLOUD_GRID_HPP
