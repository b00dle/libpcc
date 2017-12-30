//
// Created by basti on 29.12.17.
//

#ifndef LIBPCC_BIT_VEC_POINT_CLOUD_GRID_HPP
#define LIBPCC_BIT_VEC_POINT_CLOUD_GRID_HPP

#include "../include/BitVec.hpp"
#include "../include/PointCloud.hpp"

struct BitVecGridCell {
    BitVecGridCell()
        : points(nullptr)
        , colors(nullptr)
    {
        points = new BitVecArray<BIT_8,BIT_8,BIT_8>;
        colors = new BitVecArray<BIT_8,BIT_8,BIT_8>;
    }

    ~BitVecGridCell()
    {
        delete points;
        delete colors;
    }

    template<BitCount NX, BitCount NY, BitCount NZ>
    void initPoints()
    {
        if(points->getNX() != NX || points->getNY() != NY || points->getNZ() != NZ) {
            delete points;
            points = new BitVecArray<NX,NY,NZ>;
        }
        points->clear();
    }

    template<BitCount NX, BitCount NY, BitCount NZ>
    void initColors()
    {
        if(colors->getNX() != NX || colors->getNY() != NY || colors->getNZ() != NZ) {
            delete colors;
            colors = new BitVecArray<NX,NY,NZ>;
        }
        colors->clear();
    }

    void addVoxel(const Vec<uint64_t>& p, const Vec<uint64_t>& c)
    {
        points->push_back(p);
        colors->push_back(c);
    }

    unsigned size()
    {
        return points->size();
    }

    void resize(unsigned s)
    {
        points->resize(s);
        colors->resize(s);
    }

    void clear()
    {
        points->clear();
        colors->clear();
    }

    AbstractBitVecArray* points;
    AbstractBitVecArray* colors;
};

struct BitVecPointCloudGrid {
    explicit BitVecPointCloudGrid(Vec8 const& t_dimensions=Vec8(4,4,4), const BoundingBox& t_bb=BoundingBox())
        : dimensions(t_dimensions)
        , bounding_box(t_bb)
        , cells()
    {
        initCells();
    }

    ~BitVecPointCloudGrid() {
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

    BitVecGridCell* operator[](unsigned cell_idx) {
        return cells[cell_idx];
    }

    Vec8 dimensions;
    BoundingBox bounding_box;
    std::vector<BitVecGridCell*> cells;

private:
    void initCells() {
        unsigned idx_count = dimensions.x * dimensions.y * dimensions.z;
        for(unsigned i=0; i < idx_count; ++i) {
            cells.push_back(new BitVecGridCell);
        }
    }
};

#endif //LIBPCC_BIT_VEC_POINT_CLOUD_GRID_HPP
