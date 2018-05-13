#ifndef LIBPCC_POINT_CLOUD_GRID_HPP
#define LIBPCC_POINT_CLOUD_GRID_HPP

#include "BitVecArray.hpp"

/**
 * Data transfer object defining a cell within a PointCloudGrid.
 * a BitVecArray is used as the container type for both points and colors.
 * Convenience functions are provided for initialization of
 * point and color component bit counts and basic container interaction.
*/
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

/**
 * Data transfer object defining a point cloud grid as used by
 * PointCloudGridEncoder to compress a point cloud.
 * Position and color values are assigned to GridCells.
 * Convenience functions are given for resizing of the grid
 * and basic container interaction.
*/
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
        if(t_dimensions == dimensions) {
            clear();
            return;
        }
        deleteCells();
        dimensions = t_dimensions;
        initCells();
    }

    GridCell* operator[](unsigned cell_idx) {
        return cells[cell_idx];
    }

    Vec<float> const getQuantizationStepSize(int cell_idx) const
    {
        if(cell_idx >= cells.size() || cell_idx < 0) {
            std::cout << "NOTIFICATION: invalid cell_idx for call to getQuantizationStepSize" << std::endl;
            std::cout << "  > got:" << cell_idx;
            std::cout << "  > valid range: [0," << cells.size()-1 << "]." << std::endl;
        }

        Vec<int> num_quant_values(
            static_cast<int>(pow(2, static_cast<int>(cells[cell_idx]->points.getNX()))),
            static_cast<int>(pow(2, static_cast<int>(cells[cell_idx]->points.getNY()))),
            static_cast<int>(pow(2, static_cast<int>(cells[cell_idx]->points.getNZ())))
        );

        Vec<float> quant_step;
        quant_step.x = ((bounding_box.max.x-bounding_box.min.x) / dimensions.x) / num_quant_values.x;
        quant_step.y = ((bounding_box.max.y-bounding_box.min.y) / dimensions.y) / num_quant_values.y;
        quant_step.z = ((bounding_box.max.z-bounding_box.min.z) / dimensions.z) / num_quant_values.z;
        return quant_step;
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

/**
 * Data transfer object to encode structural data of a PointCloudGrid.
 * Held as a member inside PointCloudGridEncoder::EncoderSettings.
*/
struct GridPrecisionDescriptor {
    explicit GridPrecisionDescriptor(const Vec8& t_dimensions=Vec8(4,4,4),
                                     const BoundingBox& t_bounding_box=BoundingBox(),
                                     const Vec<BitCount>& t_point_prec=Vec<BitCount>(BIT_4,BIT_4,BIT_4),
                                     const Vec<BitCount>& t_color_prec=Vec<BitCount>(BIT_4,BIT_4,BIT_4))
        : dimensions(t_dimensions)
        , bounding_box(t_bounding_box)
        , default_point_precision(t_point_prec)
        , default_color_precision(t_color_prec)
        , point_precision()
        , color_precision()
    {
        initCells();
    }

    void resize(const Vec8& dim) {
        point_precision.clear();
        color_precision.clear();
        dimensions = dim;
        initCells();
    }

    Vec8 dimensions;
    BoundingBox bounding_box;
    Vec<BitCount> default_point_precision;
    Vec<BitCount> default_color_precision;
    std::vector<Vec<BitCount>> point_precision;
    std::vector<Vec<BitCount>> color_precision;
private:
    void initCells() {
        unsigned idx_count = dimensions.x * dimensions.y * dimensions.z;
        point_precision.resize(idx_count, default_point_precision);
        color_precision.resize(idx_count, default_color_precision);
    }
};

#endif //LIBPCC_POINT_CLOUD_GRID_HPP
