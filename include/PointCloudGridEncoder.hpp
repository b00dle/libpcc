//
// Created by basti on 14.12.17.
//

#ifndef LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
#define LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP

#include "../include/Encoder.hpp"
#include "../include/PointCloud.hpp"
#include "../include/PointCloudGrid.hpp"

#include <zmq.hpp>

#include <vector>
#include <string>


class PointCloudGridEncoder : public Encoder {

private:
    struct CellHeader {
        unsigned cell_idx; // not added to message (debug use)
        VariantValueType point_encoding; // TODO: smaller size for point_encoding
        VariantValueType color_encoding; // TODO: smaller size for color_encoding
        unsigned num_elements;
        static size_t getByteSize() {
            return 2*sizeof(unsigned)+ 2*sizeof(VariantValueType);
        }
    };

    struct GlobalHeader {
        Vec8 dimensions;
        BoundingBox bounding_box;
        unsigned num_blacklist;
        static size_t getByteSize() {
            return 3*sizeof(uint8_t) + 6*sizeof(float) + sizeof(unsigned);
        }
    };

public:
    PointCloudGridEncoder();
    ~PointCloudGridEncoder();

    /* Compresses given PointCloud and creates message from it.
     * P and C have to be primitive types to be used as TYPE
     * in Vec<TYPE>.
     * P will be position type.
     * C will be color type.
    */
    template <typename P, typename C>
    zmq::message_t encode(PointCloud<Vec32, Vec32>* point_cloud, const Vec8& grid_dimensions) {
        // Set properties for new grid
        pc_grid_->resize(grid_dimensions);
        pc_grid_->bounding_box = point_cloud->bounding_box;
        // build new grid
        buildPointCloudGrid<P, C>(point_cloud);
        return encodePointCloudGrid();
    };

    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, PointCloud<Vec32, Vec32>* point_cloud);

private:
    /* Fills pc_grid_ from given point_cloud and settings */
    template <typename P, typename C>
    void buildPointCloudGrid(PointCloud<Vec32, Vec32>* point_cloud) {
        Vec32 cell_range = pc_grid_->bounding_box.calcRange();
        Vec<float> pos_cell;
        cell_range.x /= (float) pc_grid_->dimensions.x;
        cell_range.y /= (float) pc_grid_->dimensions.y;
        cell_range.z /= (float) pc_grid_->dimensions.z;
        BoundingBox bb_cell(Vec32(0.0f,0.0f,0.0f), cell_range);
        BoundingBox bb_clr(Vec32(0.0f,0.0f,0.0f), Vec32(1.0f,1.0f,1.0f));
        Vec<P> compressed_pos, compressed_clr;
        VariantValue v_pos, v_clr;
        unsigned progress = 0, new_progress = 0;
        for(unsigned i=0; i < point_cloud->size(); ++i) {
            if (!pc_grid_->bounding_box.contains(point_cloud->points[i]))
                continue;
            unsigned cell_idx = calcGridCellIndex(point_cloud->points[i], cell_range);
            pos_cell = mapToCell(point_cloud->points[i], cell_range);
            compressed_pos = mapVec<float, P>(pos_cell, bb_cell);
            compressed_clr = mapVec<float, C>(point_cloud->colors[i], bb_clr);
            v_pos.set<P>(compressed_pos);
            v_clr.set<C>(compressed_clr);
            pc_grid_->addVoxel(cell_idx, v_pos, v_clr);
        }
    }

    /* Creates a zmq message from current point_cloud grid */
    zmq::message_t encodePointCloudGrid();

    /*
     * Helper function for encodePointCloudGrid to encode GlobalHeader
     * Returns updated offset.
    */
    size_t encodeGlobalHeader(zmq::message_t& msg, size_t offset=0);

    /*
     * Helper function for encodePointCloudGrid to encode black_list
     * Returns updated offset.
    */
    size_t encodeBlackList(zmq::message_t& msg, std::vector<unsigned> bl, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to encode a CellHeader
     * Returns updated offset.
    */
    size_t encodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to encode a GridCell
     * with variant value type.
     * Returns updated offset.
    */
    size_t encodeVariantCell(zmq::message_t& msg, GridCell<VariantValue,VariantValue>* cell, size_t offset);

    /* Calculates the index of the cell a point belongs to */
    unsigned calcGridCellIndex(const Vec32& pos, const Vec32& cell_range) const;

    /* Maps a global position into local cell coordinates. */
    const Vec<float> mapToCell(const Vec32& pos, const Vec32& cell_range);

    /* Calculates the overall message size in bytes */
    size_t calcMessageSize(const std::vector<CellHeader*>&) const;

    VariantPointCloudGrid* pc_grid_;
    GlobalHeader* header_;
};


#endif //LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
