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
        VariantVecType point_encoding; // TODO: smaller size for point_encoding
        VariantVecType color_encoding; // TODO: smaller size for color_encoding
        unsigned num_elements;
        static size_t getByteSize() { // cell_idx not encoded
            return 1*sizeof(unsigned)+ 2*sizeof(VariantVecType);
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
    virtual ~PointCloudGridEncoder();

    /* Compresses given PointCloud and creates message from it.
     * P and C have to be primitive types to be used as TYPE
     * in Vec<TYPE>.
     * P will be position type.
     * C will be color type.
    */
    template <typename P, typename C>
    zmq::message_t encode(PointCloud<Vec<float>, Vec<float>>* point_cloud, const Vec8& grid_dimensions) {
        // Set properties for new grid
        pc_grid_->resize(grid_dimensions);
        pc_grid_->bounding_box = point_cloud->bounding_box;
        // build new grid
        buildPointCloudGrid<P, C>(point_cloud);
        return encodePointCloudGrid();
    };

    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, PointCloud<Vec<float>, Vec<float>>* point_cloud);

private:
    /* Fills pc_grid_ from given point_cloud and settings */
    template <typename P, typename C>
    void buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud) {
        Vec<float> cell_range = pc_grid_->bounding_box.calcRange();
        Vec<float> pos_cell;
        cell_range.x /= (float) pc_grid_->dimensions.x;
        cell_range.y /= (float) pc_grid_->dimensions.y;
        cell_range.z /= (float) pc_grid_->dimensions.z;
        std::cout << "CELLS encode\n  > RANGE " << cell_range.x << "," << cell_range.y << "," << cell_range.z << std::endl;
        BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
        BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));
        Vec<P> compressed_pos;
        Vec<C> compressed_clr;
        VariantVec v_pos, v_clr;
        unsigned progress = 0, new_progress = 0;
        std::cout << "  > CELL POSITIONS\n";
        for(unsigned i=0; i < point_cloud->size(); ++i) {
            if (!pc_grid_->bounding_box.contains(point_cloud->points[i]))
                continue;
            unsigned cell_idx = calcGridCellIndex(point_cloud->points[i], cell_range);
            pos_cell = mapToCell(point_cloud->points[i], cell_range);
            std::cout << "    ===\n";
            std::cout << "    > global " << point_cloud->points[i].x << "," << point_cloud->points[i].y << "," << point_cloud->points[i].z << std::endl;
            std::cout << "    > local " << pos_cell.x << "," << pos_cell.y << "," << pos_cell.z << std::endl;
            // TODO: Handle float type for pos and clr as DST type
            compressed_pos = mapVec<float, P>(pos_cell, bb_cell);
            std::cout << "    > mapped " << (int) compressed_pos.x << "," << (int) compressed_pos.y << "," << (int) compressed_pos.z << std::endl;
            Vec<float> test = mapVecToFloat(compressed_pos, bb_cell);
            std::cout << "    > local (back) " << test.x << "," << test.y << "," << test.z << std::endl;
            compressed_clr = mapVec<float, C>(point_cloud->colors[i], bb_clr);
            v_pos.set<P>(compressed_pos);
            v_clr.set<C>(compressed_clr);
            pc_grid_->addVoxel(cell_idx, v_pos, v_clr);
        }
    }

    /*
     * Extracts a PointCloud from pc_grid_.
     * Results are stored in pc parameter.
     * Returns success of operation.
    */
    bool extractPointCloudFromGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud);

    /* Creates a zmq message from current point_cloud grid */
    zmq::message_t encodePointCloudGrid();

    /*
     * Helper function for decode, to extract a VariantPointCloudGrid
     * from given zmq message, into pc_grid_.
     * Returns success of operation.
 *  */
    bool decodePointCloudGrid(zmq::message_t& msg);

    /*
     * Helper function for encodePointCloudGrid to encode GlobalHeader
     * Returns updated offset.
    */
    size_t encodeGlobalHeader(zmq::message_t& msg, size_t offset=0);

    /*
     * Helper function for decode to extract GlobalHeader
     * Returns updated offset.
    */
    size_t decodeGlobalHeader(zmq::message_t& msg, size_t offset=0);

    /*
     * Helper function for encodePointCloudGrid to encode black_list
     * Returns updated offset.
    */
    size_t encodeBlackList(zmq::message_t& msg, std::vector<unsigned> bl, size_t offset);

    /*
     * Helper function for decodePointCloudGrid to decode black_list
     * Returns updated offset.
    */
    size_t decodeBlackList(zmq::message_t& msg, std::vector<unsigned>& bl, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to encode a CellHeader
     * Returns updated offset.
    */
    size_t encodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to decode a CellHeader
     * Returns updated offset.
    */
    size_t decodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to encode a GridCell
     * with variant value type.
     * Returns updated offset.
    */
    size_t encodeVariantCell(zmq::message_t& msg, GridCell<VariantVec,VariantVec>* cell, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to decode a GridCell
     * Cell properties are read from given CellHeader and result
     * is stored in pc_grid_.
     * Returns updated offset.
    */
    size_t decodeVariantCell(zmq::message_t& msg, CellHeader* c_header, size_t offset);

    /* Calculates the index of the cell a point belongs to */
    unsigned calcGridCellIndex(const Vec<float>& pos, const Vec<float>& cell_range) const;

    /* Maps a global position into local cell coordinates. */
    const Vec<float> mapToCell(const Vec<float>& pos, const Vec<float>& cell_range);

    /* Calculates the overall message size in bytes */
    size_t calcMessageSize(const std::vector<CellHeader*>&) const;

    VariantPointCloudGrid* pc_grid_;
    GlobalHeader* header_;
};


#endif //LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
