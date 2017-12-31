//
// Created by basti on 14.12.17.
//

#ifndef LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
#define LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP

#include "../include/Encoder.hpp"
#include "../include/PointCloud.hpp"
#include "PointCloudGrid.hpp"

#include <zmq.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>

class PointCloudGridEncoder : public Encoder {

private:
    struct GlobalHeader {
        Vec8 dimensions;
        BoundingBox bounding_box;
        unsigned num_blacklist;
        static size_t getByteSize() {
            return 3*sizeof(uint8_t) + 6*sizeof(float) + sizeof(unsigned);
        }
        const std::string toString() const {
            std::stringstream ss;
            ss << "GlobalHeader(dim=[" << (int) dimensions.x << "," << (int) dimensions.y << "," << (int) dimensions.z << "], ";
            ss << "bb={[" << bounding_box.min.x << "," << bounding_box.min.y << "," << bounding_box.min.z << "];";
            ss << "[" << bounding_box.max.x << "," << bounding_box.max.y << "," << bounding_box.max.z << "]}, ";
            ss << "num_bl=" << num_blacklist << ")";
            return ss.str();
        }
    };

    struct CellHeader {
        unsigned cell_idx; // not added to message (debug use)
        // TODO: pack
        BitCount point_encoding_x;
        BitCount point_encoding_y;
        BitCount point_encoding_z;
        BitCount color_encoding_x;
        BitCount color_encoding_y;
        BitCount color_encoding_z;
        unsigned num_elements;
        static size_t getByteSize() { // cell_idx not encoded
            return 1*sizeof(unsigned)+ 6*sizeof(BitCount);
        }
        const std::string toString() const {
            std::stringstream ss;
            ss << "CellHeader(c_idx=" << cell_idx << ", ";
            ss << "p_enc_x=" << point_encoding_x << ", ";
            ss << "p_enc_y=" << point_encoding_y << ", ";
            ss << "p_enc_z=" << point_encoding_z << ", ";
            ss << "c_enc_x=" << color_encoding_x << ", ";
            ss << "c_enc_y=" << color_encoding_y << ", ";
            ss << "c_enc_z=" << color_encoding_z << ", ";
            ss << "num_elmts=" << num_elements << ")";
            return ss.str();
        }
    };

public:
    PointCloudGridEncoder();
    virtual ~PointCloudGridEncoder();

    /*
     * Compresses given PointCloud and creates message from it.
     * M_P and M_C are the maximum precision used to
     * encode components of position (M_P) and color (M_C) in bits.
    */
    zmq::message_t encode(PointCloud<Vec<float>, Vec<float>>* point_cloud, const Vec8& grid_dimensions, const Vec<BitCount>& M_P, const Vec<BitCount>& M_C) {
        // Set properties for new grid
        pc_grid_->resize(grid_dimensions);
        pc_grid_->bounding_box = point_cloud->bounding_box;
        buildPointCloudGrid(point_cloud, M_P, M_C);
        return encodePointCloudGrid();
    };

    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, PointCloud<Vec<float>, Vec<float>>* point_cloud);

private:
    /* Fills pc_grid_ from given point_cloud and settings */
    void buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud, const Vec<BitCount>& M_P, const Vec<BitCount>& M_C) {
        // init all cells to default BitCount
        for(auto c : pc_grid_->cells) {
            c->initPoints(M_P.x, M_P.y, M_P.z);
            c->initColors(M_C.x, M_C.y, M_C.z);
        }
        Vec<float> cell_range = pc_grid_->bounding_box.calcRange();
        Vec<float> pos_cell;
        cell_range.x /= (float) pc_grid_->dimensions.x;
        cell_range.y /= (float) pc_grid_->dimensions.y;
        cell_range.z /= (float) pc_grid_->dimensions.z;
        BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
        BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));
        Vec<uint64_t> compressed_pos;
        Vec<uint64_t> compressed_clr;
        Vec<uint8_t> p_bits(M_P.x, M_P.y, M_P.z);
        Vec<uint8_t> c_bits(M_C.x, M_C.y, M_C.z);
        unsigned progress = 0, new_progress = 0;
        for(unsigned i=0; i < point_cloud->size(); ++i) {
            if (!pc_grid_->bounding_box.contains(point_cloud->points[i]))
                continue;
            unsigned cell_idx = calcGridCellIndex(point_cloud->points[i], cell_range);
            pos_cell = mapToCell(point_cloud->points[i], cell_range);
            compressed_pos = mapVec(pos_cell, bb_cell, p_bits);
            compressed_clr = mapVec(point_cloud->colors[i], bb_clr, c_bits);
            (*pc_grid_)[cell_idx]->addVoxel(compressed_pos, compressed_clr);
        }
        std::cout << "DONE building grid\n";
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
     * Returns updated offset.
    */
    size_t encodeCell(zmq::message_t &msg, GridCell* cell, size_t offset);

    /*
     * Helper function for encodePointCloudGrid to decode a GridCell
     * Cell properties are read from given CellHeader and result
     * is stored in pc_grid_.
     * Returns updated offset.
    */
    size_t decodeCell(zmq::message_t &msg, CellHeader *c_header, size_t offset);

    /* Calculates the index of the cell a point belongs to */
    unsigned calcGridCellIndex(const Vec<float>& pos, const Vec<float>& cell_range) const;

    /* Maps a global position into local cell coordinates. */
    const Vec<float> mapToCell(const Vec<float>& pos, const Vec<float>& cell_range);

    /* Calculates the overall message size in bytes */
    size_t calcMessageSize(const std::vector<CellHeader*>&) const;

    PointCloudGrid* pc_grid_;
    GlobalHeader* header_;
};


#endif //LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
