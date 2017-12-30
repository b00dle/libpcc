//
// Created by basti on 14.12.17.
//

#ifndef LIBPCC_POINT_CLOUD_BIT_GRID_ENCODER_HPP
#define LIBPCC_POINT_CLOUD_BIT_GRID_ENCODER_HPP

#include "../include/Encoder.hpp"
#include "../include/PointCloud.hpp"
#include "../include/PointCloudGrid.hpp"
#include "../include/BitVecPointCloudGrid.hpp"

#include <zmq.hpp>

#include <vector>
#include <string>
#include <sstream>

class PointCloudBitGridEncoder : public Encoder {

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
        ComponentPrecision point_encoding;
        ComponentPrecision color_encoding;
        unsigned num_elements;
        static size_t getByteSize() { // cell_idx not encoded
            return 1*sizeof(unsigned)+ 2*sizeof(ComponentPrecision);
        }
        const std::string toString() const {
            std::stringstream ss;
            ss << "CellHeader(c_idx=" << cell_idx << ", ";
            ss << "p_enc=" << point_encoding << ", ";
            ss << "c_enc=" << color_encoding << ", ";
            ss << "num_elmts=" << num_elements << ")";
            return ss.str();
        }
    };

public:
    PointCloudBitGridEncoder();
    virtual ~PointCloudBitGridEncoder();

    /*
     * Compresses given PointCloud and creates message from it.
     * M_P and M_C are the maximum precision used to
     * encode components of position (M_P) and color (M_C) in bits.
    */
    template <ComponentPrecision M_P, ComponentPrecision M_C>
    zmq::message_t encode(PointCloud<Vec<float>, Vec<float>>* point_cloud, const Vec8& grid_dimensions) {
        // Set properties for new grid
        pc_grid_->resize(grid_dimensions);
        pc_grid_->bounding_box = point_cloud->bounding_box;
        buildPointCloudGrid<M_P, M_C>(point_cloud);
        return encodePointCloudGrid();
    };

    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, PointCloud<Vec<float>, Vec<float>>* point_cloud);

private:
    /* Fills pc_grid_ from given point_cloud and settings */
    template <ComponentPrecision M_P, ComponentPrecision M_C>
    void buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud) {
        // init all cells to default ComponentPrecision
        // TODO: make adaptive
        for(auto c : pc_grid_->cells) {
            c->initPoints<M_P, M_P, M_P>();
            c->initColors<M_C, M_C, M_C>();
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
        VariantVec v_pos, v_clr;
        Vec<uint8_t> p_bits(M_P, M_P, M_P);
        Vec<uint8_t> c_bits(M_C, M_C, M_C);
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
     * Helper function for encodePointCloudGrid to encode a BitVecGridCell
     * Returns updated offset.
    */
    size_t encodeCell(zmq::message_t &msg, BitVecGridCell* cell, size_t offset);

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

    /* Determines the ComponentPrecision of given AbstractBitVecArray */
    ComponentPrecision getComponentPrecision(AbstractBitVecArray* arr);

    BitVecPointCloudGrid* pc_grid_;
    GlobalHeader* header_;
};


#endif //LIBPCC_POINT_CLOUD_BIT_GRID_ENCODER_HPP
