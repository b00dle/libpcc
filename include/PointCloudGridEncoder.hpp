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

/*
 * Provides interface to point cloud compression
 * based on grid segmentation and adaptive quantization
 * of grid cells.
*/
class PointCloudGridEncoder : public Encoder {

public:
    /*
     * Data transfer object used for exposing
     * configurable settings for encoding process.
     * The value will not be changed from within this class,
     * thus it will only ever be changed by user.
    */
    struct EncodingSettings {
        EncodingSettings()
            : grid_precision()
            , num_threads(24)
        {}

        EncodingSettings(const EncodingSettings&) = default;

        GridPrecisionDescriptor grid_precision;
        int num_threads;
    };

    EncodingSettings settings;

private:
    template<typename C>
    using GridVec = std::vector<std::vector<Vec<C>>>;

    /*
     * Data transfer object for encoding first chunk in a message
     * which contains a PointCloudGrid.
     * Holds general meta info about a PointCloudGrid
    */
    struct GlobalHeader {
        Vec8 dimensions;
        BoundingBox bounding_box;
        unsigned num_blacklist;

        static size_t getByteSize()
        {
            return 3*sizeof(uint8_t) + 6*sizeof(float) + sizeof(unsigned);
        }

        const std::string toString() const
        {
            std::stringstream ss;
            ss << "GlobalHeader(dim=[" << (int) dimensions.x << "," << (int) dimensions.y << "," << (int) dimensions.z << "], ";
            ss << "bb={[" << bounding_box.min.x << "," << bounding_box.min.y << "," << bounding_box.min.z << "];";
            ss << "[" << bounding_box.max.x << "," << bounding_box.max.y << "," << bounding_box.max.z << "]}, ";
            ss << "num_bl=" << num_blacklist << ")";
            return ss.str();
        }
    };

    /*
     * Data transfer object for encoding meta info
     * about a GridCell in a PointCloudGrid.
    */
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

        static size_t getByteSize()
        { // cell_idx not encoded
            return 1*sizeof(unsigned)+6*sizeof(BitCount);
        }

        const std::string toString() const
        {
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
    explicit PointCloudGridEncoder(const EncodingSettings& s = EncodingSettings());
    ~PointCloudGridEncoder();

    /* Compresses given PointCloud and creates message from it. */
    zmq::message_t encode(PointCloud<Vec<float>, Vec<float>>* point_cloud);
    /* Compresses given UncompressedPointCloud and creates message from it. */
    zmq::message_t encode(const std::vector<UncompressedVoxel>& point_cloud);

    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, PointCloud<Vec<float>, Vec<float>>* point_cloud);
    /* Decodes given message into point_cloud. Returns success. */
    bool decode(zmq::message_t& msg, std::vector<UncompressedVoxel>* point_cloud);

private:
    /* Fills pc_grid_ from given point_cloud and settings */
    void buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud);

    /* Fills pc_grid_ from given point_cloud and settings */
    void buildPointCloudGrid(const std::vector<UncompressedVoxel>& point_cloud);

    /*
     * Extracts a PointCloud from pc_grid_.
     * Results are stored in pc parameter.
     * Returns success of operation.
    */
    bool extractPointCloudFromGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud);

    /*
     * Extracts a PointCloud from pc_grid_.
     * Results are stored in pc parameter.
     * Returns success of operation.
    */
    bool extractPointCloudFromGrid(std::vector<UncompressedVoxel>* point_cloud);

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
    size_t encodeBlackList(zmq::message_t& msg, std::vector<unsigned> bl,
                           size_t offset);

    /*
     * Helper function for decodePointCloudGrid to decode black_list
     * Returns updated offset.
    */
    size_t decodeBlackList(zmq::message_t& msg, std::vector<unsigned>& bl,
                           size_t offset);

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

    /* Calculates the index of the cell a point belongs to */
    unsigned calcGridCellIndex(const float pos[3], const Vec<float>& cell_range) const;

    /* Maps a global position into local cell coordinates. */
    const Vec<float> mapToCell(const Vec<float>& pos, const Vec<float>& cell_range);

    /* Maps a global position into local cell coordinates. */
    const Vec<float> mapToCell(const float pos[3], const Vec<float>& cell_range);

    /* Calculates the overall message size in bytes */
    size_t calcMessageSize(const std::vector<CellHeader*>&) const;

    PointCloudGrid* pc_grid_;
    GlobalHeader* header_;
};


#endif //LIBPCC_POINT_CLOUD_GRID_ENCODER_HPP
