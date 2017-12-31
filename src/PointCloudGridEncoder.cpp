#include "../include/PointCloudGridEncoder.hpp"
#include <set>

//
// Created by basti on 14.12.17.
//
PointCloudGridEncoder::PointCloudGridEncoder()
    : settings()
    , pc_grid_()
    , header_()
{
    pc_grid_ = new PointCloudGrid(Vec8(1,1,1));
    header_ = new GlobalHeader;
}

PointCloudGridEncoder::~PointCloudGridEncoder()
{
    delete pc_grid_;
    delete header_;
}

zmq::message_t PointCloudGridEncoder::encode(PointCloud<Vec<float>, Vec<float>>* point_cloud)
{
    // Set properties for new grid
    pc_grid_->resize(settings.grid_dimensions);
    pc_grid_->bounding_box = point_cloud->bounding_box;
    buildPointCloudGrid(point_cloud, settings.positions_precision, settings.color_precision);
    return encodePointCloudGrid();
};

bool PointCloudGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec<float>, Vec<float>> *point_cloud)
{
    if(!decodePointCloudGrid(msg))
        return false;
    std::cout << "DECODE GRID done\n";
    return extractPointCloudFromGrid(point_cloud);
}

void PointCloudGridEncoder::buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud, const Vec<BitCount>& M_P, const Vec<BitCount>& M_C) {
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

bool PointCloudGridEncoder::extractPointCloudFromGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud)
{
    // calc num total points once
    // to resize point_cloud
    unsigned num_grid_points = 0;
    for(auto cell: pc_grid_->cells)
        num_grid_points += cell->size();
    point_cloud->clear();
    point_cloud->resize(num_grid_points);
    // calc cell range for local point mapping
    Vec<float> cell_range = pc_grid_->bounding_box.calcRange();
    Vec<float> pos_cell, clr, glob_cell_min;
    cell_range.x /= (float) pc_grid_->dimensions.x;
    cell_range.y /= (float) pc_grid_->dimensions.y;
    cell_range.z /= (float) pc_grid_->dimensions.z;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));
    Vec<uint64_t> v_pos, v_clr;
    unsigned cell_idx;
    GridCell* cell;
    unsigned point_idx=0;
    Vec<uint8_t> p_bits;
    Vec<uint8_t> c_bits;
    for(unsigned x_idx=0; x_idx < pc_grid_->dimensions.x; ++x_idx) {
        for(unsigned y_idx=0; y_idx < pc_grid_->dimensions.y; ++y_idx) {
            for(unsigned z_idx=0; z_idx < pc_grid_->dimensions.z; ++z_idx) {
                cell_idx = x_idx +
                    y_idx * pc_grid_->dimensions.x +
                    z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
                cell = pc_grid_->cells[cell_idx];
                p_bits.x = cell->points.getNX();
                p_bits.y = cell->points.getNY();
                p_bits.z = cell->points.getNZ();
                c_bits.x = cell->colors.getNX();
                c_bits.y = cell->colors.getNY();
                c_bits.z = cell->colors.getNZ();
                glob_cell_min = Vec<float>(cell_range.x*x_idx, cell_range.y*y_idx, cell_range.z*z_idx);
                glob_cell_min += pc_grid_->bounding_box.min;
                for(unsigned i=0; i < cell->size(); ++i) {
                    v_pos = cell->points[i];
                    v_clr = cell->colors[i];
                    pos_cell = mapVecToFloat(v_pos, bb_cell, p_bits);
                    clr = mapVecToFloat(v_clr, bb_clr, c_bits);
                    point_cloud->points[point_idx] = pos_cell+glob_cell_min;
                    point_cloud->colors[point_idx] = clr;
                    ++point_idx;
                }
            }
        }
    }
    return point_idx == point_cloud->size();
}

zmq::message_t PointCloudGridEncoder::encodePointCloudGrid() {
    std::vector<unsigned> black_list;
    std::vector<CellHeader*> cell_headers;
    // initialize cell headers
    int total_elements = 0;
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        if(pc_grid_->cells[cell_idx]->size() == 0) {
            black_list.push_back(cell_idx);
            continue;
        }
        auto c_header = new CellHeader;
        c_header->cell_idx = cell_idx;
        // TODO extend header with precise encoding
        c_header->point_encoding_x = (*pc_grid_)[cell_idx]->points.getNX();
        c_header->point_encoding_y = (*pc_grid_)[cell_idx]->points.getNY();
        c_header->point_encoding_z = (*pc_grid_)[cell_idx]->points.getNZ();
        c_header->color_encoding_x = (*pc_grid_)[cell_idx]->colors.getNX();
        c_header->color_encoding_y = (*pc_grid_)[cell_idx]->colors.getNY();
        c_header->color_encoding_z = (*pc_grid_)[cell_idx]->colors.getNZ();
        c_header->num_elements = pc_grid_->cells[cell_idx]->size();
        cell_headers.push_back(c_header);
        total_elements += c_header->num_elements;
    }

    // fill global header
    header_->num_blacklist = static_cast<unsigned>(black_list.size());
    header_->dimensions = pc_grid_->dimensions;
    header_->bounding_box = pc_grid_->bounding_box;

    // calc overall message size and init message
    size_t message_size_bytes = calcMessageSize(cell_headers);

    zmq::message_t message(message_size_bytes);

    size_t offset = encodeGlobalHeader(message);
    offset = encodeBlackList(message, black_list, offset);

    for(CellHeader* c_header: cell_headers) {
        offset = encodeCellHeader(message, c_header, offset);
        offset = encodeCell(message, pc_grid_->cells[c_header->cell_idx], offset);
    }

    // Cleanup
    while(!cell_headers.empty()) {
        delete cell_headers.back();
        cell_headers.pop_back();
    }

    return message;
}

bool PointCloudGridEncoder::decodePointCloudGrid(zmq::message_t& msg)
{
    size_t old_offset = 0;
    size_t offset = decodeGlobalHeader(msg, 0);
    if(offset == old_offset)
        return false;

    pc_grid_->resize(header_->dimensions);
    pc_grid_->bounding_box = header_->bounding_box;

    std::vector<unsigned> black_list;
    offset = decodeBlackList(msg, black_list, offset);

    std::set<unsigned> black_set;
    for(unsigned idx : black_list)
        black_set.insert(idx);

    auto c_header = new CellHeader;
    auto num_cells = static_cast<unsigned>(pc_grid_->cells.size());
    for(unsigned c_idx = 0; c_idx < num_cells; ++c_idx) {
        if(black_set.find(c_idx) != black_set.end())
            continue;
        c_header->cell_idx = c_idx;
        // extract cell_header
        old_offset = offset;
        offset = decodeCellHeader(msg, c_header, offset);
        if(offset == old_offset)
            return false;
        // extract cell using cell_header
        offset = decodeCell(msg, c_header, offset);
    }
    delete c_header;

    return true;
}

size_t PointCloudGridEncoder::encodeGlobalHeader(zmq::message_t &msg, size_t offset) {
    auto dim = new unsigned char[3];
    size_t bytes_dim_size(3 * sizeof(unsigned char));
    dim[0] = header_->dimensions.x;
    dim[1] = header_->dimensions.y;
    dim[2] = header_->dimensions.z;
    memcpy((unsigned char*) msg.data() + offset, dim, bytes_dim_size);
    offset += bytes_dim_size;

    auto bb = new float[6];
    size_t bytes_bb_size(6 * sizeof(float));
    bb[0] = header_->bounding_box.min.x;
    bb[1] = header_->bounding_box.min.y;
    bb[2] = header_->bounding_box.min.z;
    bb[3] = header_->bounding_box.max.x;
    bb[4] = header_->bounding_box.max.y;
    bb[5] = header_->bounding_box.max.z;

    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) bb, bytes_bb_size);
    offset += bytes_bb_size;

    auto num_blacklist = new unsigned[1];
    size_t bytes_num_bl_size = sizeof(unsigned);
    num_blacklist[0] = header_->num_blacklist;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) num_blacklist, bytes_num_bl_size);
    offset += bytes_num_bl_size;

    // cleanup
    delete [] dim;
    delete [] bb;
    delete [] num_blacklist;
    return offset;
}

size_t PointCloudGridEncoder::decodeGlobalHeader(zmq::message_t& msg, size_t offset)
{
    auto dim = new unsigned char[3];
    size_t bytes_dim_size(3 * sizeof(unsigned char));
    memcpy(dim, (unsigned char*) msg.data() + offset, bytes_dim_size);
    header_->dimensions.x = dim[0];
    header_->dimensions.y = dim[1];
    header_->dimensions.z = dim[2];
    offset += bytes_dim_size;

    auto bb = new float[6];
    size_t bytes_bb(6 * sizeof(float));
    memcpy((unsigned char*) bb, (unsigned char*) msg.data() + offset, bytes_bb);
    header_->bounding_box.min.x = bb[0];
    header_->bounding_box.min.y = bb[1];
    header_->bounding_box.min.z = bb[2];
    header_->bounding_box.max.x = bb[3];
    header_->bounding_box.max.y = bb[4];
    header_->bounding_box.max.z = bb[5];
    offset += bytes_bb;

    auto num_blacklist = new unsigned[1];
    size_t bytes_num_bl(sizeof(unsigned));
    memcpy((unsigned char*) num_blacklist, (unsigned char*) msg.data() + offset, bytes_num_bl);
    header_->num_blacklist = num_blacklist[0];
    offset += bytes_num_bl;

    // cleanup
    delete [] dim;
    delete [] bb;
    delete [] num_blacklist;
    return offset;
}

size_t PointCloudGridEncoder::encodeBlackList(zmq::message_t &msg, std::vector<unsigned> bl, size_t offset) {
    auto black_list = new unsigned[bl.size()];
    size_t bytes_bl(bl.size() * sizeof(unsigned));
    unsigned i=0;
    for (unsigned elmt: bl) {
        black_list[i] = elmt;
        ++i;
    }
    memcpy((unsigned char*) msg.data() + offset,(unsigned char*) black_list, bytes_bl);
    offset += bytes_bl;

    // cleanup
    delete [] black_list;
    return offset;
}

size_t PointCloudGridEncoder::decodeBlackList(zmq::message_t &msg, std::vector<unsigned>& bl, size_t offset) {
    bl.resize(header_->num_blacklist);

    auto black_list = new unsigned[bl.size()];
    size_t bytes_bl(bl.size() * sizeof(unsigned));
    memcpy((unsigned char*) black_list, (unsigned char*) msg.data() + offset, bytes_bl);
    for (unsigned i = 0; i < header_->num_blacklist; ++i)
        bl[i] = black_list[i];
    offset += bytes_bl;

    // cleanup
    delete [] black_list;
    return offset;
}

size_t PointCloudGridEncoder::encodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    auto num_elmts = new unsigned[1];
    size_t bytes_num_elmts(sizeof(unsigned));
    num_elmts[0] = c_header->num_elements;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) num_elmts , bytes_num_elmts);
    offset += bytes_num_elmts;

    auto encoding = new BitCount[6];
    size_t bytes_enc(6*sizeof(BitCount));
    encoding[0] = c_header->point_encoding_x;
    encoding[1] = c_header->point_encoding_y;
    encoding[2] = c_header->point_encoding_z;
    encoding[3] = c_header->color_encoding_x;
    encoding[4] = c_header->color_encoding_y;
    encoding[5] = c_header->color_encoding_z;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) encoding, bytes_enc);
    offset += bytes_enc;

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudGridEncoder::decodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    auto num_elmts = new unsigned[1];
    size_t bytes_num_elmts(sizeof(unsigned));
    memcpy((unsigned char*) num_elmts, (unsigned char*) msg.data() + offset, bytes_num_elmts);
    c_header->num_elements = num_elmts[0];
    offset += bytes_num_elmts;

    auto encoding = new BitCount[6];
    size_t bytes_enc(6*sizeof(BitCount));
    memcpy((unsigned char*) encoding, (unsigned char*) msg.data() + offset, bytes_enc);
    c_header->point_encoding_x = encoding[0];
    c_header->point_encoding_y = encoding[1];
    c_header->point_encoding_z = encoding[2];
    c_header->color_encoding_x = encoding[3];
    c_header->color_encoding_y = encoding[4];
    c_header->color_encoding_z = encoding[5];
    offset += bytes_enc;

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudGridEncoder::encodeCell(zmq::message_t &msg, GridCell* cell, size_t offset)
{
    if(cell->size() == 0)
        return offset;

    // pack positions
    unsigned char* p_packed = cell->points.pack();
    size_t bytes_p(cell->points.getByteSize());
    memcpy((unsigned char*) msg.data() + offset, p_packed, bytes_p);
    offset += bytes_p;

    // pack colors
    unsigned char* c_packed = cell->colors.pack();
    size_t bytes_c(cell->colors.getByteSize());
    memcpy((unsigned char*) msg.data() + offset, c_packed, bytes_c);
    offset += bytes_c;

    delete [] p_packed;
    delete [] c_packed;

    return offset;
}

size_t PointCloudGridEncoder::decodeCell(zmq::message_t &msg, CellHeader *c_header, size_t offset)
{
    if(c_header->num_elements == 0)
        return offset;

    // TODO precise encoding
    GridCell* cell = pc_grid_->cells[c_header->cell_idx];

    // set BitCount and element count for position data
    cell->initPoints(
        c_header->point_encoding_x,
        c_header->point_encoding_y,
        c_header->point_encoding_z
    );
    cell->points.resize(c_header->num_elements);

    // set BitCount and element count for color data
    cell->initColors(
            c_header->color_encoding_x,
            c_header->color_encoding_y,
            c_header->color_encoding_z
    );
    cell->colors.resize(c_header->num_elements);

    // extract position data
    size_t bytes_p(cell->points.getByteSize());
    auto p_arr = new unsigned char[bytes_p];
    memcpy(p_arr, (unsigned char*) msg.data() + offset, bytes_p);
    offset += bytes_p;
    cell->points.unpack(p_arr, c_header->num_elements);

    // extract color data
    size_t bytes_c(cell->colors.getByteSize());
    auto c_arr = new unsigned char[bytes_c];
    memcpy(c_arr, (unsigned char*) msg.data() + offset, bytes_c);
    offset += bytes_c;
    cell->colors.unpack(c_arr, c_header->num_elements);

    // cleanup
    delete [] p_arr;
    delete [] c_arr;

    return offset;
}


unsigned PointCloudGridEncoder::calcGridCellIndex(const Vec<float> &pos, const Vec<float>& cell_range) const {
    Vec<float> temp(pos);
    temp -= pc_grid_->bounding_box.min;
    auto x_idx = static_cast<unsigned>(floor(static_cast<double>(temp.x / cell_range.x)));
    auto y_idx = static_cast<unsigned>(floor(static_cast<double>(temp.y / cell_range.y)));
    auto z_idx = static_cast<unsigned>(floor(static_cast<double>(temp.z / cell_range.z)));
    return x_idx +
        y_idx * pc_grid_->dimensions.x +
        z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
}

const Vec<float> PointCloudGridEncoder::mapToCell(const Vec<float> &pos, const Vec<float> &cell_range)
{
    Vec<float> cell_pos(pos.x, pos.y, pos.z);
    cell_pos -= pc_grid_->bounding_box.min;
    float x_steps = cell_pos.x / cell_range.x;
    float y_steps = cell_pos.y / cell_range.y;
    float z_steps = cell_pos.z / cell_range.z;
    // normalized cell_pos ([0,0,0]-[1,1,1])
    cell_pos.x = x_steps - static_cast<float>(floor(static_cast<double>(x_steps)));
    cell_pos.y = y_steps - static_cast<float>(floor(static_cast<double>(y_steps)));
    cell_pos.z = z_steps - static_cast<float>(floor(static_cast<double>(z_steps)));
    // actual pos
    cell_pos.x *= cell_range.x;
    cell_pos.y *= cell_range.y;
    cell_pos.z *= cell_range.z;
    return cell_pos;
}

size_t PointCloudGridEncoder::calcMessageSize(const std::vector<CellHeader *> & cell_headers) const {
    size_t header_size = GlobalHeader::getByteSize();
    std::cout << "HEADER SIZE (bytes) " << header_size << std::endl;
    // header size
    size_t message_size = header_size;
    // blacklist size
    size_t blacklist_size = header_->num_blacklist*sizeof(unsigned);
    message_size += blacklist_size;
    std::cout << "BLACKLIST SIZE (bytes) " << blacklist_size << std::endl;
    if(cell_headers.empty())
        return message_size;
    // size of one cell header
    size_t cell_header_size = CellHeader::getByteSize();
    // cell header sizes
    message_size += cell_header_size * cell_headers.size();
    unsigned num_elements=0;
    for(auto c_header: cell_headers) {
        num_elements += c_header->num_elements;
        // size of elements for one cell
        message_size += BitVecArray::getByteSize(
            c_header->num_elements,
            c_header->point_encoding_x,
            c_header->point_encoding_y,
            c_header->point_encoding_z
        );
        message_size += BitVecArray::getByteSize(
                c_header->num_elements,
                c_header->color_encoding_x,
                c_header->color_encoding_y,
                c_header->color_encoding_z
        );
    }
    std::cout << "CELLS\n";
    std::cout << " > ELEMENT COUNT " << num_elements << std::endl;
    return message_size;
}


