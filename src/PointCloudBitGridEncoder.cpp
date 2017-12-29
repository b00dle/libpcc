#include "../include/PointCloudBitGridEncoder.hpp"
#include <set>

//
// Created by basti on 14.12.17.
//
PointCloudBitGridEncoder::PointCloudBitGridEncoder()
    : pc_grid_()
    , header_()
{
    pc_grid_ = new BitVecPointCloudGrid(Vec8(1,1,1));
    header_ = new GlobalHeader;
}

PointCloudBitGridEncoder::~PointCloudBitGridEncoder()
{
    delete pc_grid_;
    delete header_;
}

bool PointCloudBitGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec<float>, Vec<float>> *point_cloud)
{
    if(!decodePointCloudGrid(msg))
        return false;
    return extractPointCloudFromGrid(point_cloud);
}

bool PointCloudBitGridEncoder::extractPointCloudFromGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud)
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
    BitVecGridCell* cell;
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
                p_bits.x = static_cast<uint8_t>(cell->points->getNX());
                p_bits.y = static_cast<uint8_t>(cell->points->getNY());
                p_bits.z = static_cast<uint8_t>(cell->points->getNZ());
                c_bits.x = static_cast<uint8_t>(cell->colors->getNX());
                c_bits.y = static_cast<uint8_t>(cell->colors->getNY());
                c_bits.z = static_cast<uint8_t>(cell->colors->getNZ());
                glob_cell_min = Vec<float>(cell_range.x*x_idx, cell_range.y*y_idx, cell_range.z*z_idx);
                glob_cell_min += pc_grid_->bounding_box.min;
                for(unsigned i=0; i < cell->size(); ++i) {
                    v_pos = (*cell->points)[i];
                    v_clr = (*cell->colors)[i];
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

zmq::message_t PointCloudBitGridEncoder::encodePointCloudGrid() {
    std::vector<unsigned> black_list;
    std::vector<CellHeader*> cell_headers;
    // initialize cell headers
    int total_elements = 0;
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        if(pc_grid_->cells[cell_idx]->size() == NONE) {
            black_list.push_back(cell_idx);
            continue;
        }
        auto c_header = new CellHeader;
        c_header->cell_idx = cell_idx;
        c_header->point_encoding = getComponentPrecision((*pc_grid_)[cell_idx]->points);
        c_header->color_encoding = getComponentPrecision((*pc_grid_)[cell_idx]->colors);
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

bool PointCloudBitGridEncoder::decodePointCloudGrid(zmq::message_t& msg)
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

size_t PointCloudBitGridEncoder::encodeGlobalHeader(zmq::message_t &msg, size_t offset) {
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

size_t PointCloudBitGridEncoder::decodeGlobalHeader(zmq::message_t& msg, size_t offset)
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

size_t PointCloudBitGridEncoder::encodeBlackList(zmq::message_t &msg, std::vector<unsigned> bl, size_t offset) {
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

size_t PointCloudBitGridEncoder::decodeBlackList(zmq::message_t &msg, std::vector<unsigned>& bl, size_t offset) {
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

size_t PointCloudBitGridEncoder::encodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    auto num_elmts = new unsigned[1];
    size_t bytes_num_elmts(sizeof(unsigned));
    num_elmts[0] = c_header->num_elements;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) num_elmts , bytes_num_elmts);
    offset += bytes_num_elmts;

    auto encoding = new ComponentPrecision[2];
    size_t bytes_enc(2*sizeof(VariantVecType));
    encoding[0] = c_header->point_encoding;
    encoding[1] = c_header->color_encoding;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) encoding, bytes_enc);
    offset += bytes_enc;

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudBitGridEncoder::decodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    auto num_elmts = new unsigned[1];
    size_t bytes_num_elmts(sizeof(unsigned));
    memcpy((unsigned char*) num_elmts, (unsigned char*) msg.data() + offset, bytes_num_elmts);
    c_header->num_elements = num_elmts[0];
    offset += bytes_num_elmts;

    auto encoding = new ComponentPrecision[2];
    size_t bytes_enc(2* sizeof(VariantVecType));
    memcpy((unsigned char*) encoding, (unsigned char*) msg.data() + offset, bytes_enc);
    c_header->point_encoding = encoding[0];
    c_header->color_encoding = encoding[1];
    offset += bytes_enc;

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudBitGridEncoder::encodeCell(zmq::message_t &msg, BitVecGridCell* cell, size_t offset)
{
    if(cell->size() == 0)
        return offset;

    // pack positions
    unsigned char* p_packed = cell->points->pack();
    size_t bytes_p(cell->points->getByteSize());
    memcpy((unsigned char*) msg.data() + offset, p_packed, bytes_p);
    offset += bytes_p;

    // pack colors
    unsigned char* c_packed = cell->colors->pack();
    size_t bytes_c(cell->colors->getByteSize());
    memcpy((unsigned char*) msg.data() + offset, c_packed, bytes_c);
    offset += bytes_c;

    delete [] p_packed;
    delete [] c_packed;

    return offset;
}

size_t PointCloudBitGridEncoder::decodeCell(zmq::message_t &msg, CellHeader *c_header, size_t offset)
{
    if(c_header->num_elements == 0)
        return offset;

    BitVecGridCell* cell = pc_grid_->cells[c_header->cell_idx];

    switch (c_header->point_encoding) {
        case BIT_1:
            cell->initPoints<BIT_1, BIT_1, BIT_1>(); break;
        case BIT_2:
            cell->initPoints<BIT_2, BIT_2, BIT_2>(); break;
        case BIT_3:
            cell->initPoints<BIT_3, BIT_3, BIT_3>(); break;
        case BIT_4:
            cell->initPoints<BIT_4, BIT_4, BIT_4>(); break;
        case BIT_5:
            cell->initPoints<BIT_5, BIT_5, BIT_5>(); break;
        case BIT_6:
            cell->initPoints<BIT_6, BIT_6, BIT_6>(); break;
        case BIT_7:
            cell->initPoints<BIT_7, BIT_7, BIT_7>(); break;
        case BIT_8:
            cell->initPoints<BIT_8, BIT_8, BIT_8>(); break;
        case BIT_9:
            cell->initPoints<BIT_9, BIT_9, BIT_9>(); break;
        case BIT_10:
            cell->initPoints<BIT_10, BIT_10, BIT_10>(); break;
        case BIT_11:
            cell->initPoints<BIT_11, BIT_11, BIT_11>(); break;
        case BIT_12:
            cell->initPoints<BIT_12, BIT_12, BIT_12>(); break;
        case BIT_13:
            cell->initPoints<BIT_13, BIT_13, BIT_13>(); break;
        case BIT_14:
            cell->initPoints<BIT_14, BIT_14, BIT_14>(); break;
        case BIT_15:
            cell->initPoints<BIT_15, BIT_15, BIT_15>(); break;
        case BIT_16:
            cell->initPoints<BIT_16, BIT_16, BIT_16>(); break;
        default:
            break;
    }

    switch (c_header->color_encoding) {
        case BIT_1:
            cell->initColors<BIT_1, BIT_1, BIT_1>(); break;
        case BIT_2:
            cell->initColors<BIT_2, BIT_2, BIT_2>(); break;
        case BIT_3:
            cell->initColors<BIT_3, BIT_3, BIT_3>(); break;
        case BIT_4:
            cell->initColors<BIT_4, BIT_4, BIT_4>(); break;
        case BIT_5:
            cell->initColors<BIT_5, BIT_5, BIT_5>(); break;
        case BIT_6:
            cell->initColors<BIT_6, BIT_6, BIT_6>(); break;
        case BIT_7:
            cell->initColors<BIT_7, BIT_7, BIT_7>(); break;
        case BIT_8:
            cell->initColors<BIT_8, BIT_8, BIT_8>(); break;
        case BIT_9:
            cell->initColors<BIT_9, BIT_9, BIT_9>(); break;
        case BIT_10:
            cell->initColors<BIT_10, BIT_10, BIT_10>(); break;
        case BIT_11:
            cell->initColors<BIT_11, BIT_11, BIT_11>(); break;
        case BIT_12:
            cell->initColors<BIT_12, BIT_12, BIT_12>(); break;
        case BIT_13:
            cell->initColors<BIT_13, BIT_13, BIT_13>(); break;
        case BIT_14:
            cell->initColors<BIT_14, BIT_14, BIT_14>(); break;
        case BIT_15:
            cell->initColors<BIT_15, BIT_15, BIT_15>(); break;
        case BIT_16:
            cell->initColors<BIT_16, BIT_16, BIT_16>(); break;
        default:
            break;
    }

    size_t bytes_p(cell->points->getByteSize());
    auto p_arr = new unsigned char[bytes_p];
    memcpy(p_arr, (unsigned char*) msg.data() + offset, bytes_p);
    offset += bytes_p;
    cell->points->unpack(p_arr, c_header->num_elements);

    size_t bytes_c(cell->colors->getByteSize());
    auto c_arr = new unsigned char[bytes_c];
    memcpy(c_arr, (unsigned char*) msg.data() + offset, bytes_p);
    offset += bytes_c;
    cell->points->unpack(c_arr, c_header->num_elements);

    delete [] p_arr;
    delete [] c_arr;

    return offset;
}


unsigned PointCloudBitGridEncoder::calcGridCellIndex(const Vec<float> &pos, const Vec<float>& cell_range) const {
    Vec<float> temp(pos);
    temp -= pc_grid_->bounding_box.min;
    auto x_idx = static_cast<unsigned>(floor(static_cast<double>(temp.x / cell_range.x)));
    auto y_idx = static_cast<unsigned>(floor(static_cast<double>(temp.y / cell_range.y)));
    auto z_idx = static_cast<unsigned>(floor(static_cast<double>(temp.z / cell_range.z)));
    return x_idx +
        y_idx * pc_grid_->dimensions.x +
        z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
}

const Vec<float> PointCloudBitGridEncoder::mapToCell(const Vec<float> &pos, const Vec<float> &cell_range)
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

size_t PointCloudBitGridEncoder::calcMessageSize(const std::vector<CellHeader *> & cell_headers) const {
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
        message_size += AbstractBitVecArray::getByteSize(
            c_header->num_elements,
            c_header->point_encoding,
            c_header->point_encoding,
            c_header->point_encoding
        );
        message_size += AbstractBitVecArray::getByteSize(
                c_header->num_elements,
                c_header->color_encoding,
                c_header->color_encoding,
                c_header->color_encoding
        );
    }
    std::cout << "CELLS\n";
    std::cout << " > ELEMENT COUNT " << num_elements << std::endl;
    return message_size;
}

PointCloudBitGridEncoder::ComponentPrecision PointCloudBitGridEncoder::getComponentPrecision(AbstractBitVecArray *arr) {
    return static_cast<ComponentPrecision>(arr->getNX());
}


