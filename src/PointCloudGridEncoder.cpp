#include "../include/PointCloudGridEncoder.hpp"
#include <set>

//
// Created by basti on 14.12.17.
//
PointCloudGridEncoder::PointCloudGridEncoder()
    : pc_grid_()
    , header_()
{
    pc_grid_ = new VariantPointCloudGrid(Vec8(1,1,1));
    header_ = new GlobalHeader;
}

PointCloudGridEncoder::~PointCloudGridEncoder()
{
    delete pc_grid_;
    delete header_;
}

bool PointCloudGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec<float>, Vec<float>> *point_cloud)
{
    if(!decodePointCloudGrid(msg))
        return false;
    return extractPointCloudFromGrid(point_cloud);
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
    std::cout << "CELLS decode\n  > RANGE " << cell_range.x << "," << cell_range.y << "," << cell_range.z << std::endl;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));
    VariantVec v_pos, v_clr;
    unsigned cell_idx;
    GridCell<VariantVec, VariantVec>* cell;
    unsigned point_idx=0;
    for(unsigned x_idx=0; x_idx < pc_grid_->dimensions.x; ++x_idx) {
        for(unsigned y_idx=0; y_idx < pc_grid_->dimensions.y; ++y_idx) {
            for(unsigned z_idx=0; z_idx < pc_grid_->dimensions.z; ++z_idx) {
                cell_idx = x_idx +
                    y_idx * pc_grid_->dimensions.x +
                    z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
                cell = pc_grid_->cells[cell_idx];
                glob_cell_min = Vec<float>(cell_range.x*x_idx, cell_range.y*y_idx, cell_range.z*z_idx);
                glob_cell_min += pc_grid_->bounding_box.min;
                bool ok=true;
                for(unsigned i=0; i < pc_grid_->cells[cell_idx]->points.size(); ++i) {
                    v_pos = cell->points[i];
                    v_clr = cell->colors[i];
                    if(v_pos.getType() == NONE || v_clr.getType() == NONE)
                        continue;
                    pos_cell = mapToFloat(v_pos, bb_cell, ok);
                    if(!ok)
                        continue;
                    clr = mapToFloat(v_clr, bb_clr, ok);
                    if(!ok)
                        continue;
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
    VariantVecType pos_type, clr_type;
    // initialize cell headers
    int total_elements = 0;
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        pos_type = pc_grid_->getPointType(cell_idx);
        clr_type = pc_grid_->getColorType(cell_idx);
        if(pos_type == NONE || clr_type == NONE) {
            black_list.push_back(cell_idx);
            continue;
        }
        auto c_header = new CellHeader;
        c_header->cell_idx = cell_idx;
        c_header->point_encoding = pos_type;
        c_header->color_encoding = clr_type;
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
        offset = encodeVariantCell(message, pc_grid_->cells[c_header->cell_idx], offset);
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

    std::cout << "DECODED header\n";
    std::cout << "  > bb\n";
    std::cout << "    > min " << header_->bounding_box.min.x << "," << header_->bounding_box.min.y << "," << header_->bounding_box.min.z << std::endl;
    std::cout << "    > max " << header_->bounding_box.max.x << "," << header_->bounding_box.max.y << "," << header_->bounding_box.max.z << std::endl;
    std::cout << "  > dim\n";
    std::cout << "    > " << (int) header_->dimensions.x << "," << (int) header_->dimensions.y << "," << (int) header_->dimensions.z << std::endl;
    std::cout << "  > num_bl " << header_->num_blacklist << std::endl;

    pc_grid_->resize(header_->dimensions);
    pc_grid_->bounding_box = header_->bounding_box;

    std::vector<unsigned> black_list;
    old_offset = offset;
    offset = decodeBlackList(msg, black_list, offset);
    //if(offset == old_offset)
    //    return false;

    std::cout << "DECODED blacklist\n";
    std::cout << "  > size " << black_list.size() << std::endl;

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
        offset = decodeVariantCell(msg, c_header, offset);
    }
    delete c_header;

    unsigned total_num_points = 0;
    //std::cout << "DONE decoding\n";
    //std::cout << "  > num_cells " << pc_grid_->cells.size() << std::endl;
    //std::cout << "  > cell_sizes\n";
    for(auto cell: pc_grid_->cells) {
        total_num_points += cell->size();
    /*    std::cout << "===\n";
        std::cout << "    > idx " << i << std::endl;
        std::cout << "    > point_type " << pc_grid_->getPointType(i) << std::endl;
        std::cout << "    > color_type " << pc_grid_->getColorType(i) << std::endl;
        std::cout << "    > points " << pc_grid_->cells[i]->size() << std::endl;*/
    }
    //std::cout << "  > total num points " << total_num_points << std::endl;

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

    std::cout << "ENCODED bb\n";
    std::cout << "  > min " << bb[0] << "," << bb[1] << "," << bb[2] << std::endl;
    std::cout << "  > max " << bb[3] << "," << bb[4] << "," << bb[5] << std::endl;

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

    auto encoding = new VariantVecType[2];
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

size_t PointCloudGridEncoder::decodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    auto num_elmts = new unsigned[1];
    size_t bytes_num_elmts(sizeof(unsigned));
    memcpy((unsigned char*) num_elmts, (unsigned char*) msg.data() + offset, bytes_num_elmts);
    c_header->num_elements = num_elmts[0];
    offset += bytes_num_elmts;

    auto encoding = new VariantVecType[2];
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

size_t PointCloudGridEncoder::encodeVariantCell(zmq::message_t& msg, GridCell<VariantVec,VariantVec>* cell, size_t offset)
{
    if(cell->size() == 0)
        return offset;

    VariantVecType p_type = cell->points[0].getType();
    VariantVecType c_type = cell->colors[0].getType();

    if(p_type == NONE || c_type == NONE)
        return offset;

    // encode points
    if(p_type == VEC_FLOAT) {
        auto p_arr = new float[cell->size()*3];
        size_t bytes_p_arr(3*cell->size()*sizeof(float));
        bool ok = true;
        Vec<float> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->points[i].toVecFloat(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            p_arr[i*3] = v.x;
            p_arr[i*3+1] = v.y;
            p_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) p_arr, bytes_p_arr);
        offset += bytes_p_arr;
        delete [] p_arr;
    }
    else if(p_type == VEC_UINT16) {
        auto p_arr = new uint16_t[cell->size()*3];
        size_t bytes_p_arr(cell->size()*3*sizeof(uint16_t));
        bool ok = true;
        Vec<uint16_t> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->points[i].toVecUInt16(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            p_arr[i*3] = v.x;
            p_arr[i*3+1] = v.y;
            p_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) p_arr, bytes_p_arr);
        offset += bytes_p_arr;
        delete [] p_arr;
    }
    else if(p_type == VEC_UINT8) {
        auto p_arr = new uint8_t[cell->size()*3];
        size_t bytes_p_arr(cell->size()*3*sizeof(uint8_t));
        bool ok = true;
        Vec<uint8_t> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->points[i].toVecUInt8(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            p_arr[i*3] = v.x;
            p_arr[i*3+1] = v.y;
            p_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, p_arr, bytes_p_arr);
        offset += bytes_p_arr;
        delete [] p_arr;
    }

    // encode points
    if(c_type == VEC_FLOAT) {
        auto c_arr = new float[cell->size()*3];
        size_t bytes_c_arr(cell->size()*3*sizeof(float));
        bool ok = true;
        Vec<float> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->colors[i].toVecFloat(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            c_arr[i*3] = v.x;
            c_arr[i*3+1] = v.y;
            c_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) c_arr, bytes_c_arr);
        offset += bytes_c_arr;
        delete [] c_arr;
    }
    else if(c_type == VEC_UINT16) {
        auto c_arr = new uint16_t[cell->size()*3];
        size_t bytes_c_arr(cell->size()*3*sizeof(uint16_t));
        bool ok = true;
        Vec<uint16_t> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->colors[i].toVecUInt16(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            c_arr[i*3] = v.x;
            c_arr[i*3+1] = v.y;
            c_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) c_arr, bytes_c_arr);
        offset += bytes_c_arr;
        delete [] c_arr;
    }
    else if(c_type == VEC_UINT8) {
        auto c_arr = new uint8_t[cell->size()*3];
        size_t bytes_c_arr(cell->size()*3*sizeof(uint8_t));
        bool ok = true;
        Vec<uint8_t> v;
        for(unsigned i=0; i < cell->size(); ++i) {
            v = cell->colors[i].toVecUInt8(ok);
            if(!ok) {
                std::cerr << "Failure: couldn't cast vector from variant." << std::endl;
                continue;
            }
            c_arr[i*3] = v.x;
            c_arr[i*3+1] = v.y;
            c_arr[i*3+2] = v.z;
        }
        memcpy((unsigned char*) msg.data() + offset, c_arr, bytes_c_arr);
        offset += bytes_c_arr;
        delete [] c_arr;
    }

    return offset;
}

size_t PointCloudGridEncoder::decodeVariantCell(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    if(c_header->num_elements == 0)
        return offset;

    GridCell<VariantVec, VariantVec>* cell = pc_grid_->cells[c_header->cell_idx];
    cell->clear();
    cell->resize(c_header->num_elements);

    VariantVecType p_type = c_header->point_encoding;
    VariantVecType c_type = c_header->color_encoding;

    if(p_type == NONE || c_type == NONE)
        return offset;

    // encode points
    if(p_type == VEC_FLOAT) {
        auto p_arr = new float[c_header->num_elements*3];
        size_t bytes_p_arr(c_header->num_elements*3*sizeof(float));
        memcpy((unsigned char*) p_arr, (unsigned char*) msg.data() + offset, bytes_p_arr);
        offset += bytes_p_arr;

        Vec<float> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = p_arr[i*3];
            v.y = p_arr[i*3+1];
            v.z = p_arr[i*3+2];
            cell->points[i].set(v);
        }

        delete [] p_arr;
    }
    else if(p_type == VEC_UINT16) {
        auto p_arr = new uint16_t[c_header->num_elements*3];
        size_t bytes_p_arr(c_header->num_elements*3*sizeof(uint16_t));
        memcpy((unsigned char*) p_arr, (unsigned char*) msg.data() + offset, bytes_p_arr);
        offset += bytes_p_arr;

        Vec<uint16_t> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = p_arr[i*3];
            v.y = p_arr[i*3+1];
            v.z = p_arr[i*3+2];
            cell->points[i].set(v);
        }

        delete [] p_arr;
    }
    else if(p_type == VEC_UINT8) {
        auto p_arr = new uint8_t[c_header->num_elements*3];
        size_t bytes_p_arr(c_header->num_elements*3*sizeof(uint8_t));
        memcpy(p_arr, (unsigned char*) msg.data() + offset, bytes_p_arr);
        offset += bytes_p_arr;

        Vec<uint8_t> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = p_arr[i*3];
            v.y = p_arr[i*3+1];
            v.z = p_arr[i*3+2];
            cell->points[i].set(v);
        }

        delete [] p_arr;
    }

    // encode points
    if(c_type == VEC_FLOAT) {
        auto c_arr = new float[c_header->num_elements*3];
        size_t bytes_c_arr(c_header->num_elements*3*sizeof(float));
        memcpy((unsigned char*) c_arr, (unsigned char*) msg.data() + offset, bytes_c_arr);
        offset += bytes_c_arr;

        Vec<float> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = c_arr[i*3];
            v.y = c_arr[i*3+1];
            v.z = c_arr[i*3+2];
            cell->colors[i].set(v);
        }

        delete [] c_arr;
    }
    else if(c_type == VEC_UINT16) {
        auto c_arr = new uint16_t[c_header->num_elements*3];
        size_t bytes_c_arr(c_header->num_elements*3*sizeof(uint16_t));
        memcpy((unsigned char*) c_arr, (unsigned char*) msg.data() + offset, bytes_c_arr);
        offset += bytes_c_arr;

        Vec<uint16_t> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = c_arr[i*3];
            v.y = c_arr[i*3+1];
            v.z = c_arr[i*3+2];
            cell->colors[i].set(v);
        }

        delete [] c_arr;
    }
    else if(c_type == VEC_UINT8) {
        auto c_arr = new uint8_t[c_header->num_elements*3];
        size_t bytes_c_arr(c_header->num_elements*3*sizeof(uint8_t));
        memcpy(c_arr, (unsigned char*) msg.data() + offset, bytes_c_arr);
        offset += bytes_c_arr;

        Vec<uint8_t> v;
        for(unsigned i=0; i < c_header->num_elements; ++i) {
            v.x = c_arr[i*3];
            v.y = c_arr[i*3+1];
            v.z = c_arr[i*3+2];
            cell->colors[i].set(v);
        }

        delete [] c_arr;
    }

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
    //std::cout << "HEADER SIZE " << header_size << std::endl;
    // header size
    size_t message_size = header_size;
    // blacklist size
    size_t blacklist_size = header_->num_blacklist*sizeof(unsigned);
    message_size += blacklist_size;
    //std::cout << "BLACKLIST SIZE " << blacklist_size << std::endl;
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
        message_size += c_header->num_elements * (
            VariantVec::getByteSize(c_header->point_encoding) +
            VariantVec::getByteSize(c_header->color_encoding)
        );
    }
    //std::cout << "CELLS\n";
    //std::cout << " > ELEMENT COUNT " << num_elements << std::endl;
    //std::cout << " > POINT ENCODING " << VariantVec::getByteSize(cell_headers[0]->point_encoding) << std::endl;
    //std::cout << " > POINT ENCODING " << VariantVec::getByteSize(cell_headers[0]->color_encoding) << std::endl;
    return message_size;
}


