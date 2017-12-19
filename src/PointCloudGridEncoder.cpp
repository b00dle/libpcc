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

bool PointCloudGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec32, Vec32> *point_cloud)
{
    if(!decodePointCloudGrid(msg))
        return false;
    return true;
}

zmq::message_t PointCloudGridEncoder::encodePointCloudGrid() {
    std::vector<unsigned> black_list;
    std::vector<CellHeader*> cell_headers;
    VariantValueType pos_type, clr_type;
    // initialize cell headers
    int total_elements = 0;
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        pos_type = pc_grid_->getPointType(cell_idx);
        clr_type = pc_grid_->getColorType(cell_idx);
        if(pos_type == NONE || clr_type == NONE) {
            black_list.push_back(cell_idx);
            continue;
        }
        CellHeader* c_header = new CellHeader;
        c_header->cell_idx = cell_idx;
        c_header->point_encoding = pos_type;
        c_header->color_encoding = clr_type;
        c_header->num_elements = pc_grid_->cells[cell_idx]->size();
        cell_headers.push_back(c_header);
        total_elements += c_header->num_elements;
    }

    // fill global header
    header_->num_blacklist = black_list.size();
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
    while(cell_headers.size() > 0) {
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
    old_offset = offset;
    offset = decodeBlackList(msg, black_list, offset);
    if(offset == old_offset)
        return false;

    std::set<unsigned> black_set;
    for(unsigned idx : black_list)
        black_set.insert(idx);

    CellHeader* c_header;
    unsigned num_cells = pc_grid_->cells.size();
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

    unsigned total_num_points = 0;
    std::cout << "DONE decoding\n";
    std::cout << "  > num_cells " << pc_grid_->cells.size() << std::endl;
    //std::cout << "  > cell_sizes\n";
    for(unsigned i = 0; i < pc_grid_->cells.size(); ++i) {
        total_num_points += pc_grid_->cells[i]->size();
    /*    std::cout << "===\n";
        std::cout << "    > idx " << i << std::endl;
        std::cout << "    > point_type " << pc_grid_->getPointType(i) << std::endl;
        std::cout << "    > color_type " << pc_grid_->getColorType(i) << std::endl;
        std::cout << "    > points " << pc_grid_->cells[i]->size() << std::endl;*/
    }
    std::cout << "  > total num points " << total_num_points << std::endl;

    return true;
}

size_t PointCloudGridEncoder::encodeGlobalHeader(zmq::message_t &msg, size_t offset) {
    unsigned char* dim = new unsigned char[3];
    dim[0] = header_->dimensions.x;
    dim[1] = header_->dimensions.y;
    dim[2] = header_->dimensions.z;
    memcpy((unsigned char*) msg.data() + offset, dim, sizeof(dim));
    offset += sizeof(dim);

    float* bb = new float[6];
    bb[0] = header_->bounding_box.min.x;
    bb[1] = header_->bounding_box.min.y;
    bb[2] = header_->bounding_box.min.z;
    bb[3] = header_->bounding_box.max.x;
    bb[4] = header_->bounding_box.max.y;
    bb[5] = header_->bounding_box.max.z;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) bb, sizeof(bb));
    offset += sizeof(bb);

    unsigned* num_blacklist = new unsigned[1];
    num_blacklist[0] = header_->num_blacklist;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) num_blacklist, sizeof(num_blacklist));
    offset += sizeof(num_blacklist);

    // cleanup
    delete [] dim;
    delete [] bb;
    delete [] num_blacklist;
    return offset;
}

size_t PointCloudGridEncoder::decodeGlobalHeader(zmq::message_t& msg, size_t offset)
{
    unsigned char* dim = new unsigned char[3];
    memcpy(dim, (unsigned char*) msg.data() + offset, sizeof(dim));
    header_->dimensions.x = dim[0];
    header_->dimensions.y = dim[1];
    header_->dimensions.z = dim[2];
    offset += sizeof(dim);

    float* bb = new float[6];
    memcpy((unsigned char*) bb, (unsigned char*) msg.data() + offset, sizeof(bb));
    header_->bounding_box.min.x = bb[0];
    header_->bounding_box.min.y = bb[1];
    header_->bounding_box.min.z = bb[2];
    header_->bounding_box.max.x = bb[3];
    header_->bounding_box.max.y = bb[4];
    header_->bounding_box.max.z = bb[5];
    offset += sizeof(bb);

    unsigned* num_blacklist = new unsigned[1];
    memcpy((unsigned char*) num_blacklist, (unsigned char*) msg.data() + offset, sizeof(num_blacklist));
    header_->num_blacklist = num_blacklist[0];
    offset += sizeof(num_blacklist);

    // cleanup
    delete [] dim;
    delete [] bb;
    delete [] num_blacklist;
    return offset;
}

size_t PointCloudGridEncoder::encodeBlackList(zmq::message_t &msg, std::vector<unsigned> bl, size_t offset) {
    unsigned* black_list = new unsigned[bl.size()];
    unsigned i=0;
    for (unsigned elmt: bl) {
        black_list[i] = elmt;
        ++i;
    }
    memcpy((unsigned char*) msg.data() + offset,(unsigned char*) black_list, sizeof(black_list));
    offset += sizeof(black_list);

    // cleanup
    delete [] black_list;
    return offset;
}

size_t PointCloudGridEncoder::decodeBlackList(zmq::message_t &msg, std::vector<unsigned>& bl, size_t offset) {
    bl.resize(header_->num_blacklist);

    unsigned* black_list = new unsigned[bl.size()];
    memcpy((unsigned char*) black_list, (unsigned char*) msg.data() + offset, sizeof(black_list));
    for (unsigned i = 0; i < header_->num_blacklist; ++i)
        bl[i] = black_list[i];
    offset += sizeof(black_list);

    // cleanup
    delete [] black_list;
    return offset;
}

size_t PointCloudGridEncoder::encodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    unsigned* num_elmts = new unsigned[1];
    num_elmts[0] = c_header->num_elements;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) num_elmts , sizeof(num_elmts));
    offset += sizeof(num_elmts );

    VariantValueType* encoding = new VariantValueType[2];
    encoding[0] = c_header->point_encoding;
    encoding[1] = c_header->color_encoding;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) encoding, sizeof(encoding));
    offset += sizeof(encoding);

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudGridEncoder::decodeCellHeader(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    unsigned* num_elmts = new unsigned[1];
    memcpy((unsigned char*) num_elmts, (unsigned char*) msg.data() + offset, sizeof(num_elmts));
    c_header->num_elements = num_elmts[0];
    offset += sizeof(num_elmts );

    VariantValueType* encoding = new VariantValueType[2];
    memcpy((unsigned char*) encoding, (unsigned char*) msg.data() + offset, sizeof(encoding));
    c_header->point_encoding = encoding[0];
    c_header->color_encoding = encoding[1];
    offset += sizeof(encoding);

    // cleanup
    delete [] num_elmts;
    delete [] encoding;
    return offset;
}

size_t PointCloudGridEncoder::encodeVariantCell(zmq::message_t& msg, GridCell<VariantValue,VariantValue>* cell, size_t offset)
{
    if(cell->size() == 0)
        return offset;

    VariantValueType p_type = cell->points[0].getType();
    VariantValueType c_type = cell->colors[0].getType();

    if(p_type == NONE || c_type == NONE)
        return offset;

    // encode points
    if(p_type == VEC_FLOAT) {
        float* p_arr = new float[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) p_arr, sizeof(p_arr));
        offset += sizeof(p_arr);
        delete [] p_arr;
    }
    else if(p_type == VEC_UINT16) {
        uint16_t* p_arr = new uint16_t[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) p_arr, sizeof(p_arr));
        offset += sizeof(p_arr);
        delete [] p_arr;
    }
    else if(p_type == VEC_UINT8) {
        uint8_t* p_arr = new uint8_t[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) p_arr, sizeof(p_arr));
        offset += sizeof(p_arr);
        delete [] p_arr;
    }

    // encode points
    if(c_type == VEC_FLOAT) {
        float* c_arr = new float[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) c_arr, sizeof(c_arr));
        offset += sizeof(c_arr);
        delete [] c_arr;
    }
    else if(c_type == VEC_UINT16) {
        uint16_t* c_arr = new uint16_t[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) c_arr, sizeof(c_arr));
        offset += sizeof(c_arr);
        delete [] c_arr;
    }
    else if(c_type == VEC_UINT8) {
        uint8_t* c_arr = new uint8_t[cell->size()*3];
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
        memcpy((unsigned char*) msg.data() + offset, (unsigned char*) c_arr, sizeof(c_arr));
        offset += sizeof(c_arr);
        delete [] c_arr;
    }

    return offset;
}

size_t PointCloudGridEncoder::decodeVariantCell(zmq::message_t& msg, CellHeader* c_header, size_t offset)
{
    if(c_header->num_elements == 0)
        return offset;

    GridCell<VariantValue, VariantValue>* cell = pc_grid_->cells[c_header->cell_idx];
    cell->clear();
    cell->resize(c_header->num_elements);

    VariantValueType p_type = c_header->point_encoding;
    VariantValueType c_type = c_header->color_encoding;

    if(p_type == NONE || c_type == NONE)
        return offset;

    // encode points
    if(p_type == VEC_FLOAT) {
        float* p_arr = new float[c_header->num_elements*3];
        memcpy((unsigned char*) p_arr, (unsigned char*) msg.data() + offset, sizeof(p_arr));
        offset += sizeof(p_arr);

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
        uint16_t* p_arr = new uint16_t[c_header->num_elements*3];
        memcpy((unsigned char*) p_arr, (unsigned char*) msg.data() + offset, sizeof(p_arr));
        offset += sizeof(p_arr);

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
        uint8_t* p_arr = new uint8_t[c_header->num_elements*3];
        memcpy(p_arr, (unsigned char*) msg.data() + offset, sizeof(p_arr));
        offset += sizeof(p_arr);

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
        float* c_arr = new float[c_header->num_elements*3];
        memcpy((unsigned char*) c_arr, (unsigned char*) msg.data() + offset, sizeof(c_arr));
        offset += sizeof(c_arr);

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
        uint16_t* c_arr = new uint16_t[c_header->num_elements*3];
        memcpy((unsigned char*) c_arr, (unsigned char*) msg.data() + offset, sizeof(c_arr));
        offset += sizeof(c_arr);

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
        uint8_t* c_arr = new uint8_t[c_header->num_elements*3];
        memcpy(c_arr, (unsigned char*) msg.data() + offset, sizeof(c_arr));
        offset += sizeof(c_arr);

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


unsigned PointCloudGridEncoder::calcGridCellIndex(const Vec32 &pos, const Vec32& cell_range) const {
    Vec32 temp(pos);
    temp -= pc_grid_->bounding_box.min;
    auto x_idx = (unsigned) floor(temp.x / cell_range.x);
    auto y_idx = (unsigned) floor(temp.y / cell_range.y);
    auto z_idx = (unsigned) floor(temp.z / cell_range.z);
    return x_idx +
        y_idx * pc_grid_->dimensions.x +
        z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
}

const Vec<float> PointCloudGridEncoder::mapToCell(const Vec32 &pos, const Vec32 &cell_range)
{
    Vec<float> cell_pos(pos.x, pos.y, pos.z);
    cell_pos -= pc_grid_->bounding_box.min;
    float x_steps = cell_pos.x / cell_range.x;
    float y_steps = cell_pos.y / cell_range.y;
    float z_steps = cell_pos.z / cell_range.z;
    // normalized cell_pos ([0,0,0]-[1,1,1])
    cell_pos.x = x_steps - (float) floor(x_steps);
    cell_pos.y = y_steps - (float) floor(y_steps);
    cell_pos.z = z_steps - (float) floor(z_steps);
    // actual pos
    cell_pos.x *= cell_range.x;
    cell_pos.y *= cell_range.y;
    cell_pos.z *= cell_range.z;
    return cell_pos;
}

size_t PointCloudGridEncoder::calcMessageSize(const std::vector<CellHeader *> & cell_headers) const {
    size_t header_size = GlobalHeader::getByteSize();
    std::cout << "HEADER SIZE " << header_size << std::endl;
    // header size
    size_t message_size = header_size;
    // blacklist size
    size_t blacklist_size = header_->num_blacklist*sizeof(unsigned);
    message_size += blacklist_size;
    std::cout << "BLACKLIST SIZE " << blacklist_size << std::endl;
    if(cell_headers.size() == 0)
        return message_size;
    // size of one cell header
    size_t cell_header_size = CellHeader::getByteSize();
    // cell header sizes
    message_size += cell_header_size * cell_headers.size();
    unsigned num_elements=0;
    for(unsigned i=0; i < cell_headers.size(); ++i) {
        num_elements += cell_headers[i]->num_elements;
        // size of elements for one cell
        message_size += cell_headers[i]->num_elements * (
            VariantValue::getByteSize(cell_headers[i]->point_encoding) +
            VariantValue::getByteSize(cell_headers[i]->color_encoding)
        );
    }
    std::cout << "CELLS\n";
    std::cout << " > ELEMENT COUNT " << num_elements << std::endl;
    std::cout << " > POINT ENCODING " << VariantValue::getByteSize(cell_headers[0]->point_encoding) << std::endl;
    std::cout << " > POINT ENCODING " << VariantValue::getByteSize(cell_headers[0]->color_encoding) << std::endl;
    return message_size;
}


