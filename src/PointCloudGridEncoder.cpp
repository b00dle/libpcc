#include "../include/PointCloudGridEncoder.hpp"

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
    return false;
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
        std::cout << "cell elmts" << c_header->num_elements << std::endl;
    }

    std::cout << "total: " << total_elements << "\n";

    // fill global header
    header_->num_blacklist = black_list.size();
    header_->dimensions = pc_grid_->dimensions;
    header_->bounding_box = pc_grid_->bounding_box;

    // calc overall message size and init message
    size_t message_size_bytes = calcMessageSize(cell_headers);

    zmq::message_t message(message_size_bytes);

    // TODO FILL MESSAGE

    // Cleanup
    while(cell_headers.size() > 0) {
        delete cell_headers.back();
        cell_headers.pop_back();
    }

    return message;
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
    // header size
    size_t message_size = sizeof(*header_);
    // blacklist size
    message_size += header_->num_blacklist*sizeof(unsigned);
    if(cell_headers.size() == 0)
        return message_size;
    // size of one cell header
    size_t cell_header_size = sizeof(cell_headers[0]->num_elements);
    cell_header_size += sizeof(cell_headers[0]->color_encoding);
    cell_header_size += sizeof(cell_headers[0]->point_encoding);
    // cell header sizes
    message_size += cell_header_size * cell_headers.size();
    for(unsigned i=0; i < cell_headers.size(); ++i) {
        // size of elements for one cell
        message_size += cell_headers[i]->num_elements * (
            VariantValue::getSize(cell_headers[i]->point_encoding) +
            VariantValue::getSize(cell_headers[i]->color_encoding)
        );
    }
    return message_size;
}


