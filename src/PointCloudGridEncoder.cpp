#include "../include/PointCloudGridEncoder.hpp"

//
// Created by basti on 14.12.17.
//
PointCloudGridEncoder::PointCloudGridEncoder()
    : pc_grid_()
    , header_size_bytes_()
    , header_()
{
    pc_grid_ = new VariantPointCloudGrid(Vec8(1,1,1));
}

PointCloudGridEncoder::~PointCloudGridEncoder() {

}

bool PointCloudGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec32, Vec32> *point_cloud) {
    return false;
}

zmq::message_t PointCloudGridEncoder::encodePointCloudGrid() {
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        
    }

    return zmq::message_t();
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


