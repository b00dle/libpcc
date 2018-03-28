#include "../include/PointCloudGridEncoder.hpp"

#include <set>
#include <omp.h>
#include <map>

#include "zlib.h"

#include "../include/Measurement.hpp"

PointCloudGridEncoder::PointCloudGridEncoder(const EncodingSettings& s)
    : Encoder()
    , settings(s)
    , pc_grid_()
    , header_()
    , global_header_()
{
    pc_grid_ = new PointCloudGrid(Vec8(1,1,1));
    header_ = new GridHeader;
    global_header_ = new GlobalHeader;
}

PointCloudGridEncoder::~PointCloudGridEncoder()
{
    delete pc_grid_;
    delete header_;
    delete global_header_;
}

zmq::message_t PointCloudGridEncoder::encode(PointCloud<Vec<float>, Vec<float>>* point_cloud)
{
    // set properties for parallelization
    omp_set_num_threads(settings.num_threads);
    // Set properties for new grid
    pc_grid_->resize(settings.grid_precision.dimensions);
    pc_grid_->bounding_box = point_cloud->bounding_box;
    buildPointCloudGrid(point_cloud);

    if(settings.entropy_coding) {
      return entropyCompression(encodePointCloudGrid());
    } else {
      return finalizeMessage(encodePointCloudGrid());
    }
};

zmq::message_t PointCloudGridEncoder::encode(const std::vector<UncompressedVoxel>& point_cloud, int num_points)
{
    // set properties for parallelization
    omp_set_num_threads(settings.num_threads);
    // Set properties for new grid
    pc_grid_->resize(settings.grid_precision.dimensions);
    pc_grid_->bounding_box = settings.grid_precision.bounding_box;
    buildPointCloudGrid(point_cloud, num_points);

    if(settings.entropy_coding) {
      return entropyCompression(encodePointCloudGrid());
    } else {
      return finalizeMessage(encodePointCloudGrid());
    }
};

bool PointCloudGridEncoder::decode(zmq::message_t &msg, PointCloud<Vec<float>, Vec<float>> *point_cloud)
{
    // set properties for parallelization
    omp_set_num_threads(settings.num_threads);
    if(!decodePointCloudGrid(msg))
        return false;
    return extractPointCloudFromGrid(point_cloud);
}

bool PointCloudGridEncoder::decode(zmq::message_t &msg, std::vector<UncompressedVoxel>* point_cloud)
{
    // set properties for parallelization
    omp_set_num_threads(settings.num_threads);
    if(!decodePointCloudGrid(msg))
        return false;
    return extractPointCloudFromGrid(point_cloud);
}

const PointCloudGrid* PointCloudGridEncoder::getPointCloudGrid() const
{
    return pc_grid_;
}

bool PointCloudGridEncoder::writeToAppendix(zmq::message_t& msg, unsigned char* data, unsigned long size)
{
    decodeGlobalHeader(msg);
    if(size > global_header_->appendix_size)
        return false;
    memcpy((unsigned char*) msg.data() + msg.size() - global_header_->appendix_size, data, size);
    return true;
}

bool PointCloudGridEncoder::writeToAppendix(zmq::message_t& msg, const std::string& text) {
    unsigned char* data = new unsigned char[text.size()];
    memcpy(data, text.data(), text.size());
    bool success = writeToAppendix(msg, data, text.size());
    delete [] data;
    return success;
}

unsigned long PointCloudGridEncoder::readFromAppendix(zmq::message_t& msg, unsigned char*& data)
{
    decodeGlobalHeader(msg);
    if(global_header_->appendix_size > 0) {
        data = new unsigned char[global_header_->appendix_size];
        memcpy(data, (unsigned char *) msg.data() + msg.size() - global_header_->appendix_size,
               global_header_->appendix_size);
    }
    return global_header_->appendix_size;
}

zmq::message_t PointCloudGridEncoder::finalizeMessage(zmq::message_t msg) {
    global_header_->entropy_coding = false;
    global_header_->uncompressed_size = msg.size();
    global_header_->appendix_size = settings.appendix_size;

    zmq::message_t out_msg(
        msg.size() +
        GlobalHeader::getByteSize() +
        global_header_->appendix_size
    );

    int offset = encodeGlobalHeader(out_msg);

    memcpy((unsigned char*) out_msg.data() + offset,
     (unsigned char*) msg.data(), msg.size());

    return out_msg;
}

zmq::message_t PointCloudGridEncoder::entropyCompression(zmq::message_t msg) {
    Measure t;
    t.startWatch();
    global_header_->entropy_coding = true;
    global_header_->uncompressed_size = msg.size();
    global_header_->appendix_size = settings.appendix_size;

    unsigned long size_compressed = (msg.size() * 1.1) + 12;
    unsigned char* entropy_compressed = new unsigned char[size_compressed];//(unsigned char*) malloc(size_compressed);

    int z_result = compress(entropy_compressed, &size_compressed, (unsigned char*) msg.data(), msg.size());
    switch( z_result )
    {
    case Z_OK:
        /*if(settings.verbose)
            printf("***** SUCCESS! *****\n");*/
        break;

    case Z_MEM_ERROR:
        printf("FAILURE [zlib]: out of memory.\n  > Exiting.");
        exit(1);    // quit.
        break;

    case Z_BUF_ERROR:
        printf("FAILURE [zlib]: output buffer wasn't large enough!\n  > Exiting.");
        exit(1);    // quit.
        break;
    }

    zmq::message_t out_msg(
        size_compressed +
        GlobalHeader::getByteSize() +
        global_header_->appendix_size
    );
    int offset = encodeGlobalHeader(out_msg);
    memcpy((unsigned char*) out_msg.data() + offset, entropy_compressed, size_compressed);

    delete [] entropy_compressed;
    if(settings.verbose) {
        std::cout << "ENTROPY COMPRESSION done." << std::endl;
        std::cout << "  > took " << t.stopWatch() << "ms." << std::endl;
        std::cout << "  > uncompressed byte size " << msg.size() << std::endl;
        std::cout << "  > compressed byte size " << out_msg.size() << std::endl;
    }
    return out_msg;
}

zmq::message_t PointCloudGridEncoder::entropyDecompression(zmq::message_t& msg, size_t offset) {
    Measure t;
    t.startWatch();
    unsigned long compressed_size = msg.size() - offset - global_header_->appendix_size;
    zmq::message_t msg_uncompressed(global_header_->uncompressed_size);
    int z_result = uncompress((unsigned char*) msg_uncompressed.data(), &global_header_->uncompressed_size, (unsigned char*)msg.data() + offset, compressed_size);
    switch( z_result )
    {
    case Z_OK:
        /*if(settings.verbose)
            printf("***** SUCCESS! *****\n");*/
        break;

    case Z_MEM_ERROR:
        printf("FAILURE [zlib]: out of memory.\n  > Exiting.");
        exit(1);

    case Z_BUF_ERROR:
        printf("FAILURE [zlib]: output buffer wasn't large enough!\n  > Exiting.");
        exit(1);

    default:
        break;
    }

    if(settings.verbose) {
        std::cout << "ENTROPY DECOMPRESSION done." << std::endl;
        std::cout << "  > took " << t.stopWatch() << "ms." << std::endl;
    }
    return msg_uncompressed;
}

void PointCloudGridEncoder::buildPointCloudGrid(PointCloud<Vec<float>, Vec<float>>* point_cloud) {
    Measure t;
    t.startWatch();

    // init all cells to default BitCount
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        Vec<BitCount> M_P = settings.grid_precision.point_precision[cell_idx];
        Vec<BitCount> M_C = settings.grid_precision.color_precision[cell_idx];
        pc_grid_->cells[cell_idx]->initPoints(M_P.x, M_P.y, M_P.z);
        pc_grid_->cells[cell_idx]->initColors(M_C.x, M_C.y, M_C.z);
    }

    Vec<float> cell_range = pc_grid_->bounding_box.calcRange();
    cell_range.x /= (float) pc_grid_->dimensions.x;
    cell_range.y /= (float) pc_grid_->dimensions.y;
    cell_range.z /= (float) pc_grid_->dimensions.z;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));

    // Create one grid per thread
    // to avoid race conditions writing to shared grid
    auto max_threads = static_cast<unsigned>(omp_get_max_threads());
    unsigned num_cells = pc_grid_->dimensions.x * pc_grid_->dimensions.y * pc_grid_->dimensions.z;
    std::vector<std::vector<size_t>> t_grid_elmts(max_threads, std::vector<size_t>(num_cells, 0));
    std::vector<unsigned> point_cell_idx(point_cloud->size());

    // calculate cell indexes for points
    // and number of elements per thread grid cell
    #pragma omp parallel for schedule(static)
    for(unsigned i=0; i < point_cloud->size(); ++i) {
        int t_num = omp_get_thread_num();
        if (!pc_grid_->bounding_box.contains(point_cloud->points[i]))
            continue;
        unsigned cell_idx = calcGridCellIndex(point_cloud->points[i], cell_range);
        t_grid_elmts[t_num][cell_idx] += 1;
        point_cell_idx[i] = cell_idx;
    }

    // resize grid cells based on summing elements per thread grid cell
    // and create offsets of thread grid cell insert into main grid
    std::vector<std::vector<unsigned>> t_curr_elmt(max_threads, std::vector<unsigned>(num_cells,0));
    size_t cell_size = 0;
    for(unsigned cell_idx=0; cell_idx < num_cells; ++cell_idx) {
        for(unsigned t_num=1; t_num < t_curr_elmt.size(); ++t_num)
            t_curr_elmt[t_num][cell_idx] += t_curr_elmt[t_num-1][cell_idx] + t_grid_elmts[t_num-1][cell_idx];
        cell_size = t_curr_elmt[max_threads-1][cell_idx] + t_grid_elmts[max_threads-1][cell_idx];
        (*pc_grid_)[cell_idx]->resize(cell_size);
    }

    time_t calc_offset = t.stopWatch();

    // insert compressed points into main grid
    #pragma omp parallel for schedule(static)
    for(unsigned i=0; i < point_cloud->size(); ++i) {
        int t_num = omp_get_thread_num();
        if (!pc_grid_->bounding_box.contains(point_cloud->points[i]))
            continue;
        Vec<float> pos_cell = mapToCell(point_cloud->points[i], cell_range);
        unsigned cell_idx = point_cell_idx[i];
        unsigned elmnt_idx = t_curr_elmt[t_num][cell_idx];
        (*pc_grid_)[cell_idx]->points[elmnt_idx] = mapVec(pos_cell, bb_cell,
                                                          settings.grid_precision.point_precision[cell_idx]);
        (*pc_grid_)[cell_idx]->colors[elmnt_idx] = mapVec(point_cloud->colors[i], bb_clr,
                                                          settings.grid_precision.color_precision[cell_idx]);
        t_curr_elmt[t_num][cell_idx] += 1;
    }

    time_t fill_grid = t.stopWatch();

    if(settings.verbose) {
        std::cout << "DONE building grid\n";
        std::cout << "  > took " << fill_grid << "ms.\n";
        std::cout << "    > offset calculation " << calc_offset << "ms.\n";
        std::cout << "    > filling grid " << fill_grid - calc_offset << "ms.\n";
    }
}


void PointCloudGridEncoder::buildPointCloudGrid(const std::vector<UncompressedVoxel>& point_cloud, int num_points) {
    Measure t;
    t.startWatch();

    // init all cells to default BitCount
    for(unsigned cell_idx = 0; cell_idx < pc_grid_->cells.size(); ++cell_idx) {
        Vec<BitCount> M_P = settings.grid_precision.point_precision[cell_idx];
        Vec<BitCount> M_C = settings.grid_precision.color_precision[cell_idx];
        pc_grid_->cells[cell_idx]->initPoints(M_P.x, M_P.y, M_P.z);
        pc_grid_->cells[cell_idx]->initColors(M_C.x, M_C.y, M_C.z);
    }

    Vec<float> cell_range = pc_grid_->bounding_box.calcRange();
    cell_range.x /= (float) pc_grid_->dimensions.x;
    cell_range.y /= (float) pc_grid_->dimensions.y;
    cell_range.z /= (float) pc_grid_->dimensions.z;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(255.0f,255.0f,255.0f));

    // Create one grid per thread
    // to avoid race conditions writing to shared grid
    auto max_threads = static_cast<unsigned>(omp_get_max_threads());
    unsigned num_cells = pc_grid_->dimensions.x * pc_grid_->dimensions.y * pc_grid_->dimensions.z;
    std::vector<std::vector<size_t>> t_grid_elmts(max_threads, std::vector<size_t>(num_cells, 0));
    std::vector<unsigned> point_cell_idx(point_cloud.size());

    if(num_points < 0)
        num_points = point_cloud.size();

    if(settings.verbose) {
        std::cout << "POINT CLOUD\n";
        std::cout << "  > size " << num_points << std::endl;
    }

    // discard overlapping points after quantization
    // and calc overall point color by incremental mean.
    // - not parallelized, thus slower.
    // - reduces number of points in grid (compared to original) for increasing coarsity of abstraction
    if(settings.irrelevance_coding) {
        std::vector<PropertyMap> cell_prop_map(num_cells);
        PropertyMap::iterator it;
        int discarded_by_bb = 0;
        int discarded_by_cell = 0;
        for(unsigned i=0; i < num_points; ++i) {
            if (!pc_grid_->bounding_box.contains(point_cloud[i].pos)) {
                discarded_by_bb++;
                continue;
            }
            Vec<float> pos_cell = mapToCell(point_cloud[i].pos, cell_range);
            unsigned cell_idx = calcGridCellIndex(point_cloud[i].pos, cell_range);
            Vec<uint64_t> comp_pos = mapVec(pos_cell, bb_cell,
                                            settings.grid_precision.point_precision[cell_idx]);
            Vec<uint64_t> comp_clr = mapVec(point_cloud[i].color_rgba, bb_clr,
                                            settings.grid_precision.color_precision[cell_idx]);
            it = cell_prop_map[cell_idx].find(comp_pos);
            if(it == cell_prop_map[cell_idx].end()) {
                cell_prop_map[cell_idx].insert(PropertyPair(
                        comp_pos, std::pair<Vec<uint64_t>, int>(comp_clr, 1)));
            } else {
                discarded_by_cell++;
                std::pair<Vec<uint64_t>, int> prop = it->second;
                prop.second += 1;
                float weight = 1 / (float) (prop.second);
                float new_r = weight * (float) comp_clr.x + (1-weight) * (float) prop.first.x;
                float new_g = weight * (float) comp_clr.y + (1-weight) * (float) prop.first.y;
                float new_b = weight * (float) comp_clr.z + (1-weight) * (float) prop.first.z;
                prop.first = Vec<uint64_t>((uint64_t) new_r,(uint64_t) new_g,(uint64_t) new_b);
                cell_prop_map[cell_idx][comp_pos] = prop;
            }
        }

        for(unsigned cell_idx = 0; cell_idx < num_cells; ++cell_idx) {
            (*pc_grid_)[cell_idx]->resize(cell_prop_map[cell_idx].size());
            int elmnt_idx = 0;
            for(it = cell_prop_map[cell_idx].begin(); it != cell_prop_map[cell_idx].end(); ++it) {
                (*pc_grid_)[cell_idx]->points[elmnt_idx] = it->first;
                (*pc_grid_)[cell_idx]->colors[elmnt_idx] = it->second.first;
                ++elmnt_idx;
            }
        }

        if(settings.verbose) {
            std::cout << "POINTS DISCARDED \n";
            std::cout << "  > took " << t.stopWatch() << "ms." << std::endl;
            std::cout << "  > BoundingBox " << discarded_by_bb << std::endl;
            std::cout << "  > Quantization " << discarded_by_cell<< std::endl;
            std::cout << "  > " << num_points - discarded_by_bb - discarded_by_cell << " voxels left.\n";
        }
    }
    // keep overlapping points
    // - parallel computation, thus faster
    // - number of points in grid equal to points in uncompressed point cloud
    else {
        std::vector<int> discarded_by_bb(max_threads, 0);
        // calculate cell indexes for points
        // and number of elements per thread grid cell
#pragma omp parallel for schedule(static)
        for(unsigned i=0; i < num_points; ++i) {
            int t_num = omp_get_thread_num();
            if (!pc_grid_->bounding_box.contains(point_cloud[i].pos)) {
                discarded_by_bb[t_num] += 1;
                continue;
            }
            unsigned cell_idx = calcGridCellIndex(point_cloud[i].pos, cell_range);
            t_grid_elmts[t_num][cell_idx] += 1;
            point_cell_idx[i] = cell_idx;
        }

        int total_discarded_by_bb = 0;
        for(auto disc_bb : discarded_by_bb) {
            total_discarded_by_bb += disc_bb;
        }

        if(settings.verbose) {
            std::cout << "POINTS DISCARDED BY BoundingBox " << total_discarded_by_bb << std::endl;
            std::cout << "  > " << num_points - total_discarded_by_bb << " voxels left.\n";
        }

        // resize grid cells based on summing elements per thread grid cell
        // and create offsets of thread grid cell insert into main grid
        std::vector<std::vector<unsigned>> t_curr_elmt(max_threads, std::vector<unsigned>(num_cells,0));
        size_t cell_size = 0;
        for(unsigned cell_idx=0; cell_idx < num_cells; ++cell_idx) {
            for(unsigned t_num=1; t_num < t_curr_elmt.size(); ++t_num)
                t_curr_elmt[t_num][cell_idx] += t_curr_elmt[t_num-1][cell_idx] + t_grid_elmts[t_num-1][cell_idx];
            cell_size = t_curr_elmt[max_threads-1][cell_idx] + t_grid_elmts[max_threads-1][cell_idx];
            (*pc_grid_)[cell_idx]->resize(cell_size);
        }

        time_t calc_offset = t.stopWatch();

        // insert compressed points into main grid
#pragma omp parallel for schedule(static)
        for(unsigned i=0; i < num_points; ++i) {
            int t_num = omp_get_thread_num();
            if (!pc_grid_->bounding_box.contains(point_cloud[i].pos))
                continue;
            Vec<float> pos_cell = mapToCell(point_cloud[i].pos, cell_range);
            unsigned cell_idx = point_cell_idx[i];
            unsigned elmnt_idx = t_curr_elmt[t_num][cell_idx];
            (*pc_grid_)[cell_idx]->points[elmnt_idx] = mapVec(pos_cell, bb_cell,
                                                              settings.grid_precision.point_precision[cell_idx]);
            (*pc_grid_)[cell_idx]->colors[elmnt_idx] = mapVec(point_cloud[i].color_rgba, bb_clr,
                                                              settings.grid_precision.color_precision[cell_idx]);
            t_curr_elmt[t_num][cell_idx] += 1;
        }

        time_t fill_grid = t.stopWatch();

        if(settings.verbose) {
            std::cout << "DONE building grid\n";
            std::cout << "  > took " << fill_grid << "ms.\n";
            std::cout << "    > offset calculation " << calc_offset << "ms.\n";
            std::cout << "    > filling grid " << fill_grid - calc_offset << "ms.\n";
        }
    }
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
    cell_range.x /= (float) pc_grid_->dimensions.x;
    cell_range.y /= (float) pc_grid_->dimensions.y;
    cell_range.z /= (float) pc_grid_->dimensions.z;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(1.0f,1.0f,1.0f));

    std::vector<std::vector<unsigned>> point_idx;
    std::vector<unsigned> white_cells;
    point_idx.resize(pc_grid_->cells.size());
    unsigned cell_offset = 0;
    for(unsigned i = 0; i < pc_grid_->cells.size(); ++i) {
        point_idx[i].resize(pc_grid_->cells[i]->size());
        for(unsigned j = 0; j < pc_grid_->cells[i]->size(); ++j)
            point_idx[i][j] = cell_offset+j;
        cell_offset += pc_grid_->cells[i]->size();
        if(pc_grid_->cells[i]->size() > 0)
            white_cells.emplace_back(i);
    }

    std::vector<Vec8> cell_idx_to_dim;
    cell_idx_to_dim.resize(pc_grid_->dimensions.x*pc_grid_->dimensions.y*pc_grid_->dimensions.z);
    for(uint8_t x_idx=0; x_idx < pc_grid_->dimensions.x; ++x_idx) {
        for (uint8_t y_idx = 0; y_idx < pc_grid_->dimensions.y; ++y_idx) {
            for (uint8_t z_idx = 0; z_idx < pc_grid_->dimensions.z; ++z_idx) {
                unsigned cell_idx = x_idx +
                    y_idx * pc_grid_->dimensions.x +
                    z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
                cell_idx_to_dim[cell_idx].x = x_idx;
                cell_idx_to_dim[cell_idx].y = y_idx;
                cell_idx_to_dim[cell_idx].z = z_idx;
            }
        }
    }

    Measure m;
    m.startWatch();

    #pragma omp parallel for
    for (unsigned i = 0; i < white_cells.size(); ++i) {
        unsigned cell_idx = white_cells[i];
        GridCell *cell = pc_grid_->cells[cell_idx];
        Vec<uint8_t> p_bits(
            cell->points.getNX(),
            cell->points.getNY(),
            cell->points.getNZ()
        );
        Vec<uint8_t> c_bits(
            cell->colors.getNX(),
            cell->colors.getNY(),
            cell->colors.getNZ()
        );
        Vec<float> glob_cell_min = Vec<float>(
            cell_range.x * cell_idx_to_dim[cell_idx].x,
            cell_range.y * cell_idx_to_dim[cell_idx].y,
            cell_range.z * cell_idx_to_dim[cell_idx].z
        );
        glob_cell_min += pc_grid_->bounding_box.min;
        Vec<float> pos_cell, clr;
        for (unsigned j = 0; j < cell->size(); ++j) {
            pos_cell = Encoder::mapVecToFloat(pc_grid_->cells[cell_idx]->points[j], bb_cell, p_bits);
            clr = Encoder::mapVecToFloat(pc_grid_->cells[cell_idx]->colors[j], bb_clr, c_bits);
            point_cloud->points[point_idx[cell_idx][j]] = pos_cell + glob_cell_min;
            point_cloud->colors[point_idx[cell_idx][j]] = clr;
        }
    }

    if(settings.verbose) {
        std::cout << "DECOMPRESSION done.\n";
        std::cout << "  > took " << m.stopWatch() << "ms.\n";
    }

    return true;//point_idx == point_cloud->size();
}

bool PointCloudGridEncoder::extractPointCloudFromGrid(std::vector<UncompressedVoxel>* point_cloud)
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
    cell_range.x /= (float) pc_grid_->dimensions.x;
    cell_range.y /= (float) pc_grid_->dimensions.y;
    cell_range.z /= (float) pc_grid_->dimensions.z;
    BoundingBox bb_cell(Vec<float>(0.0f,0.0f,0.0f), cell_range);
    BoundingBox bb_clr(Vec<float>(0.0f,0.0f,0.0f), Vec<float>(255.0f,255.0f,255.0f));

    std::vector<std::vector<unsigned>> point_idx;
    std::vector<unsigned> white_cells;
    point_idx.resize(pc_grid_->cells.size());
    unsigned cell_offset = 0;
    for(unsigned i = 0; i < pc_grid_->cells.size(); ++i) {
        point_idx[i].resize(pc_grid_->cells[i]->size());
        for(unsigned j = 0; j < pc_grid_->cells[i]->size(); ++j)
            point_idx[i][j] = cell_offset+j;
        cell_offset += pc_grid_->cells[i]->size();
        if(pc_grid_->cells[i]->size() > 0)
            white_cells.emplace_back(i);
    }

    std::vector<Vec8> cell_idx_to_dim;
    cell_idx_to_dim.resize(pc_grid_->dimensions.x*pc_grid_->dimensions.y*pc_grid_->dimensions.z);
    for(uint8_t x_idx=0; x_idx < pc_grid_->dimensions.x; ++x_idx) {
        for (uint8_t y_idx = 0; y_idx < pc_grid_->dimensions.y; ++y_idx) {
            for (uint8_t z_idx = 0; z_idx < pc_grid_->dimensions.z; ++z_idx) {
                unsigned cell_idx = x_idx +
                                    y_idx * pc_grid_->dimensions.x +
                                    z_idx * pc_grid_->dimensions.x * pc_grid_->dimensions.y;
                cell_idx_to_dim[cell_idx].x = x_idx;
                cell_idx_to_dim[cell_idx].y = y_idx;
                cell_idx_to_dim[cell_idx].z = z_idx;
            }
        }
    }

    Measure m;
    m.startWatch();

#pragma omp parallel for
    for (unsigned i = 0; i < white_cells.size(); ++i) {
        unsigned cell_idx = white_cells[i];
        GridCell *cell = pc_grid_->cells[cell_idx];
        Vec<uint8_t> p_bits(
                cell->points.getNX(),
                cell->points.getNY(),
                cell->points.getNZ()
        );
        Vec<uint8_t> c_bits(
                cell->colors.getNX(),
                cell->colors.getNY(),
                cell->colors.getNZ()
        );
        Vec<float> glob_cell_min = Vec<float>(
                cell_range.x * cell_idx_to_dim[cell_idx].x,
                cell_range.y * cell_idx_to_dim[cell_idx].y,
                cell_range.z * cell_idx_to_dim[cell_idx].z
        );
        glob_cell_min += pc_grid_->bounding_box.min;
        Vec<float> pos_cell, clr;
        for (unsigned j = 0; j < cell->size(); ++j) {
            pos_cell = Encoder::mapVecToFloat(pc_grid_->cells[cell_idx]->points[j], bb_cell, p_bits);
            pos_cell += glob_cell_min;
            clr = Encoder::mapVecToFloat(pc_grid_->cells[cell_idx]->colors[j], bb_clr, c_bits);
            (*point_cloud)[point_idx[cell_idx][j]].pos[0] = pos_cell.x;
            (*point_cloud)[point_idx[cell_idx][j]].pos[1] = pos_cell.y;
            (*point_cloud)[point_idx[cell_idx][j]].pos[2] = pos_cell.z;
            (*point_cloud)[point_idx[cell_idx][j]].color_rgba[0] = 255;
            (*point_cloud)[point_idx[cell_idx][j]].color_rgba[1] = (unsigned char) clr.x;
            (*point_cloud)[point_idx[cell_idx][j]].color_rgba[2] = (unsigned char) clr.y;
            (*point_cloud)[point_idx[cell_idx][j]].color_rgba[3] = (unsigned char) clr.z;
        }
    }

    if(settings.verbose) {
        std::cout << "DECOMPRESSION done.\n";
        std::cout << "  > took " << m.stopWatch() << "ms.\n";
    }

    return true;//point_idx == point_cloud->size();
}

zmq::message_t PointCloudGridEncoder::encodePointCloudGrid() {
    Measure m;
    m.startWatch();

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

    size_t offset = encodeGridHeader(message);
    offset = encodeBlackList(message, black_list, offset);

    time_t pre_cells = m.stopWatch();

    // Calculate offsets prior to message encoding
    // to be able to parallelize message creation
    std::vector<size_t> cell_offsets(cell_headers.size(), 0);
    for(unsigned i = 0; i < cell_headers.size(); ++i) {
        if(i == 0) {
            cell_offsets[i] += offset;
        }
        else {
            cell_offsets[i] = cell_offsets[i-1];
            cell_offsets[i] += CellHeader::getByteSize();
            cell_offsets[i] += BitVecArray::getByteSize(
                cell_headers[i-1]->num_elements,
                cell_headers[i-1]->point_encoding_x,
                cell_headers[i-1]->point_encoding_y,
                cell_headers[i-1]->point_encoding_z
            );
            cell_offsets[i] += BitVecArray::getByteSize(
                    cell_headers[i-1]->num_elements,
                    cell_headers[i-1]->color_encoding_x,
                    cell_headers[i-1]->color_encoding_y,
                    cell_headers[i-1]->color_encoding_z
            );
        }
    }

    // generate message content for cells in parallel
    #pragma omp parallel for
    for(unsigned i = 0; i < cell_headers.size(); ++i) {
        size_t temp_offset(cell_offsets[i]);
        temp_offset = encodeCellHeader(message, cell_headers[i], temp_offset);
        encodeCell(message, pc_grid_->cells[cell_headers[i]->cell_idx], temp_offset);
    }

    // Cleanup
    while(!cell_headers.empty()) {
        delete cell_headers.back();
        cell_headers.pop_back();
    }

    time_t post_cells = m.stopWatch();

    if(settings.verbose) {
        std::cout << "ENCODING done.\n";
        std::cout << "  > took " << post_cells << "ms.\n";
        std::cout << "    > pre-encode cells " << pre_cells << "ms.\n";
        std::cout << "    > encode cells " << post_cells-pre_cells << "ms.\n";
    }
    return message;
}

bool PointCloudGridEncoder::decodePointCloudGrid(zmq::message_t& msg)
{
    size_t offset = decodeGlobalHeader(msg);


    zmq::message_t decomp_msg(global_header_->uncompressed_size);
    
    if(global_header_->entropy_coding){
      decomp_msg = entropyDecompression(msg, offset);
    } else {
      memcpy((unsigned char*) decomp_msg.data(),(unsigned char*) msg.data() + offset, global_header_->uncompressed_size);
    }
    size_t old_offset = 0;
    offset = 0;
    offset = decodeGridHeader(decomp_msg, offset);
    if(offset == old_offset)
        return false;

    pc_grid_->resize(header_->dimensions);
    pc_grid_->bounding_box = header_->bounding_box;

    std::vector<unsigned> black_list;
    offset = decodeBlackList(decomp_msg, black_list, offset);

    std::set<unsigned> black_set;
    for(unsigned idx : black_list)
        black_set.insert(idx);

    Measure t;
    t.startWatch();

    // Extract Cell Headers to
    // calculate grid data offsets prior to message decoding
    // to be able to parallelize grid data extraction
    size_t num_cells = header_->dimensions.x * header_->dimensions.y * header_->dimensions.z;
    size_t num_white_cells = num_cells - black_set.size();
    // Stores message offset per whitelisted grid cell
    // offset encodes start position for memcpy to retrieve point&color data for cell
    std::vector<size_t> cell_offsets(num_white_cells, 0);
    // Stores cell header per whitelisted grid cell
    std::vector<CellHeader*> cell_headers(num_white_cells, nullptr);
    unsigned header_idx = 0;
    old_offset = offset;
    for(unsigned c_idx = 0; c_idx < num_cells; ++c_idx) {
        if (black_set.find(c_idx) != black_set.end())
            continue;
        cell_headers[header_idx] = new CellHeader;
        cell_headers[header_idx]->cell_idx = c_idx;
        if(header_idx == 0) {
            cell_offsets[header_idx] += offset;
            cell_offsets[header_idx] = decodeCellHeader(decomp_msg, cell_headers[header_idx], cell_offsets[header_idx]);
            if(cell_offsets[header_idx] == old_offset) {
                while(!cell_headers.empty()) {
                    delete cell_headers.back();
                    cell_headers.pop_back();
                }
                return false;
            }
        }
        else {
            cell_offsets[header_idx] = cell_offsets[header_idx-1];
            cell_offsets[header_idx] += BitVecArray::getByteSize(
                    cell_headers[header_idx-1]->num_elements,
                    cell_headers[header_idx-1]->point_encoding_x,
                    cell_headers[header_idx-1]->point_encoding_y,
                    cell_headers[header_idx-1]->point_encoding_z
            );
            cell_offsets[header_idx] += BitVecArray::getByteSize(
                    cell_headers[header_idx-1]->num_elements,
                    cell_headers[header_idx-1]->color_encoding_x,
                    cell_headers[header_idx-1]->color_encoding_y,
                    cell_headers[header_idx-1]->color_encoding_z
            );
            old_offset = cell_offsets[header_idx];
            cell_offsets[header_idx] = decodeCellHeader(decomp_msg, cell_headers[header_idx], cell_offsets[header_idx]);
            if(old_offset == cell_offsets[header_idx]) {
                while(!cell_headers.empty()) {
                    delete cell_headers.back();
                    cell_headers.pop_back();
                }
                return false;
            }
        }
        ++header_idx;
    }

    time_t pre_cell_decode = t.stopWatch();

    # pragma omp parallel for
    for(header_idx = 0; header_idx < cell_headers.size(); ++header_idx) {
        if(cell_offsets[header_idx] == decodeCell(decomp_msg, cell_headers[header_idx], cell_offsets[header_idx])) {
            std::cout << "WARNING: No points in cell\n  > Cell should've been blacklisted.\n";
        }
    }

    while(!cell_headers.empty()) {
        delete cell_headers.back();
        cell_headers.pop_back();
    }

    time_t post_cell_decode = t.stopWatch();

    if(settings.verbose) {
        std::cout << "DECODING CELLS done.\n  > took " << post_cell_decode << "ms.\n";
        std::cout << "    > decode headers " << pre_cell_decode << "ms.\n";
        std::cout << "    > decode cells " << post_cell_decode-pre_cell_decode << "ms.\n";
    }

    return true;
}

size_t PointCloudGridEncoder::encodeGlobalHeader(zmq::message_t &msg, size_t offset) {
    auto entropy_coding = new bool[1];
    entropy_coding[0] = global_header_->entropy_coding;
    size_t bytes_entropy(sizeof(unsigned char));
    memcpy((unsigned char*) msg.data() + offset,(unsigned char*) entropy_coding, sizeof(bool));
    offset += sizeof(bool);

    auto uncompressed_size = new unsigned long[2];
    uncompressed_size[0] = global_header_->uncompressed_size;
    uncompressed_size[1] = global_header_->appendix_size;
    memcpy((unsigned char*) msg.data() + offset, (unsigned char*) uncompressed_size, 2*sizeof(unsigned long));
    offset += 2*sizeof(unsigned long);

    // cleanup
    delete [] entropy_coding;
    delete [] uncompressed_size;
    return offset;
}

size_t PointCloudGridEncoder::decodeGlobalHeader(zmq::message_t &msg, size_t offset) {
    auto entropy_coding = new bool[1];
    memcpy((unsigned char*) entropy_coding, (unsigned char*) msg.data() + offset, sizeof(bool));
    global_header_->entropy_coding = entropy_coding[0];
    offset += sizeof(bool);

    auto uncompressed_size = new unsigned long[2];
    memcpy((unsigned char*) uncompressed_size, (unsigned char*) msg.data() + offset, 2*sizeof(unsigned long));
    global_header_->uncompressed_size = uncompressed_size[0];
    global_header_->appendix_size = uncompressed_size[1];
    offset += 2*sizeof(unsigned long);

    // cleanup
    delete [] entropy_coding;
    delete [] uncompressed_size;
    return offset;
}

size_t PointCloudGridEncoder::encodeGridHeader(zmq::message_t &msg, size_t offset) {
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

size_t PointCloudGridEncoder::decodeGridHeader(zmq::message_t& msg, size_t offset)
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

unsigned PointCloudGridEncoder::calcGridCellIndex(const float pos[3], const Vec<float>& cell_range) const {
    Vec<float> temp(pos[0], pos[1], pos[2]);
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

const Vec<float> PointCloudGridEncoder::mapToCell(const float pos[3], const Vec<float> &cell_range)
{
    Vec<float> cell_pos(pos[0], pos[1], pos[2]);
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
    // global header size
    size_t global_header_size = GlobalHeader::getByteSize();
    // message size
    size_t message_size = global_header_size;
    // header size
    size_t header_size = GridHeader::getByteSize();
    message_size += header_size;

    // blacklist size
    size_t blacklist_size = header_->num_blacklist*sizeof(unsigned);
    message_size += blacklist_size;
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

    if(settings.verbose) {
        std::cout << "HEADER SIZE (bytes) " << header_size << std::endl;
        std::cout << "BLACKLIST SIZE (bytes) " << blacklist_size << std::endl;
        std::cout << "CELLS\n";
        std::cout << " > ELEMENT COUNT " << num_elements << std::endl;
    }

    return message_size;
}
