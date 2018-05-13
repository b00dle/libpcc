#ifndef LIBPCC_UNCOMPRESSEDVOXEL_HPP
#define LIBPCC_UNCOMPRESSEDVOXEL_HPP

/**
 * Data transfer object to define basic (property) layout of a voxel.
 * Offers a 3D position as well as an 8 bit rgba-color.
*/
struct UncompressedVoxel {
    float pos[3];
    unsigned char color_rgba[4];
};

#endif //LIBPCC_UNCOMPRESSEDVOXEL_HPP
