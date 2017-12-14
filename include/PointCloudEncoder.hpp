#ifndef POINT_CLOUD_ENCODER_HPP
#define POINT_CLOUD_ENCODER_HPP

#include "Encoder.hpp"
#include "PointCloud.hpp"
#include <zmq.hpp>

#include <vector>
#include <string>

class PointCloudEncoder : public Encoder {

public:
  enum Codec {
    PC_3x32p_3x8c,
    PC_3x8p_3x8c,
    PC_1x32p_1x32c
  };

public:
  PointCloudEncoder();
  ~PointCloudEncoder();

  /* Compresses given PointCloud and creates message from it */
  zmq::message_t encode(PointCloud<Vec32, Vec32>* point_cloud, Codec codec=PC_3x32p_3x8c);  

  /* Decodes given message into point_cloud. Returns success. */
  bool decode(zmq::message_t& msg, PointCloud<Vec32, Vec32>* point_cloud);

private:
  /* Compress from_pc to 32Bit component point and 8Bit component color. */
  void compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<Vec32, Vec8>* to_pc);

  /* Compress from_pc to 8Bit component point and 8Bit component color. */
  void compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<Vec8, Vec8>* to_pc);

  /* Compress from_pc to 32Bit (x=11bit, y=10bit, z=11bit) point and 32bit (r=10bit, g=12bit, b=10bit) color. */
  void compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<uint32_t, uint32_t>* to_pc);

  /* reverse compress from_pc to to_pc */
  void decompress(PointCloud<Vec32, Vec8>* from_pc, PointCloud<Vec32, Vec32>* to_pc);

  /* reverse compress from_pc to to_pc */
  void decompress(PointCloud<Vec8, Vec8>* from_pc, PointCloud<Vec32, Vec32>* to_pc);

  /* reverse compress from_pc to to_pc */
  void decompress(PointCloud<uint32_t, uint32_t>* from_pc, PointCloud<Vec32, Vec32>* to_pc);

  /* Creates a zmq message from given template point cloud */
  template<typename P, typename C>
  zmq::message_t createMessage(PointCloud<P, C>* point_cloud);

  /* 
   * Helper function for decode,
   * extracts PointCloud (pc) from message data.
   * num_points should state the expected number of points in the PC,
   * which can be read from header.
   * start_offset indicates from which position in the message data
   * the PC data should be read.
  */
  template<typename P, typename C>
  void extractPC(PointCloud<P, C>* pc, zmq::message_t& msg, unsigned num_points, unsigned start_offset);

  size_t header_size_bytes_;
  float* header_;
};


#endif // #ifndef  POINT_CLOUD_ENCODER_HPP

