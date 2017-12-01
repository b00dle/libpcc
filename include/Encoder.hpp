#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <math.h>
#include <PointCloud.hpp>

class Encoder {

public:
  Encoder();
  ~Encoder();

  ////////////////////////////////////////////
  ////////// STATIC MAPPING HELPERS //////////
  ////////////////////////////////////////////

  static uint8_t mapTo8Bit(float value, float min, float max);
  static float mapTo32Bit(uint8_t value, float min, float max, float invalid = 0.0f);
  static float mapToRange(float value, float min, float max, float range_max);
  static uint32_t mapToBit(float value, float min, float max, uint8_t bits);
  static float mapFromBit(uint32_t value, float min, float max, uint8_t bits, float invalid = 0.0f);
  
  static Vec8 const Vec32ToVec8(Vec32 const& v);
  static Vec8 const Vec32ToVec8(Vec32 const& v, BoundingBox const& bb);
  static Vec32 const Vec8ToVec32(Vec8 const& v);
  static Vec32 const Vec8ToVec32(Vec8 const& v, BoundingBox const& bb);
  static uint32_t Vec32ToUInt32(Vec32 const& v, unsigned short x_bits, unsigned short y_bits, unsigned short z_bits);
  static uint32_t Vec32ToUInt32(Vec32 const& v, BoundingBox const& bb, unsigned short x_bits, unsigned short y_bits, unsigned short z_bits);
  static Vec32 UInt32ToVec32(uint32_t v, unsigned short x_bits, unsigned short y_bits, unsigned short z_bits);
  static Vec32 UInt32ToVec32(uint32_t v, BoundingBox const& bb, unsigned short x_bits, unsigned short y_bits, unsigned short z_bits);

private:
};


#endif // #ifndef  ENCODER_HPP

