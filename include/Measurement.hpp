#ifndef LIBPCC_MEASUREMENT_HPP
#define LIBPCC_MEASUREMENT_HPP

#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <list>
#include <cassert>
#include "PointCloud.hpp"

using namespace std::chrono;

class Measure {


public:
  Measure();
  ~Measure();

  // Time Measurement Functions
  time_point<system_clock> createTimeStamp();
  // void printTimeStamp(time_point<system_clock>);
  std::time_t calcTimeSpan(time_point<system_clock>, time_point<system_clock>);
  void printTimeSpan(std::time_t);
  void startWatch();
  std::time_t stopWatch();

  // PointCloud Analytics Functions
  std::vector<float> comparePC(PointCloud<Vec<float>, Vec<float>> const&, PointCloud<Vec<float>, Vec<float>> const&, BoundingBox const&);
  void printResultsPC(std::vector<float>);

  float colorErrorYuv(Vec<float>, Vec<float>);
  float colorErrorYuvWithoutY(Vec<float>, Vec<float>);
  float colorErrorCielab(Vec<float>, Vec<float>);
  float colorErrorXyz(Vec<float>, Vec<float>);
  float calcVariance(std::list<float>);

private:
time_point<system_clock> m_start_time;
};


#endif // #ifndef  LIBPCC_MEASUREMENT_HPP
