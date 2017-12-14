#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std::chrono;

class Measure {


public:
  Measure();
  ~Measure();

  // Time Measurement Functions
  time_point<system_clock, nanoseconds> setTimeStamp();
  void printTimeStamp(time_point<system_clock, nanoseconds>);

  std::time_t getTimeSpanStamps(time_point<system_clock, nanoseconds>, time_point<system_clock, nanoseconds>);
  void printTimePeriod(std::time_t);

  void stopWatch(bool);

private:
};


#endif // #ifndef  MEASUREMENT_HPP
