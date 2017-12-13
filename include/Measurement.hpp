#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <math.h>
#include <iostream>
#include <chrono>

using namespace std::chrono;

class Measure {


public:
  Measure();
  ~Measure();

  long setTimeStamp();
  long getTimeSpanStamps(long, long);
  void stopWatch(bool);

private:
};


#endif // #ifndef  MEASUREMENT_HPP
