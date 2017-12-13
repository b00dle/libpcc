#include "Measurement.hpp"
#include <cassert>
#include <iostream>


Measure::Measure()
{}

Measure::~Measure()
{}

  long Measure::setTimeStamp() {
    auto current_time = high_resolution_clock::now;
    std::cout << "Current Time Stamp: " << current_time << std::endl;

    long return_current_time = (long) current_time;

    return return_current_time;
  }

  long Measure::getTimeSpanStamps(long start_time, long end_time) {
    auto time_span = start_time - end_time;

    return time_span;
  }

  void Measure::stopWatch(bool running) {
    long start_time;
    long end_time;
    long time_span;

    if(running == true) {
      auto start_time_point = high_resolution_clock::now;
      start_time = (long) start_time_point;
    } else {
      auto end_time_point = high_resolution_clock::now;
      end_time = (long) end_time_point;

      time_span = start_time - end_time;

      std::cout << "Stopwatch counter: " << time_span << std::endl;
    }
  }

// uint8_t Measure::mapTo8Bit(float value, float min, float max)
// {
//     if(value < min) {
//         return 0;
//     }
//     else if(value > max) {
//         return 255;
//     }
//     else {
//         // map between 0-255;
//         float range = max - min;
//         value = (value - min) / range * 255;
//         value = std::max(0.0f, std::min(value, 255.0f));
//         return (uint8_t) value;
//     }
// }
