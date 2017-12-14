#include "Measurement.hpp"
#include <cassert>
#include <iostream>


Measure::Measure()
{}

Measure::~Measure()
{}

  // Sets current TimeStamp and returns it as Time Point.
  time_point<system_clock, nanoseconds> Measure::setTimeStamp() {

    time_point<system_clock, nanoseconds> time_point;
    time_point = time_point_cast<nanoseconds>(system_clock::now());

    return time_point;
  }

  void Measure::printTimeStamp(time_point<system_clock, nanoseconds> time_point){
    std::time_t print_time = system_clock::to_time_t(time_point);

    std::cout << "TimeStamp Value: " << print_time / 1000000000 << std::endl;
    std::cout << "TimeStamp Value (in nanoseconds): " << print_time  << std::endl;
  }

  // Takes two longs as TimeStamps and calculates TimeSpan.
  std::time_t Measure::getTimeSpanStamps(time_point<system_clock, nanoseconds> start_point,
                                        time_point<system_clock, nanoseconds> end_point) {

    std::time_t start_time = system_clock::to_time_t(start_point);
    std::time_t end_time = system_clock::to_time_t(end_point);

    std::time_t period_time = end_time - start_time;

    return period_time;
  }

  // Prints TimeSpan.
  void Measure::printTimePeriod(std::time_t period_time) {
    std::cout << "TimeStamp Value: " << period_time / 1000000000 << std::endl;
    std::cout << "TimeStamp Value (in nanoseconds): " << period_time  << std::endl;
  }

  // Takes bool. For 'true' sets first TimeStamp.
  // For 'false' sets second TimeStamp, calculates and prints TimeSpan.
  void Measure::stopWatch(bool running) {
    time_point<system_clock, nanoseconds> time_point;
    std::time_t start_time;
    std::time_t end_time;

    if(running == true) {
      time_point = time_point_cast<nanoseconds>(system_clock::now());
      start_time = system_clock::to_time_t(time_point);
    } else {
      time_point = time_point_cast<nanoseconds>(system_clock::now());
      end_time = system_clock::to_time_t(time_point);

      if(start_time > 0) {
        std::time_t period_time = end_time - start_time;
        std::cout << "Stopwatch counter: " << period_time << std::endl;
      }
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
