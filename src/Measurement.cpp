#include "Measurement.hpp"
#include <cassert>
#include <iostream>


Measure::Measure()
  : m_start_time()
{}

Measure::~Measure()
{}

  // Sets current TimeStamp and returns it as Time Point.
  time_point<system_clock> Measure::createTimeStamp() {

    time_point<system_clock> time_point;
    time_point = time_point_cast<milliseconds>(system_clock::now());

    return time_point;
  }

  void Measure::printTimeStamp(time_point<system_clock> time_point){

    // auto print_time = duration<milliseconds>(time_point);
    std::time_t print_time = system_clock::to_time_t(time_point);


    std::cout << "TimeStamp Value (in seconds): " << print_time / 1000 << std::endl;
    std::cout << "TimeStamp Value (in milliseconds): " << print_time  << std::endl;
  }

  std::time_t Measure::calcTimeSpan(time_point<system_clock> start_point,
                                        time_point<system_clock> end_point) {

    auto mil_sec = duration_cast<milliseconds>(end_point - start_point).count();

    return mil_sec;
  }

  // Prints TimeSpan.
  void Measure::printTimeSpan(std::time_t period_time) {
    std::cout << "TimePeriod Value (in seconds): " << period_time / 1000 << std::endl;
    std::cout << "TimePeriod Value (in milliseconds): " << period_time  << std::endl;
  }

  // Start and stopWatch Functions
  void Measure::startWatch() {
    m_start_time = time_point_cast<milliseconds>(system_clock::now());
  }

  std::time_t Measure::stopWatch() {
    time_point<system_clock> time_point = time_point_cast<milliseconds>(system_clock::now());;
    auto mil_sec = duration_cast<milliseconds>(time_point - m_start_time).count();

    return mil_sec;
  }
