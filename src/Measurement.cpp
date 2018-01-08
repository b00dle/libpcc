#include "../include/Measurement.hpp"
#include "../include/Encoder.hpp"

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
  // void Measure::printTimeStamp(time_point<system_clock> time_point){
  //
  //   // auto print_time = duration<milliseconds>(time_point);
  //   std::time_t print_time = system_clock::to_time_t(time_point);
  //
  //
  //   std::cout << "TimeStamp Value (in seconds): " << print_time / 1000 << std::endl;
  //   std::cout << "TimeStamp Value (in milliseconds): " << print_time  << std::endl;
  // }
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
  // Start and stopWatch Functions.
  void Measure::startWatch() {
    m_start_time = time_point_cast<milliseconds>(system_clock::now());
  }
  std::time_t Measure::stopWatch() {
    time_point<system_clock> time_point = time_point_cast<milliseconds>(system_clock::now());;
    auto mil_sec = duration_cast<milliseconds>(time_point - m_start_time).count();

    return mil_sec;
  }


  //Calculate Mean Squared Error between two PointClouds
  std::vector<float> Measure::meanSquaredErrorPC(PointCloud<Vec<float>, Vec<float>> p1, PointCloud<Vec<float>, Vec<float>> p2) {
    std::vector<Vec<float>> p1_data_points = p1.points;
    std::vector<Vec<float>> p2_data_points = p2.points;
    std::vector<Vec<float>> p1_data_points_color = p1.colors;
    std::vector<Vec<float>> p2_data_points_color = p2.colors;
    //std::cout << p1_data_points.size() << std::endl;
    //std::cout << p2_data_points.size() << std::endl;

    std::list<float> min_distance_list;
    std::list<float> color_error_list;
    std::vector<float> data;
    float avg_error = 0;
    float color_error = 0;
    float avg_color_error = 0;

    for(unsigned p1_index = 0; p1_index < p1_data_points.size(); ++p1_index) {
      float nearest_pair = 10000;
      for(unsigned p2_index = 0; p2_index < p2_data_points.size(); ++p2_index) {
        float x_pair = (p1_data_points[p1_index].x - p2_data_points[p2_index].x) *
          (p1_data_points[p1_index].x - p2_data_points[p2_index].x);
        float y_pair = (p1_data_points[p1_index].y - p2_data_points[p2_index].y) *
          (p1_data_points[p1_index].y - p2_data_points[p2_index].y);
        float z_pair = (p1_data_points[p1_index].z - p2_data_points[p2_index].z) *
          (p1_data_points[p1_index].z - p2_data_points[p2_index].z);
        float distance_pair =  sqrt(x_pair + y_pair + z_pair);

        if(distance_pair < nearest_pair) {
          nearest_pair = distance_pair;
          color_error = colorErrorYuv(p1_data_points_color[p1_index], p2_data_points_color[p2_index]);
        }
      }
      min_distance_list.push_back(nearest_pair);
      color_error_list.push_back(color_error);
    }
    for(auto const& l : min_distance_list) {
      avg_error += l;
    }
    for(auto const& k : color_error_list) {
      avg_color_error += k;
    }

    avg_error = avg_error / min_distance_list.size();
    avg_color_error = avg_color_error / color_error_list.size();

    data.push_back(avg_error);
    data.push_back(avg_color_error);

    return data;
  }

  float Measure::colorErrorYuv(Vec<float> point1, Vec<float> point2) {
    //Calculate YUV Color values for point1 and point2
    std::cout << "-----------------" << std::endl;
    std::cout << "Original Color Values: " << std::endl;
    std::cout << point1.x << " : " << point1.y << " : " << point1.z << std::endl;
    std::cout << point2.x << " : " << point2.y << " : " << point2.z << std::endl;
    Vec<float> point1_yuv = Encoder::rgbToYuv(point1);

    std::cout << std::endl;
    std::cout << "Point 1:" << std::endl;
    std::cout << point1_yuv.x << std::endl;
    std::cout << point1_yuv.y << std::endl;
    std::cout << point1_yuv.z << std::endl;

    Vec<float> point2_yuv = Encoder::rgbToYuv(point2);

    std::cout << std::endl;
    std::cout << "Point 2:" << std::endl;
    std::cout << point2_yuv.x << std::endl;
    std::cout << point2_yuv.y << std::endl;
    std::cout << point2_yuv.z << std::endl;

    float color_y = (point1_yuv.x - point2_yuv.x) * (point1_yuv.x - point2_yuv.x);
    float color_u = (point1_yuv.y - point2_yuv.y) * (point1_yuv.y - point2_yuv.y);
    float color_v = (point1_yuv.z - point2_yuv.z) * (point1_yuv.z - point2_yuv.z);

    std::cout << std::endl;
    std::cout << "Color Difference: (by component)" << std::endl;
    std::cout << sqrt(color_y) << std::endl;
    std::cout << sqrt(color_u) << std::endl;
    std::cout << sqrt(color_v) << std::endl;

    float color_deviation = sqrt(color_y + color_u + color_v);

    std::cout << std::endl;
    std::cout << "Result:" << std::endl;
    std::cout << color_deviation << std::endl;

    return color_deviation;
  }

  float Measure::colorErrorYuvWithoutY(Vec<float> point1, Vec<float> point2) {
    //Calculate YUV Color values for point1 and point2
    Vec<float> point1_yuv = Encoder::rgbToYuv(point1);
    Vec<float> point2_yuv = Encoder::rgbToYuv(point2);

    float color_u = (point1_yuv.y - point2_yuv.y) * (point1_yuv.y - point2_yuv.y);
    float color_v = (point1_yuv.z - point2_yuv.z) * (point1_yuv.z - point2_yuv.z);

    float color_deviation = sqrt(color_u + color_v);

    return color_deviation;
  }
