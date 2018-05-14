#include "Measure.hpp"

Measure::Measure()
  : start_time_()
{}

Measure::~Measure()
{}

Measure::TimePoint Measure::now()
{
    return std::chrono::time_point_cast<Milliseconds>(Clock::now());
}

std::time_t Measure::span(TimePoint start_point, TimePoint end_point)
{
    return std::chrono::duration_cast<Milliseconds>(end_point - start_point).count();
}

void Measure::print(std::time_t period_time)
{
    std::cout << "TimePeriod Value (in seconds): " << period_time / 1000 << std::endl;
    std::cout << "TimePeriod Value (in milliseconds): " << period_time  << std::endl;
}

void Measure::print(const Measure::ComparisonResult & data)
{
    std::cout << "PC Differences " << std::endl;
    std::cout << "  > average position error " << data.avg_pos_error << std::endl;
    std::cout << "  > average color error " << data.avg_clr_error << std::endl;
    std::cout << "  > position variance " << data.pos_variance << std::endl;
    std::cout << "  > color variance " << data.clr_variance << std::endl;
    std::cout << "  > max position error " << data.max_pos_error << std::endl;
    std::cout << "  > max color error " << data.max_clr_error << std::endl;
}

void Measure::startWatch()
{
    start_time_ = std::chrono::time_point_cast<Milliseconds>(Clock::now());
}

std::time_t Measure::stopWatch()
{
    TimePoint time_point = std::chrono::time_point_cast<Milliseconds>(Clock::now());;
    return std::chrono::duration_cast<Milliseconds>(time_point - start_time_).count();
}

const Measure::ComparisonResult Measure::compare(std::vector<UncompressedVoxel> const &p1,
                                                 std::vector<UncompressedVoxel> const &p2, BoundingBox const &bb)
{
    std::vector<float> min_distances(p1.size());
    std::vector<float> color_errors(p1.size());

#pragma omp parallel for
    for(size_t p1_idx = 0; p1_idx < p1.size(); ++p1_idx) {
        if(!bb.contains(p1[p1_idx].pos))
            continue;
        float closest_distance = 100000;
        float clr_error = 0;
        for(auto p2_voxel : p2) {
            if(!bb.contains(p2_voxel.pos))
                continue;
            float x_dist = (p1[p1_idx].pos[0] - p2_voxel.pos[0]) *
                           (p1[p1_idx].pos[0] - p2_voxel.pos[0]);
            float y_dist = (p1[p1_idx].pos[1] - p2_voxel.pos[1]) *
                           (p1[p1_idx].pos[1] - p2_voxel.pos[1]);
            float z_dist = (p1[p1_idx].pos[2] - p2_voxel.pos[2]) *
                           (p1[p1_idx].pos[2] - p2_voxel.pos[2]);
            auto distance = static_cast<float>(sqrt(x_dist + y_dist + z_dist));
            if(distance < closest_distance) {
                closest_distance = distance;
                clr_error = colorErrorCielab(p1[p1_idx], p2_voxel);
            }
        }
        min_distances[p1_idx] = closest_distance;
        color_errors[p1_idx] = clr_error;
    }

    float max_pos_error = 0;
    float avg_pos_error = 0;
    for(auto const& l : min_distances) {
        avg_pos_error += l;
        if(max_pos_error < l)
            max_pos_error = l;
    }
    avg_pos_error = avg_pos_error / min_distances.size();

    float max_clr_error = 0;
    float avg_clr_error = 0;
    for(auto const& k : color_errors) {
        avg_clr_error += k;
        if(max_clr_error < k) {
            max_clr_error = k;
        }
    }
    avg_clr_error = avg_clr_error / color_errors.size();

    float pos_variance = calcVariance(min_distances);
    float clr_variance = calcVariance(color_errors);

    ComparisonResult data;
    data.avg_pos_error = avg_pos_error;
    data.pos_variance = pos_variance;
    data.max_pos_error = max_pos_error;
    data.avg_clr_error = avg_clr_error;
    data.clr_variance = clr_variance;
    data.max_clr_error = max_clr_error;

    return data;
}

float Measure::colorErrorYuv(const Vec<float>& point1, const Vec<float>& point2)
{
    Vec<float> point1_yuv = Encoder::rgbToYuv(point1);
    Vec<float> point2_yuv = Encoder::rgbToYuv(point2);

    float color_y = (point1_yuv.x - point2_yuv.x) * (point1_yuv.x - point2_yuv.x);
    float color_u = (point1_yuv.y - point2_yuv.y) * (point1_yuv.y - point2_yuv.y);
    float color_v = (point1_yuv.z - point2_yuv.z) * (point1_yuv.z - point2_yuv.z);

    auto color_deviation = static_cast<float>(sqrt(color_y + color_u + color_v));

    return color_deviation;
}

float Measure::colorErrorYuv(const UncompressedVoxel &v1, const UncompressedVoxel &v2) {
    return colorErrorYuv(
        Encoder::bit8ToRgb(v1.color_rgba),
        Encoder::bit8ToRgb(v2.color_rgba)
    );
}

float Measure::colorErrorYuvWithoutY(const Vec<float>& point1, const Vec<float>& point2)
{
    Vec<float> point1_yuv = Encoder::rgbToYuv(point1);
    Vec<float> point2_yuv = Encoder::rgbToYuv(point2);

    float color_u = (point1_yuv.y - point2_yuv.y) * (point1_yuv.y - point2_yuv.y);
    float color_v = (point1_yuv.z - point2_yuv.z) * (point1_yuv.z - point2_yuv.z);

    auto color_deviation = static_cast<float>(sqrt(color_u + color_v));

    return color_deviation;
}

float Measure::colorErrorYuvWithoutY(const UncompressedVoxel &v1, const UncompressedVoxel &v2) {
    return colorErrorYuvWithoutY(
        Encoder::bit8ToRgb(v1.color_rgba),
        Encoder::bit8ToRgb(v2.color_rgba)
    );
}

float Measure::colorErrorXyz(const Vec<float>& point1, const Vec<float>& point2) {
    Vec<float> point1_lab = Encoder::rgbToXyz(point1);
    Vec<float> point2_lab = Encoder::rgbToXyz(point2);

    float color_x = (point1_lab.x - point2_lab.x) * (point1_lab.x - point2_lab.x);
    float color_y = (point1_lab.y - point2_lab.y) * (point1_lab.y - point2_lab.y);
    float color_z = (point1_lab.z - point2_lab.z) * (point1_lab.z - point2_lab.z);

    auto color_deviation = static_cast<float>(sqrt(color_x + color_y + color_z));

    return color_deviation;
}

float Measure::colorErrorXyz(const UncompressedVoxel &v1, const UncompressedVoxel &v2) {
    return colorErrorXyz(
        Encoder::bit8ToRgb(v1.color_rgba),
        Encoder::bit8ToRgb(v2.color_rgba)
    );
}

float Measure::colorErrorCielab(const Vec<float>& point1, const Vec<float>& point2) {
    //Calculate YUV Color values for point1 and point2
    Vec<float> point1_lab = Encoder::rgbToCieLab(point1);
    Vec<float> point2_lab = Encoder::rgbToCieLab(point2);

    float color_L = (point1_lab.x - point2_lab.x) * (point1_lab.x - point2_lab.x);
    float color_a = (point1_lab.y - point2_lab.y) * (point1_lab.y - point2_lab.y);
    float color_b = (point1_lab.z - point2_lab.z) * (point1_lab.z - point2_lab.z);

    auto color_deviation = static_cast<float>(sqrt(color_L + color_a + color_b));

    return color_deviation;
}

float Measure::colorErrorCielab(const UncompressedVoxel &v1, const UncompressedVoxel &v2) {
    return colorErrorCielab(
        Encoder::bit8ToRgb(v1.color_rgba),
        Encoder::bit8ToRgb(v2.color_rgba)
    );
}

float Measure::calcVariance(const std::vector<float>& values)
{
    float avg_error = 0;
    float variance = 0;

    for(auto const& l : values)
        avg_error += l;

    avg_error = avg_error / values.size();

    for(auto const& l : values)
        variance +=  (l - avg_error) * (l - avg_error);

    return variance / values.size();
}
