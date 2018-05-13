#ifndef LIBPCC_MEASUREMENT_HPP
#define LIBPCC_MEASUREMENT_HPP

#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <cassert>

#include "Encoder.hpp"

/**
 * Wraps simple time measurement tasks and
 * provides a static interface to compare point clouds (points & colors).
*/
class Measure {
public:
    /**
     * Data transfer object to encase results
     * of a Measure::compare operation.
    */
    struct ComparisonResult {
        ComparisonResult()
            : avg_pos_error(-1)
            , avg_clr_error(-1)
            , pos_variance(-1)
            , clr_variance(-1)
            , max_pos_error(-1)
            , max_clr_error(-1)
        {}

        float avg_pos_error;
        float avg_clr_error;
        float pos_variance;
        float clr_variance;
        float max_pos_error;
        float max_clr_error;
    };

    typedef std::chrono::system_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;
    typedef std::chrono::milliseconds Milliseconds;

public:
    Measure();
    ~Measure();

    /**
     * Starts the internal watch of this instance.
    */
    void startWatch();

    /**
     * Stops the internal watch of this instance and
     * returns the time difference since starting.
    */
    std::time_t stopWatch();

    /**
     * Returns a timestamp to current time (in milliseconds).
    */
    static Measure::TimePoint now();

    /**
     * Returns the time difference between given time_points (in milliseconds).
     *
    */
    static std::time_t span(Measure::TimePoint, Measure::TimePoint);

    /**
     * Prints given timestamp.
    */
    static void print(std::time_t);

    /**
     * Prints given Measure::ComparisonResult.
    */
    static void print(const Measure::ComparisonResult&);

    /**
     * Finds closest voxel in p2 for each voxel in p1. Distances are then stored
     * For each closest distance a color error is computed and stored.
     * Average, variance and max is returned for point and color errors.
     * Result is wrapped in Measure::ComparisonResult struct.
     *
    */
    static const ComparisonResult compare(std::vector<UncompressedVoxel> const & p1,
                                          std::vector<UncompressedVoxel> const & p2,
                                          BoundingBox const & bb);

    /**
     * Computes color error in YUV space.
     * Values should be given in RGB space with component values in range [0,1].
    */
    static float colorErrorYuv(const Vec<float>&, const Vec<float>&);

    /**
     * Computes color error between given voxels in YUV space.
    */
    static float colorErrorYuv(const UncompressedVoxel& v1, const UncompressedVoxel& v2);

    /**
     * Computes color error in YUV space without taking Y into account.
     * Values should be given in RGB space with component values in range [0,1].
    */
    static float colorErrorYuvWithoutY(const Vec<float>&, const Vec<float>&);

    /**
     * Computes color error between given voxels in YUV space
     * without taking Y into account.
    */
    static float colorErrorYuvWithoutY(const UncompressedVoxel& v1, const UncompressedVoxel& v2);

    /**
     * Computes color error in Cielab space.
     * Values should be given in RGB space with component values in range [0,1].
    */
    static float colorErrorCielab(const Vec<float>&, const Vec<float>&);

    /**
     * Computes color error between given voxels in Cielab space.
    */
    static float colorErrorCielab(const UncompressedVoxel& v1, const UncompressedVoxel& v2);

    /**
     * Computes color error in XYZ space.
     * Values should be given in RGB space with component values in range [0,1].
    */
    static float colorErrorXyz(const Vec<float>&, const Vec<float>&);

    /**
     * Computes color error between given voxels in XYZ space.
    */
    static float colorErrorXyz(const UncompressedVoxel& v1, const UncompressedVoxel& v2);

    /**
     * Computes variance for values in given std::vector.
    */
    static float calcVariance(const std::vector<float>&);

private:
    Measure::TimePoint start_time_;
};


#endif // #ifndef  LIBPCC_MEASUREMENT_HPP
