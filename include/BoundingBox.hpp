//
// Created by basti on 31.12.17.
//

#ifndef LIBPCC_BOUNDINGBOX_HPP
#define LIBPCC_BOUNDINGBOX_HPP

struct BoundingBox {
    explicit BoundingBox(float x_min_t=.0f, float x_max_t=.0f, float y_min_t=.0f, float y_max_t=.0f, float z_min_t=.0f, float z_max_t=.0f)
            : min(x_min_t, y_min_t, z_min_t)
            , max(x_max_t, y_max_t, z_max_t)
    {}

    explicit BoundingBox(const Vec<float>& min_t, const Vec<float>& max_t)
            : min(min_t)
            , max(max_t)
    {}

    BoundingBox(BoundingBox const& bb) = default;

    ~BoundingBox() = default;

    bool contains(Vec<float> const& v) const
    {
        return v.x > min.x && v.x < max.x &&
               v.y > min.y && v.y < max.y &&
               v.z > min.z && v.z < max.z;
    }

    Vec<float> const calcRange() const {
        return max - min;
    }

    Vec<float> min;
    Vec<float> max;
};

#endif //LIBPCC_BOUNDINGBOX_HPP
