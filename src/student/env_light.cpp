
#include "../rays/env_light.h"
#include "debug.h"

#include <limits>
#include <algorithm>

namespace PT {

/// Converts a point within [range_lo, range_hi] into a range [0, 1].
float to_range(float point, float range_lo, float range_hi)
{
    float range = (range_hi - range_lo);
    return ((point - range_lo) / range);
}

Light_Sample Env_Map::sample() const {

    Light_Sample ret;
    ret.distance = std::numeric_limits<float>::infinity();

    // TODO (PathTracer): Task 7
    // Uniformly sample the sphere. Tip: implement Samplers::Sphere::Uniform
    Samplers::Sphere::Uniform uniform;
    ret.direction = uniform.sample(ret.pdf);

    // Once you've implemented Samplers::Sphere::Image, remove the above and
    // uncomment this line to use importance sampling instead.
    // ret.direction = sampler.sample(ret.pdf);

    ret.radiance = sample_direction(ret.direction);
    return ret;
}

/// @brief Perform linear interpolation between the values LHS and RHS according
/// to the following formula:
///
/// (t * LHS) + (1 - t) * RHS
Spectrum lerp(const Spectrum& lhs, const Spectrum rhs, float t)
{
    return (t * lhs) + (1 - t) * rhs;
}

Spectrum Env_Map::sample_direction(Vec3 dir) const {

    // TODO (PathTracer): Task 7
    // Find the incoming light along a given direction by finding the
    // corresponding place in the enviornment image. You should bi-linearly
    // interpolate the value between the 4 image pixels nearest to the exact
    // direction.

    if(dir.norm() < 0.5f)
    {
        float pdf;
        Samplers::Sphere::Uniform uniform;
        dir = uniform.sample(pdf);
    }

    // Find the spherical coordinates from (x, y, z)
    // printf("Vec: [%.2f, %.2f, %.2f] (Norm: %.2f)\n", dir.x, dir.y, dir.z, dir.norm());
    assert(dir.norm() >= .99f);
 
    const float theta = acosf(
        dir.z /
        (sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z))
        );


   float phi =((atan2(dir.y, dir.x)) + PI_F);

    // Samples in environment map.
    const float sample_x = to_range(phi, 0, 2 * PI_F);
    const float sample_y = to_range(theta, 0, PI_F);

    // Find lerp t-values
    float sample_x_t = to_range(sample_x, floorf(sample_x), ceilf(sample_x));
    float sample_y_t = to_range(sample_y, floorf(sample_y), ceilf(sample_y));

    // Pixels in environment image map ()
    const std::pair<size_t, size_t>& dimensions =  image.dimension();
    size_t x = std::clamp((size_t)(sample_x * dimensions.first), 1UL, dimensions.first - 1);
    size_t y = std::clamp((size_t)(sample_y * dimensions.second), 1UL,  dimensions.second - 1);
    // y = dimensions.second - y;

    printf("(X, Y) : (%zu, %zu) | (%zu, %zu)\n", x, y,
           dimensions.first, dimensions.second);

    // Perform bilinear interpolation.
    const size_t x_1 = (x >= dimensions.first - 2) ? x : x + 1;
    const size_t y_1 = (y >= dimensions.second - 2) ? y : y + 1;

    const Spectrum& bot_left = image.at(x, y);
    const Spectrum& bot_right = image.at(x_1, y);
    const Spectrum& top_left = image.at(x, y_1);
    const Spectrum& top_right = image.at(x_1, y_1);

    const Spectrum bottom = lerp(bot_left, bot_right, sample_x_t);
    const Spectrum top = lerp(top_left, top_right, sample_x_t);

    const Spectrum bilerp = lerp(bottom, top, sample_y_t);

    return bilerp;
}

Light_Sample Env_Hemisphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Hemisphere::sample_direction(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Light_Sample Env_Sphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Sphere::sample_direction(Vec3) const {
    return radiance;
}

} // namespace PT
