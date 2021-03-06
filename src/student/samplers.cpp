#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace Samplers {

Vec2 Rect::Uniform::sample(float& pdf) const {
    // TODO (PathTracer): Task 1

    // The PDF should integrate to 1 over the whole rectangle. Because we're
    // uniformly sampling over a rectangle, each point
    float area = size.x * size.y;
    pdf = 1.0f / area;

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    float rand_x = RNG::unit() * size.x;
    float rand_y = RNG::unit() * size.y;
    return Vec2(rand_x, rand_y);
}

Vec3 Hemisphere::Cosine::sample(float& pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.
    return Vec3();
}

Vec3 Sphere::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 7
    //
    // Generate a uniformly random point on the unit sphere (or equivalently,
    // direction) Tip: start with Hemisphere::Uniform

    // Essentially, generate a hemisphere point, and with a 50/50 chance, negate
    // it.
    Vec3 sampled_vec = hemi.sample(pdf);
    if(RNG::coin_flip())
    {
        sampled_vec *= -1;
    }
    printf("{Sphere::Sample}\n\tVec: [%.2f, %.2f, %.2f] (Norm: %.2f)\n",
           sampled_vec.x, sampled_vec.y, sampled_vec.z, sampled_vec.norm());
    assert(sampled_vec.norm() >= 0.98f);

    pdf = 1/(4 * PI_F); // what was the PDF at the chosen direction?
    return sampled_vec;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
}

Vec3 Sphere::Image::sample(float& out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    out_pdf = 1.0f; // what was the PDF (again, PMF here) of your chosen sample?
    return Vec3();
}

Vec3 Point::sample(float& pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float& pmf) const {
    if(RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float& pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
