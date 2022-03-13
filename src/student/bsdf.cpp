
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {
    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    Vec3 normal = Vec3(0, 1, 0);

    return -1 * dir + 2 * (dot(dir, normal) * normal);
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // Critical angle for glass is about 41 degrees
    /// @todo  How to generalize if we're _only_ given index of refraction?
    // (Source: https://www.physicsclassroom.com/class/refrn/Lesson-3/The-Critical-Angle)
    const float glass_crit_angle_deg = 41.1;
    // in radians
    const float glass_crit_angle = (PI_F * glass_crit_angle_deg) / 180.0f;

    // Since glass_crit_angle is a constant, we can precompute the cosine. If
    // dot(N, d) is < cos(theta_crit), then that means the angle was larger than
    // the critical angle, meaning TIR occurred.
    const float cos_crit_angle = cos(glass_crit_angle);

    const Vec3 normal = Vec3(0, 1, 0);
    if(dot(out_dir, normal) < cos_crit_angle)
    {
        was_internal = false;
        return reflect(out_dir);
    }
    was_internal = true;
    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to
    // a ray exiting the surface into vaccum (ior = 1). However, note that you
    // should actually treat this case as _entering_ the surface, because you
    // want to compute the 'input' direction that would cause this output, and
    // to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    // Otherwise, use snell's law.

    
    Vec3 refraction_dir;
    return refraction_dir;
}


/// @brief Given a unit normal with directions (x, y, z), samples a hemisphere
/// for a direction. Additionally, it ensures the sampled vector is oriented in
/// the same direction as the normal (dot product is 0).
BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    // (2) Randomly select a new ray direction (it may be reflection or
    // transmittance ray depending on surface type) using bsdf.sample()
    float pdf;

    Vec3 hemisphere_dir = sampler.sample(pdf);

    if(dot(out_dir, hemisphere_dir) < 0)
        hemisphere_dir *= -1;

    BSDF_Sample ret;

    // What is the ratio of reflected/incoming light?
    ret.attenuation = albedo;

    // What direction should we sample incoming light from?
    ret.direction = hemisphere_dir.unit();
    ret.emissive = albedo;
    // Was was the PDF of the sampled direction?
    ret.pdf = pdf;
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;

    // What is the ratio of reflected/incoming light?
    ret.attenuation = Spectrum(1.0);

    // What direction should we sample incoming light from?
    ret.direction = reflect(out_dir);

    // Was was the PDF of the sampled direction? (In this case, the PMF)
    ret.pdf = 1.0f;
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement glass BSDF.
    const Vec3 normal(0, 1, 0);

    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // https://en.wikipedia.org/wiki/Schlick%27s_approximation
    const float cos_angle = dot(out_dir.unit(), normal);

    // Because the interface n1 is usually air, we can assume n_1 = 1.
    const float n_air = 1;
    const float n_glass = index_of_refraction;

    const float tmp = (n_air - n_glass) / (n_air + n_glass);
    const float r_0 = (tmp * tmp);

    // R_0 in Schlick approximation.
    const float reflection_coeff = r_0 + (1 - r_0) * powf(1 - cos_angle, 5);

    // (2) Reflect or refract probabilistically based on Fresnel coefficient.
    // Tip: RNG::coin_flip
    Vec3 ray;
    bool reflection = RNG::coin_flip(reflection_coeff);
    bool was_internal = true;
    if(reflection)
    { // Generate reflection ray (PDF = 1)
        ray = reflect(out_dir);

        // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
        BSDF_Sample ret;
        ret.attenuation = reflectance; // What is the ratio of reflected/incoming light?
        ret.direction = ray;       // What direction should we sample incoming light from?
        ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
        return ret;
    }

    // Generate refraction ray.
    ray = refract(out_dir, n_glass, was_internal);
    // (3) Compute attenuation based on reflectance or transmittance
    
    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    BSDF_Sample ret;
    ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
    ret.direction = ray;       // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the
    // surface?
    float refraction_index = index_of_refraction;

    /// If the "out direction" is on the same side as the normal, we can assume
    /// the out direction is exiting the surface into the vacuum, so we invert
    /// the refraction index.
    const Vec3 normal = Vec3(0, 1, 0);
    if(dot(out_dir, normal) > 0)
    {
        refraction_index = 1 / refraction_index;
    }

    bool was_internal;
    BSDF_Sample ret;
    ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
    ret.direction = refract(out_dir, refraction_index, was_internal);       // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
