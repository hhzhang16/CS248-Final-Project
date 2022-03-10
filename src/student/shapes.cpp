
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

bool in_bounds(float value, float low, float high)
{
    return value < high && value >= low;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    // Source:
    // https://stanford-cs248.github.io/Cardinal3D/pathtracer_extra/ray_sphere_intersection
    // Explanation: For a sphere, f(x) = |x|^2 - r^2. Plugging in x = r(t), we
    // get (after simplifying):
    // f(r(t)) = |d|^2 t^2 + 2(o^T d)t + |o|^2 - r^2 = 0
    // If we let a = |d|^2, b = 2(o^T d), c = |o|^2 - r^2, we can solve as a
    // quadratic equation: t = -b +/- sqrt(b^2 - 4ac) / (2a)
    //
    // Because |d| = 1, a = a, which simplifies some things:
    // t = -(o^T d) +/- sqrt((o^T d)^2 - (|o|^2 - r^2))

    const Vec3& o = ray.point;
    const Vec3& d = ray.dir;

    const float a = dot(d, d);
    const float b = 2 * dot(o, d);
    const float c = dot(o, o) - (radius * radius);

    // If b^2 - 4ac < 0, then there's no (real) solution.
    float determinant = (b * b) - (4 * a * c);
    if(determinant < 0)
    {
        return Trace{};
    }

    // Solve for both solutions.
    float t_1 = (-b + sqrtf(determinant)) / (2 * a); // larger soln
    float t_2 = (-b - sqrtf(determinant)) / (2 * a); // smaller soln

    float t_large = std::max(t_1, t_2);
    float t_small = std::min(t_1, t_2);

    // If small is larger than the 'high' point, or large is smaller than the
    // 'small' point, the ray was OOB.
    float bound_small = ray.dist_bounds.x;
    float bound_large = ray.dist_bounds.y;
    if(t_small >= bound_large || t_large <= bound_small )
    {
        return Trace{};
    }

    // By this point, we know t_small < bound_large and t_large > bound_small.
    float t_soln;
    if(in_bounds(t_small, bound_small, bound_large))
    {
        t_soln = t_small;
    }
    else if(in_bounds(t_large, bound_small, bound_large))
    {
        t_soln = t_large;
    }
    else
    {
        return Trace{};
    }

    Vec3 intersection = ray.point + t_soln * ray.dir;

    // If the intersection was at point p, with the sphere centered at the
    // origin, then the normal is just the vector pointing from the origin to
    // p (scaled to be unit length).
    Vec3 normal = (intersection).normalize();

    Trace ret;
    ret.origin = ray.point;
    ret.hit = true ;         // was there an intersection?
    ret.distance = t_soln;   // at what distance did the intersection occur?
    ret.position = intersection; // where was the intersection?
    ret.normal = normal;     // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
