
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    // Find intersection points
    Vec3 o = ray.point;
    Vec3 d = ray.dir;
    Vec3 minIntersect = (min - o) / d;
    Vec3 maxIntersect = (max - o) / d;

    // minIntersect will hold all the min x, y, z; maxIntersect will hold the maxes
    if (minIntersect.x > maxIntersect.x) std::swap(minIntersect.x, maxIntersect.x);
    if (minIntersect.y > maxIntersect.y) std::swap(minIntersect.y, maxIntersect.y);
    if (minIntersect.z > maxIntersect.z) std::swap(minIntersect.z, maxIntersect.z);

    // Check whether the ray intersects the box at all
    if ((minIntersect.x > maxIntersect.y) || (minIntersect.y > maxIntersect.x)) {
        return false; 
    }
    float tMin = std::min(minIntersect.x, minIntersect.y);
    float tMax = std::max(maxIntersect.x, maxIntersect.y);
    if ((tMin > maxIntersect.z) || (minIntersect.z > tMax)) {
        return false; 
    }

    // Check whether the ray intersects within the bounding box range
    tMin = std::min(tMin, minIntersect.z);
    if (tMin < times.x) {
        return false;
    }
    tMax = std::max(tMax, maxIntersect.z);
    if (tMax > times.y) {
        return false;
    }

    times = Vec2(tMin, tMax);
    return false;
}
