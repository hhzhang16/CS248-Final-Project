
#include "../lib/mathlib.h"
#include "debug.h"

using namespace std;

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
    if (minIntersect.x > maxIntersect.x) swap(minIntersect.x, maxIntersect.x);
    if (minIntersect.y > maxIntersect.y) swap(minIntersect.y, maxIntersect.y);
    if (minIntersect.z > maxIntersect.z) swap(minIntersect.z, maxIntersect.z);

    // Check whether the ray intersects within the bounding box range
    int tMin = std::min(std::min(minIntersect.x, minIntersect.y), minIntersect.z);
    if (tMin < times.x) {
        return false;
    }
    int tMax = std::max(std::max(maxIntersect.x, maxIntersect.y), maxIntersect.z);
    if (tMax > times.y) {
        return false;
    }

    times = Vec2(tMin, tMax);
    return false;
}
