
#pragma once

#include <map>
#include <vector>

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

namespace PT {

template<typename Primitive> class BVH {
public:
    BVH() = default;
    BVH(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    void build(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);

    BVH(BVH&& src) = default;
    BVH& operator=(BVH&& src) = default;

    BVH(const BVH& src) = delete;
    BVH& operator=(const BVH& src) = delete;

    BBox bbox() const;
    Trace hit(const Ray& ray) const;

    BVH copy() const;
    size_t visualize(GL::Lines& lines, GL::Lines& active, size_t level, const Mat4& trans) const;

    std::vector<Primitive> destructure();
    void clear();

private:
    class Node {
        BBox bbox;
        size_t start, size, l, r;

        bool is_leaf() const;
        friend class BVH<Primitive>;
    };
    size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0,
                    size_t l = 0, size_t r = 0);

    // Custom struct for buckets (for SAH)
    struct Bucket {
        BBox bbox;
        int prim_count;
    };

    // our added recursive helper for build
    void buildRec(size_t node_index, size_t max_leaf_size);
    float calcSAH(float mainSurfaceArea, const std::vector<Bucket>& partition);
    bool findLowestSAH(float mainSurfaceArea, float space_start, float bucket_size,
                       const std::vector<Bucket>& buckets, 
                       float& lowest_partition_cost, float& best_partition_value);

    void closest_hit(const Ray& ray, const Node& node, Trace* closest) const;
    void find_closest_hit(Ray* ray, Node* node, Trace* closest);

    std::vector<Node> nodes;
    std::vector<Primitive> primitives;
    size_t root_idx = 0;
    int num_buckets = 16;
    float big_epsilon = 0.01;
    float small_epsilon = 0.00002;
};

} // namespace PT

#ifdef CARDINAL3D_BUILD_REF
#include "../reference/bvh.inl"
#else
#include "../student/bvh.inl"
#endif
