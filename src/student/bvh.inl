
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

using namespace std;

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    //
    // This BVH is parameterized on the type of the primitive it contains. This
    // allows us to build a BVH over any type that defines a certain interface.
    // Specifically, we use this to both build a BVH over triangles within each
    // Tri_Mesh, and over a variety of Objects (which might be Tri_Meshes,
    // Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes
    // don't contain pointers to children, but rather indicies. This is because
    // instead of allocating each node individually, the BVH class contains a
    // vector that holds all of the nodes. Hence, to get the child of a node,
    // you have to look up the child index in this vector (e.g. nodes[node.l]).
    // Similarly, to create a new node, don't allocate one yourself - use
    // BVH::new_node, which returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    preparse();

    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    size_t node_index = new_node(box, 0, primitives.size(), 0, 0);
    buildRec(node_index, max_leaf_size);

    root_idx = 0;
}

/* Change bvh_min and bvh_max so that they are the min/max possible values
 */
void setMinMax(const BBox& bbox, Vec3& bvh_min, Vec3& bvh_max) {
    if (bbox.min.x < bvh_min.x) {
        bvh_min.x = bbox.min.x;
    }
    if (bbox.min.y < bvh_min.y) {
        bvh_min.y = bbox.min.y;
    }
    if (bbox.min.z < bvh_min.z) {
        bvh_min.z = bbox.min.z;
    }

    if (bbox.max.x > bvh_max.x) {
        bvh_max.x = bbox.max.x;
    }
    if (bbox.max.y > bvh_max.y) {
        bvh_max.y = bbox.max.y;
    }
    if (bbox.max.z > bvh_max.z) {
        bvh_max.z = bbox.max.z;
    }
}

/* Go through primitives twice: once to set bboxes and figure out ranges of 
 * x, y, z buckets, and once to figure out which buckets each primitive 
 * belongs to
 */
template<typename Primitive>
void BVH<Primitive>::preparse() {
    // First pass
    bvh_min = Vec3(INT_MAX);
    bvh_max = Vec3(INT_MIN);
    for (Primitive prim : primitives) {
        BBox bbox = prim.bbox();
        primitive_bboxes[prim] = bbox;
        Vec3 centroid = (bbox.max + bbox.min) / 2;
        primitive_centroids[prim] = centroid;
        setMinMax(bbox, bvh_min, bvh_max);
    }

    // Second pass
    // Vec3 diff = bvh_max - bvh_min;
    // Vec3 bucket_size = diff / num_buckets;
    // for (Primitive prim : primitives) {
    //     Vec3 centroid = primitive_centroids[prim];
    //     Vec3 prim_bucket = (centroid - bvh_min) / bucket_size;
    //     primitive_buckets[prim] = Vec3(floor(prim_bucket.x), floor(prim_bucket.y), floor(prim_bucket.z));
    // }
}

template<typename Primitive>
float BVH<Primitive>::calcSAH(float mainSurfaceArea, const vector<BVH<Primitive>::Bucket>& partition) {
    BBox bbox;
    int num_prims = 0;
    for (Bucket bucket : partition) {
        bbox.enclose(bucket.bbox);
        num_prims += bucket.prim_count;
    }

    float subSurfaceArea = bbox.surface_area();
    return subSurfaceArea / mainSurfaceArea * num_prims;
}

template<typename Primitive>
void BVH<Primitive>::findLowestSAH(float mainSurfaceArea, //const Vec3& space_start, const Vec3& bucket_size,
                                   const vector<BVH<Primitive>::Bucket>& buckets,
                                   float& lowest_partition_cost, float& partition_index) {
    for (int i = 1; i < num_buckets; i++) {
        // float partition_value = ;
        float partition_cost = calcSAH(mainSurfaceArea, subList(buckets.begin(), buckets.begin() + i)) +
                               calcSAH(mainSurfaceArea, subList(buckets.begin() + i, buckets.end()));
        if (partition_cost < lowest_partition_cost) {
            lowest_partition_cost = partition_cost;
            // best_partition_value = space_start + bucket_size * i;
            partition_index = i;
        }
    }
}

/* Each recursive call starts out with the (already made) node.
 * Taking that node, it checks whether it's a leaf node or an 
 * interior node. If it's an interior node, it creates both children
 * then makes two recursive calls
 */
template<typename Primitive>
void BVH<Primitive>::buildRec(size_t node_index, size_t max_leaf_size) {
    Node node = nodes[node_index];

    // base case: node size is <= max_leaf_size -- make node a leaf
    if (node.size <= max_leaf_size) {
        // node is leaf by default
        return;
    }

    // recursive case: interior node
    // for each x, y, z; bvh_min/max are just the bounding box's min/max
    float mainSurfaceArea = nodes[node_index].bbox.surface_area();
    Vec3 diff = node.bbox.max - node.bbox.min;
    Vec3 bucket_size = diff / num_buckets;
    float lowest_partition_cost = INT_MAX;
    float partition_index = -1;
    // x
    vector<Bucket> buckets(num_buckets);
    findLowestSAH(mainSurfaceArea, )
}

template<typename Primitive>
void BVH<Primitive>::find_closest_hit(Ray* ray, Node* node, Trace* closest)
{
    
}


template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
