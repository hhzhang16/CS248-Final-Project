
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <vector>

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
    // Vec3 bvh_min = Vec3(INT_MAX);
    // Vec3 bvh_max = Vec3(INT_MIN);
    for (size_t i = 0; i < primitives.size(); i++) {
        BBox bbox = primitives[i].bbox();
        primitive_bboxes[i] = bbox;
        // Vec3 centroid = (bbox.max + bbox.min) / 2;
        // primitive_centroids[i] = centroid;
        // setMinMax(bbox, bvh_min, bvh_max);
    }
}

template<typename Primitive>
float BVH<Primitive>::calcSAH(float mainSurfaceArea, const std::vector<BVH<Primitive>::Bucket>& partition) {
    BBox bbox;
    int num_prims = 0;
    for (Bucket bucket : partition) {
        bbox.enclose(bucket.bbox);
        num_prims += bucket.prim_count;
    }

    float subSurfaceArea = bbox.surface_area();
    return subSurfaceArea / mainSurfaceArea * num_prims;
}

/* Returns true if lowest_partition_cost was updated in this iteration of the function
 */
template<typename Primitive>
bool BVH<Primitive>::findLowestSAH(float mainSurfaceArea, float space_start, float bucket_size,
                                   const std::vector<BVH<Primitive>::Bucket>& buckets,
                                   float& lowest_partition_cost, float& best_partition_value) {
    bool changed = false;
    for (int i = 1; i < num_buckets; i++) {
        // float partition_value = ;
        std::vector<BVH<Primitive>::Bucket> leftHalf(buckets.begin(), buckets.begin() + i);
        std::vector<BVH<Primitive>::Bucket> rightHalf(buckets.begin() + i, buckets.end());
        float partition_cost = calcSAH(mainSurfaceArea, leftHalf) +
                               calcSAH(mainSurfaceArea, rightHalf);
        if (partition_cost < lowest_partition_cost) {
            lowest_partition_cost = partition_cost;
            best_partition_value = space_start + bucket_size * i;
            // partition_index = i;
            changed = true;
        }
    }
    return changed;
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
    // for each x, y, z
    float mainSurfaceArea = nodes[node_index].bbox.surface_area();
    Vec3 diff = node.bbox.max - node.bbox.min;
    Vec3 bucket_size = diff / num_buckets;
    float lowest_partition_cost = INT_MAX;
    float pivot = -1;

    // x
    std::vector<Bucket> bucketsX(num_buckets);
    std::vector<Bucket> bucketsY(num_buckets);
    std::vector<Bucket> bucketsZ(num_buckets);
    for (size_t i = node.start; i < node.start + node.size; i++) {
        Vec3 centroid = primitive_bboxes[i].center();
        Vec3 bucket_indices = (centroid - node.bbox.min) / bucket_size;
        
        // x
        bucketsX[bucket_indices.x].bbox.enclose(primitives[i].bbox());
        bucketsX[bucket_indices.x].prim_count++;
        // y
        bucketsY[bucket_indices.y].bbox.enclose(primitives[i].bbox());
        bucketsY[bucket_indices.y].prim_count++;
        // z
        bucketsZ[bucket_indices.z].bbox.enclose(primitives[i].bbox());
        bucketsZ[bucket_indices.z].prim_count++;
    }

    // do the partitions
    int changed_param = 0; // 0 for x, 1 for y, 2 for z
    findLowestSAH(mainSurfaceArea, node.bbox.min.x, bucket_size.x, bucketsX, lowest_partition_cost, pivot);
    if (findLowestSAH(mainSurfaceArea, node.bbox.min.y, bucket_size.y, bucketsY, lowest_partition_cost, pivot)) {
        changed_param = 1;
    }
    if (findLowestSAH(mainSurfaceArea, node.bbox.min.z, bucket_size.z, bucketsZ, lowest_partition_cost, pivot)) {
        changed_param = 2;
    }
    // float pivot = node.bbox.min.x + bucket_size.x * partition_index;
    auto node_begin = primitives.begin() + node.start;
    auto it = partition(node_begin, node_begin + node.size, 
                        [pivot, changed_param](const auto& prim){ 
                            if (changed_param == 0) {
                                return prim.bbox().center().x < pivot; 
                            } else if (changed_param == 1) {
                                return prim.bbox().center().y < pivot; 
                            } else {
                                return prim.bbox().center().z < pivot; 
                            }
                        });

    size_t new_size = it - node_begin;

    // create two child nodes
    BBox left_box;
    for (size_t i = 0; i < new_size; i++) {
        left_box.enclose(primitives[node.start + i].bbox());
    }
    size_t left_child = new_node(left_box, node.start, new_size, 0, 0);

    BBox right_box;
    for (size_t i = new_size; i < node.size; i++) {
        right_box.enclose(primitives[node.start + i].bbox());
    }
    size_t right_child = new_node(right_box, node.start + new_size, node.size - new_size, 0, 0);

    // set children of current node
    nodes[node_index].l = left_child;
    nodes[node_index].r = right_child;

    // make recursive calls
    buildRec(left_child, max_leaf_size);
    buildRec(right_child, max_leaf_size);
}

/* Note: to even enter this function, we presuppose that ray intersects the
 * current node
 */
template<typename Primitive>
void BVH<Primitive>::find_closest_hit(Ray* ray, Node* node, Trace* closest)
{
    // base case
    if (node->is_leaf()) {
        for (int i = node->start; i < node->start + node->size; i++) {
            Primitive prim = primitives[i];
            Trace prim_hit = prim.hit(*ray);
            if (prim_hit.hit) { // ray hits this primitive
                if (!closest->hit) { // this is the first primitive we've seen the ray hit
                    *closest = prim_hit;
                } else {
                    *closest = Trace::min(*closest, prim_hit);
                }
            }
        }
    }

    // Trace intersectL = nodes[node.l].bbox.intersect(ray);
    // Trace intersectR = nodes[node.r].bbox.intersect(ray);
    // Trace closestIntersect = min(intersectL, intersectR);

    Vec2 timesL(INT_MIN, INT_MAX);
    Vec2 timesR(INT_MIN, INT_MAX);
    Node leftChild = nodes[node->l];
    Node rightChild = nodes[node->r];

    bool lHit = leftChild.bbox.hit(*ray, timesL);
    bool rHit = rightChild.bbox.hit(*ray, timesR);

    if (lHit && !rHit) { // only hit left child
        find_closest_hit(ray, &leftChild, closest);
    } else if (rHit && !lHit) { // only hit right child
        find_closest_hit(ray, &rightChild, closest);
    } else {
        // lHit && rHit
        if (timesL.x < timesR.x) {
            // hit left child first, check it first
            find_closest_hit(ray, &leftChild, closest);
            if (timesR.x < closest->distance) {
                find_closest_hit(ray, &rightChild, closest);
            }
        } else {
            find_closest_hit(ray, &rightChild, closest);
            if (timesL.x < closest->distance) {
                find_closest_hit(ray, &leftChild, closest);
            }
        }
    }
}

/* Recurses to find the trace whether ray intersects our BVH
 */
template<typename Primitive>
Trace BVH<Primitive>::hitRec(const Ray& ray, size_t node_index) const {
    // base case
    Trace ret; 
    return ret;
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    // Node root = nodes[0];
    Trace ret;
    // Vec2 times(INT_MIN, INT_MAX);
    for(const Primitive& prim : primitives) {
        Trace hit = prim.hit(ray);
        ret = Trace::min(ret, hit);
    }
    // if (root.bbox.hit(ray, times)) {
    //     find_closest_hit(&ray, &root, &ret);
    // }
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
