
#include "../rays/tri_mesh.h"
#include "debug.h"
#include <array>

namespace PT {

BBox Triangle::bbox() const {

    // (PathTracer): Task 2
    // compute the bounding box of the triangle
    Vec3 min(+FLT_MAX);
    Vec3 max(-FLT_MAX);

    std::array<unsigned int, 3> vertices = {v0, v1, v2};
    for(const unsigned int& vertex_id : vertices)
    {
        const Vec3& pos = vertex_list[vertex_id].position;
        min.x = std::min(pos.x, min.x);
        min.y = std::min(pos.y, min.y);
        min.z = std::min(pos.z, min.z);

        max.x = std::max(pos.x, max.x);
        max.y = std::max(pos.y, max.y);
        max.z = std::max(pos.z, max.z);
    }

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect

    BBox box(min, max);
    return box;
}

/// @brief Part 2 implementation: Triangle hit.
/// @todo (PathTracer): Task 2
// Intersect this ray with a triangle defined by the three above points.
Trace Triangle::hit(const Ray& ray) const {
    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // Source: https://stanford-cs248.github.io/Cardinal3D/pathtracer_extra/ray_triangle_intersection
    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;
    Vec3 s = ray.point - v_0.position;

    // Calculate 1 / (e1 x d) \cdot e2
    float denom = dot(cross(e1, ray.dir), e2);

    // If dot(e1 x dir, e2) is 0, this means several things:
    // 1. e1 and the ray's dir produced a vector orthogonal to e2
    // 2. This means that e1 and the ray were on the same plane
    // 3. This means that either there were no collisions, or infinitely many.
    // 4. So for now, let's say "hit = false"
    if(fabsf(denom) < 0.00001f)
    {
        return Trace{};
    }

    Vec3 solve_vec = Vec3(
        -1 * (dot(cross(s, e2), ray.dir)),
        (dot(cross(e1, ray.dir), s)),
        -1 * (dot(cross(s, e2), e1)));


    /// u, v are barycentric coords for the triangle,
    /// t is the "time" position where the ray intersected with the triangle.
    Vec3 uvt = (1.0f / denom) * solve_vec;
    float u = uvt.x;
    float v = uvt.y;

    // (Assume that u + v + w = 1)
    /// @todo Is this a valid assumption? 
    float w = 1 - u - v;

    /// @todo (Part 2)
    // The ray only intersected the triangle if the barycentric coordinates were
    // between 0 and 1. (I think... Would be good to verify this).
    if(u < 0 || u > 1 || v < 0 || v > 1 || w < 0 || w > 1)
    {
        return Trace{};
    }

    // Hit = False if the 'hit' was outside the bounds of the ray.
    float t = uvt.z;
    float min_bound = ray.dist_bounds.x;
    float max_bound = ray.dist_bounds.y;
    if(t < min_bound || t > max_bound)
    {
        return Trace{};
    }

    // Vec3 hit_position = ray.point + t * ray.dir;

    // Interpolate the normals of each of the three vertices according to U, V,
    // and W.
    //
    // U: Vertex v1 (v1 - v0)
    // V: Vertex v2 (v2 - v0)
    // W: Vertex v0 (the remaining one, v0)
    Vec3 interpolated_normal =
        v_0.normal * w +
        v_1.normal * u +
        v_2.normal * v;
    interpolated_normal.normalize();

    Trace ret;
    ret.origin = ray.point;
    ret.hit = true;              // was there an intersection?
    ret.distance = t;            // at what distance did the intersection occur?
    ret.position = ray.at(t);    // where was the intersection?

    // what was the surface normal at the intersection?
    // (this should be interpolated between the three vertex normals)
    ret.normal = interpolated_normal;
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
