
#include <queue>
#include <set>
#include <unordered_map>
#include <iostream>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This helper function finds and returns the halfedge whose next pointer points to the current halfedge
*/
Halfedge_Mesh::HalfedgeRef Halfedge_Mesh::find_prev_halfedge(Halfedge_Mesh::HalfedgeRef first) {
    HalfedgeRef prev = first;
    while (prev->next() != first) {
        prev = prev->next();
    }
    return prev;
}

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    // if e is a boundary edge, nothing happens
    if (e->on_boundary()) {
        return std::nullopt;
    }

    // get the half edges
    HalfedgeRef he1 = e->halfedge();
    HalfedgeRef he2 = he1->twin();

    // get the 2 vertices attached to this edge
    VertexRef v1 = he1->vertex();
    VertexRef v2 = he2->vertex();
    v1->halfedge() = he1->twin()->next();
    v2->halfedge() = he2->twin()->next();

    // get the previous half-edges
    HalfedgeRef he1Prev = find_prev_halfedge(he1);
    HalfedgeRef he2Prev = find_prev_halfedge(he2);

    // Arbitrarily choose to keep he1's face
    HalfedgeRef he1Next = he1->next();
    HalfedgeRef he2Next = he2->next();
    FaceRef f1 = he1->face();
    f1->halfedge() = he1Next;
    FaceRef f2 = he2->face();

    he1Prev->next() = he2Next;
    he2Prev->next() = he1Next;

    // for each edge that used to be part of face2, reassign to face1
    HalfedgeRef temp = he2Next;
    while (temp != he1Next) {
        temp->face() = f1;
        temp = temp->next();
    }

    erase(e);
    erase(he1);
    erase(he2);
    erase(f2);
    return f1;
}

/* 
    This helper function deletes the face if it happens to be a triangle.
    It spares one halfedge and the vertex it points to (not its corresponding vertex)
*/
void Halfedge_Mesh::delete_face_if_needed(Halfedge_Mesh::FaceRef f, 
                                          Halfedge_Mesh::VertexRef v, 
                                          Halfedge_Mesh::HalfedgeRef he) {
    if (f->degree() == 3) { // delete face

        HalfedgeRef toDelete1 = he->next();
        HalfedgeRef toDelete2 = toDelete1->next();
        VertexRef v3 = toDelete2->vertex();
        if (v3->halfedge() == toDelete2) {
            v3->halfedge() = toDelete2->twin()->next();
        }

        // if v has any of the to-deleted half edges as its halfedge, change it
        if (v->halfedge() == toDelete1) {
            v->halfedge() = toDelete1->twin()->next();
        }

        // change he->next bc toDelete1 will be deleted
        he->next() = toDelete1->twin()->next();

        // change the twins of toDelete1 and toDelete2 to point at each other
        toDelete1->twin()->twin() = toDelete2->twin();
        toDelete2->twin()->twin() = toDelete1->twin();
        
        // arbitrarily choose toDelete1's edge to keep
        toDelete1->edge()->halfedge() = toDelete1->twin();
        toDelete2->twin()->edge() = toDelete1->edge();

        erase(toDelete2->edge());
        erase(toDelete1);
        erase(toDelete2);
        erase(f);
    }
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    // if e is a boundary edge, nothing happens 
    // TODO need to change this
    if (e->on_boundary()) {
        return std::nullopt;
    }

    // get the half edges
    HalfedgeRef he1 = e->halfedge();
    HalfedgeRef he2 = he1->twin();
  
    // find the half edges before h1 and h2 that come from the same vertices
    HalfedgeRef h1PrevFromV = find_prev_halfedge(he1)->twin();
    HalfedgeRef h2PrevFromV = find_prev_halfedge(he2)->twin();

    // get the 2 vertices attached to this edge
    VertexRef v1 = he1->vertex();
    VertexRef v2 = he2->vertex();

    // any vertices that have he1 or he2 as its halfedge should be changed
    v1->halfedge() = he1->twin()->next();
    v2->halfedge() = he2->twin()->next();

    // any face that has he1 or he2 as its boundary halfedge should be changed
    FaceRef face1 = he1->face();
    face1->halfedge() = he1->next();
    FaceRef face2 = he2->face();
    face2->halfedge() = he2->next();

    // Any faces that are triangles that include e should be deleted
    delete_face_if_needed(face1, v2, he1);
    delete_face_if_needed(face2, v1, he2);

    // Delete halfedges he1 and he2 by making sure the next() that points to them points to other vertices
    h1PrevFromV->twin()->next() = he1->next();
    h2PrevFromV->twin()->next() = he2->next();

    // delete the vertex with the smallest degree and have all of its half edges point to the other v
    VertexRef smallerVertex = v1->degree() < v2->degree() ? v1 : v2;
    VertexRef largerVertex = v1->degree() < v2->degree() ? v2 : v1;
    HalfedgeRef h = smallerVertex->halfedge();
    do {
        h->vertex() = largerVertex;
        h = h->twin()->next();
    } while (h != smallerVertex->halfedge());

    // Move the larger vertex to be in the middle of the edge
    largerVertex->pos = e->center();

    // Now we can safely delete the edge;
    erase(e);
    erase(he1);
    erase(he2);
    erase(smallerVertex);

    return largerVertex;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {
    
    // if f is a boundary face, nothing happens
    // TODO: add support for boundary
    if (f->is_boundary()) {
        return std::nullopt;
    }

    // Create a new vertex, set its position and halfedge
    VertexRef newV = new_vertex();
    newV->pos = f->center();

    // For every halfedge coming from vertices around f, reassign to come from newV
    HalfedgeRef firstHeFromF = f->halfedge()->twin()->next();
    HalfedgeRef tempHe = firstHeFromF;
    bool newFirstHe = false; // whether or not firstHeFromF was deleted
    do {
        if (newFirstHe) {
            firstHeFromF = tempHe;
            newFirstHe = false;
        }

        HalfedgeRef nextHe = tempHe->twin()->next();
        if (nextHe->twin()->face() == f) {
            nextHe = nextHe->next();
        }
        HalfedgeRef prevBeforeLastHe = tempHe;
        HalfedgeRef lastHe = tempHe;
        while (lastHe->next() != tempHe) {
            prevBeforeLastHe = lastHe;
            lastHe = lastHe->next();
        }

        if (lastHe->twin()->face() == f) {
            // delete halfedge that lies on an edge on F

            if (tempHe->face()->degree() == 3) {
                if (tempHe == firstHeFromF) {
                    newFirstHe = true; // will need a new loop ending
                }
                // a triangle adjacent to f, should be deleted
                delete_face_if_needed(tempHe->face(), tempHe->vertex(), lastHe);
            } else {
                // non-triangle adjacent to f, only delete halfedge lastHe
                prevBeforeLastHe->next() = tempHe;
                prevBeforeLastHe->face()->halfedge() = prevBeforeLastHe;
                tempHe->vertex() = newV;
            }
            erase(lastHe);
        } else {
            tempHe->vertex() = newV;
        }
        tempHe = nextHe;
    } while (tempHe != firstHeFromF);

    // delete every he on face as well as its V & E
    tempHe = f->halfedge();
    for (unsigned int i = 0; i < f->degree(); i++) {
        HalfedgeRef nextHe = tempHe->next();
        erase(tempHe->vertex());
        erase(tempHe->edge());
        erase(tempHe);
        tempHe = nextHe;
    }

    newV->halfedge() = firstHeFromF;
    erase(f);

    return newV;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    // “rotated” around the face, in the sense that each endpoint moves to the next vertex
    // get the halfedges and the faces on either side of e
    HalfedgeRef he1 = e->halfedge();
    FaceRef f1 = he1->face();
    HalfedgeRef he2 = he1->twin();
    FaceRef f2 = he2->face();

    // get the rest of the halfedges
    HalfedgeRef he1Next = he1->next();
    HalfedgeRef he1Prev = find_prev_halfedge(he1);
    HalfedgeRef he2Next = he2->next();
    HalfedgeRef he2Prev = find_prev_halfedge(he2);

    // reassign halfedges
    he1->vertex()->halfedge() = he2Next;
    he2->vertex()->halfedge() = he1Next;
    f1->halfedge() = he1;
    f2->halfedge() = he2;

    // describing as if we're flipping <-> to <|>
    // update vertices
    he1->vertex() = he2Next->next()->vertex();
    he2->vertex() = he1Next->next()->vertex();
    // lower left
    he1->next() = he1Next->next();
    he1Next->face() = f2;
    he1Next->next() = he2;
    // upper right
    he2->next() = he2Next->next();
    he2Next->next() = he1;
    he2Next->face() = f1;

    he1Prev->next() = he2Next;
    he2Prev->next() = he1Next;

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    // if e is a boundary edge, nothing happens
    if (e->on_boundary()) {
        return std::nullopt;
    }

    // painting the picture: right now it looks like <|>, we'll make it <+>
    // the variable naming assumes a vertical edge and CCW direction but should work no matter the setup

    // get the halfedges and the faces on either side of e
    // arbitrarily assigning the current edge to be the "top edge" coming out of the new vertex
    HalfedgeRef heNorthLeft = e->halfedge();
    FaceRef fTopLeft = heNorthLeft->face();
    HalfedgeRef heNorthRight = heNorthLeft->twin();
    FaceRef fTopRight = heNorthRight->face();

    // make a vertex, two new faces
    FaceRef fBottomLeft = new_face();
    FaceRef fBottomRight = new_face();
    VertexRef newV = new_vertex();
    newV->halfedge() = heNorthLeft;
    newV->pos = e->center();

    // do the left side first
    HalfedgeRef heNorthwest= heNorthLeft->next();
    HalfedgeRef heSouthwest = heNorthwest->next();
    HalfedgeRef heSouthLeft = new_halfedge();
    // new edge and new halfedges
    EdgeRef eLeft = new_edge();
    HalfedgeRef heWestUpper = new_halfedge();
    HalfedgeRef heWestLower = new_halfedge();
    heWestUpper->twin() = heWestLower;
    heWestLower->twin() = heWestUpper;
    heWestUpper->edge() = eLeft;
    heWestLower->edge() = eLeft;
    eLeft->halfedge() = heWestUpper;
    // Lower left triangle
    heSouthLeft->vertex() = heNorthLeft->vertex();
    heSouthLeft->next() = heWestLower;
    heSouthLeft->face() = fBottomLeft;
    heWestLower->vertex() = newV;
    heWestLower->next() = heSouthwest;
    heWestLower->face() = fBottomLeft;
    heSouthwest->next() = heSouthLeft;
    heSouthwest->face() = fBottomLeft;
    fBottomLeft->halfedge() = heSouthLeft;
    // Upper left triangle
    heNorthLeft->vertex() = newV;
    heNorthwest->next() = heWestUpper;
    heWestUpper->next() = heNorthLeft;
    heWestUpper->face() = fTopLeft;
    heWestUpper->vertex() = heSouthwest->vertex();
    fTopLeft->halfedge() = heNorthLeft;

    // next, the right half
    HalfedgeRef heSoutheast = heNorthRight->next();
    HalfedgeRef heNortheast = heSoutheast->next();
    HalfedgeRef heSouthRight = new_halfedge();
    heSouthRight->twin() = heSouthLeft;
    heSouthLeft->twin() = heSouthRight;
    EdgeRef eBottom = new_edge();
    eBottom->halfedge() = heSouthLeft;
    heSouthLeft->edge() = eBottom;
    heSouthRight->edge() = eBottom;
    // new edge and halfedges
    EdgeRef eRight = new_edge();
    HalfedgeRef heEastUpper = new_halfedge();
    HalfedgeRef heEastLower = new_halfedge();
    heEastUpper->twin() = heEastLower;
    heEastLower->twin() = heEastUpper;
    heEastUpper->edge() = eRight;
    heEastLower->edge() = eRight;
    eRight->halfedge() = heEastUpper;
    // Lower right triangle
    heSouthRight->vertex() = newV;
    heSouthRight->next() = heSoutheast;
    heSouthRight->face() = fBottomRight;
    heSoutheast->next() = heEastLower;
    heSoutheast->face() = fBottomRight;
    heEastLower->vertex() = heNortheast->vertex();
    heEastLower->next() = heSouthRight;
    heEastLower->face() = fBottomRight;
    fBottomRight->halfedge() = heSouthRight;
    // Upper right triangle
    heNorthRight->next() = heEastUpper;
    heEastUpper->vertex() = newV;
    heEastUpper->next() = heNortheast;
    heEastUpper->face() = fTopRight;
    fTopRight->halfedge() = heNorthRight;

    return newV;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)

    Suggestion: For next time, I recommend the teaching staff define custom
     'iterators' for the 'half-edges'. It'd go a long way to improving the
     quality of the code created.
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef face)
{
    // Defines a pair of vertices where the 'from' vertex has a half-edge
    // pointing to 'to'.
    struct VertexPair { VertexRef from; VertexRef to; };

    // Defines the information we need to perform operations on the side of a
    // beveled face.
    struct BevelSide
    {
        // Constructor creates:
        // 1. New face
        // 2. Edge corresponding to the link between old_.from and new_.from
        // 3. Three halfedges (the fourth one that completes the rectangle loop is
        // already in the mesh between old_.from and old_.to). 
        BevelSide(Halfedge_Mesh& hm, HalfedgeRef old_halfedge, VertexPair old_, VertexPair new_)
            {
                old_verts = old_;
                new_verts = new_;

                side_face = hm.new_face();
                side_edge = hm.new_edge();

                // Let the "First" halfedge be the existing halfedge between the
                // vertices in old_pair
                halfedges[0] = old_halfedge;
                halfedges[0]->face() = side_face;

                // Generate the rest of the halfedges.
                for(size_t i = 1; i < halfedges.size(); i++)
                {
                    halfedges[i] = hm.new_halfedge();
                    halfedges[i]->_face = side_face;

                    // Note: Don't assign edges to half-edges; at this point, we
                    // don't have enough information to know which halfedge
                    // corresponds to which edge.
                }
                side_face->_halfedge = halfedges[0];

                // Assign vertices: Note that there's a halfedge from old_.to to
                // new_.from and from new_.to and old_.from. 
                halfedges[0]->vertex() = old_.from;
                halfedges[1]->vertex() = old_.to;
                halfedges[2]->vertex() = new_.from;
                halfedges[3]->vertex() = new_.to;

                // Link halfedges together.
                for(size_t i = 0; i < halfedges.size(); i++)
                {
                    halfedges[i]->next() = halfedges[(i+1) % halfedges.size()];
                }
            }

        // One edge/face per bevel side.
        FaceRef side_face;
        EdgeRef side_edge;

        // Beveling creates three new half-edges on the 'inside' of each beveled
        // face: The 'fourth' halfedge in the rectangle loop is the halfedge
        // that already exists between old_verts.from and old_verts.to. This
        // "existing edge" will be labeled as 'ExistingEdge', or 'E' in
        // documentation.
        //
        // The ordering of the halfedges is:
        // 1. E (halfedges[0]) -> halfedges[1]
        // 2. halfedges[1] -> halfedges[2]
        // 3. halfedges[2] -> halfedges[3]
        // 4. halfedges[3] -> E (halfedges[0])
        //
        // Because of this ordering, the "twins" are as follows:
        //  halfedges[0]: Unchanged
        //  halfedges[1]: Next Side::halfedges[3] (last halfedge)
        //  halfedges[2]: Corresponding inside halfedge
        //  halfedges[3]: Prev Side::halfedges[1] (first created halfedge)
        std::array<HalfedgeRef, 4> halfedges;

        // Pair of vertices in the old face.
        VertexPair old_verts;

        // Pair of vertices in the 'new' face.
        VertexPair new_verts;
    };

    struct BevelInside
    {
        BevelInside(Halfedge_Mesh& hm, const std::vector<VertexRef>& vertices)
            {
                const size_t N = vertices.size();
                face = hm.new_face();

                for(size_t i = 0; i < N; i++)
                {
                    edges.push_back(hm.new_edge());
                    halfedges.push_back(hm.new_halfedge());

                    halfedges[i]->_face = face;
                    halfedges[i]->_edge = edges[i];
                    edges[i]->halfedge() = halfedges[i];

                    halfedges[i]->_vertex = vertices[i];
                }
                // Arbitrarily set the face's halfedge. 
                face->halfedge() = halfedges[0];

                // Create a loop with the created half-edges by linking their
                // 'next' pointers.
                //
                // NOTE: The "twin" of each halfedge corresponds to an entry in
                // the SideBevel::halfedges struct.
                for(size_t i =0; i < N; i++)
                {
                    halfedges[i]->_next = halfedges[(i+1) % N];
                }
            }

        FaceRef face;

        // "Inside edges" that form a loop around the face.
        std::vector<EdgeRef> edges;
        std::vector<HalfedgeRef> halfedges;
    };

    // Pseudocode:
    //
    // This function operates in several phases:
    // Phase 1: Collect vertices on current face, and create N new vertices
    // Phase 2: Create the N side-faces corresponding to the bevel.
    // Phase 3: Wire the edges and half-edges of the side-faces
    // Phase 4: Create edges and half-edges for the newly created "inside" face.
    // Phase 5: Wire the half-edges for the "inside" face and the half-edges for
    //          the "side" face.
    

    // Phase 1: Collect vertices on current face, and create N new vertices.
    // Each vertex in the 'inside' face corresponds to one in the original
    // 'outside' face. 
    std::vector<VertexRef> beveled_vertices;
    std::vector<VertexRef> original_vertices;
    std::vector<HalfedgeRef> original_halfedges;

    const HalfedgeRef init_edge = face->halfedge();
    auto edge_iter = init_edge;
    do {
        VertexRef curr_vertex = edge_iter->vertex();

        // Reminder: You should set the positions of new vertices (v->pos) to be
        // exactly the same as wherever they "started from."
        VertexRef created_vertex = new_vertex();
        created_vertex->pos = curr_vertex->pos;

        beveled_vertices.push_back(created_vertex);
        original_vertices.push_back(edge_iter->vertex());
        original_halfedges.push_back(*curr_vertex->FindHalfedgeOnFace(face));

        edge_iter = edge_iter->next();
    } while(edge_iter != init_edge);

    const size_t n_vertices = beveled_vertices.size();

    // Phase 2:Create edges and half-edges for the newly created "inside" face.
    // Create one BevelSide per vertex, each element represents a "side" face of
    // the bevel created. 
    std::vector<BevelSide> bevel_sides;
    for(size_t k = 0; k < n_vertices; k++)
    {
        // If there's a link between vertices i and j in the old face, then
        // there will be a link between j and i in a beveled (side) face. (draw
        // a rectangle with clockwise arrows to convince yourself. Opposite
        // sides have arrows pointing in opposite directions).
        size_t i = k;

        // "Next" vertex, including wrap-around.
        size_t j = (i + 1) % n_vertices;
        VertexPair old_pair{
            .from = original_vertices[i],
            .to = original_vertices[j]
        };

        VertexPair new_pair{
            .from = beveled_vertices[j],
            .to = beveled_vertices[i]
        };

        bevel_sides.push_back(BevelSide(*this, original_halfedges[i],
                                        old_pair, new_pair));
    }

    // Phase 4: Create edges and half-edges for the newly created "inside" face.
    // One half-edge per inside face vertex.
    BevelInside bevel_inside(*this, beveled_vertices);
    for(size_t i = 0; i < n_vertices; i++)
    {
        beveled_vertices[i]->_halfedge = bevel_inside.halfedges[i];
    }

    // Phase 3:  Wire the edges and half-edges of the side-faces
    for(size_t i = 0; i < n_vertices; i++)
    {
        BevelSide& curr_side = bevel_sides[i];
        BevelSide& prev_side = bevel_sides[(i + n_vertices - 1) % n_vertices];
        BevelSide& next_side = bevel_sides[(i + 1) % n_vertices];

        // Wire Twins.
        curr_side.halfedges[1]->twin() = next_side.halfedges[3];
        curr_side.halfedges[2]->twin() = bevel_inside.halfedges[i];
        curr_side.halfedges[3]->twin() = prev_side.halfedges[1];

        // Wire edges.
        curr_side.halfedges[1]->edge() = next_side.side_edge;
        curr_side.halfedges[2]->edge() = bevel_inside.edges[i];
        curr_side.halfedges[3]->edge() = curr_side.side_edge;

        // Assign edge. 
        curr_side.side_edge->halfedge() = curr_side.halfedges[3];
    }

    // Phase 5: Wire the half-edges for the "inside" face and the half-edges for
    // the "side" face.
    for(size_t i = 0; i < n_vertices; i++)
    {
        bevel_inside.halfedges[i]->twin() = bevel_sides[i].halfedges[2];
    }
    erase(face);
    return bevel_inside.face;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face,
                                         float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->new_pos = e->center();
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        f->new_pos = f->center();
    }

    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        Vec3 face1Pos = e->halfedge()->face()->new_pos;
        Vec3 face2Pos = e->halfedge()->twin()->face()->new_pos;
        e->new_pos = (2 * e->center() + face1Pos + face2Pos) / 4;
    }

    // Vertices
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        unsigned int n = v->degree();
        // Q is the average of all new face position for faces containing v
        Vec3 Q;
        // R is the average of all original edge midpoints for edges containing v
        Vec3 R;
        HalfedgeRef h = v->halfedge();
        do {
            Q += h->face()->new_pos / n;
            R += h->edge()->center() / n;
            h = h->twin()->next();
        } while(h != v->halfedge());

        // S is the original vertex position for vertex v
        Vec3 S = v->pos;
        v->new_pos = (Q + 2 * R + (n - 3) * S) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        unsigned int n = v->degree();
        float u = (n == 3) ? 3.0f / 16.0f : 3.0f / (8.0f * n);
        v->new_pos = (1.0f - u * n) * v->pos;
        
        HalfedgeRef h = v->halfedge();
        do {
            VertexRef neighbor = h->twin()->vertex();
            v->new_pos += u / n * neighbor->pos();
            h = h->twin()->next();
        } while(h != v->halfedge());

        v->is_new = false;
    }
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef he1 = e->halfedge();
        HalfedgeRef he2 = he1->twin();
        VertexRef v1 = he1->vertex();
        VertexRef v2 = he2->vertex();
        VertexRef oppositeV1 = he1->next()->next()->vertex();
        VertexRef oppositeV2 = he2->next()->next()->vertex();

        e->new_pos = 3.0f/8.0f * (v1->pos + v2->pos) + 1.0f/8.0f * (oppositeV1->pos + oppositeV2->pos);
    }
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // iterate over all edges in the mesh
    int n = n_edges();
    EdgeRef e = edges_begin();
    for (int i = 0; i < n; i++) {

        // get the next edge NOW!
        EdgeRef nextEdge = e;
        nextEdge++;

        // now, even if splitting the edge deletes it...
        if (some condition is met) {
            
            split_edge(e);
        }

        // ...we still have a valid reference to the next edge.
        e = nextEdge;
    }
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
