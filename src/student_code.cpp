#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    std::vector<Vector2D> nextPoints;
    for (size_t i = 0; i < points.size() - 1; i++) {
        Vector2D interpolated = points[i] * (1 - t) + points[i + 1] * t;
        nextPoints.push_back(interpolated);
    }
    return nextPoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> nextPoints;
    for (size_t i = 0; i < points.size() - 1; i++) {
        Vector3D interpolated = points[i] * (1 - t) + points[i + 1] * t;
        nextPoints.push_back(interpolated);
    }
    return nextPoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    if (points.size() == 1) {
        return points.front(); // Directly return the only point.
    }

    // Recursive step with LERP'd points
    std::vector<Vector3D> intermediatePoints;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        Vector3D lerpPoint = points[i] * (1 - t) + points[i + 1] * t;
        intermediatePoints.push_back(lerpPoint);
    }

    return evaluate1D(intermediatePoints, t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    return Vector3D();
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D weighted_normal(0,0,0); // Initialize weighted normal to zero
    HalfedgeCIter h = halfedge(); // Start with one of the half-edges emanating from the vertex

    HalfedgeCIter h_orig = h; // Keep the original half-edge to check for the loop's end
    do {
      if (!h->face()->isBoundary()) { // Skip boundary faces
        // Get the positions of the triangle vertices
        Vector3D v0 = position;
        Vector3D v1 = h->next()->vertex()->position;
        Vector3D v2 = h->next()->next()->vertex()->position;

        // Calculate the area of the face using the cross product (area of parallelogram) and halve it
        double face_area = cross(v1 - v0, v2 - v0).norm() / 2.0;

        // Add the area-weighted normal of the face to the weighted normal
        weighted_normal += face_area * h->face()->normal();
      }
      h = h->twin()->next(); // Move to the next half-edge around the vertex
    } while (h != h_orig); // Check if we've made a full loop

    // Normalize and return the weighted normal
    return weighted_normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    if (!e0->isBoundary()) {
    // Define halfedges related to e0
    HalfedgeIter h0 = e0->halfedge(); // Halfedge from e0
    HalfedgeIter h3 = h0->twin(); // Twin of h0
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next(); // Completes the triangle on one side
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next(); // Completes the triangle on the opposite side

    // Outer halfedges for re-linking
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    // Vertices of the quadrilateral
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    // Reassigning the halfedge neighbors to flip the edge
    h0->setNeighbors(h1, h3, v2, e0, h0->face());
    h1->setNeighbors(h2, h9, v3, h1->edge(), h0->face());
    h2->setNeighbors(h0, h6, v1, h2->edge(), h0->face());

    h3->setNeighbors(h4, h0, v3, e0, h3->face());
    h4->setNeighbors(h5, h7, v2, h4->edge(), h3->face());
    h5->setNeighbors(h3, h8, v0, h5->edge(), h3->face());

    // Update the twin links for outer halfedges
    h6->setNeighbors(h6->next(), h2, h6->vertex(), h6->edge(), h6->face());
    h7->setNeighbors(h7->next(), h4, h7->vertex(), h7->edge(), h7->face());
    h8->setNeighbors(h8->next(), h5, h8->vertex(), h8->edge(), h8->face());
    h9->setNeighbors(h9->next(), h1, h9->vertex(), h9->edge(), h9->face());

    // Update vertices to point to one of their outgoing halfedges
    v0->halfedge() = h5;
    v1->halfedge() = h2;
    v2->halfedge() = h4;
    v3->halfedge() = h1;

    // Edges keep pointing to one of their halfedges
    e0->halfedge() = h0;

    // Faces are updated to point to one of their halfedges
    h0->face()->halfedge() = h0;
    h3->face()->halfedge() = h3;
    }
    return e0; // Return the flipped edge
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (!e0->isBoundary()) {
        // Retrieve and define the original halfedge elements associated with the edge to be split.
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->next();
        HalfedgeIter h2 = h1->next();
        HalfedgeIter h3 = h0->twin();
        HalfedgeIter h4 = h3->next();
        HalfedgeIter h5 = h4->next();

        // Define the twin halfedges for the surrounding area.
        HalfedgeIter h6 = h1->twin();
        HalfedgeIter h7 = h2->twin();
        HalfedgeIter h8 = h4->twin();
        HalfedgeIter h9 = h5->twin();

        // Identify the vertices involved in the split operation.
        VertexIter v0 = h0->vertex();
        VertexIter v1 = h3->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h5->vertex();

        // Enumerate the edges surrounding the edge to be split.
        EdgeIter e1 = h1->edge();
        EdgeIter e2 = h2->edge();
        EdgeIter e3 = h4->edge();
        EdgeIter e4 = h5->edge();

        // Reference the faces on either side of the edge.
        FaceIter f1 = h0->face();
        FaceIter f2 = h3->face();

        // Generate new halfedge elements as part of the split.
        HalfedgeIter h10 = newHalfedge();
        HalfedgeIter h11 = newHalfedge();
        HalfedgeIter h12 = newHalfedge();
        HalfedgeIter h13 = newHalfedge();
        HalfedgeIter h14 = newHalfedge();
        HalfedgeIter h15 = newHalfedge();

        // Introduce a new vertex at the midpoint of the edge.
        VertexIter v = newVertex();
        v->position = (v0->position + v1->position) * 0.5; // Midpoint calculation.

        // Create new edges as a result of the split.
        EdgeIter e5 = newEdge();
        EdgeIter e6 = newEdge();
        EdgeIter e7 = newEdge();

        // Form new faces to accommodate the additional geometry.
        FaceIter f3 = newFace();
        FaceIter f4 = newFace();

        // Reconfigure connectivity among the halfedges, vertices, edges, and faces.
        h0->setNeighbors(h1, h3, v, e0, f1);
        h1->setNeighbors(h2, h6, v1, e1, f1);
        h2->setNeighbors(h0, h11, v2, e5, f1);
        h3->setNeighbors(h4, h0, v1, e0, f2);
        h4->setNeighbors(h5, h15, v, e7, f2);
        h5->setNeighbors(h3, h9, v3, e4, f2);
        h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
        h7->setNeighbors(h7->next(), h12, v0, e2, h7->face());
        h8->setNeighbors(h8->next(), h14, v3, e3, h8->face());
        h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
        h10->setNeighbors(h11, h13, v0, e6, f3);
        h11->setNeighbors(h12, h2, v, e5, f3);
        h12->setNeighbors(h10, h7, v2, e2, f3);
        h13->setNeighbors(h14, h10, v, e6, f4);
        h14->setNeighbors(h15, h8, v0, e3, f4);
        h15->setNeighbors(h13, h4, v3, e7, f4);

        // Update the newly created vertex's halfedge pointer.
        v->halfedge() = h0; // Points to one of the halfedges originating from it.

        // Adjust the halfedge pointers for vertices to ensure the mesh remains valid.
        v0->halfedge() = h10;
        v1->halfedge() = h1;
        v2->halfedge() = h12;
        v3->halfedge() = h5;

        // Link the new edges to their respective halfedges.
        e5->halfedge() = h2;
        e6->halfedge() = h10;
        e7->halfedge() = h4;

        // Update isNew flags for the edges accordingly.
        e0->isNew = false; // Original edge, now half split.
        e5->isNew = true; // New edge created by the split.
        e6->isNew = false; // New edge, parallel to the original, half split.
        e7->isNew = true; // New edge created by the split.

        // Assign halfedges to the new and adjusted faces.
        f1->halfedge() = h0;
        f2->halfedge() = h3;
        f3->halfedge() = h10;
        f4->halfedge() = h13;

        // The function returns the new vertex created at the midpoint of the edge.
        return v;
    }

    // If the edge is a boundary, return an invalid iterator.
    return VertexIter();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
