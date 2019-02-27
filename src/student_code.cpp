#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
	  int level = evaluatedLevels.size();
	  vector<Vector2D> oldpoints = evaluatedLevels[level - 1];
	  if (oldpoints.size() == 1) {
		  return;
	  }
	  vector<Vector2D> newpoints = vector<Vector2D>();
	  for (int i = 1; i < oldpoints.size(); ++i) {
		  Vector2D val = (1 - t)*oldpoints[i - 1] + t*oldpoints[i];
		  newpoints.push_back(val);
	  }
	  evaluatedLevels.push_back(newpoints);
	  return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
	  vector<Vector3D> idPoints = vector<Vector3D>();
	  for (int i = 0; i < controlPoints.size(); ++i) {
		  idPoints.push_back(evaluate1D(controlPoints[i], u));
	  }
	  if (idPoints.size() <= 0) {
		  return Vector3D();
	  }
	  return evaluate1D(idPoints, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
	  if (points.size() == 1) {
		  return points[0];
	  }
	//  vector<Vector3D> newpoints = vector<Vector3D>();
	//  for (int i = 1; i < points.size(); ++i) {
	//	  Vector3D val = (1 - t)*points[i - 1] + t*points[i];
	//	  newpoints.push_back(val);
	//  }
	//  return evaluate1D(newpoints, t);
	// Teapot is taking a while to load so ima try not using recursion
	  while (points.size() > 1) {
		  vector<Vector3D> newpoints = vector<Vector3D>();
		  for (int i = 1; i < points.size(); ++i) {
			  Vector3D val = (1 - t)*points[i - 1] + t * points[i];
			  newpoints.push_back(val);
		  }
		  points = newpoints;
	  }
	  if (points.size() == 1) {
		  return points[0];
	  }
	  return Vector3D();
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
	  
	  Vector3D n(0.,0.,0.);
	  HalfedgeCIter h = halfedge();
	  h = h->twin();
	  HalfedgeCIter h_origin = h;
	  do {
		  n += h->face()->normal();
		  h = h->next();
		  h = h->twin();
	  } while (h != h_origin);
	  return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
	  FaceIter f0 = e0->halfedge()->face();
	  FaceIter f1 = e0->halfedge()->twin()->face();
	  if (f0->isBoundary() || f1->isBoundary()) {
		  return e0;
	  }
	  // use e0 to find all of the edges in our two faces
	  HalfedgeIter h0 = e0->halfedge();
	  HalfedgeIter h1 = h0->next();
	  HalfedgeIter h2 = h1->next();
	  HalfedgeIter h3 = h0->twin();
	  HalfedgeIter h4 = h3->next();
	  HalfedgeIter h5 = h4->next();
	  HalfedgeIter h6 = h1->twin();
	  HalfedgeIter h7 = h2->twin();
	  HalfedgeIter h8 = h4->twin();
	  HalfedgeIter h9 = h5->twin();
	  VertexIter v0 = h0->vertex();
	  VertexIter v1 = h3->vertex();
	  VertexIter v2 = h2->vertex();
	  VertexIter v3 = h5->vertex();
	  EdgeIter e1 = h1->edge();
	  EdgeIter e2 = h2->edge();
	  EdgeIter e3 = h4->edge();
	  EdgeIter e4 = h5->edge();
	  //start setting all of the new pointers:
	  h0->setNeighbors(h1, h3, v3, e0, f0);
	  h1->setNeighbors(h2, h7, v2, e2, f0);
	  h2->setNeighbors(h0, h8, v0, e3, f0);
	  h3->setNeighbors(h4, h0, v2, e0, f1);
	  h4->setNeighbors(h5, h9, v3, e4, f1);
	  h5->setNeighbors(h3, h6, v1, e1, f1);
	  h6->setNeighbors(h6->next(),h5,v2,e1,h6->face());
	  h7->setNeighbors(h7->next(),h1,v0,e2,h7->face());
	  h8->setNeighbors(h8->next(),h2,v3,e3,h8->face());
	  h9->setNeighbors(h9->next(),h4,v1,e4,h9->face());
	  //setting vertexes
	  v0->halfedge() = h2;
	  v1->halfedge() = h5;
	  v2->halfedge() = h3;
	  v3->halfedge() = h0;
	  //seting edges
	  e0->halfedge() = h0;
	  e1->halfedge() = h5;
	  e2->halfedge() = h1;
	  e3->halfedge() = h2;
	  e4->halfedge() = h4;
	  //setting faces
	  f0->halfedge() = h0;
	  f1->halfedge() = h3;
	  return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
	  //halfedges
	  HalfedgeIter h0 = e0->halfedge();
	  HalfedgeIter h1 = h0->next();
	  HalfedgeIter h2 = h1->next();
	  HalfedgeIter h3 = h0->twin();
	  HalfedgeIter h4 = h3->next();
	  HalfedgeIter h5 = h4->next();
	  HalfedgeIter h6 = h1->twin();
	  HalfedgeIter h7 = h2->twin();
	  HalfedgeIter h8 = h4->twin();
	  HalfedgeIter h9 = h5->twin();
	  //vertexes
	  VertexIter v0 = h0->vertex();
	  VertexIter v1 = h3->vertex();
	  VertexIter v2 = h2->vertex();
	  VertexIter v3 = h5->vertex();
	  //edges
	  EdgeIter e1 = h1->edge();
	  EdgeIter e2 = h2->edge();
	  EdgeIter e3 = h4->edge();
	  EdgeIter e4 = h5->edge();
	  //faces
	  FaceIter f0 = h0->face();
	  FaceIter f1 = h3->face();
	  //chech boundary conditions.
	  if (f0->isBoundary() || f1->isBoundary()) {
		  return newVertex();
	  }
	  // Allocate new stuff
	  VertexIter v4 = this->newVertex();
	  EdgeIter e5 = this->newEdge();
	  EdgeIter e6 = this->newEdge();
	  EdgeIter e7 = this->newEdge();
	  FaceIter f2 = this->newFace();
	  FaceIter f3 = this->newFace();
	  HalfedgeIter h10 = this->newHalfedge();
	  HalfedgeIter h11 = this->newHalfedge();
	  HalfedgeIter h12 = this->newHalfedge();
	  HalfedgeIter h13 = this->newHalfedge();
	  HalfedgeIter h14 = this->newHalfedge();
	  HalfedgeIter h15 = this->newHalfedge();
	  //Find the location of our new vertex v4:
	  v4->position = (v0->position + v1->position)/2.;
	  //now lets set the pointers lol
	  //setNeighbors(next, twin, vertex, edge, face)
	  h0->setNeighbors(h10, h3, v0, e0, f0);
	  h1->setNeighbors(h11, h6, v1, e1, f2);
	  h2->setNeighbors(h0, h7, v2, e2, f0);
	  h3->setNeighbors(h4, h0, v4, e0, f3);
	  h4->setNeighbors(h15, h8, v0, e3, f3);
	  h5->setNeighbors(h13, h9, v3, e4, f1);
	  h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
	  h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
	  h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
	  h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
	  h10->setNeighbors(h2, h11, v4, e6, f0);
	  h11->setNeighbors(h12, h10, v2, e6, f2);
	  h12->setNeighbors(h1, h13, v4, e7, f2);
	  h13->setNeighbors(h14, h12, v1, e7, f1);
	  h14->setNeighbors(h5, h15, v4, e5, f1);
	  h15->setNeighbors(h3, h14, v3, e5, f3);
	  //set the vertex pointers
	  v0->halfedge() = h0;
	  v1->halfedge() = h1;
	  v2->halfedge() = h2;
	  v3->halfedge() = h5;
	  v4->halfedge() = h3;
	  //set the edges
	  e0->halfedge() = h0;
	  e1->halfedge() = h1;
	  e2->halfedge() = h2;
	  e3->halfedge() = h4;
	  e4->halfedge() = h5;
	  e5->halfedge() = h14;
	  e6->halfedge() = h11;
	  e7->halfedge() = h12;
	  //setting faces
	  f0->halfedge() = h0;
	  f1->halfedge() = h5;
	  f2->halfedge() = h1;
	  f3->halfedge() = h3;
	  return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.
	//first lets find the old vertexes new positions:
	 
	for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		int n = v->degree();
		float u = 3./(8.*n);
		if (n == 3) {
			u = 3. / 16.;
		}
		HalfedgeCIter h = v->halfedge();
		HalfedgeCIter h_orig = h;
		Vector3D sum(0., 0., 0.);
		do {
			h = h->next();
			sum += h->vertex()->position;
			h = h->next();
			h = h->twin();
		} while (h != h_orig);
		v->newPosition = (1 - n * u)*v->position + u * sum;
		v->isNew = false;
	}
	//now lets iterate over the edges to get the positions for the new vertecies were going to add to each edge:
	for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		HalfedgeCIter h = e->halfedge();
		HalfedgeCIter hn2 = h->next()->next();
		HalfedgeCIter t = h->twin();
		HalfedgeCIter tn2 = t->next()->next();
		e->newPosition = 3 * (h->vertex()->position + t->vertex()->position) / 8. + (hn2->vertex()->position + tn2->vertex()->position) / 8.;
		e->isNew = false;
	}
	//Now lets split all of the old edges 
	for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		HalfedgeIter eh = e->halfedge();
		HalfedgeIter twin = eh->twin();
		if (!(eh->vertex()->isNew) && !(twin->vertex()->isNew)) {
			if (!(e->isNew)) {
				Vector3D newpos = e->newPosition;
				VertexIter newV = mesh.splitEdge(e);
				newV->newPosition = newpos;
				newV->isNew = true;
				HalfedgeIter h = newV->halfedge();
				HalfedgeIter h_orig = h;
				int i = 0;
				do {
					if (i == 0) {
						h->edge()->isNew = false;
						i += 1;
					}
					else {
						i -= 1;
						h->edge()->isNew = true;
					}
					h = h->next()->next()->twin();
				} while (h != h_orig);
			}
		}
	}
	//now lets flip the edges we need to flip
	for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		HalfedgeCIter h = e->halfedge();
		HalfedgeCIter t = h->twin();
		if (h->vertex()->isNew != t->vertex()->isNew) {
			if (e->isNew) {
				mesh.flipEdge(e);
			}
		}
	}
	//finaly lets reweight all of the verticeis:
	for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		v->position = v->newPosition;
	}
    return;
  }
}
