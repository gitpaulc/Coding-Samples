/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include "edge_list.h"

#include "point_cloud.h"

namespace ComputationalGeometry
{

  class DoublyConnectedEdgeList::Impl
  {
  public:
    DoublyConnectedEdgeList* pDcel = nullptr;
    Impl(DoublyConnectedEdgeList* pParent) : pDcel(pParent) {}

    struct HalfEdge;
    struct Vertex
    {
      point2d coords;
      /** \brief The half-edge with this vertex as source. */
      HalfEdge* halfEdgeFrom = nullptr;
    };
    struct Face;
    struct HalfEdge
    {
      /** \brief The vertex which is the source of this half-edge. */
      Vertex* source = nullptr;
      /** \brief This same edge traversed backwards. */
      HalfEdge* reverse = nullptr;
      /** A face which lies to the left of this half-edge.
       *  This half-edge is on the (inner portion of the) face's outer boundary.
       */
      Face* faceFrom = nullptr;
      /** \brief A half-edge whose destination is this half-edge's source. */
      HalfEdge* prev = nullptr;
      /** \brief A half-edge whose source is this half-edge's destination. */
      HalfEdge* next = nullptr;
      /** \brief The vertex which is the destination of this half-edge. */
      Vertex* getDest();
    };
    struct Face
    {
      /** \brief A half-edge on (the inner portion of) this face's outer boundary. */
      HalfEdge* outerComponent = nullptr;
      /** A set of half-edges.
       * Each half-edge is on (the inner portion of) this face's inner boundary on a distinct hole.
       * So there are as many elements in the set as there are holes on this face.
       */
      std::set<HalfEdge*> holes;
    };

    std::vector<Vertex> vertices;
    std::vector<HalfEdge> halfEdges;
    std::vector<Face> faces;
  };

  DoublyConnectedEdgeList::DoublyConnectedEdgeList() : pImpl(std::make_unique<DoublyConnectedEdgeList::Impl>(this))
  {
    // unique_ptr requires C++ 11.
    // make_unique requires C++ 14.
  }

  DoublyConnectedEdgeList::Impl::Vertex* DoublyConnectedEdgeList::Impl::HalfEdge::getDest()
  {
    if (reverse == nullptr) { return nullptr; }
    return reverse->source;
  }

}
