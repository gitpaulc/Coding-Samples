
#include "voronoi.h"

#include "includes.h"

namespace ComputationalGeometry
{

  HalfEdge::HalfEdge(Edge* e, int pm)
  {
    edgeListEdge = e;
    edgeListPm = pm;
  }

} // end namespace ComputationalGeometry
