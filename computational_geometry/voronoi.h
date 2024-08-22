/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef VORONOI_H
#define VORONOI_H

#include "point_cloud.h"

#include <map>

namespace ComputationalGeometry
{
  class Site // e.g., Voronoi site
  {
  public:
    point2d coord;
    int siteNumber = 0;
    int refCount = 0;
  };

  class Edge
  {
  public:
    point3d abc;
    Site* endpoint0 = nullptr;
    Site* endpoint1 = nullptr;
    Site* reg0 = nullptr;
    Site* reg1 = nullptr;
    int edgeNumber = 0;
  };

  enum class EdgeStatus
  {
    LE = 0,
    RE = 1,
    DELETED = -2
  };

  class HalfEdge
  {
  public:
    HalfEdge* edgeListLeft = nullptr;
    HalfEdge* edgeListRight = nullptr;
    Edge* edgeListEdge = nullptr;
    char edgeListPm = 0;
    Site* vertex = nullptr;
    double ystar  = 0.0;
    HalfEdge* pqNext = nullptr;

    HalfEdge(Edge* e = nullptr, int pm = 0);
  };

  class EdgeList
  {
  public:
    Site* bottomSite = nullptr;
    std::map<int, HalfEdge> body;
    HalfEdge* leftEnd = nullptr;
    HalfEdge* rightEnd = nullptr;
  };

} // end namespace ComputationalGeometry

#endif //def VORONOI_H
