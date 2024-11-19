/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef MESH_EDGE_LIST_H
#define MESH_EDGE_LIST_H

#include "includes.h"

namespace ComputationalGeometry
{
  class point3d; // Forward declaration.
  class Plane3d; // Forward declaration.
}

namespace MeshRenderer
{
  class DoublyConnectedEdgeList
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      DoublyConnectedEdgeList();
      DoublyConnectedEdgeList(const std::string& filename);
      bool Export(const std::string& filename) const;
      void getBoundingBox(ComputationalGeometry::point3d& maxCorner, ComputationalGeometry::point3d& minCorner) const;
      int getNumEdges() const;
      int getNumFaces() const;
      int getNumHalfEdges() const;
      int getNumVertices() const;
      static void Run(const std::string& filename);
  };
}

#endif //def MESH_EDGE_LIST
