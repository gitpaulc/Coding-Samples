/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef MESH_EDGE_LIST_H
#define MESH_EDGE_LIST_H

#include "includes.h"

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
      int getNumEdges() const;
      int getNumFaces() const;
      int getNumHalfEdges() const;
      int getNumVertices() const;
      static void Run(const std::string& filename);
  };
}

#endif //def MESH_EDGE_LIST
