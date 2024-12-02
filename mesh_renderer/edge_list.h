/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef MESH_EDGE_LIST_H
#define MESH_EDGE_LIST_H

#include "includes.h"

namespace ComputationalGeometry
{
  class Edge2d;  // Forward declaration.
  class point3d; // Forward declaration.
  class Plane3d; // Forward declaration.
}

namespace MeshRenderer
{
  class Camera; // Forward declaration.

  class DoublyConnectedEdgeList
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      enum class RenderMode
      {
        Opaque = 0,
        Wireframe = 1
      };
      DoublyConnectedEdgeList();
      DoublyConnectedEdgeList(const std::string& filename);
      static void Create(const std::string& filename);
      static DoublyConnectedEdgeList& Get();
      bool Export(const std::string& filename = "") const;
      void getBoundingBox(ComputationalGeometry::point3d& maxCorner, ComputationalGeometry::point3d& minCorner) const;
      int getNumEdges() const;
      int getNumFaces() const;
      int getNumHalfEdges() const;
      int getNumVertices() const;
      /** \brief Project mesh to the screen of the camera. Theta is the screen rotation. */
      bool project(const Camera&, std::vector<ComputationalGeometry::Edge2d>& wireframeOut, double theta) const;
  };
}

#endif //def MESH_EDGE_LIST
