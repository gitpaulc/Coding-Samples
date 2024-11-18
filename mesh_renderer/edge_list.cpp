/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include <fstream>

#include "edge_list.h"
#include "point_cloud.h"

namespace MeshRenderer
{
  class DoublyConnectedEdgeList::Impl
  {
  public:
    DoublyConnectedEdgeList* pDcel = nullptr;
    Impl(DoublyConnectedEdgeList* pParent) : pDcel(pParent) {}
    Impl(DoublyConnectedEdgeList* pParent, const std::string& filename);

    struct ObjCount
    {
      int vertexCount = 0;
      int faceCount = 0;
      bool success = false;
    };
    ObjCount readCounts(const std::string& filename);
    bool readObj(const std::string& filename);

    struct HalfEdge;
    struct Vertex
    {
      ComputationalGeometry::point3d coords;
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

  DoublyConnectedEdgeList::DoublyConnectedEdgeList(const std::string& filename) : pImpl(std::make_unique<DoublyConnectedEdgeList::Impl>(this, filename))
  {
  }

  DoublyConnectedEdgeList::Impl::Vertex* DoublyConnectedEdgeList::Impl::HalfEdge::getDest()
  {
    if (reverse == nullptr) { return nullptr; }
    return reverse->source;
  }

  DoublyConnectedEdgeList::Impl::Impl(DoublyConnectedEdgeList* pParent, const std::string& filename) : pDcel(pParent)
  {
    DoublyConnectedEdgeList::Impl::ObjCount counts = readCounts(filename);
    if (counts.success) { std::cout << "\nFile " << filename << " loaded."; }
    else { std::cout << "\nLoading failed."; }
    if (counts.success)
    {
      vertices.resize(counts.vertexCount);
      faces.resize(counts.faceCount);
      readObj(filename);
    }
  }

  DoublyConnectedEdgeList::Impl::ObjCount DoublyConnectedEdgeList::Impl::readCounts(const std::string& filename)
  {
    ObjCount answer;
    try
    {
      std::ifstream ifs(filename);
      answer.success = ifs.good();
      std::string line = "";
      while (std::getline(ifs, line))
      {
        if (line.empty()) { continue; }
        if (line[0] == '#') { continue; }
        if (line.length() > 2)
        {
          if ((line[0] == 'f') && (line[1] == ' '))
          {
            answer.faceCount++;
            continue;
          }
          if ((line[0] == 'v') && (line[1] == ' '))
          {
            answer.vertexCount++;
            continue;
          }
        }
      }
    }
    catch (...)
    {
      answer.success = false;
    }
    return answer;
  }

  bool DoublyConnectedEdgeList::Impl::readObj(const std::string& filename)
  {
    bool answer;
    try
    {
      std::ifstream ifs(filename);
      answer = ifs.good();
      std::string line = "";
      while (std::getline(ifs, line))
      {
        if (line.empty()) { continue; }
        if (line[0] == '#') { continue; }
        if (line.length() > 2)
        {
          if ((line[0] == 'f') && (line[1] == ' '))
          {
            // TODO: Parse faces. First parse vertices.
            continue;
          }
          if ((line[0] == 'v') && (line[1] == ' '))
          {
            // TODO: Parse vertex.
            continue;
          }
        }
      }
    }
    catch (...)
    {
      answer = false;
    }
    return answer;
  }

  int DoublyConnectedEdgeList::getNumEdges() const
  {
    if (pImpl == nullptr) { return 0; }
    return (int)(pImpl->halfEdges.size() / 2);
  }

  int DoublyConnectedEdgeList::getNumFaces() const
  {
    if (pImpl == nullptr) { return 0; }
    return (int)(pImpl->faces.size());
  }

  int DoublyConnectedEdgeList::getNumHalfEdges() const
  {
    if (pImpl == nullptr) { return 0; }
    return (int)(pImpl->halfEdges.size());
  }

  int DoublyConnectedEdgeList::getNumVertices() const
  {
    if (pImpl == nullptr) { return 0; }
    return (int)(pImpl->vertices.size());
  }

  void DoublyConnectedEdgeList::Run(const std::string& filename)
  {
    MeshRenderer::DoublyConnectedEdgeList mesh(filename);
    std::cout << "\nFilename: " << filename;
    std::cout << "\nNum. vertices: " << mesh.getNumVertices();
    std::cout << "\nNum. edges: " << mesh.getNumEdges();
    std::cout << "\nNum. half-edges: " << mesh.getNumHalfEdges();
    std::cout << "\nNum. faces: " << mesh.getNumFaces();
      
    std::cout << "\n\nPress any key to continue:\n-->  ";
    std::string dummy = "";
    std::cin >> dummy;
  }

}
