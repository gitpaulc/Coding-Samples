/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include <fstream>

#include "edge_list.h"
#include "point_cloud.h"

namespace MeshRenderer
{
  std::size_t findFirstWhitespace(const std::string&);
  bool isWhitespace(char);
  bool startsWith(const std::string& str, const std::string& prefix);
  std::string trimLeft(const std::string&);

  class DoublyConnectedEdgeList::Impl
  {
  public:
    DoublyConnectedEdgeList* pDcel = nullptr;
    Impl(DoublyConnectedEdgeList* pParent) : pDcel(pParent) {}
    Impl(DoublyConnectedEdgeList* pParent, const std::string& filename);

    bool Export(const std::string& filename) const;
    /**
     * vertexBuffer line starts with "v"
     * vertexNormals line starts with "vn"
     * vertexTextures line starts with "vt"
     * faceBuffer line starts with "f"
     */
    bool readObj(const std::string& filename,
      std::string& mtlLink,
      std::vector<std::string>& vertexBuffer,
      std::vector<std::string>& vertexNormals,
      std::vector<std::string>& vertexTextures,
      std::vector<std::string>& faceBuffer);
    bool parseObj(const std::string& filename,
      const std::string& mtlLink,
      const std::vector<std::string>& vertexBuffer,
      const std::vector<std::string>& vertexNormals,
      const std::vector<std::string>& vertexTextures,
      const std::vector<std::string>& faceBuffer);
    bool setVertex(int i, const std::string& vertexStr);

    struct HalfEdge;
    struct Vertex
    {
      ComputationalGeometry::point3d coords;
      ComputationalGeometry::point3d normal;
      ComputationalGeometry::point3d texture;
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

  bool DoublyConnectedEdgeList::Export(const std::string& filename) const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->Export(filename);
  }

  DoublyConnectedEdgeList::Impl::Vertex* DoublyConnectedEdgeList::Impl::HalfEdge::getDest()
  {
    if (reverse == nullptr) { return nullptr; }
    return reverse->source;
  }

  DoublyConnectedEdgeList::Impl::Impl(DoublyConnectedEdgeList* pParent, const std::string& filename) : pDcel(pParent)
  {
    std::string mtlLink = "";
    std::vector<std::string> vertexBuffer;
    std::vector<std::string> vertexNormals;
    std::vector<std::string> vertexTextures;
    std::vector<std::string> faceBuffer;
    bool success = readObj(filename, mtlLink, vertexBuffer, vertexNormals, vertexTextures, faceBuffer);
    if (success)
    {
      success = parseObj(filename, mtlLink, vertexBuffer, vertexNormals, vertexTextures, faceBuffer);
    }
    if (success)
    {
      std::cout << "\nFile " << filename << " loaded.";
    }
    else { std::cout << "\nLoading failed."; }
  }

  bool DoublyConnectedEdgeList::Impl::Export(const std::string& filename) const
  {
    std::ofstream obj(filename);
    if (!(obj.good())) { return false; }
    for (const auto& vertex : vertices)
    {
      obj << "\nv " << vertex.coords.x << " " << vertex.coords.y << " " << vertex.coords.z;
    }
    return true;
  }

  bool DoublyConnectedEdgeList::Impl::readObj(const std::string& filename,
    std::string& mtlLink,
    std::vector<std::string>& vertexBuffer,
    std::vector<std::string>& vertexNormals,
    std::vector<std::string>& vertexTextures,
    std::vector<std::string>& faceBuffer)
  {
    bool answer = false;
    mtlLink = "";
    vertexBuffer.resize(0);
    vertexNormals.resize(0);
    vertexTextures.resize(0);
    faceBuffer.resize(0);
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
          std::string prefix = "f ";
          if (startsWith(line, prefix))
          {
            faceBuffer.push_back(line.substr(prefix.length()));
            continue;
          }
          prefix = "vt ";
          if (startsWith(line, prefix))
          {
            vertexTextures.push_back(line.substr(prefix.length()));
            continue;
          }
          prefix = "vn ";
          if (startsWith(line, prefix))
          {
            vertexNormals.push_back(line.substr(prefix.length()));
            continue;
          }
          prefix = "v ";
          if (startsWith(line, prefix))
          {
            vertexBuffer.push_back(line.substr(prefix.length()));
            continue;
          }
          prefix = "mtllib ";
          if (startsWith(line, prefix))
          {
            mtlLink = line.substr(prefix.length());
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

  bool DoublyConnectedEdgeList::Impl::setVertex(int i, const std::string& vertexStr)
  {
    bool success = true;
    try
    {
      std::string str = vertexStr;
      std::size_t ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      std::string coord = str.substr(0, ind);
      Vertex& vertex = vertices[i];
      vertex.coords.x = std::stod(coord);
      str = trimLeft(str.substr(ind));
      ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      coord = str.substr(0, ind);
      vertex.coords.y = std::stod(coord);
      str = trimLeft(str.substr(ind));
      ind = findFirstWhitespace(str);
      //if (ind == std::string::npos) { return false; } w-coordinate is optional.
      coord = str.substr(0, ind);
      vertex.coords.z = std::stod(coord);
    }
    catch (...) { success = false; }
    return success;
  }

  bool DoublyConnectedEdgeList::Impl::parseObj(const std::string& filename,
    const std::string& mtlLink,
    const std::vector<std::string>& vertexBuffer,
    const std::vector<std::string>& vertexNormals,
    const std::vector<std::string>& vertexTextures,
    const std::vector<std::string>& faceBuffer)
  {
    bool answer = true;
    vertices.resize(vertexBuffer.size());
    faces.resize(faceBuffer.size());
    for (int i = 0; i < (int)vertexBuffer.size(); ++i)
    {
      bool success = setVertex(i, trimLeft(vertexBuffer[i]));
      answer = answer && success;
    }
    return answer;
  }

  int DoublyConnectedEdgeList::getNumEdges() const
  {
    if (pImpl == nullptr) { return 0; }
    const int numHalfEdges = (int)(pImpl->halfEdges.size());
    int numBoundaryEdges = 0;
    for (int i = 0; i < numHalfEdges; ++i)
    {
      const auto& halfEdge = (pImpl->halfEdges)[i];
      if (halfEdge.reverse != nullptr) { continue; }
      ++numBoundaryEdges;
    }
    return ((numHalfEdges - numBoundaryEdges) / 2) + numBoundaryEdges;
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
    {
      std::string logFile = filename + ".log";
      mesh.Export(logFile);
    }
    std::cout << "\n\nPress any key to continue:\n-->  ";
    std::string dummy = "";
    std::cin >> dummy;
  }

  std::size_t findFirstWhitespace(const std::string& str)
  {
    for (std::size_t ind = 0; ind < str.length(); ++ind)
    {
      if (isWhitespace(str[ind])) { return ind; }
    }
    return std::string::npos;
  }

  bool isWhitespace(char cc)
  {
    if (cc == ' ') { return true; }
    if (cc == '\t') { return true; }
    if (cc == '\n') { return true; }
    return false;
  }

  bool startsWith(const std::string& str, const std::string& prefix)
  {
    int prefixLen = (int)prefix.length();
    if (prefixLen > (int)str.length()) { return false; }
    for (int i = 0; i < prefixLen; ++i)
    {
      if (str[i] != prefix[i]) { return false; }
    }
    return true;
  }

  std::string trimLeft(const std::string& str)
  {
    std::string answer = str;
    if (answer.empty()) { return answer; }
    while (isWhitespace(answer[0]))
    {
      answer = answer.substr(1);
      if (answer.empty()) { break; }
    }
    return answer;
  }
}
