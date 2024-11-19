/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include <fstream>
#include <map>

#include "edge_list.h"
#include "point_cloud.h"

const int DcelNull = -1;

namespace MeshRenderer
{
  bool endsWith(const std::string& str, const std::string& suffix);
  std::size_t findFirstWhitespace(const std::string&);
  bool isWhitespace(char);
  std::string reverse(const std::string&);
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
    bool parseCurrentFaceVertex(int faceIndex, std::vector<int>& faceBuffer, const std::string& info, const std::vector<std::string>& vertexNormals, const std::vector<std::string>& vertexTextures);
    bool setFace(int i, const std::string& faceStr, const std::vector<std::string>& vertexNormals, const std::vector<std::string>& vertexTextures, std::map<std::pair<int, int>, int>& halfEdgesCache);
    bool setVertex(int i, const std::string& vertexStr);
    bool setTexture(int vertIdx, const std::string& vertexTexture);
    bool setNormal(int vertIdx, const std::string& vertexNormal);

    struct HalfEdge;
    typedef int HalfEdgePtr;
    struct Vertex
    {
      ComputationalGeometry::point3d coords;
      ComputationalGeometry::point3d normal;
      ComputationalGeometry::point3d texture;
      bool hasNormal = false;
      bool hasTexture = false;
      /** \brief The half-edge with this vertex as source. */
      HalfEdgePtr halfEdgeFrom = DcelNull;
      int ID = -1;
    };
    typedef int VertexPtr;
    struct Face;
    typedef int FacePtr;
    struct HalfEdge
    {
      /** \brief The vertex which is the source of this half-edge. */
      VertexPtr source = DcelNull;
      /** \brief This same edge traversed backwards. */
      HalfEdgePtr reverse = DcelNull;
      /** A face which lies to the left of this half-edge.
       *  This half-edge is on the (inner portion of the) face's outer boundary.
       */
      FacePtr faceFrom = DcelNull;
      /** \brief A half-edge whose destination is this half-edge's source. */
      HalfEdgePtr prev = DcelNull;
      /** \brief A half-edge whose source is this half-edge's destination. */
      HalfEdgePtr next = DcelNull;
    };
    /** \brief The vertex which is the destination of this half-edge. */
    VertexPtr getDest(const HalfEdge&);
    struct Face
    {
      /** \brief A half-edge on (the inner portion of) this face's outer boundary. */
      HalfEdgePtr outerComponent = DcelNull;
    };

    std::vector<Vertex> vertices;
    std::vector<HalfEdge> halfEdges;
    std::vector<Face> faces;

    std::string mtlFilepath = "";
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

  DoublyConnectedEdgeList::Impl::VertexPtr DoublyConnectedEdgeList::Impl::getDest(const HalfEdge& halfEdge)
  {
    if (halfEdge.reverse == DcelNull) { return DcelNull; }
    return halfEdges[halfEdge.reverse].source;
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
      mtlFilepath = mtlLink;
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
    obj << "\n";
    if (!mtlFilepath.empty())
    {
      obj << "\nmtllib " << mtlFilepath << "\n";
    }
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
    catch (...) { answer = false; }
    return answer;
  }

  bool DoublyConnectedEdgeList::Impl::parseCurrentFaceVertex(int faceIndex, std::vector<int>& faceBuffer, const std::string& info, const std::vector<std::string>& vertexNormals, const std::vector<std::string>& vertexTextures)
  {
    std::string str = info;
    bool success = true;
    int vertIdx = -1;
    int textureIdx = -1;
    int normalIdx = -1;
    try
    {
      std::size_t ind = str.find('/');
      vertIdx = std::stoi(str.substr(0, ind));
      vertIdx--; // Obj indices are ordered from 1 not 0.
      if (vertIdx < 0) { return false; }
      if (vertIdx >= (int)(vertices.size())) { return false; }
      faceBuffer.push_back(vertIdx);
      if (ind == std::string::npos) { return true; }
      if (ind == str.length() - 1) { return false; }
      str = str.substr(ind + 1);
      ind = str.find('/');
      std::string textureInd = trimLeft(str.substr(0, ind));
      if (!(textureInd.empty()))
      {
        textureIdx = std::stoi(textureInd);
        textureIdx--; // Obj indices are ordered from 1 not 0.
        if (textureIdx < 0) { return false; }
        if (textureIdx >= (int)(vertexTextures.size())) { return false; }
      }
      if (ind != std::string::npos)
      {
        if (ind == str.length() - 1) { return false; }
        str = str.substr(ind + 1);
        ind = str.find('/');
        if (ind != std::string::npos) { return false; }
        normalIdx = std::stoi(str);
        normalIdx--; // Obj indices are ordered from 1 not 0.
        if (normalIdx < 0) { return false; }
        if (normalIdx >= (int)(vertexNormals.size())) { return false; }
      }
    }
    catch (...) { success = false; }
    bool ok = setTexture(vertIdx, vertexTextures[textureIdx]);
    success = success && ok;
    if (!success) { return success; }
    ok = setNormal(vertIdx, vertexNormals[normalIdx]);
    success = success && ok;
    return success;
  }

  bool DoublyConnectedEdgeList::Impl::setFace(int i, const std::string& faceStr, const std::vector<std::string>& vertexNormals, const std::vector<std::string>& vertexTextures, std::map<std::pair<int, int>, int>& halfEdgesCache)
  {
    bool success = true;
    std::vector<int> faceBuffer;
    try
    {
      std::string str = faceStr;
      std::size_t ind = findFirstWhitespace(str);
      while (ind != std::string::npos)
      {
        std::string info = str.substr(0, ind);
        bool ok = parseCurrentFaceVertex(i, faceBuffer, info, vertexNormals, vertexTextures);
        success = success && ok;
        if (ind >= str.length() - 1) { break; }
        str = trimLeft(str.substr(ind + 1));
        ind = findFirstWhitespace(str);
      }
      str = trimLeft(str);
      if (!(str.empty()))
      {
        bool ok = parseCurrentFaceVertex(i, faceBuffer, str, vertexNormals, vertexTextures);
        success = success && ok;
      }
      success = success && (faceBuffer.size() >= 3);
    }
    catch (...) { success = false; }
    if (!success) { return false; }

    const int size0 = (int)halfEdges.size();
    const int faceBufferSize = (int)faceBuffer.size();
    for (int ind = 0; ind < faceBufferSize; ++ind)
    {
      int ind1 = ind + 1;
      if (ind == (faceBufferSize - 1)) { ind1 = 0; }
      std::pair<int, int> edgePair;
      edgePair.first = faceBuffer[ind];
      edgePair.second = faceBuffer[ind1];
      if (halfEdgesCache.find(edgePair) != halfEdgesCache.end()) { return false; }
      HalfEdge halfEdge;
      int halfEdgeCurrent = (int)halfEdges.size();
      if (ind == 0)
      {
        Face newFace;
        newFace.outerComponent = halfEdgeCurrent;
        faces[i] = newFace;
      }
      halfEdge.source = edgePair.first;
      halfEdge.faceFrom = i;
      if (ind == 0)
      {
        halfEdge.prev = size0 + faceBufferSize - 1;
      }
      else { halfEdge.prev = halfEdges.size() - 1; }
      if (ind1 == 0)
      {
        halfEdge.next = size0;
      }
      else { halfEdge.next = halfEdges.size() + 1; }
      // Fix reverse if applicable.
      {
        std::pair<int, int> edgeReverse;
        edgeReverse.first = edgePair.second;
        edgeReverse.second = edgePair.first;
        auto it = halfEdgesCache.find(edgeReverse);
        if (it != halfEdgesCache.end())
        {
          int revHalfEdgePtr = it->second;
          HalfEdge& revHalfEdge = halfEdges[revHalfEdgePtr];
          revHalfEdge.reverse = halfEdgeCurrent;
          halfEdge.reverse = revHalfEdgePtr;
        }
      }
      halfEdges.push_back(halfEdge);
      halfEdgesCache[edgePair] = halfEdgeCurrent;
    }

    return success;
  }

  bool DoublyConnectedEdgeList::Impl::setVertex(int i, const std::string& vertexStr)
  {
    bool success = true;
    Vertex& vertex = vertices[i];
    try
    {
      std::string str = vertexStr;
      std::size_t ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      std::string coord = str.substr(0, ind);
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
    if (success) { vertex.ID = i; }
    return success;
  }

  bool DoublyConnectedEdgeList::Impl::setTexture(int vertIdx, const std::string& vertexTexture)
  {
    bool success = true;
    if (vertIdx < 0) { return false; }
    if (vertIdx >= (int)(vertices.size())) { return false; }
    Vertex& vertex = vertices[vertIdx];
    try
    {
      std::string str = vertexTexture;
      std::size_t ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      std::string coord = str.substr(0, ind);
      vertex.texture.x = std::stod(coord);
      str = trimLeft(str.substr(ind));
      ind = findFirstWhitespace(str);
      if (ind != std::string::npos)
      {
        // 0.0 by default.
        coord = str.substr(0, ind);
        vertex.texture.y = std::stod(coord);
        str = trimLeft(str.substr(ind));
        ind = findFirstWhitespace(str);
        coord = str.substr(0, ind);
        vertex.texture.z = std::stod(coord);
      }
    }
    catch (...) { success = false; }
    if (success) { vertex.hasTexture = true; }
    return success;
  }

  bool DoublyConnectedEdgeList::Impl::setNormal(int vertIdx, const std::string& vertexNormal)
  {
    bool success = true;
    if (vertIdx < 0) { return false; }
    if (vertIdx >= (int)(vertices.size())) { return false; }
    Vertex& vertex = vertices[vertIdx];
    try
    {
      std::string str = vertexNormal;
      std::size_t ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      std::string coord = str.substr(0, ind);
      vertex.normal.x = std::stod(coord);
      str = trimLeft(str.substr(ind));
      ind = findFirstWhitespace(str);
      if (ind == std::string::npos) { return false; }
      coord = str.substr(0, ind);
      vertex.normal.y = std::stod(coord);
      str = trimLeft(str.substr(ind));
      ind = findFirstWhitespace(str);
      coord = str.substr(0, ind);
      vertex.normal.z = std::stod(coord);
    }
    catch (...) { success = false; }
    if (success) { vertex.hasNormal = true; }
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
    std::map<std::pair<int, int>, int> halfEdgesCache;
    for (int i = 0; i < (int)faceBuffer.size(); ++i)
    {
      bool success = setFace(i, trimLeft(faceBuffer[i]), vertexNormals, vertexTextures, halfEdgesCache);
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
      if (halfEdge.reverse != DcelNull) { continue; }
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
      if (endsWith(filename, ".obj")) { logFile = filename.substr(0, filename.length() - 4) + "Out.obj"; }
      if (mesh.Export(logFile))
      {
        std::cout << "\nFile " << logFile << " exported.";
      }
      else { std::cout << "\nExport failed."; }
    }
    std::cout << "\n\nPress any key to continue:\n-->  ";
    std::string dummy = "";
    std::cin >> dummy;
  }

  bool endsWith(const std::string& str, const std::string& suffix)
  {
    return startsWith(reverse(str), reverse(suffix));
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
    if (cc == '\r') { return true; }
    return false;
  }

  std::string reverse(const std::string& str)
  {
    std::string answer = str;
    int strLen = (int)str.length();
    for (int i = 0; i < strLen; ++i)
    {
      answer[i] = str[strLen - 1 - i];
    }
    return answer;
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
    std::string answer = "";
    int strLen = (int)str.length();
    if (strLen == 0) { return answer; }
    bool started = false;
    for (int i = 0; i < strLen; ++i)
    {
      if (!isWhitespace(str[i])) { started = true; }
      if (started) { answer += str[i]; }
    }
    return answer;
  }
}
