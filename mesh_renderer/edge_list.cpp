/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#include <fstream>
#include <map>
#include <sstream>

#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

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
    bool getWireframe(std::vector<ComputationalGeometry::Edge3d>& wireframeOut) const;
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
    /** \brief The optional output map sends vertexIndex |--> { faceIndex, index within face }
     */
    std::map<int, ComputationalGeometry::Face3d> getFaces() const;
    bool getSkeleton(std::vector<ComputationalGeometry::Edge3d>& meshOut, std::vector<ComputationalGeometry::Edge3d>& normalsOut) const;
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
    VertexPtr getDest(const HalfEdge&) const;
    struct Face
    {
      /** \brief A half-edge on (the inner portion of) this face's outer boundary. */
      HalfEdgePtr outerComponent = DcelNull;
    };

    std::vector<Vertex> vertices;
    std::vector<HalfEdge> halfEdges;
    std::vector<Face> faces;

    std::string originalFilename = "";
    std::string mtlFilepath = "";
    DoublyConnectedEdgeList::RenderMode renderMode = DoublyConnectedEdgeList::RenderMode::Wireframe;
  };

  DoublyConnectedEdgeList::DoublyConnectedEdgeList() : pImpl(std::make_unique<DoublyConnectedEdgeList::Impl>(this))
  {
    // unique_ptr requires C++ 11.
    // make_unique requires C++ 14.
  }

  DoublyConnectedEdgeList::DoublyConnectedEdgeList(const std::string& filename) : pImpl(std::make_unique<DoublyConnectedEdgeList::Impl>(this, filename))
  {
  }

  static DoublyConnectedEdgeList sMesh;
  void DoublyConnectedEdgeList::Create(const std::string& filename)
  {
    sMesh = DoublyConnectedEdgeList(filename);
    std::cout << "\nFilename: " << filename;
    std::cout << "\nNum. vertices: " << sMesh.getNumVertices();
    std::cout << "\nNum. edges: " << sMesh.getNumEdges();
    std::cout << "\nNum. half-edges: " << sMesh.getNumHalfEdges();
    std::cout << "\nNum. faces: " << sMesh.getNumFaces();
    std::cout << "\n\n";
  }

  DoublyConnectedEdgeList& DoublyConnectedEdgeList::Get()
  {
    return sMesh;
  }

  bool DoublyConnectedEdgeList::Export(const std::string& filename) const
  {
    if (pImpl == nullptr) { return false; }
    std::string exportName = filename;
    if (filename.empty())
    {
      exportName = pImpl->originalFilename + ".log";
      if (endsWith(pImpl->originalFilename, ".obj")) { exportName = pImpl->originalFilename.substr(0, pImpl->originalFilename.length() - 4) + "Out.obj"; }
    }
    bool success = pImpl->Export(exportName);
    if (success)
    {
      std::cout << "\nFile " << exportName << " exported.";
    }
    else { std::cout << "\nExport failed."; }
    return success;
  }

  DoublyConnectedEdgeList::Impl::VertexPtr DoublyConnectedEdgeList::Impl::getDest(const HalfEdge& halfEdge) const
  {
    if (halfEdge.next != DcelNull)
    {
      return halfEdges[halfEdge.next].source;
    }
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
      originalFilename = filename;
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
    obj << "\n\n# " << vertices.size() << " vertices in all.\n";
    std::map<int, int> texturesIndMap;
    std::map<int, int> normalsIndMap;
    {
      int textureCount = 0;
      int loopInd = -1;
      for (const auto& vertex : vertices)
      {
        ++loopInd;
        if (!vertex.hasTexture) { continue; }
        texturesIndMap[loopInd] = textureCount;
        textureCount++;
        obj << "\nvt " << vertex.texture.x << " " << vertex.texture.y << " " << vertex.texture.z;
      }
      obj << "\n\n# " << textureCount << " textures in all.\n";
    }
    {
      int normalsCount = 0;
      int loopInd = -1;
      for (const auto& vertex : vertices)
      {
        ++loopInd;
        if (!vertex.hasNormal) { continue; }
        normalsIndMap[loopInd] = normalsCount;
        normalsCount++;
        obj << "\nvn " << vertex.normal.x << " " << vertex.normal.y << " " << vertex.normal.z;
      }
      obj << "\n\n# " << normalsCount << " normals in all.\n";
    }
    for (const auto& face : faces)
    {
      if (face.outerComponent < 0) { continue; }
      if (face.outerComponent >= (int)(halfEdges.size())) { continue; }
      const HalfEdge& firstEdge = halfEdges[face.outerComponent];
      int current = firstEdge.next;
      if (current < 0) { continue; }
      if (current >= (int)(halfEdges.size())) { continue; }
      auto currentEdge = halfEdges[current];
      std::stringstream faceStrm;
      faceStrm << "\nf " << (firstEdge.source + 1);
      {
        const Vertex& vertex = vertices[currentEdge.source];
        if (vertex.hasTexture || vertex.hasNormal) { faceStrm << "/"; }
        if (vertex.hasTexture) { faceStrm << (texturesIndMap[firstEdge.source] + 1); }
        if (vertex.hasNormal) { faceStrm << "/" << (normalsIndMap[firstEdge.source] + 1); }
      }
      bool faceOk = true;
      for (int numCorners = 0; currentEdge.source != firstEdge.source; ++numCorners)
      {
        if (currentEdge.source < 0) { faceOk = false; break; }
        if (currentEdge.source >= (int)(vertices.size())) { faceOk = false; break; }
        const Vertex& vertex = vertices[currentEdge.source];
        faceStrm << " " << (currentEdge.source + 1);
        if (vertex.hasTexture || vertex.hasNormal) { faceStrm << "/"; }
        if (vertex.hasTexture) { faceStrm << (texturesIndMap[currentEdge.source] + 1); }
        if (vertex.hasNormal) { faceStrm << "/" << (normalsIndMap[currentEdge.source] + 1); }
        current = currentEdge.next;
        if (current < 0) { faceOk = false; break; }
        if (current >= (int)(halfEdges.size())) { faceOk = false; break; }
        if (numCorners >= (int)(halfEdges.size())) { faceOk = false; break; }
        currentEdge = halfEdges[current];
      }
      if (!faceOk) { continue; }
      obj << faceStrm.str();
    }
    obj << "\n\n# " << faces.size() << " faces in all.\n";
    return true;
  }

  bool DoublyConnectedEdgeList::Impl::getWireframe(std::vector<ComputationalGeometry::Edge3d>& wireframeOut) const
  {
    using namespace ComputationalGeometry;
    wireframeOut.resize(0);
    std::set<Edge3d> added;
    std::map<int, Face3d> renderFaces = getFaces();
    for (const auto& faceIt : renderFaces)
    {
      std::vector<point3d> corners;
      bool addFace = (faceIt.second.vertices.size() >= 3);
      if (!addFace) { continue; }
      for (const point3d& target : faceIt.second.vertices)
      {
        corners.push_back(target);
      }
      for (int ii = 0; ii < (int)corners.size(); ++ii)
      {
        int jj = ii + 1;
        if (jj == (int)corners.size()) { jj = 0; }
        Edge3d toAdd(corners[ii], corners[jj]);
        if (added.count(toAdd) > 0) { continue; }
        wireframeOut.push_back(toAdd);
        added.insert(toAdd);
        added.insert(Edge3d(corners[jj], corners[ii]));
      }
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
      // if (halfEdgesCache.find(edgePair) != halfEdgesCache.end()) { return false; }
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
      else { halfEdge.prev = (int)halfEdges.size() - 1; }
      if (ind1 == 0)
      {
        halfEdge.next = size0;
      }
      else { halfEdge.next = (int)halfEdges.size() + 1; }
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

  std::map<int, ComputationalGeometry::Face3d> DoublyConnectedEdgeList::Impl::getFaces() const
  {
    std::map<int, ComputationalGeometry::Face3d> faceMap;
    const int facesSize = (int)(faces.size());
    for (int ind = 0; ind < facesSize; ++ind)
    {
      HalfEdgePtr initialPtr = faces[ind].outerComponent;
      if (initialPtr == DcelNull) { continue; }
      if (initialPtr < 0) { continue; }
      if (initialPtr >= (int)(halfEdges.size())) { continue; }
      const HalfEdge& initial = halfEdges[initialPtr];
      if (initial.source == DcelNull) { continue; }
      if (initial.source < 0) { continue; }
      if (initial.source >= (int)(vertices.size())) { continue; }
      const Vertex& initialSrc = vertices[initial.source];
      ComputationalGeometry::Face3d face;
      face.vertices.push_back(initialSrc.coords);
      HalfEdge current = initial;
      for (int loopCount = 0; (current.next != initialPtr) && (loopCount < (int)vertices.size()); ++loopCount)
      {
        if (current.next == DcelNull) { break; }
        if (current.next < 0) { break; }
        if (current.next >= (int)(halfEdges.size())) { break; }
        current = halfEdges[current.next];
        if (current.source == DcelNull) { break; }
        if (current.source < 0) { break; }
        if (current.source >= (int)(vertices.size())) { break; }
        const Vertex& currentSrc = vertices[current.source];
        face.vertices.push_back(currentSrc.coords);
      }
      faceMap[ind] = face;
    }
    return faceMap;
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

  void DoublyConnectedEdgeList::getBoundingBox(ComputationalGeometry::point3d& maxCorner, ComputationalGeometry::point3d& minCorner) const
  {
    if (pImpl == nullptr) { return; }
    using namespace ComputationalGeometry;
    maxCorner = point3d();
    minCorner = point3d();
    bool started = false;
    for (const auto& vertex : pImpl->vertices)
    {
      if (!started)
      {
        maxCorner = vertex.coords;
        minCorner = vertex.coords;
        started = true;
        continue;
      }
      if (vertex.coords.x < minCorner.x) { minCorner.x = vertex.coords.x; }
      else if (vertex.coords.x > maxCorner.x) { maxCorner.x = vertex.coords.x; }
      if (vertex.coords.y < minCorner.y) { minCorner.y = vertex.coords.y; }
      else if (vertex.coords.y > maxCorner.y) { maxCorner.y = vertex.coords.y; }
      if (vertex.coords.z < minCorner.z) { minCorner.z = vertex.coords.z; }
      else if (vertex.coords.z > maxCorner.z) { maxCorner.z = vertex.coords.z; }
    }
  }

  bool DoublyConnectedEdgeList::Impl::getSkeleton(std::vector<ComputationalGeometry::Edge3d>& meshOut,
    std::vector<ComputationalGeometry::Edge3d>& normalsOut) const
  {
    using namespace ComputationalGeometry;
    meshOut.resize(0);
    struct FaceTriangle
    {
      VertexPtr a = DcelNull;
      VertexPtr b = DcelNull;
      VertexPtr c = DcelNull;
      void addTo(std::vector<FaceTriangle>& triangles, std::map<VertexPtr,
        std::set<vector3d> >& vertexNormals)
      {
        if (a == DcelNull) { return; }
        if (b == DcelNull) { return; }
        if (c == DcelNull) { return; }
        triangles.push_back(*this);
      }
    };
    std::vector<FaceTriangle> triangles;
    std::map<VertexPtr, std::set<vector3d> > vertexNormals;
    const int facesSize = (int)(faces.size());
    for (int ind = 0; ind < facesSize; ++ind)
    {
      HalfEdgePtr initialPtr = faces[ind].outerComponent;
      if (initialPtr == DcelNull) { continue; }
      if (initialPtr < 0) { continue; }
      if (initialPtr >= (int)(halfEdges.size())) { continue; }
      const HalfEdge& initial = halfEdges[initialPtr];
      if (initial.source == DcelNull) { continue; }
      if (initial.source < 0) { continue; }
      if (initial.source >= (int)(vertices.size())) { continue; }

      FaceTriangle tri;
      tri.a = initial.source;
      int ii = 1;
      HalfEdge current = initial;
      for (int loopCount = 0; (current.next != initialPtr) && (loopCount < (int)vertices.size()); ++loopCount)
      {
        if (current.next == DcelNull) { break; }
        if (current.next < 0) { break; }
        if (current.next >= (int)(halfEdges.size())) { break; }
        current = halfEdges[current.next];
        if (current.source == DcelNull) { break; }
        if (current.source < 0) { break; }
        if (current.source >= (int)(vertices.size())) { break; }

        if (ii == 1) { tri.b = current.source; }
        else if (ii == 2)
        {
          tri.c = current.source;
        }
        else if (ii >= 3)
        {
          tri.b = tri.c;
          tri.c = current.source;
        }
        tri.addTo(triangles, vertexNormals);
        ++ii;
      }
    }

    for (const auto& faceTri : triangles)
    {
      Triangle3d tri;
      if (faceTri.a == DcelNull) { continue; }
      if (faceTri.b == DcelNull) { continue; }
      if (faceTri.c == DcelNull) { continue; }
      if (faceTri.a < 0) { continue; }
      if (faceTri.b < 0) { continue; }
      if (faceTri.c < 0) { continue; }
      if (faceTri.a >= (int)(vertices.size())) { continue; }
      if (faceTri.b >= (int)(vertices.size())) { continue; }
      if (faceTri.c >= (int)(vertices.size())) { continue; }
      tri.a = vertices[faceTri.a].coords;
      tri.b = vertices[faceTri.b].coords;
      tri.c = vertices[faceTri.c].coords;
      auto edgeSet = tri.getEdges();
      if (edgeSet.size() < 3) { continue; }
      int ii = 0;
      for (const auto& edg : edgeSet)
      {
        if (ii >= 3) { break; }
        meshOut.push_back(edg);
        ++ii;
      }
    }
    
    return true;
  }

  bool DoublyConnectedEdgeList::getSkeleton(std::vector<ComputationalGeometry::Edge3d>& meshOut,
    std::vector<ComputationalGeometry::Edge3d>& normalsOut) const
  {
    meshOut.resize(0);
    if (pImpl == nullptr) { return false; }
    return pImpl->getSkeleton(meshOut, normalsOut);
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

  bool DoublyConnectedEdgeList::getWireframe(std::vector<ComputationalGeometry::Edge3d>& wireframeOut) const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->getWireframe(wireframeOut);
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
