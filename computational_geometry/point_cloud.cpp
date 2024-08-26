
#include "includes.h"
#include "point_cloud.h"
#include "gl_callbacks.h"

#ifdef _WIN32
#include <algorithm>
#endif
#include <map>

namespace ComputationalGeometry
{
  static int gWindowWidth = 1024;
  static int gWindowHeight = 1024;

  int& numRandomPoints()
  {
    static int numberOfRandomPoints = 50;
    return numberOfRandomPoints;
  }

  void SetWindowWidthHeight(int ww, int hh)
  {
    ComputationalGeometry::gWindowWidth = ww;
    if (hh < 0) { hh = ww; }
    ComputationalGeometry::gWindowHeight = hh;
  }

  void GetWindowWidthHeight(int& ww, int& hh)
  {
    ww = gWindowWidth;
    hh = gWindowHeight;
  }

  double threshold() { return 1.0e-9; }

  point3d::point3d() : x(0), y(0), z(0) {}
  point3d::point3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}

  int point3d::GetDimension() const { return 3; }

  bool point3d::operator< (const point3d& q) const
  {
    if (x > q.x) {return false;}
    if (x < q.x) {return true;}
    if (y > q.y) {return false;}
    if (y < q.y) {return true;}
    if (GetDimension() <= 2) {return false;}
    if (z > q.z) {return false;}
    if (z < q.z) {return true;}
    return false;
  }

  void point3d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y;
    if (GetDimension() > 2) {std::cout << ", " << z;}
    std::cout << ")";
  }

  double point3d::sq_distance(const point3d& P, const point3d& Q)
  {
    double answer = 0;
    double dt = (P.x - Q.x);
    answer = answer + dt * dt;
    dt = (P.y - Q.y);
    answer = answer + dt * dt;
    if ((P.GetDimension() > 2) || (Q.GetDimension() > 2))
    {
      dt = (P.z - Q.z);
      answer = answer + dt * dt;
    }
    return answer;
  }

  point2d::point2d() : point3d(0, 0, 0) {}
  point2d::point2d(const double& xx, const double& yy) : point3d(xx, yy, 0) {}

  int point2d::GetDimension() const { return 2; }

  double point2d::getOrientation(const point2d& P, const point2d& Q, const point2d& O)
  {
    return (P.x - O.x) * (Q.y - O.y) - (P.y - O.y) * (Q.x - O.x);
  }

  bool point2d::comparator(const point2d& P, const point2d& Q)
  {
    // Equivalent to P < Q
    
    double theta_P = atan2(P.y, P.x);
    double theta_Q = atan2(Q.y, Q.x);
    
    return theta_P < theta_Q;
    //Also can use return getOrientation(P, Q) < 0;
  }

  Edge2d::Edge2d(const point2d& aa, const point2d& bb)
  {
    a = aa; b = bb;
  }

  bool Edge2d::operator< (const Edge2d& rhs) const
  {
    if (rhs.a < a) {return false;}
    if (a < rhs.a) {return true;}
    if (rhs.b < b) {return false;}
    if (b < rhs.b) {return true;}
    return false;
  }

  double Edge2d::sqLength() const
  {
    return point2d::sq_distance(a, b);
  }

  double Edge2d::sq_distance(const point2d& P) const
  {
    double aSqDistP = point2d::sq_distance(a, P);
    if (sqLength() <= threshold())
    {
      return aSqDistP;
    }
    const double& x0 = P.x;
    const double& y0 = P.y;
    const double& x1 = a.x;
    const double& y1 = a.y;
    const double& x2 = b.x;
    const double& y2 = b.y;
    double numSqrt = (y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1;
    double answer = numSqrt * numSqrt / sqLength();
    if (answer > threshold()) { return answer; }
    double bSqDistP = point2d::sq_distance(b, P);
    if ((aSqDistP < sqLength()) && (bSqDistP < sqLength())) { return 0.0; }
    if (aSqDistP >= sqLength()) { return bSqDistP; }
    return aSqDistP;
  }

  /** \brief 0 = no intersection, 1 = point intersection, 2 = parallel intersection */
  int Edge2d::intersection(const Edge2d& other, point2d& intersection) const
  {
    if (point2d::sq_distance(a, b) <= threshold())
    {
      if (point2d::sq_distance(a, other.a) <= threshold())
      {
        intersection = a; return 2;
      }
      if (point2d::sq_distance(a, other.b) <= threshold())
      {
        intersection = a; return 2;
      }
      return 0;
    }
    const double& x1 = a.x;
    const double& y1 = a.y;
    const double& x2 = b.x;
    const double& y2 = b.y;
    const double& x3 = other.a.x;
    const double& y3 = other.a.y;
    const double& x4 = other.b.x;
    const double& y4 = other.b.y;
    const double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    const double absDet = (det > 0) ? det : -det;

    // t = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    //     ---------------------------------------------
    //     (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    //
    // u = (x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)
    //     ---------------------------------------------
    //     (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
      
    // For intersection as point, t and u must be between 0 and 1.
    bool bDetNonzero = (absDet > threshold());
    if ((point2d::sq_distance(a, other.a) <= threshold()) || (point2d::sq_distance(a, other.b) <= threshold()))
    {
      intersection = a;
      return bDetNonzero ? 1 : 2;
    }
    if ((point2d::sq_distance(b, other.a) <= threshold()) || (point2d::sq_distance(b, other.b) <= threshold()))
    {
      intersection = b;
      return bDetNonzero ? 1 : 2;
    }
    double tNum = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    double uNum = (x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3);
    if (bDetNonzero)
    {
      if ((tNum < 0) && (det > 0)) { return 0; }
      if ((tNum > 0) && (det < 0)) { return 0; }
      if ((uNum < 0) && (det > 0)) { return 0; }
      if ((uNum > 0) && (det < 0)) { return 0; }
      if ((tNum > det) && (det > 0)) { return 0; }
      if ((tNum < det) && (det < 0)) { return 0; }
      if ((uNum > det) && (det > 0)) { return 0; }
      if ((uNum < det) && (det < 0)) { return 0; }
      intersection = point2d(a.x + tNum * b.x / det, a.y + tNum * b.y / det);
      return 1;
    }
    // Parallel and non-collinear or else edges intersect.
    if (sq_distance(other.a) < threshold())
    {
      intersection = other.a;
      return 2;
    }
    if (sq_distance(other.b) < threshold())
    {
      intersection = other.b;
      return 2;
    }
    if (other.sq_distance(a) < threshold())
    {
      intersection = a;
      return 2;
    }
    // else if (other.sq_distance(b) < threshold())
    {
      intersection = b;
      return 2;
    }
  }

  point2d Edge2d::projection(const point2d& P) const
  {
    if (sqLength() <= threshold()) { return a; }
    point2d pp(P.x - a.x, P.y - a.y);
    point2d qq(b.x - a.x, b.y - a.y);
    double squareNorm = point2d::sq_distance(qq, point2d(0.0, 0.0));
    double tt = (qq.y * pp.x - qq.x * pp.y) / squareNorm;
    point2d rr(pp.x - tt * qq.y, pp.y + tt * qq.x);
    point2d answer(rr.x + a.x, rr.y + a.y);
    return answer;
  }

  Matrix2d::Matrix2d(const point2d& aa, const point2d& bb)
  {
    a = aa; b = bb;
  }

  double Matrix2d::det() const
  {
    return a.x * b.y - a.y * b.x;
  }

  Matrix3d::Matrix3d(const point3d& aa, const point3d& bb, const point3d& cc)
  {
    a = aa; b = bb; c = cc;
  }

  double Matrix3d::det() const
  {
    Matrix2d cof(point2d(b.y, b.z), point2d(c.y, c.z));
    double answer = a.x * cof.det();
    cof = Matrix2d(point2d(b.x, b.z), point2d(c.x, c.z));
    answer -= a.y * cof.det();
    cof = Matrix2d(point2d(b.x, b.y), point2d(c.x, c.y));
    answer += a.z * cof.det();
    return answer;
  }

  Circle2d::Circle2d(const point2d& cen, double sqRad)
  {
    if (sqRad < -threshold())
    {
      throw std::invalid_argument("Three points on circle are collinear.");
    }
    if (sqRad <= threshold()) { sqRad = 0.0; }
    center = cen;  sqRadius = sqRad;
  }

  Circle2d::Circle2d(const point2d& a, const point2d& b, const point2d& c)
  {
    Matrix3d AA(point3d(a.x, a.y, 1), point3d(b.x, b.y, 1), point3d(c.x, c.y, 1));
    double den = AA.det();
    double absDen = (den > 0) ? den : -den;
    bool bCollinear = (absDen <= threshold());
    if (bCollinear)
    {
      bool bZeroRadius = true;
      if (point2d::sq_distance(a, b) > threshold()) { bZeroRadius = false; }
      if (point2d::sq_distance(b, c) > threshold()) { bZeroRadius = false; }
      if (point2d::sq_distance(a, c) > threshold()) { bZeroRadius = false; }
      if (bZeroRadius)
      {
        center = a; sqRadius = 0;
      }
      else
      {
        throw std::invalid_argument("Three points on circle are collinear.");
      }
    }
    else
    {
      double a2 = point2d::sq_distance(a, point2d(0, 0));
      double b2 = point2d::sq_distance(b, point2d(0, 0));
      double c2 = point2d::sq_distance(c, point2d(0, 0));
      Matrix3d CX(point3d(a2, a.y, 1), point3d(b2, b.y, 1), point3d(c2, c.y, 1));
      Matrix3d CY(point3d(a.x, a2, 1), point3d(b.x, b2, 1), point3d(c.x, c2, 1));
      center = point2d(CX.det() / (den * 2.0), CY.det() / (den * 2.0));
      Matrix3d BB(point3d(a.x, a.y, a2), point3d(b.x, b.y, b2), point3d(c.x, c.y, c2));
      sqRadius = (BB.det() / den) + point2d::sq_distance(center, point2d(0, 0));
    }
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on edge */
  int Circle2d::pointIsInterior(const point2d& pt) const
  {
    double dd = point2d::sq_distance(center, pt);
    double diff = dd - sqRadius;
    double absDiff = (diff > 0) ? diff : -diff;
    if (absDiff <= threshold()) { return 2; }
    return (diff > 0) ? 0 : 1;
  }

  Triangle2d::Triangle2d(const point2d& aa, const point2d& bb, const point2d& cc)
  {
    a = aa; b = bb; c = cc;
  }

  static bool adjacentToByEdgeHelper(const Triangle2d& lhs, const Triangle2d& rhs, Edge2d& edge)
  {
    edge = Edge2d(lhs.a, lhs.b);
    edge.a = lhs.a;
    if (point2d::sq_distance(lhs.a, rhs.a) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sq_distance(lhs.b, rhs.b) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.b, rhs.c) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sq_distance(lhs.c, rhs.b) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.c, rhs.c) < threshold()) { return true; }
      return false;
    }
    if (point2d::sq_distance(lhs.a, rhs.b) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sq_distance(lhs.b, rhs.c) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.b, rhs.a) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sq_distance(lhs.c, rhs.a) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.c, rhs.c) < threshold()) { return true; }
      return false;
    }
    if (point2d::sq_distance(lhs.a, rhs.c) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sq_distance(lhs.b, rhs.a) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.b, rhs.b) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sq_distance(lhs.c, rhs.b) < threshold()) { return true; }
      if (point2d::sq_distance(lhs.c, rhs.a) < threshold()) { return true; }
      return false;
    }
    return false;
  }

  bool Triangle2d::adjacentToByEdge(const Triangle2d& rhs, Edge2d& edge) const
  {
    Edge2d edge0;
    if (adjacentToByEdgeHelper(*this, rhs, edge0)) { edge = edge0; return true; }
    Triangle2d tr(b, c, a);
    if (adjacentToByEdgeHelper(tr, rhs, edge0)) { edge = edge0; return true; }
    Triangle2d tr2(c, a, b);
    if (adjacentToByEdgeHelper(tr2, rhs, edge0)) { edge = edge0; return true; }
    return false;
  }

  double Triangle2d::sqArea() const
  {
    Edge2d u(a, b);
    Edge2d v(b, c);
    Edge2d w(c, a);
    double u2 = u.sqLength();
    double v2 = v.sqLength();
    double w2 = w.sqLength();
    double sum = u2 + v2 + w2;
    return (4.0 * (u2 * v2 + u2 * w2 + v2 * w2) - sum * sum) / 16.0;
  }

  static double safeSqrt(const double& xx)
  {
    if (xx <= threshold()) { return 0; }
    return sqrt(xx);
  }

  bool Triangle2d::operator< (const Triangle2d& rhs) const
  {
    if (rhs.a < a) {return false;}
    if (a < rhs.a) {return true;}
    if (rhs.b < b) {return false;}
    if (b < rhs.b) {return true;}
    if (rhs.c < c) {return false;}
    if (c < rhs.c) {return true;}
    return false;
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on edge, 3 = on vertex */
  int Triangle2d::pointIsInterior(const point2d& pt) const
  {
    if (point2d::sq_distance(a, pt) <= threshold()) { return 3; }
    if (point2d::sq_distance(b, pt) <= threshold()) { return 3; }
    if (point2d::sq_distance(c, pt) <= threshold()) { return 3; }
    Edge2d u(a, b);
    Edge2d v(b, c);
    Edge2d w(c, a);
    if (u.sq_distance(pt) <= threshold()) { return 2; }
    if (v.sq_distance(pt) <= threshold()) { return 2; }
    if (w.sq_distance(pt) <= threshold()) { return 2; }
    if (sqArea() <= threshold()) { return 0; }
    Triangle2d U(a, b, pt);
    Triangle2d V(b, c, pt);
    Triangle2d W(c, a, pt);
    // Can we find a way to do this without taking square roots?
    const double uArea = safeSqrt(U.sqArea());
    const double vArea = safeSqrt(V.sqArea());
    const double wArea = safeSqrt(W.sqArea());
    const double AA = safeSqrt(sqArea());
    if (uArea + vArea > AA) { return 0; }
    if (vArea + wArea > AA) { return 0; }
    if (wArea + uArea > AA) { return 0; }
    return 1;
  }

  std::set<Edge2d> Triangle2d::getEdges() const
  {
    std::set<Edge2d> edges;
    edges.insert(Edge2d(a, b));
    edges.insert(Edge2d(b, c));
    edges.insert(Edge2d(c, a));
    return edges;
  }

  class PointCloud::Impl
  {
  public:
    PointCloud* pCloud = nullptr;

    bool bConvexHullOn = true;
    bool bDelaunayOn = false;
    bool bNearestNeighborOn = false;
    bool bPointsVisible = true;
    bool bTrianglesModeOn = false;
    bool bVoronoiOn = false;

    std::vector<point2d> hull;
    std::vector<point2d> pointArray;
    std::vector<Triangle2d> triangulation;
    std::vector<Triangle2d> delaunay;
    std::vector<Edge2d> nearestNeighbor;
    std::vector<Edge2d> voronoi;

    Impl(PointCloud* pParent) : pCloud(pParent) {}
    /**
     * This is O(n^2 log(n)) where n is the number of vertices.
     * For every vertex, iterate over the number of faces a constant number of times.
     * The number of faces in any triangulation is 2n - size(hull) - 2 which is O(n).
     * So iterating over the faces is O(n log(n))
     * This gives a total of O(n^2 log(n)).
     */
    void naiveTriangulate();
    void generateRandomPoints();
    void computeDelaunay();
    void computeNearestNeighbor();
    void computeVoronoi();
      
    /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
    void computeConvexHull();
    template <class Container> static double naiveMinSqDistance(Container& A, Container& B, point3d& A_min, point3d& B_min);
    template <class Container> static double naiveMinSqDistance(Container& arr, point3d& min_1, point3d& min_2);
    template <class Container> static double minSqDistanceHelper(Container& arr, point2d& min_1, point2d& min_2);
    template <class Container> static double minSqDistance(Container& arr, point2d& min_1, point2d& min_2);
  };

  PointCloud::PointCloud() : pImpl(std::make_unique<PointCloud::Impl>(this))
  {
    // unique_ptr requires C++ 11.
    // make_unique requires C++ 14.
  }

  const std::vector<point2d>& PointCloud::PointArray() const
  {
    if (pImpl == nullptr) // Of course this should never happen.
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->pointArray;
  }

  const std::vector<point2d>& PointCloud::ConvexHull() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->hull;
  }

  const std::vector<Edge2d>& PointCloud::NearestNeighbor() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Edge2d> dummy;
      return dummy;
    }
    return pImpl->nearestNeighbor;
  }

  const std::vector<Triangle2d>& PointCloud::Triangulation() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Triangle2d> dummy;
      return dummy;
    }
    return pImpl->triangulation;
  }

  const std::vector<Triangle2d>& PointCloud::Delaunay() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Triangle2d> dummy;
      return dummy;
    }
    return pImpl->delaunay;
  }

  const std::vector<Edge2d>& PointCloud::Voronoi() const
  {
    if (pImpl == nullptr)
    {
      static std::vector<Edge2d> dummy;
      return dummy;
    }
    return pImpl->voronoi;
  }

  bool PointCloud::getBoundingBox(point3d& min, point3d& max) const
  {
    if (pImpl == nullptr) { return false; }
    if (pImpl->pointArray.empty()) { return false; }
    min = pImpl->pointArray[0];
    max = pImpl->pointArray[0];
    int startIndex = 1;
    for (int i = startIndex, NN = (int)(pImpl->pointArray.size()); i < NN; ++i)
    {
      const point3d& current = pImpl->pointArray[i];
      if (current.x < min.x) { min.x = current.x; }
      if (current.y < min.y) { min.y = current.y; }
      if (current.z < min.z) { min.z = current.z; }
      if (current.x > max.x) { max.x = current.x; }
      if (current.y > max.y) { max.y = current.y; }
      if (current.z > max.z) { max.z = current.z; }
    }
    return true;
  }

  void PointCloud::refresh(bool bRecompute)
  {
    if (pImpl == nullptr) { return; }
    // Generate random points for display.
    if (bRecompute)
    {
      pImpl->generateRandomPoints();
      pImpl->computeConvexHull();
      pImpl->triangulation.resize(0);
      pImpl->delaunay.resize(0);
      pImpl->nearestNeighbor.resize(0);
      pImpl->voronoi.resize(0);
    }
    if (pImpl->bTrianglesModeOn)
    {
      if (pImpl->triangulation.empty())
      {
        pImpl->naiveTriangulate();
      }
    }
    if (pImpl->bDelaunayOn)
    {
      if (pImpl->delaunay.empty())
      {
        pImpl->computeDelaunay();
      }
    }
    if (pImpl->bNearestNeighborOn)
    {
      if (pImpl->nearestNeighbor.empty())
      {
        pImpl->computeNearestNeighbor();
      }
    }
    if (pImpl->bVoronoiOn)
    {
      if (pImpl->voronoi.empty())
      {
        pImpl->computeVoronoi();
      }
    }
  }

  PointCloud& PointCloud::Get()
  {
    static PointCloud pc;
    return pc;
  }

  void PointCloud::toggleConvexHull()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bConvexHullOn = !(pImpl->bConvexHullOn);
  }

  void PointCloud::toggleDelaunay()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bDelaunayOn = !(pImpl->bDelaunayOn);
  }

  void PointCloud::toggleNearestNeighbor()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bNearestNeighborOn = !(pImpl->bNearestNeighborOn);
  }

  void PointCloud::togglePointsVisibility()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bPointsVisible = !(pImpl->bPointsVisible);
  }

  void PointCloud::toggleTriangulation()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bTrianglesModeOn = !(pImpl->bTrianglesModeOn);
  }

  void PointCloud::toggleVoronoi()
  {
    if (pImpl == nullptr) { return; }
    pImpl->bVoronoiOn = !(pImpl->bVoronoiOn);
  }

  bool PointCloud::convexHullIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bConvexHullOn;
  }

  bool PointCloud::delaunayIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bDelaunayOn;
  }

  bool PointCloud::nearestNeighborIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bNearestNeighborOn;
  }

  bool PointCloud::pointsAreOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bPointsVisible;
  }

  bool PointCloud::triangulationIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bTrianglesModeOn;
  }

  bool PointCloud::voronoiIsOn() const
  {
    if (pImpl == nullptr) { return false; }
    return pImpl->bVoronoiOn;
  }

  void PointCloud::computeDelaunay()
  {
    if (pImpl != nullptr) { pImpl->computeDelaunay(); }
  }

  void PointCloud::naiveTriangulate()
  {
    if (pImpl != nullptr) { pImpl->naiveTriangulate(); }
  }

  void PointCloud::Impl::computeDelaunay() // Naive Delaunay
  {
    std::set<Triangle2d> faces;
    int numFaces = 0;
    {
      if (triangulation.empty()) { naiveTriangulate(); }
      for (const auto& face : triangulation)
      {
        faces.insert(face);
        ++numFaces;
      }
    }
    bool bDelaunayNotMet = false;
    int flips = 0;
    for (bool bStarting = true; bStarting || bDelaunayNotMet; bStarting = false)
    {
      bDelaunayNotMet = false;
      Triangle2d flip0, flip1;
      // Pre- C++ 11 style of iteration:
      std::set<Triangle2d>::iterator it = faces.begin();
      std::set<Triangle2d>::iterator jt = faces.begin();
      for (; it != faces.end(); ++it)
      {
        jt = faces.begin();
        for (; jt != faces.end(); ++jt)
        {
          if (it == jt) { continue; }

          Edge2d match;
          bool bIsAdjacent = it->adjacentToByEdge(*jt, match);
          if (!bIsAdjacent) { continue; }

          point2d iVertex = it->a;
          point2d jVertex = jt->a;
          if (match.sq_distance(iVertex) <= threshold()) { iVertex = it->b; }
          if (match.sq_distance(jVertex) <= threshold()) { jVertex = jt->b; }
          if (match.sq_distance(iVertex) <= threshold()) { iVertex = it->c; }
          if (match.sq_distance(jVertex) <= threshold()) { jVertex = jt->c; }
          if (match.sq_distance(iVertex) <= threshold()) { continue; }
          if (match.sq_distance(jVertex) <= threshold()) { continue; }

          Circle2d testCircle(it->a, it->b, it->c);
          if (testCircle.pointIsInterior(jVertex) == 1)
          {
            bDelaunayNotMet = true;
            flip0 = Triangle2d(iVertex, jVertex, match.a);
            flip1 = Triangle2d(iVertex, jVertex, match.b);
            ++flips;
          }
          if (!bDelaunayNotMet)
          {
            testCircle = Circle2d(jt->a, jt->b, jt->c);
            if (testCircle.pointIsInterior(iVertex) == 1)
            {
              bDelaunayNotMet = true;
              flip0 = Triangle2d(iVertex, jVertex, match.a);
              flip1 = Triangle2d(iVertex, jVertex, match.b);
              ++flips;
            }
          }
          if (bDelaunayNotMet) { break; }
        }
        if (bDelaunayNotMet) { break; }
      }
      if (bDelaunayNotMet)
      {
        faces.erase(it);
        faces.erase(jt);
        faces.insert(flip0);
        faces.insert(flip1);
      }
    }
    for (const auto& face : faces)
    {
      delaunay.push_back(face);
    }
    //std::cout << "\nNumber of points: " << pointArray.size() << ".";
    //std::cout << "\nNumber of faces in initial triangulation: " << numFaces << ".";
    //std::cout << "\nNumber of Delaunay flips: " << flips << ".";
    //std::cout << "\nNumber of faces in Delaunay triangulation: " << delaunay.size() << ".";
  }

  void PointCloud::Impl::naiveTriangulate()
  {
    triangulation.resize(0);
    if (hull.empty())
    {
      computeConvexHull();
    }
    int hullSize = (int) hull.size();
    if (hullSize <= 2) { return; }
    int startIndex = 2;
    std::set<Triangle2d> faces;
    for (int i = startIndex; i < hullSize; ++i)
    {
      Triangle2d face(hull[0], hull[i - 1], hull[i]);
      faces.insert(face);
    }
    for (int i = 0, NN = pointArray.size(); i < NN; ++i)
    {
      // Pre- C++ 11 style of iteration:
      std::set<Triangle2d>::iterator it = faces.begin();
      for (; it != faces.end(); ++it)
      {
        if ((it->pointIsInterior(pointArray[i])) == 1) { break; }
      }
      if (it == faces.end()) { continue; }
      Triangle2d u(pointArray[i], it->a, it->b);
      Triangle2d v(pointArray[i], it->b, it->c);
      Triangle2d w(pointArray[i], it->c, it->a);
      faces.erase(it);
      faces.insert(u);
      faces.insert(v);
      faces.insert(w);
    }
    for (const auto& face : faces)
    {
      triangulation.push_back(face);
    }
  }

  void PointCloud::Impl::generateRandomPoints()
  {
    double randMax = (double)RAND_MAX;
    pointArray.resize(0);
    std::set<point2d> initialSet; // Using a set ensures unique points and allows automatic sorting.

    for (int i = 0; i < numRandomPoints(); ++i)
    {
      double xx = (rand() * 1.8 / randMax) - 0.9;
      double yy = (rand() * 1.8 / randMax) - 0.9;
      point2d P(xx, yy);
      initialSet.insert(P);
    }
  
    // Pre- C++ 11 style of iteration:
    for (std::set<point2d>::const_iterator it = initialSet.begin(); it != initialSet.end(); ++it)
    {
      pointArray.push_back(*it);
    }
  }

  void PointCloud::computeNearestNeighbor()
  {
    if (pImpl != nullptr) { pImpl->computeNearestNeighbor(); }
  }

  void PointCloud::Impl::computeNearestNeighbor()
  {
    nearestNeighbor.resize(0);
    if (delaunay.empty())
    {
      computeDelaunay();
    }
      
    std::set<Triangle2d> faces;
    for (const auto& face : delaunay)
    {
      faces.insert(face);
    }

    for (const auto& current : pointArray)
    {
      std::set<Edge2d> edgesForThisOne;
      for (const auto& face : faces)
      {
        if (point2d::sq_distance(face.a, current) <= threshold())
        {
          Edge2d edge(face.a, face.b);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.a, face.c);
          edgesForThisOne.insert(edge);
        }
        if (point2d::sq_distance(face.b, current) <= threshold())
        {
          Edge2d edge(face.a, face.b);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.b, face.c);
          edgesForThisOne.insert(edge);
        }
        if (point2d::sq_distance(face.c, current) <= threshold())
        {
          Edge2d edge(face.b, face.c);
          edgesForThisOne.insert(edge);
          edge = Edge2d(face.c, face.a);
          edgesForThisOne.insert(edge);
        }
      }
      if (edgesForThisOne.empty()) { continue; }
      Edge2d nearest = *(edgesForThisOne.begin());
      for (const auto& edge : edgesForThisOne)
      {
        if (edge.sqLength() < nearest.sqLength()) { nearest = edge; }
      }
      nearestNeighbor.push_back(nearest);
    }
  }

  void PointCloud::computeVoronoi()
  {
    if (pImpl != nullptr) { pImpl->computeVoronoi(); }
  }

  void PointCloud::Impl::computeVoronoi()
  {
    voronoi.resize(0);
    if (delaunay.empty())
    {
      computeDelaunay();
    }

    double raySqLength = -1.0;
    {
      int ww = 0; int hh = 0;
      GetWindowWidthHeight(ww, hh);
      raySqLength = (double)(ww * ww + hh * hh);
    }

    std::map<int, point2d> sites;
    std::map<int, Triangle2d> faces;
    {
      int i = 0;
      for (const auto& face : delaunay)
      {
        point2d site;
        bool bCollinear = false;
        try
        {
          Circle2d circ(face.a, face.b, face.c);
          site = circ.center;
        }
        catch (...)
        {
          bCollinear = true;
        }
        if (bCollinear) { continue; }
        faces[i] = face;
        sites[i] = site;
        ++i;
      }
    }

    for (const auto& siteIt : sites)
    {
      std::set<point2d> sitesForThisOne;
      const auto& itsFace = faces[siteIt.first];
      const std::set<Edge2d> edges = itsFace.getEdges();
      std::set<Edge2d> nonMatching = edges;
      for (const auto& faceIt : faces)
      {
        if (faceIt.first == siteIt.first) { continue; }
        Edge2d match;
        bool bIsAdjacent = itsFace.adjacentToByEdge(faceIt.second, match);
        if (!bIsAdjacent) { continue; }
        sitesForThisOne.insert(sites.at(faceIt.first));
        for (const auto& edge : edges)
        {
          if (match.sq_distance(edge.a) > threshold()) { continue; }
          if (match.sq_distance(edge.b) > threshold()) { continue; }
          nonMatching.erase(edge);
        }
        if (sitesForThisOne.size() >= 3) { break; }
      }

      for (const auto& endpoint : sitesForThisOne)
      {
        Edge2d edge(siteIt.second, endpoint);
        voronoi.push_back(edge);
      }
      if (raySqLength < 0.0) { continue; }
      for (const auto& nonMatch : nonMatching)
      {
        point2d proj = nonMatch.projection(siteIt.second);
        //Edge2d ray(siteIt.second,
        //  point2d(raySqLength * (proj.x - siteIt.second.x) + siteIt.second.x,
        //  raySqLength * (proj.y - siteIt.second.y) + siteIt.second.y));
        //voronoi.push_back(ray);
      }
    }
  }

  void PointCloud::computeConvexHull()
  {
    if (pImpl != nullptr) { pImpl->computeConvexHull(); }
  }

  void PointCloud::Impl::computeConvexHull()
  {
    hull.resize(0);

    // 2d: Graham scan.
    hull = pointArray;
    const int NN = (int)pointArray.size();
    if (NN <= 3) {return;}

    point2d tempOrigin = hull[0];
    {
      int bestIndex = 0;
      const int startIndex = 1;
      for (int i = startIndex; i < NN; ++i)
      {
        if (hull[i].y < tempOrigin.y)
        {
          tempOrigin = hull[i];
          bestIndex = i;
        }
      }

      hull[bestIndex] = hull[1];
      hull[1] = tempOrigin;
      hull.push_back(hull[0]);
    
      for (int i = startIndex; i <= NN; ++i)
      {
        hull[i].x = hull[i].x - tempOrigin.x;
        hull[i].y = hull[i].y - tempOrigin.y;
      }
    
      // O(n log(n)):
      std::sort(hull.begin() + 1, hull.end(), point2d::comparator);
    
      for (int i = startIndex; i <= NN; ++i)
      {
        hull[i].x = hull[i].x + tempOrigin.x;
        hull[i].y = hull[i].y + tempOrigin.y;
      }
        
      hull[0] = hull[NN];
    }
    
    int hullCount = 1; // Initialize stack.
    const int startIndex = 2;
    for (int i = startIndex; i <= NN; ++i)
    {
      while (point2d::getOrientation(hull[hullCount], hull[i], hull[hullCount - 1]) <= 0)
      {
        if (hullCount > 1)
        {
          --hullCount;  continue;
        }
        if (i == NN) {break;} // Stack is empty.
        ++i; // Else keep searching.
      }
      // Otherwise point is on the boundary of the convex hull.
      ++hullCount;
      tempOrigin = hull[hullCount];
      hull[hullCount] = hull[i];
      hull[i] = tempOrigin;
    }

    hull.resize(hullCount);
  }

  template <class Container> double PointCloud::Impl::naiveMinSqDistance(Container& A, Container& B, point3d& A_min, point3d& B_min)
  {
    double min = 0;  bool started = false;
    for (typename Container::iterator it1 = A.begin(); it1 != A.end(); ++it1)
    {
      for (typename Container::iterator it2 = B.begin(); it2 != B.end(); ++it2)
      {
          // Note: set iteration takes place in sorted order.
          //std::cout << "[";  it2->print();  std::cout << "]\n";
          if (!started)
          {
              min = point3d::sq_distance(*it1, *it2);
              A_min = *it1;
              B_min = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sq_distance(*it1, *it2);
          if (candidate >= min) {continue;}
      
          min = candidate;
          A_min = *it1;
          B_min = *it2;
          if (min == 0) {break;}
      }
      if (min == 0) {break;}
    }
    return min;
  }

  template <class Container> double PointCloud::Impl::naiveMinSqDistance(Container& arr, point3d& min_1, point3d& min_2)
  {
    double min = 0;  bool started = false;
    if (arr.begin() != arr.end())
    {
      min_1 = *(arr.begin());
      min_2 = *(arr.begin());
    }

    // Iteration involving template parameters:
    for (typename Container::iterator it1 = arr.begin(); it1 != arr.end(); ++it1)
    {
      for (typename Container::iterator it2 = arr.begin(); it2 != arr.end(); ++it2)
      {
          if (it1 == it2) {continue;}
          // Note: set iteration takes place in sorted order.
          //std::cout << "[";  it2->print();  std::cout << "]\n";
          if (!started)
          {
              min = point3d::sq_distance(*it1, *it2);
              min_1 = *it1;
              min_2 = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sq_distance(*it1, *it2);
          if (candidate >= min) {continue;}
      
          min = candidate;
          min_1 = *it1;
          min_2 = *it2;
          if (min == 0) {break;}
      }
      if (min == 0) {break;}
    }
    return min;
  }

  double PointCloud::naiveMinSqDistance(std::set<point3d>& A, std::set<point3d>& B, point3d& A_min, point3d& B_min)
  {
    return Impl::naiveMinSqDistance(A, B, A_min, B_min);
  }

  double PointCloud::naiveMinSqDistance(std::set<point3d>& arr, point3d& min_1, point3d& min_2)
  {
    return Impl::naiveMinSqDistance(arr, min_1, min_2);
  }

  double PointCloud::naiveMinSqDistance(std::set<point2d>& A, std::set<point2d>& B, point2d& A_min, point2d& B_min)
  {
    return Impl::naiveMinSqDistance(A, B, A_min, B_min);
  }

  double PointCloud::naiveMinSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return Impl::naiveMinSqDistance(cloud, min_1, min_2);
  }

  double PointCloud::naiveMinSqDistance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return Impl::naiveMinSqDistance(pImpl->pointArray, min_1, min_2);
  }

  template <class Container> double PointCloud::Impl::minSqDistanceHelper(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();

    // These cases (where arrCount is 0 or 1) should never happen in the private helper method.
    //if (arrCount == 0) {return 0;}
    //if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sq_distance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sq_distance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sq_distance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sq_distance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }
    
    unsigned halfArrCount = arrCount / 2;
    unsigned remainingArrCount = arrCount - halfArrCount;
    
    Container arr_1, arr_2;
    point2d left_1, left_2, right_1, right_2;
    double min_L, min_R;
    
    {
      typename Container::iterator it = arr.begin();
      for (int i = 0; i < halfArrCount; i++)
      {
        arr_1.push_back(*it);  ++it;
      }
    
      for (int i = 0; i < remainingArrCount; i++)
      {
        arr_2.push_back(*it);  ++it;
      }
    }
    
    min_L = minSqDistanceHelper(arr_1, left_1,  left_2);
    min_R = minSqDistanceHelper(arr_2, right_1, right_2);
    
    if (min_L < min_R)
    {
      min = min_L;
      min_1 = left_1;
      min_2 = left_2;
    }
    else
    {
      min = min_R;
      min_1 = right_1;
      min_2 = right_2;
    }
    return min;
  }

  template <class Container> double PointCloud::Impl::minSqDistance(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();
    if (arrCount == 0) {return 0;}
    if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sq_distance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sq_distance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sq_distance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sq_distance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }

    std::vector<point2d > arr_;
    for (typename Container::iterator it = arr.begin(); it != arr.end(); ++it)
    {
      arr_.push_back(*it);
    }
  
    std::sort(arr_.begin(), arr_.end());
    min = minSqDistanceHelper(arr_, min_1, min_2);
    return min;
  }

  double PointCloud::minSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return Impl::minSqDistance(cloud, min_1, min_2);
  }

  double PointCloud::minSqDistance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return Impl::minSqDistance(pImpl->pointArray, min_1, min_2);
  }

  void PointCloud::unitTest()
  {
    {
      point3d P(1.0, 2.0, 3.0);
      std::cout << "\n//////\nThe dimension is " << P.GetDimension() << ".";
      P.print("\n");
      std::cout << "\n";

      point2d Q(1.0, 2.0);
      std::cout << "\n//////\nThe dimension is " << Q.GetDimension() << ".";
      Q.print("\n");
      std::cout << "\n";
    }
  
    {
      point3d P(1.0, 2.0, 3.0);
      point3d Q(1.0, -2.0, 3.0);
  
      printf("%f\n", point3d::sq_distance(P,Q));
    }
  
    {
      point2d P(1.5, 2.0);
      point2d Q(1.0, -2.0);
  
      printf("%f\n", point2d::sq_distance(P,Q));
    }
  
    {
      std::set<point3d> A;
      point3d a(1.5, 2.0, 0);
      point3d b(1.0, 3.0, 0);
      point3d c(-1.5, 7.0, 0);
    
      std::set<point3d> B;
      point3d d(4.5, 2.3, 0);
      point3d e(-1.55, 2.6, 0);
      point3d f(88.3, 0.001, 0);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point3d p, q;
    
      double min = naiveMinSqDistance(A, B, p, q);
    
      std::cout << "\n//////\n" << min;
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d> A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
    
      std::set<point2d> B;
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point2d p, q;
    
      double min = naiveMinSqDistance(A, B, p, q);
    
      std::cout << "\n//////\n" << min << "\n";
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d > A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
      A.insert(d);
      A.insert(e);
      A.insert(f);
    
      point2d p, q;

      double min = naiveMinSqDistance(A, p, q);
    
      std::cout << "\n//////\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    
      min = minSqDistance(A, p, q);
    
      std::cout << "\n/!!!!/\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    }
  } // end of unit_test function.

} // end of namespace ComputationalGeometry
