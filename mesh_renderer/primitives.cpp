
#include "includes.h"
#include "primitives.h"

#ifdef POINT_CLOUD_PROJECT
#include "point_cloud.h"
#endif // def POINT_CLOUD_PROJECT

#ifdef _WIN32
#include <algorithm>
#endif
#include <map>

#define USE_MULTI_THREADING

#ifdef USE_MULTI_THREADING
#include <mutex>
#include <thread>
#endif // USE_MULTI_THREADING

namespace ComputationalGeometry
{

#ifndef POINT_CLOUD_PROJECT
  double threshold() { return 1.0e-9; }
#endif // ndef POINT_CLOUD_PROJECT

  template <class T> T safeAbs(const T& arg)
  {
    if (arg < 0) { return -arg; }
    return arg;
  }

  point3d::point3d() : x(0), y(0), z(0) {}
  point3d::point3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}
  point3d::point3d(const point2d& P) : x(P.x), y(P.y), z(0) {}

  point3d point3d::operator+(const vector3d& rhs) const
  {
    return point3d(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  vector3d point3d::operator-(const point3d& rhs) const
  {
    return vector3d(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  bool point3d::operator< (const point3d& q) const
  {
    if (x > q.x) {return false;}
    if (x < q.x) {return true;}
    if (y > q.y) {return false;}
    if (y < q.y) {return true;}
    if (z > q.z) {return false;}
    if (z < q.z) {return true;}
    return false;
  }

  void point3d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y << ", " << z << ")";
  }

  double point3d::sqDistance(const point3d& P, const point3d& Q)
  {
    return P.sqDistance(Q);
  }

  double point3d::sqDistance(const point3d& Q) const
  {
    double answer = 0;
    auto P = *this;
    double dt = (P.x - Q.x);
    answer = answer + dt * dt;
    dt = (P.y - Q.y);
    answer = answer + dt * dt;
    {
      dt = (P.z - Q.z);
      answer = answer + dt * dt;
    }
    return answer;
  }

  vector3d::vector3d() : x(0), y(0), z(0) {}
  vector3d::vector3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}

  bool vector3d::operator< (const vector3d& q) const
  {
    point3d pp(x, y, z); point3d qq(q.x, q.y, q.z);
    return (pp < qq);
  }

  void vector3d::print(const std::string& prequel) const
  {
    point3d(x, y, z).print(prequel);
  }

  double vector3d::dot(const vector3d& P) const
  {
    return x * P.x + y * P.y + z * P.z;
  }

  vector3d vector3d::cross(const vector3d& P) const
  {
    auto crossX = y * P.z - z * P.y;
    auto crossY = z * P.x - x * P.z;
    auto crossZ = x * P.y - y * P.x;
    return vector3d(crossX, crossY, crossZ);
  }

  double vector3d::sqNorm() const { return (*this).dot(*this); }

  vector3d vector3d::operator+(const vector3d& rhs) const
  {
    return vector3d(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  vector3d vector3d::operator-(const vector3d& rhs) const
  {
    return vector3d(x - rhs.x, y - rhs.y, z - rhs.z);
  }

  vector3d vector3d::operator*(const double& rhs) const
  {
    return vector3d(x * rhs, y * rhs, z * rhs);
  }

  vector3d& vector3d::operator*=(const double& scal) { x *= scal; y *= scal; z *= scal; return *this; }

  point2d::point2d() : x(0), y(0) {}
  point2d::point2d(const double& xx, const double& yy) : x(xx), y(yy) {}

  vector2d point2d::operator-(const point2d& rhs) const
  {
    return vector2d(x - rhs.x, y - rhs.y);
  }

  bool point2d::operator< (const point2d& q) const
  {
    if (x > q.x) {return false;}
    if (x < q.x) {return true;}
    if (y > q.y) {return false;}
    if (y < q.y) {return true;}
    return false;
  }

  double point2d::getOrientation(const point2d& P, const point2d& Q, const point2d& O)
  {
    return P.orientation(Q, O);
  }
  double point2d::orientation(const point2d& Q, const point2d& O) const
  {
    point2d P = *this;
    return (P.x - O.x) * (Q.y - O.y) - (P.y - O.y) * (Q.x - O.x);
  }
  bool point2d::comparator(const point2d& P, const point2d& Q) { return P.compare(Q); }

  bool point2d::compare(const point2d& Q) const
  {
    // Equivalent to *this < Q
    double theta_P = atan2(y, x);
    double theta_Q = atan2(Q.y, Q.x);
    return theta_P < theta_Q; // Also can use return getOrientation(*this, Q) < 0;
  }

  void point2d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y << ")";
  }

  double point2d::sqDistance(const point2d& P, const point2d& Q)
  {
    return P.sqDistance(Q);
  }

  double point2d::sqDistance(const point2d& Q) const
  {
    double answer = 0;
    auto P = *this;
    double dt = (P.x - Q.x);
    answer = answer + dt * dt;
    dt = (P.y - Q.y);
    answer = answer + dt * dt;
    return answer;
  }

  vector2d::vector2d() : x(0), y(0) {}
  vector2d::vector2d(const double& xx, const double& yy) : x(xx), y(yy) {}

  bool vector2d::operator< (const vector2d& q) const
  {
    point2d pp(x, y); point2d qq(q.x, q.y);
    return (pp < qq);
  }

  void vector2d::print(const std::string& prequel) const
  {
    return point2d(x, y).print(prequel);
  }

  double vector2d::dot(const vector2d& P) const
  {
    return x * P.x + y * P.y;
  }

  double vector2d::sqNorm() const { return (*this).dot(*this); }
  vector2d& vector2d::operator*=(const double& scal) { x *= scal; y *= scal; return *this; }

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
    return a.sqDistance(b);
  }

  double Edge2d::sqDistance(const point2d& P) const
  {
    double aSqDistP = a.point2d::sqDistance(P);
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
    double bSqDistP = b.sqDistance(P);
    if ((aSqDistP < sqLength()) && (bSqDistP < sqLength())) { return 0.0; }
    if (aSqDistP >= sqLength()) { return bSqDistP; }
    return aSqDistP;
  }

  /** \brief 0 = no intersection, 1 = point intersection, 2 = parallel intersection */
  int Edge2d::intersection(const Edge2d& other, point2d& intersection) const
  {
    if (point2d::sqDistance(a, b) <= threshold())
    {
      if (point2d::sqDistance(a, other.a) <= threshold())
      {
        intersection = a; return 2;
      }
      if (point2d::sqDistance(a, other.b) <= threshold())
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
    if ((point2d::sqDistance(a, other.a) <= threshold()) || (point2d::sqDistance(a, other.b) <= threshold()))
    {
      intersection = a;
      return bDetNonzero ? 1 : 2;
    }
    if ((point2d::sqDistance(b, other.a) <= threshold()) || (point2d::sqDistance(b, other.b) <= threshold()))
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
    if (sqDistance(other.a) < threshold())
    {
      intersection = other.a;
      return 2;
    }
    if (sqDistance(other.b) < threshold())
    {
      intersection = other.b;
      return 2;
    }
    if (other.sqDistance(a) < threshold())
    {
      intersection = a;
      return 2;
    }
    // else if (other.sqDistance(b) < threshold())
    {
      intersection = b;
      return 2;
    }
  }

  point2d Edge2d::projection(const point2d& P) const
  {
    if (sqLength() <= threshold()) { return a; }
    vector2d pp = P - a;
    vector2d qq = b - a;
    double coeff = pp.dot(qq) / sqLength();
    qq *= coeff;
    return point2d(qq.x + a.x, qq.y + a.y);
  }

  Edge3d::Edge3d(const point3d& aa, const point3d& bb)
  {
    a = aa; b = bb;
  }

  bool Edge3d::operator<(const Edge3d& rhs) const
  {
    if (rhs.a < a) { return false; }
    if (a < rhs.a) { return true; }
    if (rhs.b < b) { return false; }
    if (b < rhs.b) { return true; }
    return false;
  }

  double Edge3d::sqLength() const
  {
    return a.sqDistance(b);
  }

  double Edge3d::sqDistance(const point3d& P) const
  {
    auto proj = projection(P);
    vector3d orthog(P.x - proj.x, P.y - proj.y, P.z - proj.z);
    return orthog.sqNorm();
  }

  point3d Edge3d::projection(const point3d& P) const
  {
    if (sqLength() <= threshold()) { return a; }
    vector3d pp(P.x - a.x, P.y - a.y, P.z - a.z);
    vector3d qq(b.x - a.x, b.y - a.y, b.z - a.z);
    double coeff = pp.dot(qq) / sqLength();
    qq *= coeff;
    return point3d(qq.x + a.x, qq.y + a.y, qq.z + a.z);
  }

  Matrix2d::Matrix2d(const vector2d& aa, const vector2d& bb)
  {
    a = aa; b = bb;
  }

  double Matrix2d::det() const
  {
    return a.x * b.y - a.y * b.x;
  }

  Matrix2d Matrix2d::inverse(bool& bSuccess) const
  {
    auto determinant = det();
    if (determinant <= threshold()) { bSuccess = false; return Matrix2d(); }
    bSuccess = true;
    Matrix2d inv(vector2d(b.y, -a.y), vector2d(-b.x, a.x));
    auto factor = 1.0 / determinant;
    inv.a *= factor;
    inv.b *= factor;
    return inv;
  }

  void Matrix2d::takeTranspose()
  {
    auto temp = b.x;
    b.x = a.y;
    a.y = temp;
  }

  vector2d Matrix2d::operator*(const vector2d& rhs) const { return vector2d(a.dot(rhs), b.dot(rhs)); }

  Matrix3d::Matrix3d(const vector3d& aa, const vector3d& bb, const vector3d& cc)
  {
    a = aa; b = bb; c = cc;
  }

  double Matrix3d::det() const
  {
    Matrix2d cof(vector2d(b.y, b.z), vector2d(c.y, c.z));
    double answer = a.x * cof.det();
    cof = Matrix2d(vector2d(b.x, b.z), vector2d(c.x, c.z));
    answer -= a.y * cof.det();
    cof = Matrix2d(vector2d(b.x, b.y), vector2d(c.x, c.y));
    answer += a.z * cof.det();
    return answer;
  }

  Matrix3d Matrix3d::inverse(bool& bSuccess) const
  {
    auto determinant = det();
    if (determinant <= threshold()) { bSuccess = false; return Matrix3d(); }
    bSuccess = true;
    auto aa = a.x;
    auto bb = a.y;
    auto cc = a.z;
    auto dd = b.x;
    auto ee = b.y;
    auto ff = b.z;
    auto gg = c.x;
    auto hh = c.y;
    auto ii = c.z;
    Matrix3d inv(vector3d(ee * ii - ff * hh, cc * hh - ii * bb, ff * bb - cc * ee),
      vector3d(gg * ff - ii * dd, aa * ii - cc * gg, cc * dd - aa * ff),
      vector3d(dd * hh - gg * ee, bb * gg - aa * hh, aa * ee - bb * dd));
    inv.a *= (1.0 / determinant);
    inv.b *= (1.0 / determinant);
    inv.c *= (1.0 / determinant);
    return inv;
  }

  void Matrix3d::takeTranspose()
  {
    auto aa = a;
    auto bb = b;
    auto cc = c;
    a = vector3d(aa.x, bb.x, cc.x);
    b = vector3d(aa.y, bb.y, cc.y);
    c = vector3d(aa.z, bb.z, cc.z);
  }

  vector3d Matrix3d::operator*(const vector3d& rhs) const { return vector3d(a.dot(rhs), b.dot(rhs), c.dot(rhs)); }

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
    Matrix3d AA(vector3d(a.x, a.y, 1), vector3d(b.x, b.y, 1), vector3d(c.x, c.y, 1));
    double den = AA.det();
    double absDen = (den > 0) ? den : -den;
    bool bCollinear = (absDen <= threshold());
    if (bCollinear)
    {
      bool bZeroRadius = true;
      if (point2d::sqDistance(a, b) > threshold()) { bZeroRadius = false; }
      if (point2d::sqDistance(b, c) > threshold()) { bZeroRadius = false; }
      if (point2d::sqDistance(a, c) > threshold()) { bZeroRadius = false; }
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
      double a2 = point2d::sqDistance(a, point2d(0, 0));
      double b2 = point2d::sqDistance(b, point2d(0, 0));
      double c2 = point2d::sqDistance(c, point2d(0, 0));
      Matrix3d CX(vector3d(a2, a.y, 1), vector3d(b2, b.y, 1), vector3d(c2, c.y, 1));
      Matrix3d CY(vector3d(a.x, a2, 1), vector3d(b.x, b2, 1), vector3d(c.x, c2, 1));
      center = point2d(CX.det() / (den * 2.0), CY.det() / (den * 2.0));
      Matrix3d BB(vector3d(a.x, a.y, a2), vector3d(b.x, b.y, b2), vector3d(c.x, c.y, c2));
      sqRadius = (BB.det() / den) + point2d::sqDistance(center, point2d(0, 0));
    }
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on edge */
  int Circle2d::pointIsInterior(const point2d& pt) const
  {
    double dd = point2d::sqDistance(center, pt);
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
    if (point2d::sqDistance(lhs.a, rhs.a) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sqDistance(lhs.b, rhs.b) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.b, rhs.c) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sqDistance(lhs.c, rhs.b) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.c, rhs.c) < threshold()) { return true; }
      return false;
    }
    if (point2d::sqDistance(lhs.a, rhs.b) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sqDistance(lhs.b, rhs.c) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.b, rhs.a) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sqDistance(lhs.c, rhs.a) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.c, rhs.c) < threshold()) { return true; }
      return false;
    }
    if (point2d::sqDistance(lhs.a, rhs.c) < threshold())
    {
      edge.b = lhs.b;
      if (point2d::sqDistance(lhs.b, rhs.a) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.b, rhs.b) < threshold()) { return true; }
      edge.b = lhs.c;
      if (point2d::sqDistance(lhs.c, rhs.b) < threshold()) { return true; }
      if (point2d::sqDistance(lhs.c, rhs.a) < threshold()) { return true; }
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
  __host__ __device__ int pointIsInteriorHelper(const Triangle2d& tri, const point2d& pt)
  {
    vector2d pp = tri.a - tri.b;
    vector2d qq = tri.c - tri.b;
    vector2d rr = pt - tri.b;
    Matrix2d AA(pp, qq);
    AA.takeTranspose();
    bool bInvertible = true;
    auto BB = AA.inverse(bInvertible);
    if (bInvertible)
    {
      vector2d testPoint = BB * rr;
      if ((testPoint.x < -threshold()) || (testPoint.y < -threshold())) { return 0; }
      if ((testPoint.x + testPoint.y) > 1 + threshold()) { return 0; }
      if ((testPoint.x >= threshold()) && (testPoint.y >= threshold())
              && ((testPoint.x + testPoint.y) <= 1 - threshold()))
      {
        return 1;
      }
      return 2;
    }
    Edge2d u(tri.a, tri.b);
    Edge2d v(tri.b, tri.c);
    Edge2d w(tri.c, tri.a);
    if (u.sqDistance(pt) <= threshold()) { return 2; }
    if (v.sqDistance(pt) <= threshold()) { return 2; }
    if (w.sqDistance(pt) <= threshold()) { return 2; }
    return 0;
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on edge, 3 = on vertex */
  int Triangle2d::pointIsInterior(const point2d& pt) const
  {
    if (pt.sqDistance(a) <= threshold()) { return 3; }
    if (pt.sqDistance(b) <= threshold()) { return 3; }
    if (pt.sqDistance(c) <= threshold()) { return 3; }
    double oneThird = 1.0 / 3.0;
    auto barycenter = point2d(oneThird * (a.x + b.x + c.x), oneThird * (a.y + b.y + c.y));
    if (pointIsInteriorHelper(*this, barycenter) == 0)
    {
      Triangle2d face0(b, a, c); // Orient properly.
      return pointIsInteriorHelper(face0, pt);
    }
    return pointIsInteriorHelper(*this, pt);
  }

  std::set<Edge2d> Triangle2d::getEdges() const
  {
    std::set<Edge2d> edges;
    edges.insert(Edge2d(a, b));
    edges.insert(Edge2d(b, c));
    edges.insert(Edge2d(c, a));
    return edges;
  }

  bool Face2d::isValid() const
  {
    return (vertices.size() >= 3);
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on edge, 3 = on vertex
   *  Uses criterion: Cast a ray. If number of intersections == 0 (mod 2) point is exterior.
   */
  int Face2d::pointIsInterior(const point2d& pt) const
  {
    const int numVertices = (int)vertices.size();
    for (int ii = 0; ii < numVertices; ++ii)
    {
      if (pt.sqDistance(vertices[ii]) <= threshold()) { return 3; }
    }
    std::set<Edge2d> edges = getEdges();
    for (const auto& edge : edges)
    {
      if (edge.sqDistance(pt) <= threshold()) { return 2; }
    }
    std::set<double> angles;
    // TODO: Can we do the same thing without using square root or inverse trig?
    // Calculate all angles subtended by lines from point to vertices.
    for (int ii = 0; ii < numVertices; ++ii)
    {
      vector2d diff = vertices[ii] - pt;
      angles.insert(atan2(diff.y, diff.x));
    }
    // Now get a unique angle:
    double fullAngle = 2.0 * 3.14159;
    double angle = fullAngle / ((double)(numVertices + 2));
    for (int ii = 0; ii < numVertices; ++ii)
    {
      if (angles.count(angle) == 0) { break; }
      angle = (2 + ii) / ((double)(numVertices + 2));
    }
    int numIntersections = 0;
    for (const auto& edge : edges)
    {
      double rr = edge.a.sqDistance(pt);
      if (rr < 2) { rr = 2; } // > distance.
      {
        double rr1 = edge.b.sqDistance(pt);
        if (rr1 < 2) { rr1 = 2; } // > distance.
        if (rr1 > rr) { rr = rr1; }
      }
      Edge2d intersector(pt, point2d(rr * cos(angle), rr * sin(angle)));
      point2d intersection;
      int intersectValue = intersector.intersection(edge, intersection);
      if (intersectValue != 0) { ++numIntersections; }
    }
    if ((numIntersections % 2) == 0) { return 0; }
    return 1;
  }

  std::set<Edge2d> Face2d::getEdges() const
  {
    std::set<Edge2d> edges;
    const int numVertices = (int)vertices.size();
    if (numVertices < 2) { return edges; }
    for (int ii = 0; ii < numVertices; ++ii)
    {
      int jj = ii + 1;
      if (ii == (numVertices - 1)) { jj = 0; }
      edges.insert(Edge2d(vertices[ii], vertices[jj]));
    }
    return edges;
  }

  Plane3d::Plane3d(const point3d& aa, const point3d& bb, const point3d& cc)
  {
    const auto& origin = aa;
    const auto v0 = bb - aa;
    const auto v1 = cc - aa;
    vector3d normal(v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x);
    A = normal.x;
    B = normal.y;
    C = normal.z;
    D = -normal.dot(vector3d(origin.x, origin.y, origin.z));
  }

  Plane3d Plane3d::fromPointAndNormal(const point3d& origin, const vector3d& normal)
  {
    Plane3d plan;
    plan.A = normal.x;
    plan.B = normal.y;
    plan.C = normal.z;
    plan.D = -normal.dot(vector3d(origin.x, origin.y, origin.z));
    return plan;
  }

  bool Plane3d::isInPlane(const point3d& ptIn) const
  {
    auto quantity = A * ptIn.x + B * ptIn.y + C * ptIn.z + D;
    return (safeAbs(quantity) <= threshold());
  }

  bool Plane3d::isValid() const
  {
    vector3d normal(A, B, C);
    return (normal.sqNorm() > threshold());
  }

  point3d Plane3d::pointInPlane() const
  {
    if ((D <= threshold()) && ((-D) <= threshold())) { return point3d(0, 0, 0); }
    if ((C > threshold()) || ((-C) > threshold())) { return point3d(0, 0, -D / C); }
    if ((B > threshold()) || ((-B) > threshold())) { return point3d(0, -D / B, 0); }
    if ((A > threshold()) || ((-A) > threshold())) { return point3d(-D / A, 0, 0); }
    return point3d(0, 0, 0);
  }

  /** \brief Which side of the plane is the point on? 2 for left, 1 for right, 0 for on plane. */
  int Plane3d::getSide(const point3d& pt) const
  {
    vector3d normal(A, B, C);
    auto origin = pointInPlane();
    auto discriminant = normal.dot(pt - origin);
    if (safeAbs(discriminant) <= threshold()) { return 0; }
    if (discriminant > threshold()) { return 1; }
    return 2;
  }

  void Plane3d::getOrthonormalBasis(vector3d& e1, vector3d& e2) const
  {
    auto nn = getNormal();
    if (safeAbs(nn.x) <= threshold())
    {
      e1 = vector3d(0, -nn.z, nn.y);
      e2 = nn.cross(e1);
      return;
    }
    if (safeAbs(nn.y) <= threshold())
    {
      e1 = vector3d(-nn.z, 0, nn.x);
      e2 = nn.cross(e1);
      return;
    }
    if (safeAbs(nn.z) <= threshold())
    {
      e1 = vector3d(-nn.y, nn.x, 0);
      e2 = nn.cross(e1);
      return;
    }
    vector3d f1(2 * nn.y * nn.z, -nn.x * nn.z, -nn.y * nn.x);
    auto mag = safeSqrt(f1.sqNorm());
    e1 = vector3d(f1.x / mag, f1.y / mag, f1.z / mag);
    e2 = nn.cross(e1);
  }

  vector3d Plane3d::getNormal() const
  {
    if (!isValid()) { return vector3d(0, 0, 0); }
    auto mag = vector3d(A, B, C).sqNorm();
    mag = safeSqrt(mag);
    return vector3d(A / mag, B / mag, C / mag);
  }

  point3d Plane3d::getRayCastResult(const Edge3d& ray, double& tVal, bool& parallel, bool& success) const
  {
    point2d xyOut;
    return getRayCastResult(ray, tVal, pointInPlane(), xyOut, parallel, success);
  }

  point3d Plane3d::getRayCastResult(const Edge3d& ray, double& tVal, const point3d& origin, point2d& xyOut, bool& parallel, bool& success) const
  {
    if (!isValid()) { success = false; return point3d(0, 0, 0); }
    if (!isInPlane(origin)) { success = false; return point3d(0, 0, 0); }
    success = true;
    const point3d& pp = ray.a;
    vector3d vv = ray.b - ray.a;
    const auto nn = getNormal();
    const auto vDotN = vv.dot(nn);
    parallel = (safeAbs(vDotN) <= threshold());
    if (parallel)
    {
      success = isInPlane(pp);
      if (success) { tVal = 0.0; }
      vector3d e1, e2;
      getOrthonormalBasis(e1, e2);
      xyOut.x = (pp - origin).dot(e1);
      xyOut.y = (pp - origin).dot(e2);
      return pp;
    }
    const auto& p0 = origin;
    const vector3d pMinusP0 = pp - p0;
    tVal = -pMinusP0.dot(nn) / vDotN;
    vector3d e1, e2;
    getOrthonormalBasis(e1, e2);
    double& ss = xyOut.x;
    double& rr = xyOut.y;
    ss = pMinusP0.dot(e1) + tVal * vv.dot(e1);
    rr = pMinusP0.dot(e2) + tVal * vv.dot(e2);
    point3d answer;
    answer.x = p0.x + ss * e1.x + rr * e2.x;
    answer.y = p0.y + ss * e1.y + rr * e2.y;
    answer.z = p0.z + ss * e1.z + rr * e2.z;
    return answer;
  }

  bool Face3d::isValid() const
  {
    if (vertices.size() < 3) { return false; }
    Plane3d span = getPlane();
    for (const point3d& vertex : vertices)
    {
      if (!(span.isInPlane(vertex))) { return false; }
    }
    return true;
  }

  int Face3d::pointIsInterior(const point3d& pt) const
  {
    Plane3d span = getPlane();
    point3d span0 = span.pointInPlane();
    bool success = false;  bool parallel = false;
    double tVal = 0.0;
    point2d xyOut;
    point3d answer = span.getRayCastResult(Edge3d(pt, pt), tVal, span0, xyOut, parallel, success);
    if (!parallel) { return 0; }
    if (!success) { return 0; }
    point2d tester = xyOut;
    Face2d planar;
    for (const point3d& vertex : vertices)
    {
      answer = span.getRayCastResult(Edge3d(vertex, vertex), tVal, span0, xyOut, parallel, success);
      planar.vertices.push_back(xyOut);
    }
    return planar.pointIsInterior(tester);
  }

  std::set<Edge3d> Face3d::getEdges() const
  {
    std::set<Edge3d> edges;
    const int numVertices = (int)vertices.size();
    if (numVertices < 2) { return edges; }
    for (int ii = 0; ii < numVertices; ++ii)
    {
      int jj = ii + 1;
      if (ii == (numVertices - 1)) { jj = 0; }
      edges.insert(Edge3d(vertices[ii], vertices[jj]));
    }
    return edges;
  }

  Plane3d Face3d::getPlane() const
  {
    if (vertices.size() == 0) { return Plane3d(); }
    if (vertices.size() == 1) { return Plane3d(vertices[0]); }
    if (vertices.size() == 2) { return Plane3d(vertices[0], vertices[1]); }
    return Plane3d(vertices[0], vertices[1], vertices[2]);
  }

  point3d Face3d::getRayCastResult(const Edge3d& ray, double& tVal, const point3d& origin, point2d& xyOut, bool& parallel, int& interior) const
  {
    Plane3d span = getPlane();
    // if (!span.isValid()) { interior = 0; return point3d(); }
    bool success = true;
    point3d answer = span.getRayCastResult(ray, tVal, origin, xyOut, parallel, success);
    if (!success) { interior = 0; return answer; }
    interior = pointIsInterior(answer);
    return answer;
  }

  point3d Face3d::getRayCastResult(const Edge3d& ray, double& tVal, bool& parallel, int& interior) const
  {
    if (vertices.size() == 0) { interior = 0; return point3d(); }
    point2d xyOut;
    return getRayCastResult(ray, tVal, vertices[0], xyOut, parallel, interior);
  }

  Triangle3d::Triangle3d(const point3d& aa, const point3d& bb, const point3d& cc)
  {
    a = aa; b = bb; c = cc;
  }

  bool Triangle3d::operator<(const Triangle3d& rhs) const
  {
    if (rhs.a < a) { return false; }
    if (a < rhs.a) { return true; }
    if (rhs.b < b) { return false; }
    if (b < rhs.b) { return true; }
    if (rhs.c < c) { return false; }
    if (c < rhs.c) { return true; }
    return false;
  }

  std::set<Edge3d> Triangle3d::getEdges() const
  {
    std::set<Edge3d> edges;
    edges.insert(Edge3d(a, b));
    edges.insert(Edge3d(b, c));
    edges.insert(Edge3d(c, a));
    return edges;
  }

  Tetrahedron3d::Tetrahedron3d(const point3d& aa, const point3d& bb, const point3d& cc, const point3d& dd)
  {
    a = aa; b = bb; c = cc; d = dd;
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on face, 3 = on edge, 4 = on vertex */
  __host__ __device__ int pointIsInteriorHelper(const Tetrahedron3d& tri, const point3d& pt)
  {
    auto pp = tri.a - tri.b;
    auto qq = tri.c - tri.b;
    auto rr = tri.d - tri.b;
    auto pointShifted = pt - tri.b;
    Matrix3d AA(pp, qq, rr);
    AA.takeTranspose();
    bool bInvertible = true;
    auto BB = AA.inverse(bInvertible);
    if (bInvertible)
    {
      vector3d testPoint = BB * pointShifted;
      if ((testPoint.x < -threshold()) || (testPoint.y < -threshold()) || (testPoint.z < -threshold())) { return 0; }
      if ((testPoint.x + testPoint.y + testPoint.z) > 1 + threshold()) { return 0; }
      if ((testPoint.x >= threshold()) && (testPoint.y >= threshold()) && (testPoint.z >= threshold())
        && ((testPoint.x + testPoint.y + testPoint.z) <= 1 - threshold()))
      {
        return 1;
      }
      return 2; // TODO: Refine this to distinguish between 2 or 3.
    }
    // TODO: Implement cases of returning 2 or 3.
    return 0;
  }

  /** \brief 0 = exterior, 1 = interior, 2 = on face, 3 = on edge, 4 = on vertex */
  int Tetrahedron3d::pointIsInterior(const point3d& pt) const
  {
    if (pt.point3d::sqDistance(a) <= threshold()) { return 4; }
    if (pt.point3d::sqDistance(b) <= threshold()) { return 4; }
    if (pt.point3d::sqDistance(c) <= threshold()) { return 4; }
    if (pt.point3d::sqDistance(d) <= threshold()) { return 4; }
    double oneFourth = 1.0 / 4.0;
    auto barycenter = point3d(oneFourth * (a.x + b.x + c.x + d.x),
      oneFourth * (a.y + b.y + c.y + d.y), oneFourth * (a.z + b.z + c.z + d.z));
    if (pointIsInteriorHelper(*this, barycenter) == 0)
    {
      Tetrahedron3d face0(b, a, c, d); // Orient properly.
      return pointIsInteriorHelper(face0, pt);
    }
    return pointIsInteriorHelper(*this, pt);
  }

} // end of namespace ComputationalGeometry
