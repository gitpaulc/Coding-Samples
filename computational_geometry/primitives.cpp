
#include "includes.h"
#include "primitives.h"

#include "point_cloud.h"

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
  point3d::point3d() : x(0), y(0), z(0) {}
  point3d::point3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}

#ifdef USE_VIRTUAL_FUNC_POINT2D
  int point3d::GetDimension() const { return 3; }
#endif // def USE_VIRTUAL_FUNC_POINT2D

  bool point3d::operator< (const point3d& q) const
  {
    if (x > q.x) {return false;}
    if (x < q.x) {return true;}
    if (y > q.y) {return false;}
    if (y < q.y) {return true;}
#ifdef USE_VIRTUAL_FUNC_POINT2D
    if (GetDimension() <= 2) { return false; }
#else
#endif // def USE_VIRTUAL_FUNC_POINT2D
    if (z > q.z) {return false;}
    if (z < q.z) {return true;}
    return false;
  }

  void point3d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y;
#ifdef USE_VIRTUAL_FUNC_POINT2D
    if (GetDimension() > 2) { std::cout << ", " << z; }
#else
    std::cout << ", " << z;
#endif // def USE_VIRTUAL_FUNC_POINT2D
    std::cout << ")";
  }

  double point3d::dot(const point3d& P) const
  {
    double answer = x * P.x + y * P.y;
#ifdef USE_VIRTUAL_FUNC_POINT2D
    if ((GetDimension() > 2) || (P.GetDimension() > 2))
#else
#endif // def USE_VIRTUAL_FUNC_POINT2D
    {
      answer = answer + z * P.z;
    }
    return answer;
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
#ifdef USE_VIRTUAL_FUNC_POINT2D
    if ((P.GetDimension() > 2) || (Q.GetDimension() > 2))
#else
#endif // def USE_VIRTUAL_FUNC_POINT2D
    {
      dt = (P.z - Q.z);
      answer = answer + dt * dt;
    }
    return answer;
  }

  double point3d::sqNorm() const { return (*this).dot(*this); }
  point3d& point3d::operator*=(const double& scal) { x *= scal; y *= scal; z *= scal; return *this; }

  point2d::point2d() : point3d(0, 0, 0) {}
  point2d::point2d(const double& xx, const double& yy) : point3d(xx, yy, 0) {}

#ifdef USE_VIRTUAL_FUNC_POINT2D
  int point2d::GetDimension() const { return 2; }
#endif // def USE_VIRTUAL_FUNC_POINT2D

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
    point2d pp(P.x - a.x, P.y - a.y);
    point2d qq(b.x - a.x, b.y - a.y);
    double coeff = pp.dot(qq) / sqLength();
    point2d rr(qq.x * coeff, qq.y * coeff);
    return point2d(rr.x + a.x, rr.y + a.y);
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
    point3d orthog(P.x - proj.x, P.y - proj.y, P.z - proj.z);
    return orthog.sqNorm();
  }

  point3d Edge3d::projection(const point3d& P) const
  {
    if (sqLength() <= threshold()) { return a; }
    point3d pp(P.x - a.x, P.y - a.y, P.z - a.z);
    point3d qq(b.x - a.x, b.y - a.y, b.z - a.z);
    double coeff = pp.dot(qq) / sqLength();
    point3d rr(qq.x * coeff, qq.y * coeff, qq.z * coeff);
    return point3d(rr.x + a.x, rr.y + a.y, rr.y + a.z);
  }

  Matrix2d::Matrix2d(const point2d& aa, const point2d& bb)
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
    Matrix2d inv(point2d(b.y, -a.y), point2d(-b.x, a.x));
    inv.a.x = inv.a.x / determinant;
    inv.a.y = inv.a.y / determinant;
    inv.b.x = inv.b.x / determinant;
    inv.b.y = inv.b.y / determinant;
    return inv;
  }

  void Matrix2d::takeTranspose()
  {
    auto temp = b.x;
    b.x = a.y;
    a.y = temp;
  }

  point2d Matrix2d::operator*(const point2d& rhs) const { return point2d(a.dot(rhs), b.dot(rhs)); }

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
    Matrix3d inv(point3d(ee * ii - ff * hh, cc * hh - ii * bb, ff * bb - cc * ee),
        point3d(gg * ff - ii * dd, aa * ii - cc * gg, cc * dd - aa * ff),
        point3d(dd * hh - gg * ee, bb * gg - aa * hh, aa * ee - bb * dd));
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
    a = point3d(aa.x, bb.x, cc.x);
    b = point3d(aa.y, bb.y, cc.y);
    c = point3d(aa.z, bb.z, cc.z);
  }

  point3d Matrix3d::operator*(const point3d& rhs) const { return point3d(a.dot(rhs), b.dot(rhs), c.dot(rhs)); }

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
      Matrix3d CX(point3d(a2, a.y, 1), point3d(b2, b.y, 1), point3d(c2, c.y, 1));
      Matrix3d CY(point3d(a.x, a2, 1), point3d(b.x, b2, 1), point3d(c.x, c2, 1));
      center = point2d(CX.det() / (den * 2.0), CY.det() / (den * 2.0));
      Matrix3d BB(point3d(a.x, a.y, a2), point3d(b.x, b.y, b2), point3d(c.x, c.y, c2));
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
    auto pp = point2d(tri.a.x - tri.b.x, tri.a.y - tri.b.y);
    auto qq = point2d(tri.c.x - tri.b.x, tri.c.y - tri.b.y);
    auto rr = point2d(pt.x - tri.b.x, pt.y - tri.b.y);
    Matrix2d AA(pp, qq);
    AA.takeTranspose();
    bool bInvertible = true;
    auto BB = AA.inverse(bInvertible);
    if (bInvertible)
    {
      point2d testPoint = BB * rr;
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
    if (pt.point2d::sqDistance(a) <= threshold()) { return 3; }
    if (pt.point2d::sqDistance(b) <= threshold()) { return 3; }
    if (pt.point2d::sqDistance(c) <= threshold()) { return 3; }
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

  Plane3d::Plane3d(const point3d& aa, const point3d& bb, const point3d& cc)
  {
    const auto& origin = aa;
    const auto v0 = point3d(bb.x - aa.x, bb.y - aa.y, bb.z - aa.z);
    const auto v1 = point3d(cc.x - aa.x, cc.y - aa.y, cc.z - aa.z);
    point3d normal(v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x);
    A = normal.x;
    B = normal.y;
    C = normal.z;
    D = -normal.dot(origin);
  }

  bool Plane3d::isValid() const
  {
    point3d normal(A, B, C);
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
    point3d normal(A, B, C);
    auto origin = pointInPlane();
    auto discriminant = normal.dot(point3d(pt.x - origin.x, pt.y - origin.y, pt.z - origin.z));
    if ((discriminant <= threshold()) && ((-discriminant) <= threshold())) { return 0; }
    if (discriminant > threshold()) { return 1; }
    return 2;
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
    auto pp = point3d(tri.a.x - tri.b.x, tri.a.y - tri.b.y, tri.a.z - tri.b.z);
    auto qq = point3d(tri.c.x - tri.b.x, tri.c.y - tri.b.y, tri.c.z - tri.b.z);
    auto rr = point3d(tri.d.x - tri.d.x, tri.d.y - tri.b.y, tri.d.z - tri.b.z);
    auto pointShifted = point3d(pt.x - tri.b.x, pt.y - tri.b.y, pt.z - tri.b.z);
    Matrix3d AA(pp, qq, rr);
    AA.takeTranspose();
    bool bInvertible = true;
    auto BB = AA.inverse(bInvertible);
    if (bInvertible)
    {
      point3d testPoint = BB * pointShifted;
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
