/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <memory>
#include <set>
#include <string>
#include <vector>

//#define USE_CUDA

#ifdef USE_CUDA
#include <cuda_runtime.h>
// __CUDA_RUNTIME_H__ is now a defined macro.
#else
#define __host__
#define __device__
#define __global__
#define __constant__
#endif // def USE_CUDA

//#define USE_VIRTUAL_FUNC_POINT2D

namespace ComputationalGeometry
{

class point3d
{
public:
    double x;  double y;  double z;
    __host__ __device__ point3d();
    __host__ __device__ point3d(const double& xx, const double& yy, const double& zz);
#ifdef USE_VIRTUAL_FUNC_POINT2D
    virtual int GetDimension() const; // Causes a crash when managing memory with malloc.
#endif // def USE_VIRTUAL_FUNC_POINT2D
    /** \brief Necessary for set insertion to work. */
    bool operator< (const point3d& q) const;
    void print(const std::string& prequel = "") const;
    __host__ __device__ double dot(const point3d& P) const;
    static double sqDistance(const point3d& P, const point3d& Q);
    __host__ __device__ double sqDistance(const point3d& Q) const;
    __host__ __device__ double sqNorm() const;
    __host__ __device__ point3d& operator*=(const double& scal);
};

class point2d : public point3d
{
public:
    __host__ __device__ point2d();
    __host__ __device__ point2d(const double& xx, const double& yy);
#ifdef USE_VIRTUAL_FUNC_POINT2D
    virtual int GetDimension() const; // Causes a crash when managing memory with malloc.
#endif // def USE_VIRTUAL_FUNC_POINT2D
    static double getOrientation(const point2d& P, const point2d& Q, const point2d& O = point2d());
    __host__ __device__ double orientation(const point2d& Q, const point2d& O = point2d()) const;
    static bool comparator(const point2d& P, const point2d& Q);
    __host__ __device__ bool compare(const point2d& Q) const;
};

class Edge2d
{
public:
  point2d a, b;
  __host__ __device__ Edge2d(const point2d& aa = point2d(), const point2d& bb = point2d());
  /** \brief For set and map insertion. */
  bool operator< (const Edge2d& rhs) const;
  /** \brief (sqLength.a < sqLength.b) if and only if (length.a < length.b). a^2 - b^2 = (a - b)(a + b) */
  __host__ __device__ double sqLength() const;
  __host__ __device__ double sqDistance(const point2d& P) const;
  /** \brief 0 = no intersection, 1 = point intersection, 2 = parallel intersection */
  int intersection(const Edge2d& other, point2d& intersection) const;
  /** \brief Point that attains square distance from point P to line spanned by edge. */
  point2d projection(const point2d& P) const;
};

class Edge3d
{
public:
  point3d a, b;
  __host__ __device__ Edge3d(const point3d& aa = point3d(), const point3d& bb = point3d());
  /** \brief For set and map insertion. */
  bool operator< (const Edge3d& rhs) const;
  /** \brief (sqLength.a < sqLength.b) if and only if (length.a < length.b). a^2 - b^2 = (a - b)(a + b) */
  __host__ __device__ double sqLength() const;
  __host__ __device__ double sqDistance(const point3d& P) const;
  /** \brief Point that attains square distance from point P to line spanned by edge. */
  __host__ __device__ point3d projection(const point3d& P) const;
};

class Matrix2d
{
public:
    point2d a, b; /**< Rows. */
    __host__ __device__ Matrix2d(const point2d& aa = point2d(), const point2d& bb = point2d());
    __host__ __device__ double det() const;
    __host__ __device__ Matrix2d inverse(bool& bSuccess) const;
    __host__ __device__ void takeTranspose();
    __host__ __device__ point2d operator*(const point2d& rhs) const;
};

class Matrix3d
{
public:
    point3d a, b, c; /**< Rows. */
    __host__ __device__ Matrix3d(const point3d& aa = point3d(), const point3d& bb = point3d(), const point3d& cc = point3d());
    __host__ __device__ double det() const;
    __host__ __device__ Matrix3d inverse(bool& bSuccess) const;
    __host__ __device__ void takeTranspose();
    __host__ __device__ point3d operator*(const point3d& rhs) const;
};

class Circle2d
{
public:
    point2d center;
    double sqRadius = 1.0;
    /** \brief Throws invalid argument if square radius is negative. */
    Circle2d(const point2d& cen, double sqRad);
    /** \brief Throws invalid argument if points are collinear. */
    Circle2d(const point2d& a, const point2d& b, const point2d& c);
    /** \brief 0 = exterior, 1 = interior, 2 = on edge */
    int pointIsInterior(const point2d& pt) const;
};

class Triangle2d
{
public:
  point2d a, b, c;
  __host__ __device__ Triangle2d(const point2d& aa = point2d(), const point2d& bb = point2d(), const point2d& cc = point2d());
  bool adjacentToByEdge(const Triangle2d& rhs, Edge2d& edge) const;
  double sqArea() const;
  /** \brief For set and map insertion. */
  bool operator< (const Triangle2d& rhs) const;
  /** \brief 0 = exterior, 1 = interior, 2 = on edge, 3 = on vertex */
  __host__ __device__ int pointIsInterior(const point2d& pt) const;
  std::set<Edge2d> getEdges() const;
};

/** \class Unique plane with the equation Ax + By + Cz + D = 0 */
class Plane3d
{
public:
  double A = 0;
  double B = 0;
  double C = 1;
  double D = 0;
  __host__ __device__ Plane3d(const point3d& aa = point3d(), const point3d& bb = point3d(), const point3d& cc = point3d());
  __host__ __device__ bool isValid() const;
  __host__ __device__ point3d pointInPlane() const; /** \brief Not necessarily unique. */
  /** \brief Which side of the plane is the point on? 2 for left, 1 for right, 0 for on plane. */
  __host__ __device__ int getSide(const point3d& pt) const;
};

class Triangle3d
{
public:
  point3d a, b, c;
  __host__ __device__ Triangle3d(const point3d& aa = point3d(), const point3d& bb = point3d(), const point3d& cc = point3d());
  /** \brief For set and map insertion. */
  bool operator<(const Triangle3d& rhs) const;
  std::set<Edge3d> getEdges() const;
};

class Tetrahedron3d
{
public:
    point3d a, b, c, d;
    __host__ __device__ Tetrahedron3d(const point3d& aa = point3d(),
                                      const point3d& bb = point3d(), const point3d& cc = point3d(), const point3d& dd = point3d());
    /** \brief 0 = exterior, 1 = interior, 2 = on face, 3 = on edge, 4 = on vertex */
    __host__ __device__ int pointIsInterior(const point3d& pt) const;
};

}

#endif //def PRIMITIVES_H
