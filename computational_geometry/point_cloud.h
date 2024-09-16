/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

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
  int& numRandomPoints();
  void SetWindowWidthHeight(int ww, int hh = -1);
  void GetWindowWidthHeight(int& ww, int& hh);
  __host__ __device__ double threshold();

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
      double dot(const point3d& P) const;
      static double sqDistance(const point3d& P, const point3d& Q);
      __host__ __device__ double sqDistance(const point3d& Q) const;
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

  /**
   *  `input` two convex polygons, A and B of respective sizes iConvexASize and iConvexBSize.
   *  They are ASSUMED to be separated by a line of the form { x = const. } with max{iConvexA.x} < min{iConvexB.x}.
   *  The output convex polygon oMerged is ASSUMED to have max. size (iConvexASize + iConvexBSize) preallocated.
   *  `output` oMerged as the polygon, oMergedSize as the actual size. oMergedSize pointer is ASSUMED not null.
   *  Running time is linear: O(iConvexASize + iConvexBSize).
   */
  void MergeConvex(point2d* iConvexA, int iConvexASize,
                   point2d* iConvexB, int iConvexBSize,
                   point2d* oMerged, int* oMergedSize);

  std::vector<point2d> ConvexHullDivideAndConquer(std::vector<point2d>& iCloud);

  __global__ void CenterOfMass(point2d* aa, int N, point2d* bb);
  __host__ __device__ void computeConvexHullCuda(point2d* iPointArray, int iNumPoints, int* isInHull);
  __global__ void ComputeConvexHulls(point2d* pointArrays, int* arraySizes, int maxArrSize, int numArrays, int* oIndicatorArrays);

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

  class Matrix2d
  {
  public:
    point2d a, b; /**< Rows. */
    Matrix2d(const point2d& aa = point2d(), const point2d& bb = point2d());
    double det() const;
  };

  class Matrix3d
  {
  public:
    point3d a, b, c; /**< Rows. */
    Matrix3d(const point3d& aa = point3d(), const point3d& bb = point3d(), const point3d& cc = point3d());
    double det() const;
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
    int pointIsInterior(const point2d& pt) const;
    /** \brief 0 = exterior, 1 = interior, 2 = on edge */
    __host__ __device__ int pointIsInterior2(const point2d& pt) const;
    std::set<Edge2d> getEdges() const;
  };

  class PointCloud
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      PointCloud();
      PointCloud(point2d* iPoints, int iNumPoints);
      
      const std::vector<point2d>& PointArray() const;
      const std::vector<point2d>& ConvexHull() const;
      const std::vector<Triangle2d>& Delaunay() const;
      const std::vector<Edge2d>& NearestNeighbor() const;
      const std::vector<Triangle2d>& Triangulation() const;
      const std::vector<Edge2d>& Voronoi() const;
      bool getBoundingBox(point3d& min, point3d& max) const;
      void refresh(bool bRecompute = true);
      static PointCloud& Get();
      static bool pointIsInConvexSorted(const std::vector<point2d>& iHull, const point2d& iPoint);
      
      void toggleConvexHull();
      void toggleDelaunay();
      void toggleNearestNeighbor();
      void togglePointsVisibility();
      void toggleTriangulation();
      void toggleVoronoi();
      
      bool convexHullIsOn() const;
      bool delaunayIsOn() const;
      bool nearestNeighborIsOn() const;
      bool pointsAreOn() const;
      bool triangulationIsOn() const;
      bool voronoiIsOn() const;

      /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
      void computeConvexHull();

    private: // These methods are declared only for the sake of exposition:

      /** \brief A naive O(n^2 log(n)) triangulation. */
      void naiveTriangulate();
      /** \brief Delaunay triangulation maximizes the minimum angle of the triangulation. */
      void computeDelaunay();
      
      void computeNearestNeighbor();
      void computeVoronoi();
      
      /** \brief Naively search among pairs. */
      static double naiveMinSqDistance(std::set<point3d>& A, std::set<point3d>& B, point3d& A_min, point3d& B_min);
      static double naiveMinSqDistance(std::set<point3d>& cloud, point3d& min_1, point3d& min_2);
      static double naiveMinSqDistance(std::set<point2d>& A, std::set<point2d>& B, point2d& A_min, point2d& B_min);
      static double naiveMinSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2);
      double naiveMinSqDistance(point2d& min_1, point2d& min_2);

      /** \brief O(n log(n)) Divide-and-conquer implementation of min. sq. dist. in point-set. */
      static double minSqDistance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2);
      double minSqDistance(point2d& min_1, point2d& min_2);

    public: // For testing.
      void unitTest();
  };
}

#endif //def POINT_CLOUD_H
