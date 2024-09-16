/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "primitives.h"

namespace ComputationalGeometry
{
  int& numRandomPoints();
  void SetWindowWidthHeight(int ww, int hh = -1);
  void GetWindowWidthHeight(int& ww, int& hh);
  __host__ __device__ double threshold();

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

  void MergeConvex3d(point3d* iConvexA, int iConvexASize,
                     point3d* iConvexB, int iConvexBSize,
                     point3d* oMerged, int* oMergedSize);

  std::vector<point2d> ConvexHullDivideAndConquer(std::vector<point2d>& iCloud);
  std::vector<point3d> ConvexHull3d(std::vector<point3d>& iCloud);

  __global__ void CenterOfMass(point2d* aa, int N, point2d* bb);
  __host__ __device__ void computeConvexHullCuda(point2d* iPointArray, int iNumPoints, int* isInHull);
  __global__ void ComputeConvexHulls(point2d* pointArrays, int* arraySizes, int maxArrSize, int numArrays, int* oIndicatorArrays);
  __host__ __device__ void computeConvexHull3d(point3d* iPointArray, int iNumPoints, int* isInHull);
  __global__ void ComputeConvexHulls3d(point3d* pointArrays, int* arraySizes, int maxArrSize, int numArrays, int* oIndicatorArrays);

  class PointCloud
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      PointCloud();
      PointCloud(int iNumPoints); // Use default constructor.
      PointCloud(point2d* iPoints, int iNumPoints);
      
      const std::vector<point2d>& PointArray() const;
      const std::vector<point2d>& ConvexHull() const;
      const std::vector<Triangle2d>& Delaunay() const;
      const std::vector<Edge2d>& NearestNeighbor() const;
      const std::vector<Triangle2d>& Triangulation() const;
      const std::vector<Edge2d>& Voronoi() const;
      bool getBoundingBox(point3d& min, point3d& max) const;
      void refresh(bool bRecompute = true);
      void setPoint(int index, const point2d& iPoint);
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
