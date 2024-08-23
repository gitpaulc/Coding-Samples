/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <memory>
#include <string>

namespace ComputationalGeometry
{
  int& numRandomPoints();
  void SetWindowWidthHeight(int ww, int hh = -1);
  void GetWindowWidthHeight(int& ww, int& hh);
  double threshold();

  class point3d
  {
    public:
    double x;  double y;  double z;
    point3d();
    point3d(const double& xx, const double& yy, const double& zz);
    virtual int GetDimension() const;
	/** \brief Necessary for set insertion to work. */
    bool operator< (const point3d& q) const;
	void print(const std::string& prequel = "") const;
    static double sq_distance(const point3d& P, const point3d& Q);
  };
	
  class point2d : public point3d
  {
    public:
      point2d();
      point2d(const double& xx, const double& yy);
      int GetDimension() const override;
      static double getOrientation(const point2d& P, const point2d& Q, const point2d& O = point2d());
      static bool comparator(const point2d& P, const point2d& Q);
  };

  class Edge2d
  {
  public:
    point2d a, b;
    Edge2d(const point2d& aa = point2d(), const point2d& bb = point2d());
    double sqLength() const;
    double sq_distance(const point2d& P) const;
    /** \brief 0 = no intersection, 1 = point intersection, 2 = parallel intersection */
    int intersection(const Edge2d& other, point2d& intersection) const;
  };

  class Triangle2d
  {
  public:
    point2d a, b, c;
    Triangle2d(const point2d& aa = point2d(), const point2d& bb = point2d(), const point2d& cc = point2d());
    bool adjacentToByEdge(const Triangle2d& rhs) const;
    double sqArea() const;
    /** \brief For set and map insertion. */
    bool operator< (const Triangle2d& rhs) const;
    /** \brief 0 = exterior, 1 = interior, 2 = on edge, 3 = on vertex */
    int pointIsInterior(const point2d& pt) const;
  };

  class PointCloud
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      PointCloud();
      
      const std::vector<point2d>& PointArray() const;
      const std::vector<point2d>& ConvexHull() const;
      const std::vector<Triangle2d>& Triangulation() const;
      bool getBoundingBox(point3d& min, point3d& max) const;
      void refresh(bool bRecompute = true);
      static PointCloud& Get();
      
      void toggleTriangulation();
      /** \brief An arbitrary triangulation. */
      void naiveTriangulate();
      
    private: // These methods are declared only for the sake of exposition:

      /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
      void computeConvexHull();

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
