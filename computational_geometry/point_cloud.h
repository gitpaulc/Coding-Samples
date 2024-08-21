/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <memory>
#include <set>
#include <vector>

namespace ComputationalGeometry
{
  int& numRandomPoints();
  void SetWindowWidthHeight(int ww, int hh = -1);
  void GetWindowWidthHeight(int& ww, int& hh);

  class point3d
  {
    public:
    double x;  double y;  double z;
    point3d();
    point3d(const double& xx, const double& yy, const double& zz);
    virtual int GetDimension() const;
	/** \brief Necessary for set insertion to work. */
    bool operator< (const point3d& q) const;
	void print(const std::string& prequel="") const;
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

  class PointCloud
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      PointCloud();
      
      const std::vector<point2d>& PointArray();
      const std::vector<point2d>& ConvexHull();
      void refresh();
      static PointCloud& Get();
      
    private: // These methods are declared only for the sake of exposition:

      /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
      void computeConvexHull();

      /** \brief Naively search among pairs. */
      static double naive_min_sq_distance(std::set<point3d>& A, std::set<point3d>& B, point3d& A_min, point3d& B_min);
      static double naive_min_sq_distance(std::set<point3d>& cloud, point3d& min_1, point3d& min_2);
      static double naive_min_sq_distance(std::set<point2d>& A, std::set<point2d>& B, point2d& A_min, point2d& B_min);
      static double naive_min_sq_distance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2);
      double naive_min_sq_distance(point2d& min_1, point2d& min_2);

      /** \brief O(n log(n)) Divide-and-conquer implementation of min. sq. dist. in point-set. */
      static double min_sq_distance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2);
      double min_sq_distance(point2d& min_1, point2d& min_2);

    public: // For testing.
      void unit_test();
  };
}

#endif //def POINT_CLOUD_H
