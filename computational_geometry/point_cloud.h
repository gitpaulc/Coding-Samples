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

  class point_3d
  {
    public:
    double x;  double y;  double z;
    point_3d();
    point_3d(const double& xx, const double& yy, const double& zz);
    virtual int GetDimension() const;
	/** \brief Necessary for set insertion to work. */
    bool operator< (const point_3d& q) const;
	void print(const std::string& prequel="") const;
    static double sq_distance(const point_3d& P, const point_3d& Q);
    static double naive_min_sq_distance(std::set<point_3d>& A, std::set<point_3d>& B, point_3d& A_min, point_3d& B_min);
    static double naive_min_sq_distance(std::set<point_3d>& arr, point_3d& min_1, point_3d& min_2);
  };
	
  class point_2d : public point_3d
  {
    public:
      point_2d();
      point_2d(const double& xx, const double& yy);
      int GetDimension() const override;

      static double naive_min_sq_distance(std::set<point_2d>& A, std::set<point_2d>& B, point_2d& A_min, point_2d& B_min);
      static double naive_min_sq_distance(std::set<point_2d>& arr, point_2d& min_1, point_2d& min_2);

      /** \brief Divide-and-conquer implementation of min. sq. dist. in point-set. */
      static double min_sq_distance(std::set<point_2d>& arr, point_2d& min_1, point_2d& min_2);
      static void generate_random_points(std::vector<point_2d>& container, const unsigned int& N);
      static void graham_scan(std::vector<point_2d> & hull, const std::vector<point_2d> & points, const unsigned int& N);
		
    private:
      
      static double get_orientation(const point_2d& P, const point_2d& Q, const point_2d& O = point_2d());
      static bool comparator(const point_2d& P, const point_2d& Q);
  };

  class PointCloud
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      PointCloud();
      
      std::vector<point_2d>& PointArray();
      std::vector<point_2d>& ConvexHull();
      void refresh();
      static PointCloud& Get();
      
      // For testing.
      void unit_test();
  };
}

#endif //def POINT_CLOUD_H
