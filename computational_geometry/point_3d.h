/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef POINT_3D_H
#define POINT_3D_H

#include "includes.h"

int& numRandomPoints();
void SetWindowWidthHeight(int ww, int hh = -1);
void GetWindowWidthHeight(int& ww, int& hh);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void render();
void recompute();
void initialize_glut(int* argc_ptr, char** argv);

namespace ComputationalGeometry
{
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
		
		template <class Container> static double naive_min_sq_distance(Container& A, Container& B, point_3d& A_min, point_3d& B_min)
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
						min = point_3d::sq_distance(*it1, *it2);
						A_min = *it1;
						B_min = *it2;
						started = true;
						continue;
					}
					double candidate = point_3d::sq_distance(*it1, *it2);
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
		
		template <class Container> static double naive_min_sq_distance(Container& arr, point_3d& min_1, point_3d& min_2)
		{
			double min = 0;  bool started = false;
			if (arr.begin() != arr.end())
			{
				min_1 = *(arr.begin());
				min_2 = *(arr.begin());
			}
			for (typename Container::iterator it1 = arr.begin(); it1 != arr.end(); ++it1)
			{
				for (typename Container::iterator it2 = arr.begin(); it2 != arr.end(); ++it2)
				{
					if (it1 == it2) {continue;}
					// Note: set iteration takes place in sorted order.
					//std::cout << "[";  it2->print();  std::cout << "]\n";
					if (!started)
					{
						min = point_3d::sq_distance(*it1, *it2);
						min_1 = *it1;
						min_2 = *it2;
						started = true;
						continue;
					}
					double candidate = point_3d::sq_distance(*it1, *it2);
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
		
	};
	
  class point_2d : public point_3d
  {
    public:
      point_2d();
      point_2d(const double& xx, const double& yy);
      int GetDimension() const override;
      static void generate_random_points(std::vector<point_2d>& container, const unsigned int& N);
      static void graham_scan(std::vector<point_2d> & hull, const std::vector<point_2d> & points, const unsigned int& N);
		
      template <class Container> static double min_sq_distance(Container& arr, point_2d& min_1, point_2d& min_2)
		{
			double min = 0;
		
			unsigned arr_count = (unsigned) arr.size();
		
			if (arr_count == 0) {return 0;}
			if (arr_count == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
			if (arr_count == 2)
			{
				typename Container::iterator it = arr.begin();
				min_1 = *it;  ++it;  min_2 = *it;
				return point_2d::sq_distance(min_1, min_2);
			}
			if (arr_count == 3)
			{
				typename Container::iterator it = arr.begin();
				point_2d a = *it;  ++it;  point_2d b = *it;
				double min_ = point_2d::sq_distance(a, b);
				min_1 = a;  min_2 = b;
				++it;
				double candidate = point_2d::sq_distance(a, *it);
				if (candidate < min_)
				{
					min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
				}
				candidate = point_2d::sq_distance(*it, b);
				if (candidate < min_)
				{
					min_ = candidate;  min_1 = *it;  min_2 = b;
				}
				return min_;
			}
			
			std::vector<point_2d > arr_;
			for (typename Container::iterator it = arr.begin(); it != arr.end(); ++it)
			{
				arr_.push_back(*it);
			}
			
			std::sort(arr_.begin(), arr_.end());
			
			min = min_sq_distance_(arr_, min_1, min_2);
		
			return min;
		}
		
    private:
      static double get_orientation(const point_2d& P, const point_2d& Q);
      static double get_orientation(const point_2d& O, const point_2d& P, const point_2d& Q);
      static bool comparator(const point_2d& P, const point_2d& Q);
	  template <class Container> static double min_sq_distance_(Container& arr, point_2d& min_1, point_2d& min_2)
		{
			double min = 0;
		
			unsigned arr_count = (unsigned) arr.size();
		
			// These cases (where arr_count is 0 or 1) should never happen in the private method.
			//if (arr_count == 0) {return 0;}
			//if (arr_count == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
			if (arr_count == 2)
			{
				typename Container::iterator it = arr.begin();
				min_1 = *it;  ++it;  min_2 = *it;
				return point_2d::sq_distance(min_1, min_2);
			}
			if (arr_count == 3)
			{
				typename Container::iterator it = arr.begin();
				point_2d a = *it;  ++it;  point_2d b = *it;
				double min_ = point_2d::sq_distance(a, b);
				min_1 = a;  min_2 = b;
				++it;
				double candidate = point_2d::sq_distance(a, *it);
				if (candidate < min_)
				{
					min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
				}
				candidate = point_2d::sq_distance(*it, b);
				if (candidate < min_)
				{
					min_ = candidate;  min_1 = *it;  min_2 = b;
				}
				return min_;
			}
			
			unsigned half_arr_count = arr_count / 2;
			unsigned remaining_arr_count = arr_count - half_arr_count;
			
			Container arr_1, arr_2;
			point_2d left_1, left_2, right_1, right_2;
			double min_L, min_R;
			
			{
				typename Container::iterator it = arr.begin();
				for (int i = 0; i < half_arr_count; i++)
				{
					arr_1.push_back(*it);  ++it;
				}
			
				for (int i = 0; i < remaining_arr_count; i++)
				{
					arr_2.push_back(*it);  ++it;
				}
			}
			
			min_L = min_sq_distance_(arr_1, left_1,  left_2);
			min_R = min_sq_distance_(arr_2, right_1, right_2);
			
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

	};
	
}

std::vector<ComputationalGeometry::point_2d>& PointArray();
std::vector<ComputationalGeometry::point_2d>& ConvexHull();

#endif //def POINT_3D_H
