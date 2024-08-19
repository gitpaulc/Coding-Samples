/*  Copyright Paul Cernea, May 30, 2017.
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
	template<class T> class point_3d
	{
	public:
		T x;  T y;  T z;
		point_3d() : x(0), y(0), z(0) {}
		point_3d(const T& xx, const T& yy, const T& zz) : x(xx), y(yy), z(zz) {}
		
		virtual int GetDimension() const { return 3; }
		
		/*
		 * Necessary for set insertion to work.
		 */
		bool operator <(const point_3d<T>& q) const
		{
			if (x > q.x) {return false;}
			if (x < q.x) {return  true;}
			if (y > q.y) {return false;}
			if (y < q.y) {return  true;}
			if (GetDimension() <= 2) {return false;}
			if (z > q.z) {return false;}
			if (z < q.z) {return  true;}
			return false;
		}
		
		void print(const std::string& prequel="") const
		{
			std::cout << prequel << "(" << x << ", " << y;
			if (GetDimension() > 2) {std::cout << ", " << z;}
			std::cout << ")";
		}
		
		static T sq_distance(const point_3d<T>& P, const point_3d<T>& Q)
		{
			T answer = 0;
			T dt = (P.x - Q.x);
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
		
		template <class Container> static T naive_min_sq_distance(Container& A, Container& B, point_3d<T>& A_min, point_3d<T>& B_min)
		{
			T min = 0;  bool started = false;
			for (typename Container::iterator it1 = A.begin(); it1 != A.end(); ++it1)
			{
				for (typename Container::iterator it2 = B.begin(); it2 != B.end(); ++it2)
				{
					// Note: set iteration takes place in sorted order.
					//std::cout << "[";  it2->print();  std::cout << "]\n";
					if (!started)
					{
						min = point_3d<T>::sq_distance(*it1, *it2);
						A_min = *it1;
						B_min = *it2;
						started = true;
						continue;
					}
					T candidate = point_3d<T>::sq_distance(*it1, *it2);
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
		
		template <class Container> static T naive_min_sq_distance(Container& arr, point_3d<T>& min_1, point_3d<T>& min_2)
		{
			T min = 0;  bool started = false;
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
						min = point_3d<T>::sq_distance(*it1, *it2);
						min_1 = *it1;
						min_2 = *it2;
						started = true;
						continue;
					}
					T candidate = point_3d<T>::sq_distance(*it1, *it2);
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
	
	template<class T> class point_2d : public point_3d<T>
	{
	public:
		point_2d() : point_3d<T>(0, 0, 0) {}
		point_2d(const T& xx, const T& yy) : point_3d<T>(xx, yy, 0) {}
		
		/*virtual*/ int GetDimension() const { return 2; }
		
		static void generate_random_points(std::vector<point_2d<T> > & container, const unsigned & N)
		{
			double randMax = RAND_MAX;
		
			container.resize(0);
			std::set<point_2d<T> > initial_set; //Using a set initially ensures unique points and allows automatic sorting.
		
			for (int i = 0; i < N; ++i)
			{
				T xx = (rand() * 1.8 / randMax) - 0.9;
				T yy = (rand() * 1.8 / randMax) - 0.9;
				point_2d<T> P(xx, yy);
				initial_set.insert(P);
			}
			
			for (typename std::set<point_2d<T> >::iterator it = initial_set.begin(); it != initial_set.end(); ++it)
			{
				container.push_back(*it);
			}
		}
		
		static void graham_scan(std::vector<point_2d<T> > & hull, const std::vector<point_2d<T> > & points, const unsigned & N)
		{	
			hull = points;
			
			if (points.size() <= 3) {return;}
			
			point_2d<T> temp_origin_ = hull[0];
			{
				int best_index = 0;
				
				for (int i = 1; i < N; i++)
				{
					if (hull[i].y < temp_origin_.y)
					{
						temp_origin_ = hull[i];
						best_index = i;
					}
				}
				
				hull[best_index] = hull[1];
				hull[1] = temp_origin_;
				hull.push_back(hull[0]);
			
				/*std::cout << "\n/// LIST:\n";
				for (int i = 0; i <= N; i++)
				{
					hull[i].print("\n");
				}
				std::cout << "\n";*/
			
				for (int i = 1; i <= N; i++)
				{
					hull[i].x = hull[i].x - temp_origin_.x;
					hull[i].y = hull[i].y - temp_origin_.y;
				}
			
				std::sort(hull.begin() + 1, hull.end(), comparator);
				
				/*std::cout << "\n/// LIST:\n";
				for (int i = 1; i <= N; i++)
				{
					printf("\n%f", atan2(hull[i].y, hull[i].x));
				}
				std::cout << "\n";*/
			
				for (int i = 1; i <= N; i++)
				{
					hull[i].x = hull[i].x + temp_origin_.x;
					hull[i].y = hull[i].y + temp_origin_.y;
				}
				
				hull[0] = hull[N];
			}
			
			int hull_count = 1; // Initialize stack.
			
			for (int i = 2; i <= N; i++)
			{
				while (get_orientation(hull[hull_count - 1], hull[hull_count], hull[i]) <= 0)
				{
					if (hull_count > 1)
					{
						hull_count--;  continue;
					}
					else if (i == N) {break;} // Stack is empty.
					else {i++;} // Keep searching.
				}
				// Otherwise point is on the boundary of the convex hull.
				hull_count++;
				temp_origin_ = hull[hull_count];
				hull[hull_count] = hull[i];
				hull[i] = temp_origin_;
			}
			
			hull.resize(hull_count);
		}
		
		template <class Container> static T min_sq_distance(Container& arr, point_2d<T>& min_1, point_2d<T>& min_2)
		{
			T min = 0;
		
			unsigned arr_count = (unsigned) arr.size();
		
			if (arr_count == 0) {return 0;}
			if (arr_count == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
			if (arr_count == 2)
			{
				typename Container::iterator it = arr.begin();
				min_1 = *it;  ++it;  min_2 = *it;
				return point_2d<T>::sq_distance(min_1, min_2);
			}
			if (arr_count == 3)
			{
				typename Container::iterator it = arr.begin();
				point_2d<T> a = *it;  ++it;  point_2d<T> b = *it;
				T min_ = point_2d<T>::sq_distance(a, b);
				min_1 = a;  min_2 = b;
				++it;
				T candidate = point_2d<T>::sq_distance(a, *it);
				if (candidate < min_)
				{
					min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
				}
				candidate = point_2d<T>::sq_distance(*it, b);
				if (candidate < min_)
				{
					min_ = candidate;  min_1 = *it;  min_2 = b;
				}
				return min_;
			}
			
			std::vector<point_2d<T> > arr_;
			for (typename Container::iterator it = arr.begin(); it != arr.end(); ++it)
			{
				arr_.push_back(*it);
			}
			
			std::sort(arr_.begin(), arr_.end());
			
			min = min_sq_distance_(arr_, min_1, min_2);
		
			return min;
		}
		
	private:
		
		static T get_orientation(const point_2d<T> & P, const point_2d<T> & Q)
		{
		    return P.x * Q.y - P.y * Q.x;
		}
		
		static T get_orientation(const point_2d<T> & O, const point_2d<T> & P, const point_2d<T> & Q)
		{
		    return (P.x - O.x) * (Q.y - O.y) - (P.y - O.y) * (Q.x - O.x);
		}
		
		static bool comparator(const point_2d<T> & P, const point_2d<T> & Q)
		{
			// Equivalent to P < Q
			
			double theta_P = atan2(P.y, P.x);
			double theta_Q = atan2(Q.y, Q.x);
			
			return theta_P < theta_Q;
			
			//return get_orientation(P, Q) < 0;
		}
		
		template <class Container> static T min_sq_distance_(Container& arr, point_2d<T>& min_1, point_2d<T>& min_2)
		{
			T min = 0;
		
			unsigned arr_count = (unsigned) arr.size();
		
			// These cases (where arr_count is 0 or 1) should never happen in the private method.
			//if (arr_count == 0) {return 0;}
			//if (arr_count == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
			if (arr_count == 2)
			{
				typename Container::iterator it = arr.begin();
				min_1 = *it;  ++it;  min_2 = *it;
				return point_2d<T>::sq_distance(min_1, min_2);
			}
			if (arr_count == 3)
			{
				typename Container::iterator it = arr.begin();
				point_2d<T> a = *it;  ++it;  point_2d<T> b = *it;
				T min_ = point_2d<T>::sq_distance(a, b);
				min_1 = a;  min_2 = b;
				++it;
				T candidate = point_2d<T>::sq_distance(a, *it);
				if (candidate < min_)
				{
					min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
				}
				candidate = point_2d<T>::sq_distance(*it, b);
				if (candidate < min_)
				{
					min_ = candidate;  min_1 = *it;  min_2 = b;
				}
				return min_;
			}
			
			unsigned half_arr_count = arr_count / 2;
			unsigned remaining_arr_count = arr_count - half_arr_count;
			
			Container arr_1, arr_2;
			point_2d<T> left_1, left_2, right_1, right_2;
			T min_L, min_R;
			
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

std::vector<ComputationalGeometry::point_2d<double> >& PointArray();
std::vector<ComputationalGeometry::point_2d<double> >& ConvexHull();

#endif //def POINT_3D_H
