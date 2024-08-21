
#include "includes.h"
#include "point_cloud.h"
#include "gl_callbacks.h"

namespace ComputationalGeometry
{
  static int gWindowWidth = 1024;
  static int gWindowHeight = 1024;

  int& numRandomPoints()
  {
    static int numberOfRandomPoints = 1000;
    return numberOfRandomPoints;
  }

  void SetWindowWidthHeight(int ww, int hh)
  {
    ComputationalGeometry::gWindowWidth = ww;
    if (hh < 0) { hh = ww; }
    ComputationalGeometry::gWindowHeight = hh;
  }

  void GetWindowWidthHeight(int& ww, int& hh)
  {
    ww = gWindowWidth;
    hh = gWindowHeight;
  }

  point3d::point3d() : x(0), y(0), z(0) {}
  point3d::point3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}

  int point3d::GetDimension() const { return 3; }

  bool point3d::operator< (const point3d& q) const
  {
    if (x > q.x) {return false;}
    if (x < q.x) {return true;}
    if (y > q.y) {return false;}
    if (y < q.y) {return true;}
    if (GetDimension() <= 2) {return false;}
    if (z > q.z) {return false;}
    if (z < q.z) {return true;}
    return false;
  }

  void point3d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y;
    if (GetDimension() > 2) {std::cout << ", " << z;}
    std::cout << ")";
  }

  double point3d::sq_distance(const point3d& P, const point3d& Q)
  {
    double answer = 0;
    double dt = (P.x - Q.x);
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

  point2d::point2d() : point3d(0, 0, 0) {}
  point2d::point2d(const double& xx, const double& yy) : point3d(xx, yy, 0) {}

  int point2d::GetDimension() const { return 2; }

  double point2d::getOrientation(const point2d& P, const point2d& Q, const point2d& O)
  {
    return (P.x - O.x) * (Q.y - O.y) - (P.y - O.y) * (Q.x - O.x);
  }

  bool point2d::comparator(const point2d& P, const point2d& Q)
  {
    // Equivalent to P < Q
    
    double theta_P = atan2(P.y, P.x);
    double theta_Q = atan2(Q.y, Q.x);
    
    return theta_P < theta_Q;
    //Also can use return getOrientation(P, Q) < 0;
  }

  class PointCloud::Impl
  {
  public:
    PointCloud* pCloud = nullptr;

    std::vector<point2d> hull;
    std::vector<point2d> pointArray;

    Impl(PointCloud* pParent) : pCloud(pParent) {}
    void generateRandomPoints();
      
    /** \brief O(n log(n)) Convex hull implementation. Graham scan for 2d points. */
    void computeConvexHull();
  };

  PointCloud::PointCloud() : pImpl(std::make_unique<PointCloud::Impl>(this))
  {
    // unique_ptr requires C++ 11.
    // make_unique requires C++ 14.
  }

  const std::vector<point2d>& PointCloud::PointArray()
  {
    if (pImpl == nullptr) // Of course this should never happen.
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->pointArray;
  }

  const std::vector<point2d>& PointCloud::ConvexHull()
  {
    if (pImpl == nullptr)
    {
      static std::vector<point2d> dummy;
      return dummy;
    }
    return pImpl->hull;
  }

  void PointCloud::refresh()
  {
    if (pImpl == nullptr) { return; }
    // Generate random points for display.
    pImpl->generateRandomPoints();
    pImpl->computeConvexHull();
  }

  PointCloud& PointCloud::Get()
  {
    static PointCloud pc;
    return pc;
  }

  void PointCloud::Impl::generateRandomPoints()
  {
    double randMax = (double)RAND_MAX;
    pointArray.resize(0);
    std::set<point2d> initialSet; // Using a set ensures unique points and allows automatic sorting.

    for (int i = 0; i < numRandomPoints(); ++i)
    {
      double xx = (rand() * 1.8 / randMax) - 0.9;
      double yy = (rand() * 1.8 / randMax) - 0.9;
      point2d P(xx, yy);
      initialSet.insert(P);
    }
  
    // Pre- C++ 11 style of iteration:
    for (std::set<point2d>::const_iterator it = initialSet.begin(); it != initialSet.end(); ++it)
    {
      pointArray.push_back(*it);
    }
  }

  void PointCloud::computeConvexHull()
  {
    if (pImpl != nullptr) { pImpl->computeConvexHull(); }
  }

  void PointCloud::Impl::computeConvexHull()
  {
    hull.resize(0);

    // 2d : Graham scan.
    hull = pointArray;
    const int NN = (int)pointArray.size();
    if (NN <= 3) {return;}

    point2d tempOrigin = hull[0];
    {
      int bestIndex = 0;
      const int startIndex = 1;
      for (int i = startIndex; i < NN; ++i)
      {
        if (hull[i].y < tempOrigin.y)
        {
          tempOrigin = hull[i];
          bestIndex = i;
        }
      }

      hull[bestIndex] = hull[1];
      hull[1] = tempOrigin;
      hull.push_back(hull[0]);
    
      for (int i = startIndex; i <= NN; ++i)
      {
        hull[i].x = hull[i].x - tempOrigin.x;
        hull[i].y = hull[i].y - tempOrigin.y;
      }
    
      // O(n log(n)):
      std::sort(hull.begin() + 1, hull.end(), point2d::comparator);
    
      for (int i = startIndex; i <= NN; ++i)
      {
        hull[i].x = hull[i].x + tempOrigin.x;
        hull[i].y = hull[i].y + tempOrigin.y;
      }
        
      hull[0] = hull[NN];
    }
    
    int hullCount = 1; // Initialize stack.
    const int startIndex = 2;
    for (int i = startIndex; i <= NN; ++i)
    {
      while (point2d::getOrientation(hull[hullCount], hull[i], hull[hullCount - 1]) <= 0)
      {
        if (hullCount > 1)
        {
          --hullCount;  continue;
        }
        if (i == NN) {break;} // Stack is empty.
        ++i; // Else keep searching.
      }
      // Otherwise point is on the boundary of the convex hull.
      ++hullCount;
      tempOrigin = hull[hullCount];
      hull[hullCount] = hull[i];
      hull[i] = tempOrigin;
    }

    hull.resize(hullCount);
  }

  template <class Container> static double naiveMinSqDistanceImpl(Container& A, Container& B, point3d& A_min, point3d& B_min)
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
              min = point3d::sq_distance(*it1, *it2);
              A_min = *it1;
              B_min = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sq_distance(*it1, *it2);
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

  template <class Container> static double naiveMinSqDistanceImpl(Container& arr, point3d& min_1, point3d& min_2)
  {
    double min = 0;  bool started = false;
    if (arr.begin() != arr.end())
    {
      min_1 = *(arr.begin());
      min_2 = *(arr.begin());
    }

    // Iteration involving template parameters:
    for (typename Container::iterator it1 = arr.begin(); it1 != arr.end(); ++it1)
    {
      for (typename Container::iterator it2 = arr.begin(); it2 != arr.end(); ++it2)
      {
          if (it1 == it2) {continue;}
          // Note: set iteration takes place in sorted order.
          //std::cout << "[";  it2->print();  std::cout << "]\n";
          if (!started)
          {
              min = point3d::sq_distance(*it1, *it2);
              min_1 = *it1;
              min_2 = *it2;
              started = true;
              continue;
          }
          double candidate = point3d::sq_distance(*it1, *it2);
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

  double PointCloud::naive_min_sq_distance(std::set<point3d>& A, std::set<point3d>& B, point3d& A_min, point3d& B_min)
  {
    return naiveMinSqDistanceImpl(A, B, A_min, B_min);
  }

  double PointCloud::naive_min_sq_distance(std::set<point3d>& arr, point3d& min_1, point3d& min_2)
  {
    return naiveMinSqDistanceImpl(arr, min_1, min_2);
  }

  double PointCloud::naive_min_sq_distance(std::set<point2d>& A, std::set<point2d>& B, point2d& A_min, point2d& B_min)
  {
    return naiveMinSqDistanceImpl(A, B, A_min, B_min);
  }

  double PointCloud::naive_min_sq_distance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return naiveMinSqDistanceImpl(cloud, min_1, min_2);
  }

  double PointCloud::naive_min_sq_distance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return naiveMinSqDistanceImpl(pImpl->pointArray, min_1, min_2);
  }

  template <class Container> static double min_sq_distance_helper(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();

    // These cases (where arrCount is 0 or 1) should never happen in the private helper method.
    //if (arrCount == 0) {return 0;}
    //if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sq_distance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sq_distance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sq_distance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sq_distance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }
    
    unsigned halfArrCount = arrCount / 2;
    unsigned remainingArrCount = arrCount - halfArrCount;
    
    Container arr_1, arr_2;
    point2d left_1, left_2, right_1, right_2;
    double min_L, min_R;
    
    {
      typename Container::iterator it = arr.begin();
      for (int i = 0; i < halfArrCount; i++)
      {
        arr_1.push_back(*it);  ++it;
      }
    
      for (int i = 0; i < remainingArrCount; i++)
      {
        arr_2.push_back(*it);  ++it;
      }
    }
    
    min_L = min_sq_distance_helper(arr_1, left_1,  left_2);
    min_R = min_sq_distance_helper(arr_2, right_1, right_2);
    
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

  template <class Container> static double min_sq_distance_impl(Container& arr, point2d& min_1, point2d& min_2)
  {
    double min = 0;
    unsigned arrCount = (unsigned) arr.size();
    if (arrCount == 0) {return 0;}
    if (arrCount == 1) {min_1 = *(arr.begin());  min_2 = *(arr.begin());  return 0;}
    if (arrCount == 2)
    {
      typename Container::iterator it = arr.begin();
      min_1 = *it;  ++it;  min_2 = *it;
      return point2d::sq_distance(min_1, min_2);
    }
    if (arrCount == 3)
    {
      typename Container::iterator it = arr.begin();
      point2d a = *it;  ++it;  point2d b = *it;
      double min_ = point2d::sq_distance(a, b);
      min_1 = a;  min_2 = b;
      ++it;
      double candidate = point2d::sq_distance(a, *it);
      if (candidate < min_)
      {
        min_ = candidate;  /*min_1 = a;*/  min_2 = *it;
      }
      candidate = point2d::sq_distance(*it, b);
      if (candidate < min_)
      {
        min_ = candidate;  min_1 = *it;  min_2 = b;
      }
      return min_;
    }

    std::vector<point2d > arr_;
    for (typename Container::iterator it = arr.begin(); it != arr.end(); ++it)
    {
      arr_.push_back(*it);
    }
  
    std::sort(arr_.begin(), arr_.end());
    min = min_sq_distance_helper(arr_, min_1, min_2);
    return min;
  }

  double PointCloud::min_sq_distance(std::set<point2d>& cloud, point2d& min_1, point2d& min_2)
  {
    return min_sq_distance_impl(cloud, min_1, min_2);
  }

  double PointCloud::min_sq_distance(point2d& min_1, point2d& min_2)
  {
    if (pImpl == nullptr) { return -1; }
    return min_sq_distance_impl(pImpl->pointArray, min_1, min_2);
  }

  void PointCloud::unit_test()
  {
    {
      point3d P(1.0, 2.0, 3.0);
      std::cout << "\n//////\nThe dimension is " << P.GetDimension() << ".";
      P.print("\n");
      std::cout << "\n";

      point2d Q(1.0, 2.0);
      std::cout << "\n//////\nThe dimension is " << Q.GetDimension() << ".";
      Q.print("\n");
      std::cout << "\n";
    }
  
    {
      point3d P(1.0, 2.0, 3.0);
      point3d Q(1.0, -2.0, 3.0);
  
      printf("%f\n", point3d::sq_distance(P,Q));
    }
  
    {
      point2d P(1.5, 2.0);
      point2d Q(1.0, -2.0);
  
      printf("%f\n", point2d::sq_distance(P,Q));
    }
  
    {
      std::set<point3d> A;
      point3d a(1.5, 2.0, 0);
      point3d b(1.0, 3.0, 0);
      point3d c(-1.5, 7.0, 0);
    
      std::set<point3d> B;
      point3d d(4.5, 2.3, 0);
      point3d e(-1.55, 2.6, 0);
      point3d f(88.3, 0.001, 0);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point3d p, q;
    
      double min = naive_min_sq_distance(A, B, p, q);
    
      std::cout << "\n//////\n" << min;
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d> A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
    
      std::set<point2d> B;
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
    
      B.insert(d);
      B.insert(e);
      B.insert(f);
    
      point2d p, q;
    
      double min = naive_min_sq_distance(A, B, p, q);
    
      std::cout << "\n//////\n" << min << "\n";
      p.print("\n");
      q.print("\n");
      std::cout << "\n";
    }
  
    {
      std::set<point2d > A;
      point2d a(1.5, 2.0);
      point2d b(1.0, 3.0);
      point2d c(-1.5, 7.0);
      point2d d(4.5, 2.3);
      point2d e(-1.35, 2.6);
      point2d f(88.3, 0.001);
    
      A.insert(a);
      A.insert(b);
      A.insert(c);
      A.insert(d);
      A.insert(e);
      A.insert(f);
    
      point2d p, q;

      double min = naive_min_sq_distance(A, p, q);
    
      std::cout << "\n//////\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    
      min = min_sq_distance(A, p, q);
    
      std::cout << "\n/!!!!/\n";
      std::cout << min << "\n";
      p.print();  std::cout << "\n";
      q.print();  std::cout << "\n";
    }
  } // end of unit_test function.

} // end of namespace ComputationalGeometry
