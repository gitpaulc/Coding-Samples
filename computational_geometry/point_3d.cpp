
#include "includes.h"
#include "point_3d.h"

int window_id;

int& numRandomPoints()
{
  static int numberOfRandomPoints = 1000;
  return numberOfRandomPoints;
}

static int gWindowWidth = 1024;
static int gWindowHeight = 1024;

void SetWindowWidthHeight(int ww, int hh)
{
  gWindowWidth = ww;
  if (hh < 0) { hh = ww; }
  gWindowHeight = hh;
}

void GetWindowWidthHeight(int& ww, int& hh)
{
  ww = gWindowWidth;
  hh = gWindowHeight;
}

namespace ComputationalGeometry
{
  point_3d::point_3d() : x(0), y(0), z(0) {}
  point_3d::point_3d(const double& xx, const double& yy, const double& zz) : x(xx), y(yy), z(zz) {}

  int point_3d::GetDimension() const { return 3; }

  bool point_3d::operator< (const point_3d& q) const
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

  void point_3d::print(const std::string& prequel) const
  {
    std::cout << prequel << "(" << x << ", " << y;
    if (GetDimension() > 2) {std::cout << ", " << z;}
    std::cout << ")";
  }

  double point_3d::sq_distance(const point_3d& P, const point_3d& Q)
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

  point_2d::point_2d() : point_3d(0, 0, 0) {}
  point_2d::point_2d(const double& xx, const double& yy) : point_3d(xx, yy, 0) {}

  int point_2d::GetDimension() const { return 2; }

  void point_2d::generate_random_points(std::vector<point_2d>& container, const unsigned int& N)
  {
    double randMax = RAND_MAX;
    container.resize(0);
    std::set<point_2d > initial_set; // Using a set ensures unique points and allows automatic sorting.

    for (int i = 0; i < N; ++i)
    {
      double xx = (rand() * 1.8 / randMax) - 0.9;
      double yy = (rand() * 1.8 / randMax) - 0.9;
      point_2d P(xx, yy);
      initial_set.insert(P);
    }
    
    for (std::set<point_2d>::const_iterator it = initial_set.begin(); it != initial_set.end(); ++it)
    {
      container.push_back(*it);
    }
  }

  void point_2d::graham_scan(std::vector<point_2d>& hull, const std::vector<point_2d>& points, const unsigned int& N)
  {
    hull = points;
    if (points.size() <= 3) {return;}
      
    point_2d temp_origin_ = hull[0];
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
        if (i == N) {break;} // Stack is empty.
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

  double point_2d::get_orientation(const point_2d& P, const point_2d& Q)
  {
    return P.x * Q.y - P.y * Q.x;
  }

  double point_2d::get_orientation(const point_2d& O, const point_2d& P, const point_2d& Q)
  {
    return (P.x - O.x) * (Q.y - O.y) - (P.y - O.y) * (Q.x - O.x);
  }

  bool point_2d::comparator(const point_2d& P, const point_2d& Q)
  {
    // Equivalent to P < Q
    
    double theta_P = atan2(P.y, P.x);
    double theta_Q = atan2(Q.y, Q.x);
    
    return theta_P < theta_Q;
    //Also can use return get_orientation(P, Q) < 0;
  }

} // end of namespace ComputationalGeometry

std::vector<ComputationalGeometry::point_2d>& PointArray()
{
  static std::vector<ComputationalGeometry::point_2d> pointArray;
  return pointArray;
}

std::vector<ComputationalGeometry::point_2d>& ConvexHull()
{
  static std::vector<ComputationalGeometry::point_2d> convexHull;
  return convexHull;
}

void keyboard(unsigned char key, int x, int y)
{
  if ((key == 27) //Esc
      || (key == 'q') || (key == 'Q'))
  {
    glutDestroyWindow(window_id);
    exit(0);
  }
  glutPostRedisplay();
}

void recompute()
{
  // Generate random points for display.
  ComputationalGeometry::point_2d::generate_random_points(PointArray(), numRandomPoints());
  // Compute convex hull.
  ComputationalGeometry::point_2d::graham_scan(ConvexHull(), PointArray(), numRandomPoints());
}

void initialize_glut(int* argc_ptr, char** argv)
{
  // Initialize GLUT and create a window.
  glutInit(argc_ptr, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(-1, -1);
  glutInitWindowSize(gWindowWidth, gWindowHeight);

  window_id = glutCreateWindow("Computational Geometry - Paul Cernea - Press 'q' to exit.");
    
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);
    
  recompute();
}
