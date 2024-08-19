
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

std::vector<ComputationalGeometry::point_2d<double> >& PointArray()
{
  static std::vector<ComputationalGeometry::point_2d<double> > pointArray;
  return pointArray;
}

std::vector<ComputationalGeometry::point_2d<double> >& ConvexHull()
{
  static std::vector<ComputationalGeometry::point_2d<double> > convexHull;
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
  ComputationalGeometry::point_2d<double>::generate_random_points(PointArray(), numRandomPoints());
  // Compute convex hull.
  ComputationalGeometry::point_2d<double>::graham_scan(ConvexHull(), PointArray(), numRandomPoints());
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
