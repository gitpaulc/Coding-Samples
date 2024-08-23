
#include "gl_callbacks.h"

#include "includes.h"
#include "point_cloud.h"

int& GetWindowId()
{
  static int window_id = -1;
  return window_id;
}

void initialize_glut(int* argc_ptr, char** argv)
{
  // Initialize GLUT and create a window.
  glutInit(argc_ptr, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(-1, -1);
  int ww = 0;
  int hh = 0;
  ComputationalGeometry::GetWindowWidthHeight(ww, hh);
  glutInitWindowSize(ww, hh);

  GetWindowId() = glutCreateWindow("Computational Geometry - Paul Cernea - 'D' for Delaunay Triangulation, 'q' to exit.");
    
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);

  ComputationalGeometry::PointCloud::Get().refresh();
}

void keyboard(unsigned char key, int x, int y)
{
  if ((key == 27) //Esc
      || (key == 'q') || (key == 'Q'))
  {
    glutDestroyWindow(GetWindowId());
    exit(0);
  }
  if ((key == 't') || (key == 'T'))
  {
    ComputationalGeometry::PointCloud::Get().toggleTriangulation();
    ComputationalGeometry::PointCloud::Get().refresh(false);
  }
  if ((key == 'd') || (key == 'D'))
  {
    ComputationalGeometry::PointCloud::Get().toggleDelaunay();
    ComputationalGeometry::PointCloud::Get().refresh(false);
  }
  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
    ComputationalGeometry::PointCloud::Get().refresh();
  }
  glutPostRedisplay();
}

void render()
{
  using namespace ComputationalGeometry;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glColor3f(0.0f, 0.0f, 0.0f);
  glPointSize(3.0f);

  glBegin(GL_POINTS);
  {
    int sizPointArray = (int)PointCloud::Get().PointArray().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const auto& P = PointCloud::Get().PointArray()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();
  
  glColor3f(0.0f, 0.0f, 1.0f);

  //Triangulation.
  {
    int numTriangles = (int)PointCloud::Get().Triangulation().size();

    for (int i = 0; i < numTriangles; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& tri = PointCloud::Get().Triangulation()[i];
      glVertex2f(tri.a.x, tri.a.y);
      glVertex2f(tri.b.x, tri.b.y);
      glVertex2f(tri.c.x, tri.c.y);
      //P.print("\n");
      glEnd();
    }
  }
    
  glColor3f(0.0f, 1.0f, 0.0f);

  //Delaunay.
  {
    int numTriangles = (int)PointCloud::Get().Delaunay().size();

    for (int i = 0; i < numTriangles; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& tri = PointCloud::Get().Delaunay()[i];
      glVertex2f(tri.a.x, tri.a.y);
      glVertex2f(tri.b.x, tri.b.y);
      glVertex2f(tri.c.x, tri.c.y);
      //P.print("\n");
      glEnd();
    }
  }

  glColor3f(1.0f, 0.0f, 0.0f);

  glBegin(GL_LINE_LOOP);
  // Convex hull.
  {
    int sizPointArray = PointCloud::Get().ConvexHull().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const auto& P = PointCloud::Get().ConvexHull()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();

  glutSwapBuffers();
}
