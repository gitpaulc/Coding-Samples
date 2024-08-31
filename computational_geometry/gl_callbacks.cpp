
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

  GetWindowId() = glutCreateWindow("Computational Geometry - Paul Cernea - 'D' Delaunay, 'V' Voronoi, 'N' Nearest Neighbor, 'q' to exit.");
    
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
    glutPostRedisplay();
    return;
  }
  if ((key == 'p') || (key == 'P'))
  {
    ComputationalGeometry::PointCloud::Get().togglePointsVisibility();
  }
  if ((key == 'c') || (key == 'C'))
  {
    ComputationalGeometry::PointCloud::Get().toggleConvexHull();
  }
  if ((key == 't') || (key == 'T'))
  {
    ComputationalGeometry::PointCloud::Get().toggleTriangulation();
  }
  if ((key == 'd') || (key == 'D'))
  {
    ComputationalGeometry::PointCloud::Get().toggleDelaunay();
  }
  if ((key == 'n') || (key == 'N'))
  {
    ComputationalGeometry::PointCloud::Get().toggleNearestNeighbor();
  }
  if ((key == 'v') || (key == 'V'))
  {
    if (ComputationalGeometry::PointCloud::Get().convexHullIsOn())
    {
      ComputationalGeometry::PointCloud::Get().toggleConvexHull();
    }
    ComputationalGeometry::PointCloud::Get().toggleVoronoi();
  }
  ComputationalGeometry::PointCloud::Get().refresh(false);
  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
    if (!(ComputationalGeometry::PointCloud::Get().pointsAreOn()))
    {
      ComputationalGeometry::PointCloud::Get().togglePointsVisibility();
    }
    ComputationalGeometry::PointCloud::Get().refresh();
  }
  glutPostRedisplay();
}

void render()
{
  using namespace ComputationalGeometry;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPointSize(3.0f);

  if (PointCloud::Get().pointsAreOn())
  {
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    int sizPointArray = (int)PointCloud::Get().PointArray().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const auto& P = PointCloud::Get().PointArray()[i];
      glVertex2f((GLfloat)P.x, (GLfloat)P.y);
      //P.print("\n");
    }
    glEnd();
  }

  if (PointCloud::Get().triangulationIsOn())
  {
    glColor3f(0.0f, 0.0f, 1.0f);
    int numTriangles = (int)PointCloud::Get().Triangulation().size();

    for (int i = 0; i < numTriangles; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& tri = PointCloud::Get().Triangulation()[i];
      glVertex2f((GLfloat)tri.a.x, (GLfloat)tri.a.y);
      glVertex2f((GLfloat)tri.b.x, (GLfloat)tri.b.y);
      glVertex2f((GLfloat)tri.c.x, (GLfloat)tri.c.y);
      //P.print("\n");
      glEnd();
    }
  }

  if (PointCloud::Get().delaunayIsOn())
  {
    glColor3f(0.0f, 1.0f, 0.0f);
    int numTriangles = (int)PointCloud::Get().Delaunay().size();

    for (int i = 0; i < numTriangles; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& tri = PointCloud::Get().Delaunay()[i];
      glVertex2f((GLfloat)tri.a.x, (GLfloat)tri.a.y);
      glVertex2f((GLfloat)tri.b.x, (GLfloat)tri.b.y);
      glVertex2f((GLfloat)tri.c.x, (GLfloat)tri.c.y);
      //P.print("\n");
      glEnd();
    }
  }

  if (PointCloud::Get().nearestNeighborIsOn())
  {
    glColor3f(0.5f, 0.5f, 0.5f);
    int numEdges = (int)PointCloud::Get().NearestNeighbor().size();

    for (int i = 0; i < numEdges; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& edge = PointCloud::Get().NearestNeighbor()[i];
      glVertex2f((GLfloat)edge.a.x, (GLfloat)edge.a.y);
      glVertex2f((GLfloat)edge.b.x, (GLfloat)edge.b.y);
      //P.print("\n");
      glEnd();
    }
  }

  if (PointCloud::Get().voronoiIsOn())
  {
    glColor3f(1.0f, 0.647f, 0.0f); // orange
    int numEdges = (int)PointCloud::Get().Voronoi().size();

    for (int i = 0; i < numEdges; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& edge = PointCloud::Get().Voronoi()[i];
      glVertex2f((GLfloat)edge.a.x, (GLfloat)edge.a.y);
      glVertex2f((GLfloat)edge.b.x, (GLfloat)edge.b.y);
      //P.print("\n");
      glEnd();
    }
  }

  if (PointCloud::Get().convexHullIsOn())
  {
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINE_LOOP);
    int sizPointArray = (int)PointCloud::Get().ConvexHull().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const auto& P = PointCloud::Get().ConvexHull()[i];
      glVertex2f((GLfloat)P.x, (GLfloat)P.y);
      //P.print("\n");
    }
    glEnd();
  }

  glutSwapBuffers();
}
