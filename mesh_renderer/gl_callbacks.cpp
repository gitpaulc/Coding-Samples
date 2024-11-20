
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

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
  MeshRenderer::GetWindowWidthHeight(ww, hh);
  glutInitWindowSize(ww, hh);

  GetWindowId() = glutCreateWindow("Mesh Renderer - Paul Cernea - 'R' to redraw, 'E' to export, 'q' to exit.");
    
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);
}

void keyboard(unsigned char key, int x, int y)
{
  using namespace MeshRenderer;
  if ((key == 'e') || (key == 'E'))
  {
    DoublyConnectedEdgeList::Get().Export();
    return;
  }
  if ((key == 27) //Esc
      || (key == 'q') || (key == 'Q'))
  {
    glutDestroyWindow(GetWindowId());
    exit(0);
    glutPostRedisplay();
    return;
  }
  if ((key == 'r') || (key == 'R'))
  {
    MeshRenderer::Camera cam(DoublyConnectedEdgeList::Get());
  }
  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
  }
  glutPostRedisplay();
}

void render()
{
  using namespace ComputationalGeometry;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPointSize(3.0f);

  /*if (PointCloud::Get().pointsAreOn())
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
  }*/

  glutSwapBuffers();
}
