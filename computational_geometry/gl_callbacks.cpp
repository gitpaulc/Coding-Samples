
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

  GetWindowId() = glutCreateWindow("Computational Geometry - Paul Cernea - Press 'q' to exit.");
    
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);
    
  ComputationalGeometry::recompute();
}

void keyboard(unsigned char key, int x, int y)
{
  if ((key == 27) //Esc
      || (key == 'q') || (key == 'Q'))
  {
    glutDestroyWindow(GetWindowId());
    exit(0);
  }
  glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
    ComputationalGeometry::recompute();
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
    int sizPointArray = point_2d::PointArray().size();

    for (int i = 0; i < sizPointArray; ++i)
    {
      const point_2d& P = point_2d::PointArray()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();
  
  glColor3f(1.0f, 0.0f, 0.0f);

  glBegin(GL_LINE_LOOP);
  {
    int sizPointArray = point_2d::ConvexHull().size();

    for (int i = 0; i < sizPointArray; i++)
    {
      point_2d P = point_2d::ConvexHull()[i];
      glVertex2f(P.x, P.y);
      //P.print("\n");
    }
  }
  glEnd();

  glutSwapBuffers();
}
