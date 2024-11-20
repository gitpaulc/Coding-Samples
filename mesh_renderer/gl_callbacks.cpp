
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

namespace MeshRenderer
{
std::vector<ComputationalGeometry::Edge2d> gWireframe;
bool gPointsHidden = false;
bool gEdgesHidden = false;
}

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
  if ((key == 'h') || (key == 'H'))
  {
    gPointsHidden = !gPointsHidden;
    gEdgesHidden = !gEdgesHidden;
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
    const auto& mesh = DoublyConnectedEdgeList::Get();
    Camera cam(mesh);
    bool success = mesh.project(cam, gWireframe);
    std::cout << "\nWireframe size = " << gWireframe.size();
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
  using namespace MeshRenderer;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPointSize(3.0f);
    
  int numEdges = (int)gWireframe.size();
  if (!gPointsHidden)
  {
    glColor3f(0.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);

    for (int i = 0; i < numEdges; ++i)
    {
      const auto& P = gWireframe[i].a;
      glVertex2f((GLfloat)P.x, (GLfloat)P.y);
    }
    glEnd();
  }

  if (!gEdgesHidden)
  {
    glColor3f(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < numEdges; ++i)
    {
      glBegin(GL_LINE_LOOP);
      const auto& edge = gWireframe[i];
      glVertex2f((GLfloat)edge.a.x, (GLfloat)edge.a.y);
      glVertex2f((GLfloat)edge.b.x, (GLfloat)edge.b.y);
      glEnd();
    }
  }

  glutSwapBuffers();
}
