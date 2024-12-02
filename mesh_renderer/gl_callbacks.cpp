
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

namespace MeshRenderer
{
  static std::vector<ComputationalGeometry::Edge2d> gWireframe;
  static bool gOrthogonal = false;
  static bool gPointsHidden = false;
  static bool gEdgesHidden = false;

  // World to viewer:
  ComputationalGeometry::point3d gOrigin = ComputationalGeometry::point3d(0.0, 0.0, 0.0);
  double gRot = 0.0; // Rotation parallel to screen.
  double gYaw = 0.0; // Rotate screen from left to right.
  double gPitch = 0.0; // Rotate screen up and down.
}

void recalculate();

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

  GetWindowId() = glutCreateWindow("Mesh Renderer - Paul Cernea - 'E' to export, 'O' toggle orthogonal, 'q' to exit.");
    
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);

  recalculate();
  glutPostRedisplay();
}

void recalculate()
{
  using namespace MeshRenderer;
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  auto cam = MeshRenderer::Camera(mesh);
  {
    using namespace ComputationalGeometry;
    auto eye = cam.getEye();
    auto screen = cam.getScreen();
    auto normal = screen.getNormal();
    Edge3d ray(eye, eye + normal);
    double tVal = 0.0;
    bool parallel = false;
    bool success = true;
    auto screenPt = screen.getRayCastResult(ray, tVal, parallel, success);
    if (success)
    {
      double rr = (screenPt - eye).sqNorm();
      if (rr >= 1.0e-9)
      {
        rr = sqrt(rr);
        vector3d e1, e2;
        screen.getOrthonormalBasis(e1, e2);
        normal = (normal * cos(gYaw)) + (e1 * sin(gYaw) * cos(gPitch)) + (e2 * sin(gYaw) * sin(gPitch));
        screenPt = eye + (normal * rr);
      }
    }
    if (success)
    {
      double gZoom = gOrigin.z;
      eye = eye + (normal * gZoom);
      screenPt = screenPt + (normal * gZoom);
      screen = ComputationalGeometry::Plane3d::fromPointAndNormal(screenPt, normal);
      cam.setEye(eye);
      cam.setScreen(screen);
    }
  }

  if (gOrthogonal) { cam.setViewOrthogonal(true); }
  bool success = DoublyConnectedEdgeList::Get().project(cam, gWireframe, gRot);
  for (auto& edge : gWireframe)
  {
    edge.a.x -= gOrigin.x;
    edge.a.y -= gOrigin.y;
    edge.b.x -= gOrigin.x;
    edge.b.y -= gOrigin.y;
  }
  //std::cout << "\nWireframe size = " << gWireframe.size();
}

void keyboard(unsigned char key, int x, int y)
{
  using namespace MeshRenderer;
  if ((key == 'e') || (key == 'E'))
  {
    DoublyConnectedEdgeList::Get().Export();
    return;
  }
  if ((key == 'o') || (key == 'O'))
  {
    gOrthogonal = !gOrthogonal; recalculate();
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

  if ((key == 'j') || (key == 'J')) { gOrigin.x -= 0.1; recalculate(); } // Pan left.
  if ((key == 'l') || (key == 'L')) { gOrigin.x += 0.1; recalculate(); } // Pan right.
  if ((key == 'i') || (key == 'I')) { gOrigin.y += 0.1; recalculate(); } // Pan up.
  if ((key == 'k') || (key == 'K')) { gOrigin.y -= 0.1; recalculate(); } // Pan down.
  if ((key == 'z') || (key == 'Z')) { gOrigin.z += 0.1; recalculate(); } // Zoom in.
  if ((key == 'y') || (key == 'Y')) { gOrigin.z -= 0.1; recalculate(); } // Zoom out.

  const double fullAngle = 2.0 * 3.14159;
  if ((key == 'r') || (key == 'R')) // Rotate clockwise.
  {
    gRot += 0.1;  if (gRot >= fullAngle) { gRot -= fullAngle; }
    recalculate();
  }
  if ((key == 't') || (key == 'T')) // Rotate counter-clockwise.
  {
    gRot -= 0.1;  if (gRot <= -fullAngle) { gRot += fullAngle; }
    recalculate();
  }
  if ((key == 'a') || (key == 'A')) // Rotate yaw.
  {
    gYaw += 0.1;  if (gYaw >= fullAngle) { gYaw -= fullAngle; }
    recalculate();
  }
  if ((key == 'd') || (key == 'D')) // Rotate yaw.
  {
    gYaw -= 0.1;  if (gYaw <= -fullAngle) { gYaw += fullAngle; }
    recalculate();
  }
  if ((key == 'w') || (key == 'W')) // Rotate pitch.
  {
    gPitch += 0.1;  if (gPitch >= fullAngle) { gPitch -= fullAngle; }
    recalculate();
  }
  if ((key == 's') || (key == 'S')) // Rotate counter-clockwise.
  {
    gPitch -= 0.1;  if (gPitch <= -fullAngle) { gPitch += fullAngle; }
    recalculate();
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
