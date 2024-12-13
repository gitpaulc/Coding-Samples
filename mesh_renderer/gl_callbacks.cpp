
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

namespace MeshRenderer
{
  static std::vector<ComputationalGeometry::Edge2d> gWireframe;
  static bool gPointsHidden = false;
  static bool gEdgesHidden = false;

  static Camera& GetCamera(const DoublyConnectedEdgeList& mesh)
  {
    static Camera gCam(mesh);
    return gCam;
  }
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
  Camera& gCam = GetCamera(mesh);
  DoublyConnectedEdgeList::Get().project(gCam, gWireframe);
  //std::cout << "\nWireframe size = " << gWireframe.size();
}

void keyboard(unsigned char key, int x, int y)
{
  using namespace MeshRenderer;
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  Camera& gCam = GetCamera(mesh);
  if ((key == 'e') || (key == 'E'))
  {
    DoublyConnectedEdgeList::Get().Export();
    return;
  }
  if ((key == 'o') || (key == 'O'))
  {
    gCam.setViewOrthogonal(!(gCam.viewIsOrthogonal())); recalculate();
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

  {
    auto eye0 = gCam.getEye();
    auto eye = eye0;
    bool success;
    auto rot = gCam.getScreenAxesRotMatrix().inverse(success);
    double cX = 0; double cY = 0;
    bool recalc = false;
    if ((key == 'j') || (key == 'J')) { cY = -1; recalc = true; } // Pan left.
    if ((key == 'l') || (key == 'L')) { cY =  1; recalc = true; } // Pan right.
    if ((key == 'i') || (key == 'I')) { cX = -1; recalc = true; } // Pan up.
    if ((key == 'k') || (key == 'K')) { cX =  1; recalc = true; } // Pan down.
    if ((key == 'z') || (key == 'Z')) { eye.z += 0.1; recalc = true; } // Zoom in.
    if ((key == 'y') || (key == 'Y')) { eye.z -= 0.1; recalc = true; } // Zoom out.
    if (recalc)
    {
      auto dT = rot * ComputationalGeometry::vector2d(cX * 0.1, cY * 0.1);
      eye.x += dT.x; eye.y += dT.y;
      auto screen = gCam.getScreen();
      auto normal = screen.getNormal();
      auto screenPt = gCam.getEyeCast() + (eye - eye0);
      gCam.setEye(eye);
      gCam.setScreen(ComputationalGeometry::Plane3d::fromPointAndNormal(screenPt, normal));
      recalculate();
    }
  }

  const double fullAngle = 2.0 * 3.14159;
  if ((key == 'r') || (key == 'R')) // Rotate clockwise.
  {
    double rot = gCam.getScreenAxesRotation();
    rot += 0.1;  if (rot >= fullAngle) { rot -= fullAngle; }
    gCam.setScreenAxesRotation(rot);
    recalculate();
  }
  if ((key == 't') || (key == 'T')) // Rotate counter-clockwise.
  {
    double rot = gCam.getScreenAxesRotation();
    rot -= 0.1;  if (rot <= -fullAngle) { rot += fullAngle; }
    gCam.setScreenAxesRotation(rot);
    recalculate();
  }
  {
    double dYaw = 0.0; double dPitch = 0.0;
    bool yaw = false; bool pitch = false;
    if ((key == 'a') || (key == 'A')) // Rotate yaw.
    {
      dYaw = 0.1; yaw = true;
    }
    if ((key == 'd') || (key == 'D')) // Rotate yaw.
    {
      dYaw = -0.1; yaw = true;
    }
    if ((key == 'w') || (key == 'W')) // Rotate pitch.
    {
      dPitch = 0.1; pitch = true;
    }
    if ((key == 's') || (key == 'S')) // Rotate pitch.
    {
      dPitch = -0.1; pitch = true;
    }
    if (yaw || pitch)
    {
      auto eye = gCam.getEye();
      auto screen = gCam.getScreen();
      auto normal = screen.getNormal();
      ComputationalGeometry::Edge3d ray(eye, eye + normal);
      double tVal = 0.0;
      bool parallel = false;
      bool success = true;
      auto screenPt = screen.getRayCastResult(ray, tVal, parallel, success);
      if (success)
      {
        using namespace ComputationalGeometry;
        double rr = (screenPt - eye).sqNorm();
        if (rr >= 1.0e-9) { rr = sqrt(rr); }
        vector3d e1, e2;
        screen.getOrthonormalBasis(e1, e2);
        if (yaw) { normal = normal * cos(dYaw) + e1 * sin(dYaw); }
        if (pitch) { normal = normal * cos(dPitch) + e2 * sin(dPitch); }
        screenPt = eye + (normal * rr);
        gCam.setScreen(Plane3d::fromPointAndNormal(screenPt, normal));
      }
      recalculate();
    }
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
