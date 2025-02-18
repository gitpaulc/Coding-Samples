
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

namespace MeshRenderer
{
  static std::vector<ComputationalGeometry::Edge3d> gWireframe;
  static GLuint gVertexBufferObj;
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

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut);

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

  glGenBuffers(1, &MeshRenderer::gVertexBufferObj);

  recalculate();
  glutPostRedisplay();
}

void recalculate()
{
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  mesh.getWireframe(MeshRenderer::gWireframe);
  //std::cout << "\nWireframe size = " << MeshRenderer::gWireframe.size();
}

void keyboard(unsigned char key, int x, int y)
{
  using namespace MeshRenderer;
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  Camera& gCam = GetCamera(mesh);
  if ((key == 'e') || (key == 'E'))
  {
    mesh.Export();
    return;
  }
  if ((key == 'o') || (key == 'O'))
  {
    gCam.setViewOrthogonal(!(gCam.viewIsOrthogonal()));
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
    if ((key == 'j') || (key == 'J')) { cX = 1; } // Pan left.
    if ((key == 'l') || (key == 'L')) { cX = -1; } // Pan right.
    if ((key == 'i') || (key == 'I')) { cY = -1; } // Pan up.
    if ((key == 'k') || (key == 'K')) { cY = 1; } // Pan down.
    if ((key == 'z') || (key == 'Z')) { eye.z += 0.1; } // Zoom in.
    if ((key == 'y') || (key == 'Y')) { eye.z -= 0.1; } // Zoom out.

    {
      auto dT = rot * ComputationalGeometry::vector2d(cX * 0.1, cY * 0.1);
      eye.x += dT.x; eye.y += dT.y;
      auto screen = gCam.getScreen();
      auto normal = screen.getNormal();
      auto screenPt = gCam.getEyeCast() + (eye - eye0);
      gCam.setEye(eye);
      gCam.setScreen(ComputationalGeometry::Plane3d::fromPointAndNormal(screenPt, normal));
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
    }
  }
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  {
    auto eye = gCam.getEye();
    glTranslatef(eye.x, eye.y, eye.z);
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  {
    ComputationalGeometry::point3d minPt, maxPt;
    double scale = (maxPt - minPt).sqNorm();
    if (scale < 0.001) { scale = 1.0; }
    scale = 1.0 / scale;
    glScalef((float)scale, (float)scale, (float)scale);
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

  std::vector<float> vertexData;
  toVertex3dData(gWireframe, vertexData);
    
  if (gPointsHidden || gEdgesHidden) { vertexData.resize(0); }

  glPointSize(3.0f);
  glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
  glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);

  int stride = 0;
  glVertexPointer(3, GL_FLOAT, stride, NULL);
  glEnableClientState(GL_VERTEX_ARRAY);
    
  int whichArray = 0;
  glColor3f(0.0f, 0.0f, 0.0f);
  glDrawArrays(GL_POINTS, whichArray, vertexData.size() / 3);
    
  glColor3f(1.0f, 0.0f, 0.0f);
  glDrawArrays(GL_LINES, whichArray, vertexData.size() / 3);

  glDisableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glutSwapBuffers();
}

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut)
{
  const auto oldSize = dataIn.size();
  const auto newSize = dataIn.size() * 6;
  dataOut.resize(newSize);
  if (oldSize == 0) { return; }
  for (int ind = 0; ind < oldSize; ++ind)
  {
    int ind0 = ind;
    dataOut[ind * 6] = dataIn[ind0].a.x;
    dataOut[ind * 6 + 1] = dataIn[ind0].a.y;
    dataOut[ind * 6 + 2] = dataIn[ind0].a.z;
    dataOut[ind * 6 + 3] = dataIn[ind0].b.x;
    dataOut[ind * 6 + 4] = dataIn[ind0].b.y;
    dataOut[ind * 6 + 5] = dataIn[ind0].b.z;
  }
}
