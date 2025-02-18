
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
  static double gScale;
  static ComputationalGeometry::point3d gOrigin;
  static double gXAngle, gYAngle, gZAngle;
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

  {
    ComputationalGeometry::point3d minPt, maxPt;
    double scale = (maxPt - minPt).sqNorm();
    if (scale < 0.001) { scale = 1.0; }
    scale = 1.0 / scale;
    MeshRenderer::gScale = scale;
  }

  recalculate();
  glutPostRedisplay();
}

void recalculate()
{
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  mesh.getWireframe(MeshRenderer::gWireframe);
  //std::cout << "\nWireframe size = " << MeshRenderer::gWireframe.size();
}

void updateView()
{
  using namespace MeshRenderer;
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  {
    //glRotatef(gXAngle, 1.0f, 0.0f, 0.0f);
    //glRotatef(gYAngle, 0.0f, 1.0f, 0.0f);
    //glRotatef(gZAngle, 0.0f, 0.0f, 1.0f);
    //glTranslatef(gOrigin.x, gOrigin.y, gOrigin.z);

    float cosA = cos(gXAngle);
    float sinA = sin(gXAngle);
    float cosB = cos(gYAngle);
    float sinB = sin(gYAngle);
    float cosC = cos(gZAngle);
    float sinC = sin(gZAngle);

    std::vector<GLfloat> projMatrix(16, 0.0f);
    projMatrix[0] = cosB * cosC;
    projMatrix[1] = sinA * sinB * cosC - cosA * sinC;
    projMatrix[2] = cosA * sinB * cosC - sinA * cosC;
    projMatrix[4] = cosB * sinC;
    projMatrix[5] = sinA * sinB * sinC + cosA * cosC;
    projMatrix[6] = cosA * sinB * sinC - sinA * cosC;
    projMatrix[8] = -sinB;
    projMatrix[9] = sinA * cosB;
    projMatrix[10] = cosA * cosB;
    projMatrix[12] = gOrigin.x;
    projMatrix[13] = gOrigin.y;
    projMatrix[14] = gOrigin.z;
    projMatrix[15] = 1.0f;
    glLoadMatrixf(projMatrix.data());
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  {
    //glScalef(gScale, gScale, gScale);
    std::vector<GLfloat> mvMatrix(16, 0.0f);
    mvMatrix[0] = gScale;
    mvMatrix[5] = gScale;
    mvMatrix[10] = gScale;
    mvMatrix[15] = 1.0f;
    glLoadMatrixf(mvMatrix.data());
  }
  glutPostRedisplay();
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

  if ((key == 'j') || (key == 'J')) { gOrigin.x += 0.1; } // Pan left.
  if ((key == 'l') || (key == 'L')) { gOrigin.x -= 0.1; } // Pan right.
  if ((key == 'i') || (key == 'I')) { gOrigin.y -= 0.1; } // Pan up.
  if ((key == 'k') || (key == 'K')) { gOrigin.y += 0.1; } // Pan down.
  if ((key == 'b') || (key == 'B')) { gOrigin.z -= 0.1; } // Zoom out.
  if ((key == 'z') || (key == 'Z')) { gScale *= 1.1; } // Zoom in.
  if ((key == 'y') || (key == 'Y')) { gScale /= 1.1; } // Zoom out.

  const double fullAngle = 2.0 * 3.14159;
  if ((key == 'r') || (key == 'R')) // Rotate clockwise.
  {
    gZAngle += 0.1;
    if (gZAngle >= 10.0 * fullAngle) { gZAngle -= 9.0 * fullAngle; }
  }
  if ((key == 't') || (key == 'T')) // Rotate counter-clockwise.
  {
    gZAngle -= 0.1;
    if (gZAngle <= -10.0 * fullAngle) { gZAngle += 9.0 * fullAngle; }
  }
  if ((key == 'a') || (key == 'A')) // Rotate yaw.
  {
    gYAngle += 0.1;
    if (gYAngle >= 10.0 * fullAngle) { gYAngle -= 9.0 * fullAngle; }
  }
  if ((key == 'd') || (key == 'D')) // Rotate yaw.
  {
    gYAngle -= 0.1;
    if (gYAngle <= -10.0 * fullAngle) { gYAngle += 9.0 * fullAngle; }
  }
  if ((key == 'w') || (key == 'W')) // Rotate pitch.
  {
    gXAngle += 0.1;
    if (gXAngle >= 10.0 * fullAngle) { gXAngle -= 9.0 * fullAngle; }
  }
  if ((key == 's') || (key == 'S')) // Rotate pitch.
  {
    gXAngle -= 0.1;
    if (gXAngle <= -10.0 * fullAngle) { gXAngle += 9.0 * fullAngle; }
  }
  updateView();
}

void mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
    MeshRenderer::gOrigin.z += 0.1; updateView();
  }
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
