
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

#include <sstream>

namespace MeshRenderer
{
  static std::vector<ComputationalGeometry::Edge3d> gWireframe;
  static GLuint gVertexBufferObj;
  static bool gPointsHidden = false;
  static bool gDepthBuffering = true;
  static bool gEdgesHidden = false;
  static bool gWireframeOn = true;

  static Camera& GetCamera(const DoublyConnectedEdgeList& mesh)
  {
    static Camera gCam(mesh);
    return gCam;
  }
  static double gScale;
  static double gXAngle, gYAngle, gZAngle;
  static std::vector<GLfloat> gMvMatrix(16, 0.0f);
  static std::vector<GLfloat> gProjMatrix(16, 0.0f);
  static GLuint gVertexShader, gFragmentShader, gShaderProgram;
  static bool gUseShaders = false;
  static GLint gPosLocation, gMvLocation, gProjLocation;
  static ComputationalGeometry::point3d gBoundingMax, gBoundingMin;
}

void recalculate();

int& GetWindowId()
{
  static int window_id = -1;
  return window_id;
}

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut, bool triangles);
void linkShaderProgram();

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

  GetWindowId() = glutCreateWindow("Mesh Renderer - Paul Cernea - 'E' to export, 'Y' zoom out, 'Z' zoom in, 'U' toggle wireframe, 'q' to exit.");

#ifdef __GLEW_H__
  {
    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
      fprintf(stderr, "\nError initializing GLEW: %s", glewGetErrorString(err));
    }
  }
#endif
  
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  
  glutSetCursor(GLUT_CURSOR_INFO);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutDisplayFunc(render);

  glGenBuffers(1, &MeshRenderer::gVertexBufferObj);
  linkShaderProgram();

  {
    ComputationalGeometry::point3d minPt, maxPt;
    double scale = (maxPt - minPt).sqNorm();
    if (scale < 0.001) { scale = 1.0; }
    scale = 0.2 / scale;
    MeshRenderer::gScale = scale;
  }

  recalculate();
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  auto& gCam = GetCamera(mesh);
  gCam.setEye(ComputationalGeometry::point3d());
  keyboard('J', 0, 0);
}

void recalculate()
{
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  if (MeshRenderer::gWireframeOn)
  {
    mesh.getWireframe(MeshRenderer::gWireframe);
  }
  else
  {
    mesh.getSkeleton(MeshRenderer::gWireframe);
  }
  mesh.getBoundingBox(MeshRenderer::gBoundingMax, MeshRenderer::gBoundingMin);
}

void vertex2color(const float& xIn, const float& yIn, const float& zIn,
  float& rOut, float& gOut, float& bOut)
{
  ComputationalGeometry::point3d rgbMax(1, 1, 1);
  ComputationalGeometry::point3d rgbMin(0, 0, 1);
  float margin = (float)(0.25 * (MeshRenderer::gBoundingMax.z - MeshRenderer::gBoundingMin.z));
  auto MM = (float)MeshRenderer::gBoundingMax.z + margin;
  if (zIn >= MM)
  {
    rOut = (float)rgbMax.x;
    gOut = (float)rgbMax.y;
    bOut = (float)rgbMax.z;
    return;
  }
  auto mm = (float)MeshRenderer::gBoundingMin.z - margin;
  if (zIn <= mm)
  {
    rOut = (float)rgbMin.x;
    gOut = (float)rgbMin.y;
    bOut = (float)rgbMin.z;
    return;
  }
  auto tt = (zIn - mm) / ((float)(MM - mm));
  rOut = (float)(rgbMax.x * tt + rgbMin.x * (1.0f - tt));
  gOut = (float)(rgbMax.y * tt + rgbMin.y * (1.0f - tt));
  bOut = (float)(rgbMax.z * tt + rgbMin.z * (1.0f - tt));
}

void updateView()
{
  using namespace MeshRenderer;
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  Camera& gCam = GetCamera(mesh);
  auto origin = gCam.getEye();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (gCam.viewIsOrthogonal())
  {
    int ww = glutGet(GLUT_WINDOW_WIDTH);
    int hh = glutGet(GLUT_WINDOW_HEIGHT);
    glOrtho(0.0f, ww, hh, 0.0, 0.001, 10000);
  }
  else
  {
    double fov = 90.0;
    int ww = glutGet(GLUT_WINDOW_WIDTH);
    int hh = glutGet(GLUT_WINDOW_HEIGHT);
    double aspectRatio = (double)ww / (double)hh;
    double nearPlane = 0.001;
    double farPlane = 10000;
    gluPerspective(fov, aspectRatio, nearPlane, farPlane);
  }
  {
    //glRotatef(gXAngle, 1.0f, 0.0f, 0.0f);
    //glRotatef(gYAngle, 0.0f, 1.0f, 0.0f);
    //glRotatef(gZAngle, 0.0f, 0.0f, 1.0f);
    //glTranslatef(gOrigin.x, gOrigin.y, gOrigin.z);

    float cosA = (float)cos(gXAngle);
    float sinA = (float)sin(gXAngle);
    float cosB = (float)cos(gYAngle);
    float sinB = (float)sin(gYAngle);
    float cosC = (float)cos(gZAngle);
    float sinC = (float)sin(gZAngle);

    gProjMatrix[0] = cosB * cosC;
    gProjMatrix[1] = sinA * sinB * cosC - cosA * sinC;
    gProjMatrix[2] = cosA * sinB * cosC - sinA * cosC;
    gProjMatrix[4] = cosB * sinC;
    gProjMatrix[5] = sinA * sinB * sinC + cosA * cosC;
    gProjMatrix[6] = cosA * sinB * sinC - sinA * cosC;
    gProjMatrix[8] = -sinB;
    gProjMatrix[9] = sinA * cosB;
    gProjMatrix[10] = cosA * cosB;
    gProjMatrix[12] = (float)origin.x;
    gProjMatrix[13] = (float)origin.y;
    gProjMatrix[14] = (float)origin.z;
    gProjMatrix[15] = 1.0f;
    glLoadMatrixf(gProjMatrix.data());
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  {
    //glScalef(gScale, gScale, gScale);
    gMvMatrix = std::vector<GLfloat>(16, 0.0f);
    gMvMatrix[0] = (float)gScale;
    gMvMatrix[5] = (float)gScale;
    gMvMatrix[10] = (float)gScale;
    gMvMatrix[15] = 1.0f;
    glLoadMatrixf(gMvMatrix.data());
  }
  glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
  using namespace MeshRenderer;
  const auto& mesh = DoublyConnectedEdgeList::Get();
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
  if ((key == 'p') || (key == 'P'))
  {
    gDepthBuffering = !gDepthBuffering;
  }
  if ((key == 'h') || (key == 'H'))
  {
    gPointsHidden = !gPointsHidden;
    gEdgesHidden = !gEdgesHidden;
  }
  if ((key == 'u') || (key == 'U'))
  {
    gWireframeOn = !gWireframeOn;
    recalculate();
  }
  if ((key == 27) //Esc
      || (key == 'q') || (key == 'Q'))
  {
    glutDestroyWindow(GetWindowId());
    exit(0);
    glutPostRedisplay();
    return;
  }

  auto origin = gCam.getEye();

  if ((key == 'j') || (key == 'J')) { origin.x += 0.1; } // Pan left.
  if ((key == 'l') || (key == 'L')) { origin.x -= 0.1; } // Pan right.
  if ((key == 'i') || (key == 'I')) { origin.y -= 0.1; } // Pan up.
  if ((key == 'k') || (key == 'K')) { origin.y += 0.1; } // Pan down.
  if ((key == 'b') || (key == 'B')) { origin.z -= 0.1; } // Zoom out.
  if ((key == 'z') || (key == 'Z')) { gScale *= 1.1; } // Zoom in.
  if ((key == 'y') || (key == 'Y')) { gScale /= 1.1; } // Zoom out.

  gCam.setEye(origin);

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
    using namespace MeshRenderer;
    const auto& mesh = DoublyConnectedEdgeList::Get();
    Camera& gCam = GetCamera(mesh);
    auto origin = gCam.getEye();
    origin.z += 0.1;
    gCam.setEye(origin);
    updateView();
  }
}

void render()
{
  using namespace ComputationalGeometry;
  using namespace MeshRenderer;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (gDepthBuffering) { glEnable(GL_DEPTH_TEST); }
  else { glDisable(GL_DEPTH_TEST); }

  std::vector<float> vertexData;
  toVertex3dData(gWireframe, vertexData, !gWireframeOn);
    
  if (gPointsHidden || gEdgesHidden) { vertexData.resize(0); }
  else if (!gWireframeOn && gUseShaders)
  {
    glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
    glEnableVertexAttribArray(gPosLocation);
    glVertexAttribPointer(gPosLocation, 3, GL_FLOAT, GL_FALSE, sizeof(vertexData.data()[0]), (void*)0);
    glUseProgram(gShaderProgram);
    glUniformMatrix4fv(gProjLocation, 1, GL_FALSE, (const GLfloat*)gProjMatrix.data());
    glUniformMatrix4fv(gMvLocation, 1, GL_FALSE, (const GLfloat*)gMvMatrix.data());
    int whichArray = 0;
    glDrawArrays(GL_TRIANGLES, whichArray, (GLsizei)vertexData.size() / 3);

    glDisableVertexAttribArray(gPosLocation);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glutSwapBuffers();
    return;
  }
  else if (!gWireframeOn)
  {
    glUseProgram(0);
    glPointSize(3.0f);
    glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);

    int stride = 0;
    glVertexPointer(3, GL_FLOAT, stride, NULL);

    auto numTriangles = static_cast<GLsizei>(vertexData.size() / 9);
    glBegin(GL_TRIANGLES);
    for (GLsizei ii = 0; ii < numTriangles; ++ii)
    {
      float rr = 0; float gg = 0; float bb = 0;
      for (int jj = 0; jj < 3; ++jj)
      {
        auto ind = 9 * ii + 3 * jj;
        float xx = vertexData[ind]; float yy = vertexData[ind + 1]; float zz = vertexData[ind + 2];
        vertex2color(xx, yy, zz, rr, gg, bb);
        glColor3f(rr, gg, bb);
        glVertex3f(xx, yy, zz);
      }
    }
    glEnd();

    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glutSwapBuffers();
    return;
  }

  glUseProgram(0);
  glPointSize(3.0f);
  glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
  glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);

  int stride = 0;
  glVertexPointer(3, GL_FLOAT, stride, NULL);
  glEnableClientState(GL_VERTEX_ARRAY);
    
  int whichArray = 0;
  glColor3f(0.0f, 0.0f, 0.0f);
  glDrawArrays(GL_POINTS, whichArray, (GLsizei)vertexData.size() / 3);
    
  glColor3f(1.0f, 0.0f, 0.0f);
  glDrawArrays(GL_LINES, whichArray, (GLsizei)vertexData.size() / 3);

  glDisableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glutSwapBuffers();
}

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut, bool triangles)
{
  const auto oldSize = dataIn.size();
  if (triangles)
  {
    const auto newSize = dataIn.size() * 3;
    dataOut.resize(newSize);
    if (oldSize == 0) { return; }
    for (int ind = 0; ind < oldSize; ind += 3)
    {
      if (ind + 1 > oldSize) { break; }
      if (ind + 2 > oldSize) { break; }
      dataOut[ind * 3] = (float)dataIn[ind].a.x;
      dataOut[ind * 3 + 1] = (float)dataIn[ind].a.y;
      dataOut[ind * 3 + 2] = (float)dataIn[ind].a.z;
      dataOut[ind * 3 + 3] = (float)dataIn[ind + 1].a.x;
      dataOut[ind * 3 + 4] = (float)dataIn[ind + 1].a.y;
      dataOut[ind * 3 + 5] = (float)dataIn[ind + 1].a.z;
      dataOut[ind * 3 + 6] = (float)dataIn[ind + 2].a.x;
      dataOut[ind * 3 + 7] = (float)dataIn[ind + 2].a.y;
      dataOut[ind * 3 + 8] = (float)dataIn[ind + 2].a.z;
    }
    return;
  }
  const auto newSize = dataIn.size() * 6;
  dataOut.resize(newSize);
  if (oldSize == 0) { return; }
  for (int ind = 0; ind < oldSize; ++ind)
  {
    int ind0 = ind;
    dataOut[ind * 6] = (float)dataIn[ind0].a.x;
    dataOut[ind * 6 + 1] = (float)dataIn[ind0].a.y;
    dataOut[ind * 6 + 2] = (float)dataIn[ind0].a.z;
    dataOut[ind * 6 + 3] = (float)dataIn[ind0].b.x;
    dataOut[ind * 6 + 4] = (float)dataIn[ind0].b.y;
    dataOut[ind * 6 + 5] = (float)dataIn[ind0].b.z;
  }
}

std::string glslVertexShaderCode()
{
  std::stringstream glsl;
  glsl << "\n#version 110";
  glsl << "\nuniform mat4 mv;"; // mv = VIEW * MODEL
  glsl << "\nuniform mat4 proj;";
  glsl << "\nattribute vec3 posVec;";
  glsl << "\nvarying vec3 color;";
  glsl << "\nvoid main()";
  glsl << "\n{";
  glsl << "\n  gl_Position = proj * mv * vec4(posVec, 1.0);";
  glsl << "\n  color = vec3(0.0, 0.0, 0.5 + gl_Position.z);";
  glsl << "\n}";
  return glsl.str();
}

std::string glslFragmentShaderCode()
{
  std::stringstream glsl;
  glsl << "\n#version 110";
  glsl << "\nvarying vec3 color;";
  glsl << "\nvoid main()";
  glsl << "\n{";
  glsl << "\n  gl_FragColor = vec4(color, 1.0);";
  glsl << "\n}";
  return glsl.str();
}

void linkShaderProgram()
{
  using namespace MeshRenderer;
  auto vertexShaderStr = glslVertexShaderCode();
  auto fragmentShaderStr = glslFragmentShaderCode();
  const char* vertexShaderCode = vertexShaderStr.c_str();
  const char* fragmentShaderCode = fragmentShaderStr.c_str();
  gVertexShader = glCreateShader(GL_VERTEX_SHADER);
  gFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(gVertexShader, 1, &vertexShaderCode, NULL);
  glCompileShader(gVertexShader);
  int errOut = 0;
  char errorLog[512];
  glGetShaderiv(gVertexShader, GL_COMPILE_STATUS, &errOut);
  if (!errOut)
  {
    glGetShaderInfoLog(gVertexShader, 512, NULL, errorLog);
    std::cout << "\nVertex shader compilation failed:\n" << errorLog << "\n";
  }
  glShaderSource(gFragmentShader, 1, &fragmentShaderCode, NULL);
  glCompileShader(gFragmentShader);
  errOut = 0;
  glGetShaderiv(gFragmentShader, GL_COMPILE_STATUS, &errOut);
  if (!errOut)
  {
    glGetShaderInfoLog(gFragmentShader, 512, NULL, errorLog);
    std::cout << "\nFragment shader compilation failed:\n" << errorLog << "\n";
  }
  gShaderProgram = glCreateProgram();
  glAttachShader(gShaderProgram, gVertexShader);
  glAttachShader(gShaderProgram, gFragmentShader);
  glLinkProgram(gShaderProgram);
  glGetShaderiv(gShaderProgram, GL_LINK_STATUS, &errOut);
  if (!errOut)
  {
    glGetShaderInfoLog(gShaderProgram, 512, NULL, errorLog);
    std::cout << "\nShader linking failed:\n" << errorLog << "\n";
  }
  gPosLocation = glGetAttribLocation(gShaderProgram, "posVec");
  gMvLocation = glGetUniformLocation(gShaderProgram, "mv");
  gProjLocation = glGetUniformLocation(gShaderProgram, "proj");
  glDeleteShader(gVertexShader);
  glDeleteShader(gFragmentShader);
}
