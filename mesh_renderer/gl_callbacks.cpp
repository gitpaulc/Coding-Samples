
#include "gl_callbacks.h"

#include "includes.h"
#include "camera.h"
#include "edge_list.h"
#include "primitives.h"

#include <sstream>

namespace MeshRenderer
{
  static std::vector<ComputationalGeometry::Edge3d> gWireframe, gAvgNormals;
  static GLuint gVertexArrayObj, gVertexBufferObj;
  static bool gPointsHidden = false;
  static bool gDepthBuffering = true;
  static bool gEdgesHidden = false;

  static double gRenderRed = 0.0;
  static double gRenderGreen = 0.0;
  static double gRenderBlue = 1.0;

  static double gRenderRedMax = 1.0;
  static double gRenderGreenMax = 1.0;
  static double gRenderBlueMax = 1.0;

#ifdef _WIN64
  enum class RenderState
  {
    Wireframe = 0,
    Opaque = 1,
    NormalsShading = 2,
    TextureMap = 3,
    NumStates = 4
  };
#else
#ifdef _WIN32
  // Win32:
  enum class RenderState
  {
    Wireframe = 0,
    Opaque = 1,
    NormalsShading = 2,
    TextureMap = 3,
    NumStates = 4
  };
#else
  // Otherwise...
  enum class RenderState
  {
    Wireframe = 0,
    Opaque = 1,
    NumStates = 2
  };
#endif
#endif
  RenderState gRenderState = RenderState::Wireframe;
#ifdef _WIN64
  static bool stateIsNormalsShading() { return (gRenderState == RenderState::NormalsShading); }
  static bool stateIsTextureMap() { return (gRenderState == RenderState::TextureMap); }
#else
#ifdef _WIN32
  static bool stateIsNormalsShading() { return (gRenderState == RenderState::NormalsShading); }
  static bool stateIsTextureMap() { return (gRenderState == RenderState::TextureMap); }
#else
  static bool stateIsNormalsShading() { return false; }
  static bool stateIsTextureMap() { return false; }
#endif
#endif

  static bool gUseFaceNormals = false;

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
  static bool gUseShaders = true;
  static GLint gPosLocation, gNormalsLocation, gMvLocation, gProjLocation, gMinLocation, gMaxLocation;
  static GLint gAmbientLocation, gDiffuseLocation, gSpecularLocation;
  static ComputationalGeometry::point3d gBoundingMax, gBoundingMin;
  static float g_kA, g_kD, g_kS;
  static bool gTextureMapEnabled = false;
  static GLuint gTextureId;
}

void recalculate();

int& GetWindowId()
{
  static int window_id = -1;
  return window_id;
}

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut,
                    bool triangles, bool useShaders, bool normals);
void linkShaderProgram();

void enableTextureMap(bool on)
{
  MeshRenderer::gTextureMapEnabled = on;
}

void initialize_glut(int* argc_ptr, char** argv)
{
  MeshRenderer::gRenderRed = 0.0;
  MeshRenderer::gRenderGreen = 0.0;
  MeshRenderer::gRenderBlue = 1.0;

  MeshRenderer::gRenderRedMax = 1.0;
  MeshRenderer::gRenderGreenMax = 1.0;
  MeshRenderer::gRenderBlueMax = 1.0;

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

#ifdef __APPLE__
  glGenVertexArraysAPPLE(1, &MeshRenderer::gVertexArrayObj);
#else
  glGenVertexArrays(1, &MeshRenderer::gVertexArrayObj);
#endif
  glGenBuffers(1, &MeshRenderer::gVertexBufferObj);

  {
    ComputationalGeometry::point3d minPt, maxPt;
    double scale = (maxPt - minPt).sqNorm();
    if (scale < 0.001) { scale = 1.0; }
    scale = 0.2 / scale;
    MeshRenderer::gScale = scale;
  }

  MeshRenderer::g_kA = 0.0f;
  MeshRenderer::g_kD = 0.9f;
  MeshRenderer::g_kS = 0.0f;

  glGenTextures(1, &MeshRenderer::gTextureId);
  glBindTexture(GL_TEXTURE_2D, MeshRenderer::gTextureId);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  recalculate();
  linkShaderProgram();
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  auto& gCam = GetCamera(mesh);
  gCam.setEye(ComputationalGeometry::point3d());
  keyboard('J', 0, 0);
}

void recalculate()
{
  const auto& mesh = MeshRenderer::DoublyConnectedEdgeList::Get();
  if (MeshRenderer::gRenderState == MeshRenderer::RenderState::Wireframe)
  {
    mesh.getWireframe(MeshRenderer::gWireframe);
  }
  else
  {
    mesh.getSkeleton(MeshRenderer::gWireframe, MeshRenderer::gAvgNormals);
  }
  mesh.getBoundingBox(MeshRenderer::gBoundingMax, MeshRenderer::gBoundingMin);
}

void vertex2color(const float& xIn, const float& yIn, const float& zIn,
  float& rOut, float& gOut, float& bOut)
{
  ComputationalGeometry::point3d rgbMax(MeshRenderer::gRenderRedMax, MeshRenderer::gRenderGreenMax, MeshRenderer::gRenderBlueMax);
  ComputationalGeometry::point3d rgbMin(MeshRenderer::gRenderRed, MeshRenderer::gRenderGreen, MeshRenderer::gRenderBlue);
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
    gRenderState = static_cast<RenderState>(static_cast<int>(gRenderState) + 1);
    if (stateIsTextureMap() && (!gTextureMapEnabled))
    {
      gRenderState = static_cast<RenderState>(static_cast<int>(gRenderState) + 1);
    }
    if (gRenderState == RenderState::NumStates)
    {
      gRenderState = static_cast<RenderState>(0);
    }
    recalculate();
  }
  if (key == '1')
  {
    gUseShaders = !gUseShaders;
    if (gUseShaders) { std::cout << "\nOpenGL vertex and fragment shading on."; }
    else { std::cout << "\nOpenGL vertex and fragment shading off."; }
    recalculate();
  }
#ifndef __APPLE__
  else if (key == '2')
  {
    gUseFaceNormals = !gUseFaceNormals;
    if (gUseFaceNormals) { std::cout << "\nUsing face normals for shading."; }
    else { std::cout << "\nUsing average normals for shading (e.g., Gouraud shading)."; }
    recalculate();
  }
#endif
  else if ((key == 27) //Esc
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

  bool wireframeOn = (gRenderState == RenderState::Wireframe);
  std::vector<float> vertexData;
  toVertex3dData(gWireframe, vertexData, !wireframeOn, gUseShaders, stateIsNormalsShading());
    
  if (gPointsHidden || gEdgesHidden) { vertexData.resize(0); }
  else if (stateIsTextureMap())
  {
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    return;
  }
  else if (!wireframeOn && gUseShaders)
  {
#ifdef __APPLE__ // To GPU.
    glBindVertexArrayAPPLE(gVertexArrayObj);
#else
    glBindVertexArray(gVertexArrayObj);
#endif
    glBindBuffer(GL_ARRAY_BUFFER, gVertexBufferObj);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float), vertexData.data(), GL_STATIC_DRAW);
    gPosLocation = 0;
#ifdef __APPLE__
    int sizeMultiplier = 3;
#else
    int sizeMultiplier = 6; // 3 for triangle vertices, 3 for face normals at vertices.
#endif
    glVertexAttribPointer(gPosLocation, 3, GL_FLOAT, GL_FALSE,
      sizeMultiplier * sizeof(vertexData.data()[0]), (void*)0); // sizeof(float)
    glEnableVertexAttribArray(gPosLocation);

#ifdef __APPLE__
#else
    gNormalsLocation = gPosLocation + 1;
    glVertexAttribPointer(gNormalsLocation, 3, GL_FLOAT, GL_FALSE,
      sizeMultiplier * sizeof(vertexData.data()[0]), (void*)(3 * sizeof(vertexData.data()[0])));
    glEnableVertexAttribArray(gNormalsLocation);
#endif

    glUseProgram(gShaderProgram);
    glUniform1f(gMaxLocation, (float)gBoundingMax.z);
    glUniform1f(gMinLocation, (float)gBoundingMin.z);
    glUniform1f(gAmbientLocation, g_kA);
    glUniform1f(gDiffuseLocation, g_kD);
    glUniform1f(gSpecularLocation, g_kS);
    glUniformMatrix4fv(gProjLocation, 1, GL_FALSE, (const GLfloat*)gProjMatrix.data());
    glUniformMatrix4fv(gMvLocation, 1, GL_FALSE, (const GLfloat*)gMvMatrix.data());
    int whichArray = 0;
    glDrawArrays(GL_TRIANGLES, whichArray, static_cast<GLsizei>(vertexData.size() / 3));

    glDisableVertexAttribArray(gPosLocation);
#ifndef __APPLE__
    glDisableVertexAttribArray(gNormalsLocation);
    glBindVertexArray(0);
#else
    glBindVertexArrayAPPLE(0);
#endif
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glutSwapBuffers();
    return;
  }
  else if (gRenderState != RenderState::Wireframe)
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

void toVertex3dData(const std::vector<ComputationalGeometry::Edge3d>& dataIn, std::vector<float>& dataOut,
                    bool triangles, bool useShaders, bool normals)
{
  const auto oldSize = dataIn.size();
  if (triangles)
  {
#ifdef __APPLE__
    useShaders = false;
#endif
    int sizeMultiplier = useShaders ? 6 : 3;
    const auto newSize = dataIn.size() * sizeMultiplier;
    dataOut.resize(newSize);
    if (oldSize == 0) { return; }
    for (int ii = 0; ii < oldSize; ii += 3)
    {
      ComputationalGeometry::vector3d nn;
      if (useShaders && normals)
      {
        if (MeshRenderer::gUseFaceNormals || (ii >= (int)MeshRenderer::gAvgNormals.size()))
        {
          ComputationalGeometry::Plane3d plane(dataIn[ii].a, dataIn[ii + 1].a, dataIn[ii + 2].a);
          nn = plane.getNormal();
        }
        else
        {
          ComputationalGeometry::point3d zero_;
          nn = MeshRenderer::gAvgNormals[ii].a - zero_;
        }
      }
      for (int jj = 0; jj < 3; ++jj)
      {
        auto ind = ii + jj;
        if (ind > oldSize) { break; }
        dataOut[ind * sizeMultiplier] = (float)dataIn[ind].a.x;
        dataOut[ind * sizeMultiplier + 1] = (float)dataIn[ind].a.y;
        dataOut[ind * sizeMultiplier + 2] = (float)dataIn[ind].a.z;
        // Add normals, they are zero if normals == false.
        if (useShaders)
        {
          dataOut[ind * sizeMultiplier + 3] = (float)nn.x;
          dataOut[ind * sizeMultiplier + 4] = (float)nn.y;
          dataOut[ind * sizeMultiplier + 5] = (float)nn.z;
        }
      }
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
#ifdef __APPLE__
  glsl << "\n#version 110";
#else
  glsl << "\n#version 330 core";
#endif
  glsl << "\nuniform mat4 mv;"; // mv = VIEW * MODEL
  glsl << "\nuniform mat4 proj;";
  glsl << "\nuniform float maxHeight;";
  glsl << "\nuniform float minHeight;";
  glsl << "\nuniform float kA;";
  glsl << "\nuniform float kD;";
  glsl << "\nuniform float kS;";
#ifdef __APPLE__
  glsl << "\nattribute vec3 posVec;";
  glsl << "\nvec3 normalVec = vec3(0.0, 0.0, 0.0);";
  glsl << "\nvarying vec3 fragColor;";
#else
  glsl << "\nlayout(location = 0) in vec3 posVec;";
  glsl << "\nlayout(location = 1) in vec3 normalVec;";
  glsl << "\nout vec3 fragColor;";
#endif
  glsl << "\nvoid main()";
  glsl << "\n{";
  glsl << "\n  gl_Position = proj * mv * vec4(posVec, 1.0);";
  glsl << "\n  float tt = 0.0;";
  glsl << "\n  float normSq = normalVec.x * normalVec.x + normalVec.y * normalVec.y + normalVec.z * normalVec.z;";
  glsl << "\n  if (normSq < 0.5)"; // Rendering via normals is off.
  glsl << "\n  {";
  glsl << "\n    if (maxHeight != minHeight)";
  glsl << "\n    {";
  glsl << "\n      float factor = 0.7;";
  glsl << "\n      tt = (factor * posVec.z - minHeight) / (maxHeight - minHeight);";
  glsl << "\n    }";
  glsl << "\n  }";
  glsl << "\n  else"; // Rendering via normals is on.
  glsl << "\n  {";
  glsl << "\n    if (maxHeight != minHeight)";
  glsl << "\n    {";
  glsl << "\n      float factor = normalVec.z * kD;";
  glsl << "\n      if (factor < 0.0) { factor = -factor; }";
  glsl << "\n      tt = (factor * posVec.z - minHeight) / (maxHeight - minHeight);";
  glsl << "\n    }";
  glsl << "\n  }";
  double rMax = MeshRenderer::gRenderRedMax;
  double gMax = MeshRenderer::gRenderGreenMax;
  double bMax = MeshRenderer::gRenderBlueMax;
  double rMin = MeshRenderer::gRenderRed;
  double gMin = MeshRenderer::gRenderGreen;
  double bMin = MeshRenderer::gRenderBlue;
  glsl << "\n  float rr = " << rMax << " * tt" << " + " << rMin << " * (1.0 - tt);";
  glsl << "\n  float gg = " << gMax << " * tt" << " + " << gMin << " * (1.0 - tt);";
  glsl << "\n  float bb = " << bMax << " * tt" << " + " << bMin << " * (1.0 - tt);";
  glsl << "\n  fragColor = vec3(rr, gg, bb);";
  glsl << "\n}";
  return glsl.str();
}

std::string glslFragmentShaderCode()
{
  std::stringstream glsl;
#ifdef __APPLE__
  glsl << "\n#version 110";
  glsl << "\nvarying vec3 fragColor;";
#else
  glsl << "\n#version 330 core";
  glsl << "\nin vec3 fragColor;";
  glsl << "\nout vec4 outFragColor;";
#endif
  glsl << "\nvoid main()";
  glsl << "\n{";
#ifdef __APPLE__
  glsl << "\n  gl_FragColor = vec4(fragColor, 1.0);";
#else
  glsl << "\n  outFragColor = vec4(fragColor, 1.0);";
#endif
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
  gMaxLocation = glGetUniformLocation(gShaderProgram, "maxHeight");
  gMinLocation = glGetUniformLocation(gShaderProgram, "minHeight");
  gAmbientLocation = glGetUniformLocation(gShaderProgram, "kA");
  gDiffuseLocation = glGetUniformLocation(gShaderProgram, "kD");
  gSpecularLocation = glGetUniformLocation(gShaderProgram, "kS");
  gMvLocation = glGetUniformLocation(gShaderProgram, "mv");
  gProjLocation = glGetUniformLocation(gShaderProgram, "proj");
  glDeleteShader(gVertexShader);
  glDeleteShader(gFragmentShader);
}
