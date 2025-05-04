#include "graphing_math.h"
#include "math.h"

int Graphing::window_id;

void Graphing::initialize(int * argc_ptr, char **argv)
{
  // Initialize GLUT and create a window.
  glutInit(argc_ptr, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(Graphing::WindowWidth, Graphing::WindowHeight);

  const std::string windowCaption = "Graphing Math by Paul Cernea. Z/Y to zoom, R/T to rotate. M to toggle to 3d level sets: Red > 0, Blue < 0.";

  Graphing::window_id = glutCreateWindow(windowCaption.c_str());
	
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	
  glutSetCursor(GLUT_CURSOR_INFO);
	
  glutKeyboardFunc(Graphing::keyboard);
  glutMouseFunc(Graphing::mouse);
  glutDisplayFunc(Graphing::render);
}

void Graphing::keyboard(unsigned char key, int x, int y)
{
  if ((key == 27) // ESC
     || (key == 'q')
     || (key == 'Q'))
  {
    glutDestroyWindow(Graphing::window_id);
    exit(0);
  }
  else if ((key == 'm') || (key == 'M'))
  {
    if (mode == 0) { mode = 1; }
    else if (mode == 1) { mode = 0; }
  }
  else if ((key == 'z') || (key == 'Z'))
  {
    Math& math = Math::Get();
    math.scale *= 0.9;
  }
  else if ((key == 'y') || (key == 'Y'))
  {
    Math& math = Math::Get();
    math.scale *= 1.1;
  }
  else if ((key == 'r') || (key == 'R') || (key == 't') || (key == 'T'))
  {
    double factor = 1.0;
    if ((key == 't') || (key == 'T'))
    {
      factor = -1.0;
    }
    Math& math = Math::Get();
    double piNumber = 3.14159;
    math.angle += 0.1 * factor * piNumber;
    if (math.angle > 4.0 * piNumber)
    {
      math.angle -= 2.0 * piNumber;
    }
    else if (math.angle < -4.0 * piNumber)
    {
      math.angle += 2.0 * piNumber;
    }
  }
	
  glutPostRedisplay();
}

void Graphing::mouse(int button, int state, int x, int y)
{
  if((button == GLUT_LEFT_BUTTON) && (state == GLUT_UP))
  {
  }

  glutPostRedisplay();
}

bool Graphing::isNonnegative(int x, int y, const double& scaleX, const double& scaleY)
{
  auto& math = Math::Get();
  double val = math.Function(x * scaleX, y * scaleY);
  return (val >= 0);
}

bool Graphing::isBoundary(int x, int y, const double& scaleX, const double& scaleY)
{
  auto& math = Math::Get();
  double val = math.Function(x * scaleX, y * scaleY);
  if (val == 0) { return true; }
  for (int i = -1; i <= 1; ++i)
  {
    for (int j = -1; j <= 1; ++j)
    {
      if (abs(i) == abs(j)) { continue; }
      double other = math.Function((x + i) * scaleX, (y + j) * scaleY);
      if ((val > 0) && (other < 0)) { return true; }
      if ((val < 0) && (other > 0)) { return true; }
    }
  }
  return false;
}

void Graphing::render()
{
  using namespace Graphing;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPointSize(1.0f);
  glBegin(GL_POINTS);
  float bounds_x = 1.0 / WindowWidth;
  float bounds_y = 1.0 / WindowHeight;
  for (int i = -WindowWidth; i < WindowWidth; ++i)
  {
    glColor3f(0, 0, 0);
    glVertex2f(i * bounds_x, 0); // Draw x-axis.
  }
  for (int j = -WindowHeight; j < WindowHeight; ++j)
  {
    glColor3f(0, 0, 0);
    glVertex2f(0, j * bounds_y); // Draw y-axis.
  }
  for (int i = -WindowWidth; i < WindowWidth; ++i)
  {
    for (int j = -WindowHeight; j < WindowHeight; ++j)
    {
      double scale_x = 1.0 / WindowWidth;
      double scale_y = 1.0 / WindowHeight;
      if (mode == 0)
      {
        if (!isBoundary(i, j, scale_x, scale_y)) { continue; }
          
        glColor3f(255, 0, 0);
        glVertex2f(i * bounds_x, j * bounds_y);
        continue;
      }
      if (mode == 1)
      {
        glColor3f(0, 0, 255);
        if (isNonnegative(i, j, scale_x, scale_y)) {glColor3f(255, 0, 0);}
        glVertex2f(i * bounds_x, j * bounds_y);
      }
    }
  }
  glEnd();
	
  glutSwapBuffers();
}

