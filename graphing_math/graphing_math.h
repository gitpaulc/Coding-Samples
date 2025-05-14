#ifndef GRAPHING_MATH_H
#define GRAPHING_MATH_H

#include "includes.h"

namespace Graphing
{
  extern int window_id;

  const int WindowWidth = 900;
  const int WindowHeight = 900;

  /**
   * 0 = 2D Graph: f(x, y) = 0
   * 1 = Positive/Negative Regions of 3D Graph:
   *    Red = Positive, Blue = Negative, z = f(x, y)
   */
  static int mode = 0;
  static bool showingAxes = true;
	
  void initialize(int * argc_ptr, char **argv);
  void keyboard(unsigned char key, int x, int y);
  void mouse(int button, int state, int x, int y);
  bool isBoundary(int x, int y, const double& scaleX, const double& scaleY);
  bool isNonnegative(int x, int y, const double& scaleX, const double& scaleY, bool& undefined);
  void render();
};

#endif // def GRAPHING_MATH_H
