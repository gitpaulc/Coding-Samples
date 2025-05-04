#include "graphing_math.h"
#include "math.h"

int main(int argc, char **argv)
{
  std::cout << "\nUsage:";
  std::cout << "\n\tGraph a 2d function f(x, y) = 0 by default.";
  std::cout << "\n\tPress \"M\" to view the positive/negative sets of a function z = f(x, y). Red > 0, Blue < 0.";
  std::cout << "\n\tPress \"Z\" to zoom in, \"Y\" to zoom out, \"R\" to rotate clockwise, \"T\" to rotate counter-clockwise.";
  std::cout << "\n\tPress \"I\" to scroll up, \"K\" to scroll down, \"J\" to scroll left, \"L\" to scroll right.";
  std::cout << "\n\tPress \"Q\" to or ESC to exit.";
  std::cout << "\n\tGraph a line by default or enter the number 0 or the word \"line\" to graph a line. Similarly...";
  std::cout << "\n\t1. parabola";
  std::cout << "\n\t2. cubic";
  std::cout << "\n\t3. circle";
  std::cout << "\n\t4. hyperbola";
  std::cout << "\n\t5. sine";
  std::cout << std::endl;
  std::string graphType = "";
  if (argc >= 2)
  {
    graphType = argv[1];
  }
  auto& math = Math::Get();
  math.SetType(graphType);
  Graphing::initialize(&argc, argv);
  glutMainLoop();

  return 0;
}
