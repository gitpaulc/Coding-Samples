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
  std::cout << "\n\t5. sine\t6. sinc\t7. exp\t8. log";
  std::cout << "\n\t9. 9 a b OR ellipticcurve a b";
  std::cout << std::endl;
  std::string graphType = "";
  if (argc >= 2)
  {
    graphType = argv[1];
  }
  auto& math = Math::Get();
  if (argc >= 3)
  {
    std::string param1 = argv[2];
    if (!(param1.empty()))
    {
      math.hasParam1 = true;
      math.param1 = std::stod(param1);
    }
    else if ((graphType.compare("9") == 0) || (graphType.compare("ellipticcurve") == 0))
    {
      math.hasParam1 = true;
      math.param1 = -1;
      std::cout << "\n\tNo parameter \"a\" provided for the elliptic curve. Setting a = -1." << std::endl;
    }
  }
  if (argc >= 4)
  {
    std::string param2 = argv[3];
    if (!(param2.empty()))
    {
      math.hasParam2 = true;
      math.param2 = std::stod(param2);
    }
    else if ((graphType.compare("9") == 0) || (graphType.compare("ellipticcurve") == 0))
    {
      math.hasParam2 = true;
      math.param2 = 0;
      std::cout << "\n\tNo parameter \"b\" provided for the elliptic curve. Setting b = 0." << std::endl;
    }
  }
  math.SetType(graphType);
  Graphing::initialize(&argc, argv);
  glutMainLoop();

  return 0;
}
