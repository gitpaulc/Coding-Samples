#include "math.h"
#include <stdexcept>

namespace // anonymous
{
  void rescale(double& x, double& y)
  {
    Math& math = Math::Get();
    x = x - math.origin_x;
    y = y - math.origin_y;
    double cosTheta = cos(math.angle);
    double sinTheta = sin(math.angle);
    double xTemp = x * cosTheta + y * (-sinTheta);
    double yTemp = x * sinTheta + y * cosTheta;
    x = math.scale * xTemp;
    y = math.scale * yTemp;
  }
  double linear(double x, double y)
  {
    rescale(x, y);
    double ans = x;
    return y - ans;
  }

  double parabola(double x, double y)
  {
    rescale(x, y);
    double ans = x * x;
    return y - ans;
  }

  double cubic(double x, double y)
  {
    rescale(x, y);
    double ans = x * x * x;
    return y - ans;
  }

  double circle(double x, double y)
  {
    rescale(x, y);
    return 1.0 - x * x - y * y;
  }

  double hyperbola(double x, double y)
  {
    rescale(x, y);
    if (abs(x) <= 1.0e-9)
    {
      throw std::runtime_error("Division by zero.");
      return 1;
    }
    double ans = 1.0 / x;
    return y - ans;
  }

  double sine(double x, double y)
  {
    rescale(x, y);
    double ans = sin(x);
    return y - ans;
  }

  double sinc(double x, double y)
  {
    rescale(x, y);
    if (abs(x) <= 1.0e-6)
    {
      double x2 = x * x;
      double ans = 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
      return y - ans;
    }
    double ans = sin(x) / x;
    return y - ans;
  }
}

Math::Math()
{
  Function = &linear;
}

Math& Math::Get()
{
  static Math math;
  return math;
}

void Math::SetType(const std::string& mathType)
{
  if (mathType.empty() || (mathType.compare("0") == 0) || (mathType.compare("line") == 0))
  {
    Function = &linear;
    return;
  }
  if ((mathType.compare("1") == 0) || (mathType.compare("parabola") == 0))
  {
    Function = &parabola;
    return;
  }
  if ((mathType.compare("2") == 0) || (mathType.compare("cubic") == 0))
  {
    Function = &cubic;
    return;
  }
  if ((mathType.compare("3") == 0) || (mathType.compare("circle") == 0))
  {
    scale = 1.5;
    Function = &circle;
    return;
  }
  if ((mathType.compare("4") == 0) || (mathType.compare("hyperbola") == 0))
  {
    scale = 4;
    Function = &hyperbola;
    return;
  }
  if ((mathType.compare("5") == 0) || (mathType.compare("sine") == 0))
  {
    scale = 6;
    Function = &sine;
    return;
  }
  if ((mathType.compare("6") == 0) || (mathType.compare("sinc") == 0))
  {
    scale = 10;
    Function = &sinc;
    return;
  }
  Function = &linear;
}
