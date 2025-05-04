#include "math.h"

namespace // anonymous
{
  void rescale(double& x, double& y)
  {
    Math& math = Math::Get();
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
    Function = &circle;
    return;
  }
  Function = &linear;
}
