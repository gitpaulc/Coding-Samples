#include "math.h"
#include <stdexcept>
#include <cmath>

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

  double expOneVar(double x)
  {
    if (abs(x) >= 1.0)
    {
      double ans = expOneVar(x / 2.0);
      return ans * ans;
    }
    double xPower = 1.0;
    double ans = 1.0;
    double factorialNum = 1.0;
    for (int i = 1; i <= 10; ++i)
    {
      xPower *= x;
      factorialNum *= i;
      ans += xPower / factorialNum;
    }
    return ans;
  }

  double exponential(double x, double y)
  {
    rescale(x, y);
    //return y - expOneVar(x);
    return y - std::exp(x);
  }

  double natlog(double x, double y)
  {
    rescale(x, y);
    //return expOneVar(y) - x;
    return y - std::log(x);
  }

  double ellipticcurve(double x, double y)
  {
    Math& math = Math::Get();
    rescale(x, y);
    double aa = math.param1;
    double bb = math.param2;
    double ans = (x * x + aa) * x + bb;
    return y * y - ans;
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
  math_type = "line";
  if (mathType.empty() || (mathType.compare("0") == 0) || (mathType.compare("line") == 0))
  {
    math_type = "line";
    Function = &linear;
    return;
  }
  if ((mathType.compare("1") == 0) || (mathType.compare("parabola") == 0))
  {
    math_type = "parabola";
    Function = &parabola;
    return;
  }
  if ((mathType.compare("2") == 0) || (mathType.compare("cubic") == 0))
  {
    math_type = "cubic";
    Function = &cubic;
    return;
  }
  if ((mathType.compare("3") == 0) || (mathType.compare("circle") == 0))
  {
    math_type = "circle";
    scale = 1.5;
    Function = &circle;
    return;
  }
  if ((mathType.compare("4") == 0) || (mathType.compare("hyperbola") == 0))
  {
    math_type = "hyperbola";
    scale = 4;
    Function = &hyperbola;
    return;
  }
  if ((mathType.compare("5") == 0) || (mathType.compare("sine") == 0))
  {
    math_type = "sine";
    scale = 6;
    Function = &sine;
    return;
  }
  if ((mathType.compare("6") == 0) || (mathType.compare("sinc") == 0))
  {
    math_type = "sinc";
    scale = 10;
    Function = &sinc;
    return;
  }
  if ((mathType.compare("7") == 0) || (mathType.compare("exp") == 0))
  {
    math_type = "exp";
    scale = 4;
    Function = &exponential;
    return;
  }
  if ((mathType.compare("8") == 0) || (mathType.compare("log") == 0))
  {
    math_type = "log";
    scale = 4;
    Function = &natlog;
    return;
  }
  if ((mathType.compare("9") == 0) || (mathType.compare("ellipticcurve") == 0))
  {
    math_type = "ellipticcurve";
    scale = 4;
    Function = &ellipticcurve;
    return;
  }
  Function = &linear;
}

bool Math::canDivideByZero() const
{
  if (math_type.compare("hyperbola") == 0) { return true; }
  return false;
}
