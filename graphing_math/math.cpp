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

  double ellipse(double x, double y)
  {
    Math& math = Math::Get();
    rescale(x, y);
    double aa = math.hasParam1 ? math.param1 : 1;
    double bb = math.hasParam2 ? math.param2 : 1;
    if (math.math_type.compare("circle") == 0) { bb = aa; }
    return 1.0 - x * x / (aa * aa) - y * y / (bb * bb);
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

  double exponentialFunc(double x, double y)
  {
    //return y - expOneVar(x);
    return y - std::exp(x);
  }

  double natlogFunc(double x, double y)
  {
    //return expOneVar(y) - x;
    return y - std::log(x);
  }

  double exponential(double x, double y)
  {
    rescale(x, y);
    Math& math = Math::Get();
    double coeff = math.hasParam1 ? std::log(math.param1) : 1;
    return y - std::exp(coeff * x);
  }

  double natlog(double x, double y)
  {
    rescale(x, y);
    Math& math = Math::Get();
    double coeff = math.hasParam1 ? (1.0 / std::log(math.param1)) : 1;
    return y - std::log(x) * coeff;
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

  double equilateral(double x, double y)
  {
    Math& math = Math::Get();
    rescale(x, y);
    double mm = math.hasParam1 ? math.param1 : 1;
    double nn = math.hasParam2 ? math.param2 : 1;
    double halfSqrt3 = std::sqrt(3.0) * 0.5;
    double piNum = Math::getPi();
    double xx = x;
    double yy = y - 0.5;
    double uu = sin((2.0 * piNum * mm / halfSqrt3) * yy);
    uu += sin((2.0 * piNum * nn / halfSqrt3) * (halfSqrt3 * xx - 0.5 * yy));
    uu += sin((2.0 * piNum * nn / halfSqrt3) * (halfSqrt3 - halfSqrt3 * xx - 0.5 * yy));
    return uu;
  }

  double regular(double x, double y)
  {
    Math& math = Math::Get();
    rescale(x, y);
    double mm0 = math.hasParam1 ? math.param1 : 3;
    double kk = math.hasParam2 ? math.param2 : 1;
    int mm = 3;
    if (mm0 > 3.0) { mm = (int)mm0; }
    mm0 = (double) mm;

    double piNum = Math::getPi();
    double xx = x;
    double yy = y;
    double zz = 0.0;
    double coeff = kk;
    double phase = piNum / 2.0;
    if (mm == 3)
    {
      if (!math.hasParam2) { kk = 8.0 * piNum; }
      phase = kk / 6.0;
    }

    double theta = 2.0 * piNum / mm0;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    double cosAngle = 1.0;
    double sinAngle = 0.0;
    for (int ii = 0; ii < mm; ++ii)
    {
      double summand = sin(coeff * (cosAngle * xx - sinAngle * yy) + phase);
      zz += summand;
      double tempCos = cosAngle;
      double tempSin = sinAngle;
      cosAngle = tempCos * cosTheta - tempSin * sinTheta;
      sinAngle = tempSin * cosTheta + tempCos * sinTheta;
    }
    return zz;
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

  struct MathType
  {
    int key = 0;
    std::string name = "line";
    double (*Function)(double x, double y);
    double scale = 1.0;
    std::string str() const
    {
      return std::to_string(key);
    }
    typedef double (*MathFunction)(double x, double y);
    static void Add(std::map<int, MathType>& mathMap, const std::string& nameIn, MathFunction funcIn, double scaleIn = 1.0)
    {
      int keyIn = (int)mathMap.size();
      MathType mathType;
      mathType.key = keyIn;
      mathType.name = nameIn;
      mathType.Function = funcIn;
      mathType.scale = scaleIn;
      mathMap[keyIn] = mathType;
    }
  };

  std::map<int, MathType> types;
  MathType::Add(types, "line", &linear);
  MathType::Add(types, "parabola", &parabola);
  MathType::Add(types, "cubic", &cubic);
  MathType::Add(types, "circle", &ellipse, 1.5);
  MathType::Add(types, "hyperbola", &hyperbola, 5);
  MathType::Add(types, "sine", &sine, 6);
  MathType::Add(types, "sinc", &sinc, 10);
  MathType::Add(types, "exp", &exponential, 4);
  MathType::Add(types, "log", &natlog, 4);
  MathType::Add(types, "ellipse", &ellipse, 1.5);
  MathType::Add(types, "ellipticcurve", &ellipticcurve, 4);
  MathType::Add(types, "equilateral", &equilateral);
  MathType::Add(types, "regular", &regular, 4);
  if (mathType.empty())
  {
    math_type = "line";
    Function = &linear;
    return;
  }
  for (auto& mathIt : types)
  {
    auto& mathTypeStruct = mathIt.second;
    if ((mathType.compare(mathTypeStruct.str()) == 0) || (mathType.compare(mathTypeStruct.name) == 0))
    {
      scale = mathTypeStruct.scale;
      math_type = mathTypeStruct.name;
      Function = mathTypeStruct.Function;
      return;
    }
  }

  Function = &linear;
}

bool Math::canDivideByZero() const
{
  if (math_type.compare("hyperbola") == 0) { return true; }
  return false;
}

double Math::getPi()
{
  return 3.1415926535;
}
