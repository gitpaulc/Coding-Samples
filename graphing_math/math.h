#ifndef MATH_H
#define MATH_H

#include "includes.h"

class Math
{
  public:
  Math();
  static Math& Get();
  double (*Function)(double x, double y);
  void SetType(const std::string& mathType);
  std::string math_type = "line";
  double scale = 1.0;
  double angle = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  bool hasParam1 = false;
  bool hasParam2 = false;
  double param1 = 0.0;
  double param2 = 0.0;
  bool canDivideByZero() const;
  static double getPi();
};

#endif // def MATH_H
