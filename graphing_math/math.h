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
  double scale = 1.0;
  double angle = 0.0;
};

#endif // def MATH_H
