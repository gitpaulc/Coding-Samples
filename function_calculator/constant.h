/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef CONSTANT_H
#define CONSTANT_H

#include "function.h"
#include "number.h"

#include <vector>

namespace FunctionalCalculator
{

class Constant : public Function
{
  Number value;
public:
  virtual double eval(double x, double y, double z) const override;
};
}

#endif //def CONSTANT_H
