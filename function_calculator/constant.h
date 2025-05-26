/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef CONSTANT_H
#define CONSTANT_H

#include "function.h"

#include <vector>

namespace FunctionalCalculator
{

class Constant : public Function
{
  Number value;
public:
  virtual Number eval(const Number& x, const Number& y, const Number& z) const override;
};
}

#endif //def CONSTANT_H
