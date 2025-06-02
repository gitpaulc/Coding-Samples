/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

#ifndef DYNAMIC_MATRIX_H
#define DYNAMIC_MATRIX_H

#include "number.h"

#include <vector>

namespace FunctionalCalculator
{

class Matrix : public Number
{
  std::vector<std::vector<Number> > rows;
public:

  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;
};
}

#endif //def DYNAMIC_MATRIX_H
