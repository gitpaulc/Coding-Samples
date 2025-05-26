
#include "constant.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  Number Constant::eval(const Number& x, const Number& y, const Number& z) const
  {
    return value;
  }
}
