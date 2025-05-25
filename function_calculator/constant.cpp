
#include "constant.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  double Constant::eval(double x, double y, double z) const
  {
    return value.get();
  }
}
