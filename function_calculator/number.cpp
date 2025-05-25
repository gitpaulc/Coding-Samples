/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "number.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  double Number::get() const
  {
    throw std::exception("\nNot implemented.");
    return 0.0;
  }

  std::string Number::print() const
  {
    throw std::exception("\nNot implemented.");
    return "";
  }
}
