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

  std::string Number::print(bool useParentheses) const
  {
    throw std::exception("\nNot implemented.");
    return useParentheses ? "()" : "";
  }
}
