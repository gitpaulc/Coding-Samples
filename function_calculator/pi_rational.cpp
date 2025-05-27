/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "pi_rational.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{

  PiRational::PiRational(const PiPolynomial& nn, const PiPolynomial& dd)
  {
    if (dd == PiPolynomial(0))
    {
      throw std::invalid_argument("Division by zero.");
    }
    else
    {
      auto gcd_ = PiPolynomial::gcd(nn, dd);
      num = nn; denom = dd;
      if (gcd_ != PiPolynomial(0))
      {
        PiPolynomial remainder;
        num = num.division(gcd_, remainder);
        denom = denom.division(gcd_, remainder);
      }
    }
  }
}
