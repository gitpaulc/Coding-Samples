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

  std::pair<double, double> PiRational::get() const
  {
    if (denom.isReal())
    {
      auto numerGet = num.get();
      auto toDivide = denom.get().first;
      numerGet.first /= toDivide;
      numerGet.second /= toDivide;
      return numerGet;
    }
    auto denomConj = denom.conjugate();
    auto numer = (num * denomConj).get();
    auto toDivide = (denom * denomConj).get().first;
    numer.first /= toDivide;
    numer.second /= toDivide;
    return numer;
  }

  std::string PiRational::print(bool useParentheses) const
  {
  }

}
