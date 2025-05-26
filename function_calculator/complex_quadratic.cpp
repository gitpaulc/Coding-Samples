/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#include "complex_quadratic.h"

#include <stdexcept>
#include <sstream>

namespace FunctionalCalculator
{
  ComplexQuadratic::ComplexQuadratic(const QuadraticNumber& self)
  {
    re = self;
  }

  std::pair<double, double> ComplexQuadratic::get() const
  {
    return { re.get().first, im.get().first };
  }

  std::string ComplexQuadratic::print(bool useParentheses) const
  {
    std::stringstream strm;
    if (useParentheses) { strm << "("; }
    strm << re.print(true);
    strm << " + ";
    strm << im.print(true);
    strm << " * i";
    if (useParentheses) { strm << ")"; }
    return strm.str();
  }

  ComplexQuadratic ComplexQuadratic::sqrt(const Rational& radicand)
  {
    ComplexQuadratic answer;
    if (radicand < 0) { answer.im = QuadraticNumber::sqrt(-radicand); }
    else { answer.re = QuadraticNumber::sqrt(radicand); }
    return answer;
  }
}
