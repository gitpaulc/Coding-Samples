/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef COMPLEX_QUADRATIC_H
#define COMPLEX_QUADRATIC_H

#include "quadratic_number.h"

namespace FunctionalCalculator
{

/** \brief Base class from which numbers should derive. */
class ComplexQuadratic : public Number
{
  QuadraticNumber re; /**< The real part of the complex number. */
  QuadraticNumber im; /**< The imaginary part of the complex number. */
public:
  ComplexQuadratic(const QuadraticNumber& self = QuadraticNumber());
  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;
  static ComplexQuadratic sqrt(const Rational& radicand);
};
}

#endif //def COMPLEX_QUADRATIC_H
