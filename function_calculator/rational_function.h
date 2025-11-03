/*  Copyright Paul Cernea, November 2025.
All Rights Reserved.*/

#ifndef RATIONAL_FUNCTION_H
#define RATIONAL_FUNCTION_H

#include <string>

#include "algebraic_polynomial.h"

namespace FunctionalCalculator
{

/** \class Represents a rational function of x_0, x_1, ..., x_{n - 1} where n is the dimension.
 */
class RationalFunction
{
  AlgebraicPolynomial num, denom;

public:

  RationalFunction(const AlgebraicPolynomial& nn = PiRational(ComplexQuadratic(0)), const AlgebraicPolynomial& dd = PiRational(ComplexQuadratic(1)));

  virtual std::string print(bool useParentheses = false) const;

  AlgebraicPolynomial denominator() const;
  AlgebraicPolynomial numerator() const;
  RationalFunction operator+() const;
  RationalFunction operator-() const;
  RationalFunction operator+(const RationalFunction& rhs) const;
  RationalFunction operator-(const RationalFunction& rhs) const;
  RationalFunction operator*(const RationalFunction& rhs) const;
  RationalFunction operator/(const RationalFunction& rhs) const;
  RationalFunction pow(int p) const; /**< Returns the p'th power of the rational number. */
  bool operator==(const RationalFunction& rhs) const;
  bool operator!=(const RationalFunction& rhs) const;
  // TODO: This should be implemented lexicographically since it is intractable to compute in general.
  // Moreover the complex components should be compared lexicographically.
  // bool operator<(const RationalFunction& rhs) const;

  static RationalFunction constant(const PiRational& coeff);
};
}

#endif //def RATIONAL_FUNCTION_H
