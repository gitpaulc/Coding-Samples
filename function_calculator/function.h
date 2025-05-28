/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FUNCTION_H
#define FUNCTION_H

#include <string>

#include "fn_polynomial.h"

namespace FunctionalCalculator
{

/** \class Represents a rational function of x, y, z, and e^{pi * (a * x + b * y + c * z)}.
 * 
 *  Here a, b, and c are of the form A + B * sqrt(d) where d is an integer. The coefficients of
 *  the rational function are rational functions of pi and
 *  A + B * sqrt(d) where d is an integer. Here d can be -1.
 * 
 *  \remark This illustrates a practical application of the fact that sqrt(d) is nonrational if d is an
 *  integer that is not a perfect square, and that pi is transcendental: It means we can check for exact equality
 *  by checking that a polynomial in one of these variables is equal to zero.
 */
class Function
{
  FnPolynomial num, denom;

public:

  Function(const FnPolynomial& nn = PiRational(ComplexQuadratic(0)), const FnPolynomial& dd = PiRational(ComplexQuadratic(1)));

  virtual std::string print(bool useParentheses = false) const;

  FnPolynomial denominator() const;
  FnPolynomial numerator() const;
  Function operator+() const;
  Function operator-() const;
  Function operator+(const Function& rhs) const;
  Function operator-(const Function& rhs) const;
  Function operator*(const Function& rhs) const;
  Function operator/(const Function& rhs) const;
  Function pow(int p) const; /**< Returns the p'th power of the rational number. */
  bool operator==(const Function& rhs) const;
  bool operator!=(const Function& rhs) const;
  // TODO: This should be implemented lexicographically since it is intractable to compute in general.
  // Moreover the complex components should be compared lexicographically.
  // bool operator<(const Function& rhs) const;
};
}

#endif //def FUNCTION_H
