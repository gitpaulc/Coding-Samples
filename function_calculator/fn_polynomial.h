/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FN_POLYNOMIAL_H
#define FN_POLYNOMIAL_H

#include "complex_quadratic.h"

#include <map>
#include <vector>

namespace FunctionalCalculator
{

/** \class Represents a polynomial in x, y, z, e^{pi * x}, e^{pi * y}, e^{pi * z} where x, y, and z are rational complex numbers.
 * 
 *  The transcendental functions e^{pi * x}, e^{pi * y}, e^{pi * z} can effectively be treated as separate
 *  variables in the polynomial, giving us a polynomial in 6 variables. The coefficients of
 *  the polynomial are polynomials in pi with coefficients that are complex rational functions of
 *  square roots of rational numbers.
 * 
 *  \remark This illustrates a practical application of the fact that sqrt(d) is nonrational if d is an
 *  integer that is not a perfect square, and that pi is transcendental: It means we can check for exact equality
 *  by checking that a polynomial in one of these variables is equal to zero.
 */
class FnPolynomial
{
  struct Monomial
  {
    int xInd = 0;
    int yInd = 0;
    int zInd = 0;
    int ePiXInd = 0;
    int ePiYInd = 0;
    int ePiZInd = 0;
    bool isConstTerm() const;
    Monomial operator+(const Monomial& rhs) const;
    bool operator<(const Monomial& rhs) const;
  };

  std::map<Monomial, ComplexQuadratic> self;
  void clean();
public:
  FnPolynomial(const ComplexQuadratic& coeff = ComplexQuadratic());
  virtual std::string print(bool useParentheses = false) const;

  FnPolynomial operator+() const;
  FnPolynomial operator-() const;
  FnPolynomial operator+(const FnPolynomial& rhs) const;
  FnPolynomial operator-(const FnPolynomial& rhs) const;
  FnPolynomial operator*(const FnPolynomial& rhs) const;
  FnPolynomial pow(int p) const; /**< `return` The p'th power of the polynomial. */
  bool operator==(const FnPolynomial& rhs) const;
  bool operator!=(const FnPolynomial& rhs) const;
};
}

#endif //def FN_POLYNOMIAL_H
