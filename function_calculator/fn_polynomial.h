/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FN_POLYNOMIAL_H
#define FN_POLYNOMIAL_H

#include "complex_quadratic.h"

#include <map>
#include <vector>

namespace FunctionalCalculator
{

/** \class Represents a polynomial in x, y, z, and e^{pi * (a * x + b * y + c * z)}.
 * 
 *  Here a, b, and c are of the form A + B * sqrt(d) where d is an integer. The coefficients of
 *  the polynomial are themselves polynomials in pi whose coefficients are rational functions of
 *  A + B * sqrt(d) where d is an integer. Here d can be -1.
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
    ComplexQuadratic ePiXInd = 0;
    ComplexQuadratic ePiYInd = 0;
    ComplexQuadratic ePiZInd = 0;
    bool isConstTerm() const;
    Monomial operator+(const Monomial& rhs) const;
    bool operator<(const Monomial& rhs) const;
  };

  std::map<Monomial, ComplexQuadratic> self;
  void clean();
public:
  FnPolynomial(const ComplexQuadratic& coeff = ComplexQuadratic());
  virtual std::string print(bool useParentheses = false) const;

  static FnPolynomial xToPower(int p);
  static FnPolynomial yToPower(int p);
  static FnPolynomial zToPower(int p);
  static FnPolynomial eToTheATimesX(const ComplexQuadratic& A); /**< \return e^{A * x} */
  static FnPolynomial eToTheATimesY(const ComplexQuadratic& A); /**< \return e^{A * y} */
  static FnPolynomial eToTheATimesZ(const ComplexQuadratic& A); /**< \return e^{A * z} */
  /** \brief \return e^{A * x + B * y + C * z} */
  static FnPolynomial eToThe_AX_Plus_BY_CZ(const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);
  static FnPolynomial sinATimesX(const ComplexQuadratic& A); /**< \return sin(A * x) */
  static FnPolynomial sinATimesY(const ComplexQuadratic& A); /**< \return sin(A * y) */
  static FnPolynomial sinATimesZ(const ComplexQuadratic& A); /**< \return sin(A * z) */
  static FnPolynomial cosATimesX(const ComplexQuadratic& A); /**< \return cos(A * x) */
  static FnPolynomial cosATimesY(const ComplexQuadratic& A); /**< \return cos(A * y) */
  static FnPolynomial cosATimesZ(const ComplexQuadratic& A); /**< \return cos(A * z) */

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
