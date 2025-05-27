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
 *  \remark In fact, the powers of x, y, and z are allowed to be negative.
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

  static FnPolynomial xToPower(const ComplexQuadratic& coeff, int p); /**< \return coeff * x^p */
  static FnPolynomial yToPower(const ComplexQuadratic& coeff, int p); /**< \return coeff * y^p */
  static FnPolynomial zToPower(const ComplexQuadratic& coeff, int p); /**< \return coeff * z^p */
  static FnPolynomial eToTheATimesPiX(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * x} */
  static FnPolynomial eToTheATimesPiY(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * y} */
  static FnPolynomial eToTheATimesPiZ(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * e^{A * Pi * z} */
  /** \brief \return e^{Pi * (A * x + B * y + C * z)} */
  static FnPolynomial eToThePi_AX_Plus_BY_CZ(const ComplexQuadratic& coeff,
    const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);
  static FnPolynomial sinATimesPiX(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * x) */
  static FnPolynomial sinATimesPiY(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * y) */
  static FnPolynomial sinATimesPiZ(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * sin(A * Pi * z) */
  static FnPolynomial cosATimesPiX(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * x) */
  static FnPolynomial cosATimesPiY(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * y) */
  static FnPolynomial cosATimesPiZ(const ComplexQuadratic& coeff, const ComplexQuadratic& A); /**< \return coeff * cos(A * Pi * z) */

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
