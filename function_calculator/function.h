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

  static Function constant(const PiRational& coeff);

  // MATRIX COMPOSITION:

  /** \brief If the function is F(u), returns F(M * u) where M is the matrix and u is a 3d vector (x, y, z). */
  Function composeWith(const Matrix<ComplexQuadratic>& transform) const;

  // TRIG FUNCTIONS:

  static Function tanATimesPiX(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * tan(A * Pi * x) */
  static Function tanATimesPiY(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * tan(A * Pi * y) */
  static Function tanATimesPiZ(const PiRational& coeff, const ComplexQuadratic& A); /**< \return coeff * tan(A * Pi * z) */
  /** \brief \return tan(Pi * (A * x + B * y + C * z)) */
  static Function tanPi_AX_plus_BY_plus_CZ(const PiRational& coeff,
      const ComplexQuadratic& A, const ComplexQuadratic& B, const ComplexQuadratic& C);

  // DIFFERENTIATION:

  Function partial_x() const;
  Function partial_y() const;
  Function partial_z() const;
  Function laplacian() const;

  bool isLaplaceEigenfunction(PiRational& eigenvalue) const;
  bool isHarmonic() const;

  // BESSEL FUNCTIONS:

  /** \brief Solves x^2 * F''(x) + 2x * F'(x) + (x^2 - n(n+1)) * F(x) = 0 after change of variables. */
  static Function sphericalBesselATimesPiX(const ComplexQuadratic& A, int n);

  /** \brief Solves x^2 * F''(x) + 2x * F'(x) + (x^2 - n(n+1)) * F(x) = 0 after change of variables. */
  static Function sphericalNeumannATimesPiX(const ComplexQuadratic& A, int n);

  // EXACT EVALUATION:

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtX(const ComplexQuadratic& xVal, Function& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtY(const ComplexQuadratic& yVal, Function& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtZ(const ComplexQuadratic& zVal, Function& output) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a complex number and (a)^p is attempted for nonnegative integers p, it should always succeed.
   *  \remark Currently when a is a real number and e^a is attempted, evaluation only succeeds for a == 0. This is because e polynomials are unsupported.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  bool tryEvaluateAtXYZ(const ComplexQuadratic& xVal, const ComplexQuadratic& yVal, const ComplexQuadratic& zVal, PiRational& output) const;
};
}

#endif //def FUNCTION_H
