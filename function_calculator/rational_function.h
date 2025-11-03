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

  unsigned int getDimension() const;
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
  RationalFunction partial_deriv(unsigned int index) const;
  RationalFunction partial_x() const;
  RationalFunction partial_y() const;
  RationalFunction partial_z() const;
  RationalFunction partial_w() const;
  RationalFunction laplacian() const;

  bool isLaplaceEigenfunction(PiRational& eigenvalue) const;
  bool isHarmonic() const;

  // EXACT EVALUATION:

  /** \brief Evaluate the polynomial on (x_0, x_1, ..., x_{n - 1}) */
  RationalFunction evaluateAt(const std::vector<PiRational>& input) const;
  /** \brief Evaluate the polynomial on a map whose keys are variables and values are inputs. */
  RationalFunction evaluateAt(const std::map<unsigned int, PiRational>& input) const;
  /** \return `true` if and only if evaluation returns a constant. Only then is the `output` parameter written.
   */
  bool tryEvaluate(const std::vector<PiRational>& input, PiRational& output) const;
};
}

#endif //def RATIONAL_FUNCTION_H
