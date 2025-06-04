/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef PI_POLYNOMIAL_H
#define PI_POLYNOMIAL_H

#include "complex_quadratic.h"

#include <map>
#include <vector>

namespace FunctionalCalculator
{

class PiPolynomial : public Number
{
  std::map<int, ComplexQuadratic> self;
  void clean();
public:
  PiPolynomial(const ComplexQuadratic& coeff = ComplexQuadratic(), int power = 0);
  PiPolynomial(const std::vector<ComplexQuadratic>& coeffs);
  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;

  static double piValue();

  PiPolynomial operator+() const;
  PiPolynomial operator-() const;
  PiPolynomial operator+(const PiPolynomial& rhs) const;
  PiPolynomial operator-(const PiPolynomial& rhs) const;
  PiPolynomial operator*(const PiPolynomial& rhs) const;
  PiPolynomial operator*(const Rational& rhs) const;
  int degree() const;
  PiPolynomial division(const PiPolynomial& rhs, PiPolynomial& remainder) const;
  static PiPolynomial gcd(const PiPolynomial& aa, const PiPolynomial& bb);
  PiPolynomial pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const PiPolynomial& rhs) const;
  bool operator!=(const PiPolynomial& rhs) const;
  bool operator<(const PiPolynomial& rhs) const;

  PiPolynomial conjugate() const;
  bool isReal() const;
  PiPolynomial re() const;
  PiPolynomial im() const;

  /** \return The number as ComplexQuadratic if it has zero degree.
   *
   *  \exception Throws an invalid argument exception if the pi polynomial has degree > 0.
   */
  ComplexQuadratic conversion() const;
};
}

#endif //def PI_POLYNOMIAL_H
