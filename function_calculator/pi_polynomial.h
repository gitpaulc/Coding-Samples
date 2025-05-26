/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef PI_POLYNOMIAL_H
#define PI_POLYNOMIAL_H

#include "complex_quadratic.h"

#include <vector>

namespace FunctionalCalculator
{

class PiPolynomial : public Number
{
  std::vector<ComplexQuadratic> self; // TODO: Change this to a map.
  void clean();
public:
  PiPolynomial(const ComplexQuadratic& coeff = ComplexQuadratic(), int power = 0);
  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;

  static double piValue();

  PiPolynomial operator+() const;
  PiPolynomial operator-() const;
  PiPolynomial operator+(const PiPolynomial& rhs) const;
  PiPolynomial operator-(const PiPolynomial& rhs) const;
  PiPolynomial operator*(const PiPolynomial& rhs) const;
  PiPolynomial pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const PiPolynomial& rhs) const;
  bool operator!=(const PiPolynomial& rhs) const;
};
}

#endif //def PI_POLYNOMIAL_H
