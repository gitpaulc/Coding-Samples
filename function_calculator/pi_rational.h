/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef PI_RATIONAL_H
#define PI_RATIONAL_H

#include "pi_polynomial.h"

namespace FunctionalCalculator
{

class PiRational : public Number
{
  PiPolynomial num = PiPolynomial(0);
  PiPolynomial denom = PiPolynomial(1);
public:
  PiRational(const PiPolynomial& nn = PiPolynomial(0), const PiPolynomial& dd = PiPolynomial(1));

  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;

  PiPolynomial denominator() const;
  PiPolynomial numerator() const;
  PiRational operator+() const;
  PiRational operator-() const;
  PiRational operator+(const PiRational& rhs) const;
  PiRational operator-(const PiRational& rhs) const;
  PiRational operator*(const PiRational& rhs) const;
  PiRational operator/(const PiRational& rhs) const;
  PiRational pow(int p) const; /**< Returns the p'th power of the rational number. */
  bool operator==(const PiRational& rhs) const;
  bool operator!=(const PiRational& rhs) const;
  bool operator<(const PiRational& rhs) const;
  bool operator>(const PiRational& rhs) const;
  bool operator<=(const PiRational& rhs) const;
  bool operator>=(const PiRational& rhs) const;
};
}

#endif //def PI_RATIONAL_H
