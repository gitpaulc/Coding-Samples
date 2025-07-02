/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef COMPLEX_QUADRATIC_H
#define COMPLEX_QUADRATIC_H

#include "biquadratic_number.h"

namespace FunctionalCalculator
{

class ComplexQuadratic : public Number
{
  BiquadraticNumber re; /**< The real part of the complex number. */
  BiquadraticNumber im; /**< The imaginary part of the complex number. */
public:
  ComplexQuadratic(int);
  ComplexQuadratic(const BiquadraticNumber& reIn = BiquadraticNumber(), const BiquadraticNumber& imIn = BiquadraticNumber());
  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;
  BiquadraticNumber getRe() const;
  BiquadraticNumber getIm() const;
  ComplexQuadratic conjugate() const;
  bool isReal() const;
  BiquadraticNumber sqLength() const;
  static ComplexQuadratic sqrt(const Rational& radicand);
  static ComplexQuadratic sqrtOfITimes(const Rational& radicand);

  ComplexQuadratic operator+() const;
  ComplexQuadratic operator-() const;
  ComplexQuadratic operator+(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator-(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator*(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator/(const ComplexQuadratic& rhs) const;
  ComplexQuadratic pow(int p) const; /**< `return` The p'th power of the number. */
  static ComplexQuadratic aPlusBTotheP(const ComplexQuadratic& aa, const ComplexQuadratic& bb, unsigned int p);
  bool operator==(const ComplexQuadratic& rhs) const;
  bool operator!=(const ComplexQuadratic& rhs) const;
  /** \brief Uses lexicographical comparison since there is no canonical ordering on complex numbers. */
  bool operator<(const ComplexQuadratic& rhs) const;
  /** \brief Uses lexicographical comparison since there is no canonical ordering on complex numbers. */
  bool operator>(const ComplexQuadratic& rhs) const;
};
}

#endif //def COMPLEX_QUADRATIC_H
