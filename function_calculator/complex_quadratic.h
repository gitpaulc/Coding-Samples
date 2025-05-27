/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef COMPLEX_QUADRATIC_H
#define COMPLEX_QUADRATIC_H

#include "quadratic_number.h"

namespace FunctionalCalculator
{

class ComplexQuadratic : public Number
{
  QuadraticNumber re; /**< The real part of the complex number. */
  QuadraticNumber im; /**< The imaginary part of the complex number. */
public:
  ComplexQuadratic(int);
  ComplexQuadratic(const QuadraticNumber& reIn = QuadraticNumber(), const QuadraticNumber& imIn = QuadraticNumber());
  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;
  QuadraticNumber getRe() const;
  QuadraticNumber getIm() const;
  ComplexQuadratic conjugate() const;
  QuadraticNumber sqLength() const;
  static ComplexQuadratic sqrt(const Rational& radicand);
  static ComplexQuadratic sqrtOfITimes(const Rational& radicand);

  ComplexQuadratic operator+() const;
  ComplexQuadratic operator-() const;
  ComplexQuadratic operator+(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator-(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator*(const ComplexQuadratic& rhs) const;
  ComplexQuadratic operator/(const ComplexQuadratic& rhs) const;
  ComplexQuadratic pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const ComplexQuadratic& rhs) const;
  bool operator!=(const ComplexQuadratic& rhs) const;
  /** \brief Uses lexicographical comparison since there is no canonical ordering on complex numbers. */
  bool operator<(const ComplexQuadratic& rhs) const;
  /** \brief Uses lexicographical comparison since there is no canonical ordering on complex numbers. */
  bool operator>(const ComplexQuadratic& rhs) const;
};
}

#endif //def COMPLEX_QUADRATIC_H
