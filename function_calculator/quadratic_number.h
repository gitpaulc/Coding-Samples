/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef QUADRATIC_NUMBER_H
#define QUADRATIC_NUMBER_H

#include "rational.h"

#include <string>
#include <map>

namespace FunctionalCalculator
{

/** \brief A number which is the sum of square roots of integers. */
class QuadraticNumber : public Number
{
  /** \brief The keys represent which numbers the square roots are taken of. The values are coefficients.
   *
   * So, for example (33 / 4) + 2 * sqrt(2) + 4 * sqrt(3) - (20 / 7) * sqrt(6)
   * would be represented as:
   * content[1] = 33/ 4; content[2] = 2; content[3] = 4; content[6] = (-20 / 7);
   */
  std::map<int, Rational> content;

public:
  QuadraticNumber(const Rational& number = Rational(0, 1));
  virtual double get() const override;
  bool getRational(Rational& self) const; /**< \return `true` iff the number is actually rational. Only then is self redefined. */
  virtual std::string print(bool useParentheses = false) const override;
  static QuadraticNumber sqrt(const Rational& radicand);

  QuadraticNumber operator+() const;
  QuadraticNumber operator-() const;
  QuadraticNumber operator+(const QuadraticNumber& rhs) const;
  QuadraticNumber operator-(const QuadraticNumber& rhs) const;
  QuadraticNumber operator*(const QuadraticNumber& rhs) const;
  /** \brief Uses rationalization of the denominator. */
  QuadraticNumber operator/(const QuadraticNumber& rhs) const;
  QuadraticNumber pow(int p) const; /**< Returns the p'th power of the rational number. */
};
}

#endif //def QUADRATIC_NUMBER_H
