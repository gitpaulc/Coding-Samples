/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef QUADRATIC_NUMBER_H
#define QUADRATIC_NUMBER_H

#include "rational.h"

#include "dynamic_matrix.h"

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
  std::map<mp, Rational> content;
  /** \brief From Galois Theory, multiplication acts as a linear transformation upon vector space where the square roots are basis elements.
   *  \param root2Index is an output parameter that assigns a row index to its corresponding square root.
   *  \param index2Root is an output parameter that assigns to each row index its corresponding square root.
   */
  Matrix<Rational> getMultiplicationMatrix(std::map<mp, int>& root2Index, std::map<int, mp>& index2Root) const;

public:
  QuadraticNumber(const Rational& number = Rational(0, 1));
  virtual std::pair<double, double> get() const override;

  /** \return { a, b } where this number == a / b AND a has only integer coefficients. */
  std::pair<QuadraticNumber, mp> factorAsIntegral() const;
  bool getRational(Rational& self) const; /**< \return `true` iff the number is actually rational. Only then is self redefined. */
  virtual std::string print(bool useParentheses = false) const override;
  static QuadraticNumber sqrt(const Rational& radicand);
  QuadraticNumber abs() const;

  QuadraticNumber operator+() const;
  QuadraticNumber operator-() const;
  QuadraticNumber operator+(const QuadraticNumber& rhs) const;
  QuadraticNumber operator-(const QuadraticNumber& rhs) const;
  QuadraticNumber operator*(const QuadraticNumber& rhs) const;
  /** \brief Uses inversion of the multiplication operator. */
  QuadraticNumber operator/(const QuadraticNumber& rhs) const;
  QuadraticNumber pow(int p) const; /**< `return` The p'th power of the number. */
  bool operator==(const QuadraticNumber& rhs) const;
  bool operator!=(const QuadraticNumber& rhs) const;
  bool operator!=(int rhs) const;
  /** \remark Does not use algebra to determine < since it would be very inefficient. */
  bool operator<(const QuadraticNumber& rhs) const;
  bool operator>(const QuadraticNumber& rhs) const;

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  static bool tryGetCosine(const Rational& input, QuadraticNumber& output);

  /** \return `true` if and only if evaluation succeeds. Only then is the `output` parameter written.
   *  \remark Currently when a is a real QuadraticNumber and cos(pi * a) or sin(pi * a) are attempted, they should only succeed for well-known trig values.
   *  This means that a should be a multiple of 1/12 so that pi * a includes the usual values of pi / 2, pi / 4, pi / 3, and pi / 6.
   */
  static bool tryGetSine(const Rational& input, QuadraticNumber& output);
};
}

#endif //def QUADRATIC_NUMBER_H
