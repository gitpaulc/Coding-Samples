/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef RATIONAL_H
#define RATIONAL_H

#include "mp_integer.h"
#include "number.h"

#include <map>
#include <set>

namespace FunctionalCalculator
{

class Rational : public Number
{
  mp num = 0;
  mp denom = 1;
public:
  Rational(int nn = 0, int dd = 1);
  Rational(const mp& nn, const mp& dd);

  Rational(const Rational&);
  Rational(Rational&&) noexcept;
  Rational& operator=(const Rational&);
  Rational& operator=(Rational&&) noexcept;
  ~Rational() = default;

  mp denominator() const;
  mp numerator() const;
  /** \return { a, b } where a is the maximal number such that this number == a * a * b */
  std::pair<Rational, Rational> separateSquaredPart() const;
  Rational operator+() const;
  Rational operator-() const;
  Rational operator+(const Rational& rhs) const;
  Rational operator-(const Rational& rhs) const;
  Rational operator*(const Rational& rhs) const;
  Rational operator/(const Rational& rhs) const;
  Rational pow(int p) const; /**< Returns the p'th power of the rational number. */
  bool operator==(const Rational& rhs) const;
  bool operator!=(const Rational& rhs) const;
  bool operator<(const Rational& rhs) const;
  bool operator>(const Rational& rhs) const;
  bool operator<=(const Rational& rhs) const;
  bool operator>=(const Rational& rhs) const;
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  std::map<mp, int> primeFactorization() const;
  /** \brief Print the prime factorization of the rational number. */
  std::string printFactors(bool useParentheses = false) const;

  virtual std::pair<double, double> get() const override;
  virtual std::string print(bool useParentheses = false) const override;
};
}

#endif //def RATIONAL_H
