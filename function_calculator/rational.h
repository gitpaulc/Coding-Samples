/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef RATIONAL_H
#define RATIONAL_H

#include "number.h"

#include <map>
#include <set>

namespace FunctionalCalculator
{

class Rational : public Number
{
  int num = 0;
  int denom = 1;
public:
  Rational(int nn = 0, int dd = 1);

  Rational(const Rational&);
  Rational(Rational&&) noexcept;
  Rational& operator=(const Rational&);
  Rational& operator=(Rational&&) noexcept;
  ~Rational() = default;

  int denominator() const;
  int numerator() const;
  static int gcd(int aa, int bb);
  Rational operator+() const;
  Rational operator-() const;
  Rational operator+(const Rational& rhs) const;
  Rational operator-(const Rational& rhs) const;
  Rational operator*(const Rational& rhs) const;
  Rational operator/(const Rational& rhs) const;
  bool operator==(const Rational& rhs) const;
  bool operator!=(const Rational& rhs) const;
  bool operator<(const Rational& rhs) const;
  bool operator>(const Rational& rhs) const;
  bool operator<=(const Rational& rhs) const;
  bool operator>=(const Rational& rhs) const;
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  static std::map<int, int> primeFactorization(int input);
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  std::map<int, int> primeFactorization() const;
  /** \brief Print the prime factorization of the rational number. */
  std::string printFactors() const;

  virtual double get() const override;
  virtual std::string print() const override;
};
}

#endif //def RATIONAL_H
