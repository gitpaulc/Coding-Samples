/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef RATIONAL_H
#define RATIONAL_H

#include <map>
#include <set>
#include <string>

namespace FunctionalCalculator
{

class Rational
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
  double get() const;
  std::string print() const;
  /** \brief The keys are the prime factors, the values are the number of occurrences. */
  static std::map<int, int> primeFactorization(int input);
};
}

#endif //def RATIONAL
