/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef RATIONAL_H
#define RATIONAL_H

namespace ComputationalGeometry
{
class Rational
{
  int num = 0;
  int denom = 1;
public:
  Rational(int nn = 0, int dd = 1);
  
  // Rule of Five: define copy constructor, copy assignment operator,
  // destructor, move constructor, move assigment operator.
  // noexcept requires g++ compilation with at least -std=c++11
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
};
}

#endif //def RATIONAL
